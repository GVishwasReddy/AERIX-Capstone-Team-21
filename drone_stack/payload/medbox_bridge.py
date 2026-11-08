"""MedBox <-> AERIX bridge: the drone is the BLE central, the box the peripheral.

Runs as aerix-payload.service from its own venv (bleak), as user pi.

RADIO ISOLATION - the reason this file looks the way it does.
The delivery handshake (aerix-ble.service) advertises on the Pi's ON-BOARD
controller, which also shares its antenna with Wi-Fi. This bridge only ever
uses the USB dongle:

  * the adapter is chosen by sysfs bus path, never by hciN number (the kernel
    numbers controllers in registration order, so a re-plugged dongle or a
    late UART attach can swap hci0/hci1);
  * with no USB adapter present the bridge reports "no_adapter" and waits -
    it NEVER falls back to the on-board radio;
  * it never restarts bluetoothd and never powers any adapter up or down;
  * it holds no connection and runs no scan unless the selected payload is
    "medbox", so a parcel flight leaves the dongle silent.

Everything the bridge knows reaches the GCS as one hub command,
`medbox_status`, and everything it does to the box comes from /api/state
(payload_kind + the navigator's drop_stage). The box is advisory: nothing in
here can delay or block a release, and a dead bridge only greys out a card.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import signal
import time
import urllib.request

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError
from dbus_fast import BusType, Message, MessageType
from dbus_fast.aio import MessageBus

from drone_stack.payload import medbox_protocol as proto

log = logging.getLogger("medbox")

GCS_BASE_URL = os.environ.get("GCS_BASE_URL", "http://127.0.0.1:8090")
HTTP_TIMEOUT_S = 2.0
POLL_IDLE_S = 1.0         # /api/state cadence normally
POLL_DROP_S = 0.25        # ...and while a precision drop is running
SCAN_TIMEOUT_S = 8.0
HEARTBEAT_S = 3.0         # re-post link state so the hub can spot a dead bridge
CMD_ACK_TIMEOUT_S = 2.5
CMD_RETRIES = 3


# ---------------------------------------------------------------- GCS http
def _http_json(path: str, body: dict | None = None) -> dict | None:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(f"{GCS_BASE_URL}{path}", data=data,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as r:
            return json.loads(r.read())
    except Exception as exc:  # noqa: BLE001 - GCS restarting is routine
        log.debug("GCS %s failed: %s", path, exc)
        return None


async def gcs_state() -> dict | None:
    # Never block the event loop on HTTP: bleak's D-Bus traffic runs on it.
    return await asyncio.to_thread(_http_json, "/api/state")


async def gcs_post_status(payload: dict) -> None:
    await asyncio.to_thread(_http_json, "/api/command",
                            {"cmd": "medbox_status", "params": payload})


# ---------------------------------------------------------------- BlueZ
async def release_stale_links(adapter: str) -> list[str]:
    """Disconnect MedBox links BlueZ still holds on OUR adapter.

    BlueZ, not this process, owns a BLE connection: if a previous bridge died
    without disconnecting (SIGKILL, crash, restart), bluetoothd keeps the link
    up, the box - connected - stops advertising, and every later scan comes
    back empty. Only device objects under /org/bluez/<adapter>/ are touched,
    so the handshake's controller is out of reach by construction.
    """
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    dropped: list[str] = []
    try:
        reply = await bus.call(Message(destination="org.bluez", path="/",
                                       interface="org.freedesktop.DBus.ObjectManager",
                                       member="GetManagedObjects"))
        if reply.message_type != MessageType.METHOD_RETURN:
            return dropped
        prefix = f"/org/bluez/{adapter}/dev_"
        for path, ifaces in reply.body[0].items():
            dev = ifaces.get("org.bluez.Device1")
            if not path.startswith(prefix) or not dev or not dev.get("Connected", None) \
                    or not dev["Connected"].value:
                continue
            uuids = [u.lower() for u in dev["UUIDs"].value] if "UUIDs" in dev else []
            name = dev["Name"].value if "Name" in dev else ""
            if proto.SERVICE_UUID not in uuids and not name.startswith("MEDBOX-"):
                continue
            await bus.call(Message(destination="org.bluez", path=path,
                                   interface="org.bluez.Device1", member="Disconnect"))
            dropped.append(dev["Address"].value if "Address" in dev else path)
    finally:
        bus.disconnect()
    return dropped


# ---------------------------------------------------------------- bridge
class MedBoxBridge:
    def __init__(self, key: bytes | None, key_note: str) -> None:
        self.key = key
        self.key_note = key_note
        self.client: BleakClient | None = None
        self.adapter: str | None = None
        self.device_name = ""
        self.device_addr = ""
        self.rssi: int | None = None
        self.link = "starting"
        self.last_status: dict | None = None
        self._status_evt = asyncio.Event()
        self._last_post = 0.0
        self._sent: dict[tuple[str, str], str] = {}   # (order, cmd) -> result
        self._cmd_lock = asyncio.Lock()
        # Each post runs in its own worker thread, so two in flight can reach
        # the hub in either order - which showed up as a phantom
        # connected -> scanning -> connected flap. One at a time, FIFO.
        self._post_lock = asyncio.Lock()
        # Operator RESET from the GCS: the last request id handled, and its
        # answer, echoed in every report until the hub has seen it.
        self._reset_done: int | None = None
        self._reset_ack: dict | None = None

    # -- reporting
    async def report(self, link: str | None = None, note: str = "", force: bool = False) -> None:
        if link is not None and link != self.link:
            log.info("link %s -> %s %s", self.link, link, note)
            self.link = link
            force = True
        now = time.monotonic()
        if not force and now - self._last_post < HEARTBEAT_S:
            return
        self._last_post = now
        async with self._post_lock:
            # snapshot inside the lock, so whatever lands last is the newest
            await gcs_post_status({
                "link": self.link, "note": note, "adapter": self.adapter or "",
                "name": self.device_name, "addr": self.device_addr, "rssi": self.rssi,
                "key_ok": self.key is not None, "status": self.last_status,
                "reset_ack": self._reset_ack,
            })

    def _on_notify(self, _chr, data: bytearray) -> None:
        try:
            self.last_status = proto.decode_status(data)
        except ValueError as exc:
            log.warning("bad status frame: %s", exc)
            return
        self._status_evt.set()
        # fire-and-forget: a notify callback must not await the GCS
        asyncio.get_running_loop().create_task(self.report(force=True))

    # -- connection
    async def disconnect(self, why: str) -> None:
        c, self.client = self.client, None
        if c is not None:
            log.info("disconnecting: %s", why)
            try:
                await c.disconnect()
            except Exception:  # noqa: BLE001
                pass

    async def connect(self) -> bool:
        # Re-resolved on every (re)connect: a USB bus reset re-registers the
        # dongle and it may come back under a different number.
        self.adapter = proto.usb_bt_adapter()
        if self.adapter is None:
            await self.report("no_adapter", "no USB Bluetooth dongle; the on-board "
                                            "radio is reserved for the handshake")
            return False
        try:
            stale = await release_stale_links(self.adapter)
        except Exception as exc:  # noqa: BLE001 - best effort; the scan still runs
            stale = []
            log.warning("stale-link check on %s failed: %s", self.adapter, exc)
        if stale:
            log.warning("released orphaned link(s) on %s: %s", self.adapter, ", ".join(stale))
            await asyncio.sleep(1.0)   # let the box start advertising again
        await self.report("scanning")
        seen: dict[str, int] = {}

        def is_medbox(d, ad) -> bool:
            if proto.SERVICE_UUID in [u.lower() for u in ad.service_uuids]:
                seen[d.address] = ad.rssi
                return True
            return False

        try:
            dev = await BleakScanner.find_device_by_filter(
                is_medbox, timeout=SCAN_TIMEOUT_S, adapter=self.adapter)
        except (BleakError, OSError) as exc:
            await self.report("error", f"scan failed on {self.adapter}: {exc}")
            return False
        if dev is None:
            await self.report("scanning", "no MedBox in range", force=True)
            return False
        self.device_name = dev.name or ""
        self.device_addr = dev.address
        self.rssi = seen.get(dev.address)
        client = BleakClient(dev, adapter=self.adapter,
                             disconnected_callback=lambda _c: self._status_evt.set())
        try:
            await client.connect(timeout=10.0)
            await client.start_notify(proto.STATUS_UUID, self._on_notify)
            self._on_notify(None, await client.read_gatt_char(proto.STATUS_UUID))
        except (BleakError, OSError, asyncio.TimeoutError) as exc:
            await self.report("error", f"connect to {dev.address} failed: {exc}")
            try:
                await client.disconnect()
            except Exception:  # noqa: BLE001
                pass
            return False
        self.client = client
        await self.report("connected", f"{self.device_name} via {self.adapter}")
        return True

    @property
    def connected(self) -> bool:
        return self.client is not None and self.client.is_connected

    # -- commands
    async def send(self, cmd: str) -> str:
        """Signed command; waits for the box's ack in STATUS. Returns the result."""
        if self.key is None:
            return "NO_KEY"
        async with self._cmd_lock:
            for attempt in range(1, CMD_RETRIES + 1):
                if not self.connected:
                    return "NOT_CONNECTED"
                before = (self.last_status or {}).get("ack_count", -1)
                try:
                    nonce = (await self.client.read_gatt_char(proto.NONCE_UUID)).decode()
                    await self.client.write_gatt_char(
                        proto.CMD_UUID, proto.frame_command(self.key, nonce, cmd), response=True)
                except (BleakError, OSError, asyncio.TimeoutError) as exc:
                    log.warning("%s attempt %d failed: %s", cmd, attempt, exc)
                    continue
                deadline = time.monotonic() + CMD_ACK_TIMEOUT_S
                while time.monotonic() < deadline:
                    s = self.last_status or {}
                    if s.get("ack_count", -1) != before and s.get("ack_cmd") == cmd:
                        res = s.get("ack_result", "")
                        if res != "BAD_SIG":   # a stale nonce race: just retry
                            return res
                        break
                    self._status_evt.clear()
                    try:
                        await asyncio.wait_for(self._status_evt.wait(), 0.3)
                    except asyncio.TimeoutError:
                        pass
            return "NO_ACK"

    async def follow_drop(self, mission: dict) -> None:
        """Keep the box's drop window in step with the navigator's drop_stage."""
        stage = str(mission.get("drop_stage") or "")
        order = str(mission.get("drop_order_id") or "-")
        cmd = proto.STAGE_COMMAND.get(stage)
        if cmd is None or not self.connected:
            return
        s = self.last_status or {}
        if s.get("state") != "ARMED":
            return   # UNARMED: nothing to protect; COMPROMISED/DELIVERED: settled
        done = self._sent.get((order, cmd)) in ("OK", "ALREADY_RELEASED")
        if cmd == "ARM_DROP" and s.get("drop_phase") == "none":
            done = False   # box rebooted or its window lapsed while still descending
        if done:
            return
        res = await self.send(cmd)
        self._sent[(order, cmd)] = res
        log.info("drop %s/%s -> %s: %s", order, stage, cmd, res)
        await self.report(note=f"{cmd} for {order}: {res}", force=True)

    async def follow_reset(self, req: dict | None, mission: dict) -> None:
        """Carry out an operator RESET the hub has queued (it gates on armed /
        drop; the drop check here is a second lock on the same door)."""
        if not req or req.get("state") != "pending" or req.get("id") == self._reset_done:
            return
        if mission.get("drop_stage") in ("approach", "descend", "settle", "release", "climb"):
            res = "REFUSED_DROP_ACTIVE"
        else:
            res = await self.send("RESET")
        self._reset_done = req.get("id")
        self._reset_ack = {"id": self._reset_done, "result": res}
        log.warning("operator RESET #%s -> %s", self._reset_done, res)
        await self.report(note=f"RESET: {res}", force=True)

    # -- main loop
    async def _heartbeat(self) -> None:
        # Its own task: an 8 s scan in the main loop must not starve the
        # liveness signal, or the hub marks the bridge down mid-scan.
        while True:
            await asyncio.sleep(1.0)
            await self.report()

    async def run(self) -> None:
        await self.report("off" if self.key else "no_key", self.key_note, force=True)
        hb = asyncio.get_running_loop().create_task(self._heartbeat())
        try:
            await self._loop()
        finally:
            hb.cancel()

    async def _loop(self) -> None:
        while True:
            st = await gcs_state()
            if st is None:
                await asyncio.sleep(POLL_IDLE_S)
                continue
            kind = str(st.get("payload_kind") or "")
            mission = st.get("mission") or {}
            if kind != "medbox":
                if self.client is not None:
                    await self.disconnect(f"payload is {kind or 'unset'}")
                await self.report("off", f"payload kind: {kind or 'unset'}")
                await asyncio.sleep(POLL_IDLE_S)
                continue
            if not self.connected:
                if self.client is not None:
                    await self.disconnect("link lost")
                    self.last_status = None
                if not await self.connect():
                    await asyncio.sleep(2.0)
                    continue
            await self.follow_drop(mission)
            await self.follow_reset((st.get("medbox") or {}).get("reset"), mission)
            await self.report()
            await asyncio.sleep(POLL_DROP_S if mission.get("drop_stage") else POLL_IDLE_S)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--key", default="config/medbox_key.txt",
                    help="hex HMAC key shared with the box (gitignored)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        key, note = proto.load_key(args.key), ""
    except (OSError, ValueError) as exc:
        # Still run: telemetry is useful without the key; only commands need it.
        key, note = None, f"no command key ({exc}); telemetry only"
        log.warning(note)
    asyncio.run(_serve(MedBoxBridge(key, note)))


async def _serve(bridge: MedBoxBridge) -> None:
    # systemd stops us with SIGTERM: disconnect cleanly instead of leaving
    # bluetoothd holding an orphaned link (release_stale_links is the backstop).
    task = asyncio.current_task()
    asyncio.get_running_loop().add_signal_handler(signal.SIGTERM, task.cancel)
    try:
        await bridge.run()
    except asyncio.CancelledError:
        pass
    finally:
        await bridge.disconnect("service stopping")


if __name__ == "__main__":
    main()
