"""MedBox payload: wire protocol, adapter isolation, hub payload-kind + status.

The bridge itself needs bleak and a radio, so it is exercised on the bench;
everything it decides with is here.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os

import pytest

import drone_stack.gcs.hub as hub_mod
from drone_stack.gcs.hub import GcsHub
from drone_stack.payload import medbox_protocol as proto
from drone_stack.utils.config import Config

KEY = bytes.fromhex("00112233445566778899aabbccddeeff" * 2)

# A frame exactly as firmware pushStatus() prints it.
FRAME = ('{"v":2,"q":7,"s":2,"r":4,"rv":4.63,"t":28.9,"h":52,"sh":0.010,"pk":0.03,'
         '"ms":0.20,"mk":1.16,"l":1,"up":53,"bv":0.00,"bl":0,"pi":1,"d":0,"lg":0.00,'
         '"o":0,"f":2,"a":"ARM_DROP:OK","an":3}')


# ---------------------------------------------------------------- protocol
def test_sign_is_truncated_hmac_sha256_over_nonce_pipe_cmd():
    want = hmac.new(KEY, b"0123abcd0123abcd|ARM_DROP", hashlib.sha256).hexdigest()[:32]
    assert proto.sign(KEY, "0123abcd0123abcd", "ARM_DROP") == want
    assert proto.frame_command(KEY, "0123abcd0123abcd", "ARM_DROP") == \
        f"ARM_DROP|0123abcd0123abcd|{want}".encode()


def test_unknown_command_is_never_framed():
    with pytest.raises(ValueError):
        proto.frame_command(KEY, "00", "OPEN_LID")


def test_decode_status_maps_codes_to_names():
    s = proto.decode_status(FRAME.encode())
    assert s["state"] == "COMPROMISED" and s["reason"] == "impact/drop"
    assert s["lid_closed"] is True and s["armed_s"] == 53
    assert s["batt_v"] is None and s["batt_level"] == "n/a"     # divider not wired
    assert s["dht_fault"] is True and s["imu_fault"] is False
    assert (s["ack_cmd"], s["ack_result"], s["ack_count"]) == ("ARM_DROP", "OK", 3)


def test_decode_status_rejects_other_versions():
    with pytest.raises(ValueError):
        proto.decode_status(b'{"v":1,"s":0}')


def test_tables_match_firmware_enum_lengths():
    # firmware: enum State {UNARMED, ARMED, COMPROMISED, DELIVERED}; Reason has 8.
    assert proto.STATES == ["UNARMED", "ARMED", "COMPROMISED", "DELIVERED"]
    assert len(proto.REASONS) == 8


def test_every_navigator_drop_stage_has_a_box_command():
    # messages.MissionState: approach | descend | settle | release | climb | done | aborted
    stages = {"approach", "descend", "settle", "release", "climb", "done", "aborted"}
    assert set(proto.STAGE_COMMAND) == stages
    # The window must open BEFORE the servo fires (a 1 m fall beats one poll).
    for s in ("approach", "descend", "settle"):
        assert proto.STAGE_COMMAND[s] == "ARM_DROP"


# ---------------------------------------------------------------- adapter isolation
def _fake_sysfs(tmp_path, layout: dict[str, str]):
    """layout: hci name -> device path it links to (relative to tmp_path)."""
    cls = tmp_path / "class"
    cls.mkdir()
    for name, target in layout.items():
        dev = tmp_path / target / name
        dev.mkdir(parents=True)
        os.symlink(dev, cls / name)
    return cls


def test_bridge_takes_the_usb_dongle_whatever_its_number(tmp_path):
    cls = _fake_sysfs(tmp_path, {"hci0": "devices/platform/usb1/1-1/1-1:1.0/bluetooth",
                                 "hci1": "devices/platform/soc/serial0/bluetooth"})
    assert proto.usb_bt_adapter(cls) == "hci0"


def test_bridge_never_falls_back_to_the_onboard_radio(tmp_path):
    cls = _fake_sysfs(tmp_path, {"hci0": "devices/platform/soc/serial0/bluetooth"})
    assert proto.usb_bt_adapter(cls) is None


# ---------------------------------------------------------------- hub
@pytest.fixture
def kind_file(tmp_path, monkeypatch):
    f = tmp_path / "var" / "payload_kind.json"
    monkeypatch.setattr(hub_mod, "_PAYLOAD_KIND_FILE", f)
    return f


def _status(**over):
    s = proto.decode_status(FRAME)
    s.update(state="ARMED", reason="none", ack_count=0, ack_cmd="")
    s.update(over)
    return s


def test_payload_kind_defaults_from_config_and_persists(kind_file):
    hub = GcsHub(Config.load())
    assert hub.build_payload()["payload_kind"] == "parcel"
    assert hub.command("set_payload_kind", {"kind": "medbox"})["ok"]
    assert json.loads(kind_file.read_text())["kind"] == "medbox"
    assert GcsHub(Config.load()).build_payload()["payload_kind"] == "medbox"


def test_payload_kind_rejects_unknown(kind_file):
    hub = GcsHub(Config.load())
    r = hub.command("set_payload_kind", {"kind": "pizza"})
    assert not r["ok"]
    assert hub.build_payload()["payload_kind"] == "parcel"


def test_compromised_transition_logs_once(kind_file):
    hub = GcsHub(Config.load())
    hub.command("set_payload_kind", {"kind": "medbox"})
    hub.command("medbox_status", {"link": "connected", "status": _status()})

    def errors():
        return [c for c in hub.build_payload()["console"]
                if c["src"] == "medbox" and c["level"] == "ERROR"]

    bad = _status(state="COMPROMISED", reason="impact/drop", reason_value=4.6)
    hub.command("medbox_status", {"link": "connected", "status": bad})
    hub.command("medbox_status", {"link": "connected", "status": bad})
    assert len(errors()) == 1 and "impact/drop" in errors()[0]["msg"]
    m = hub.build_payload()["medbox"]
    assert m["link"] == "connected" and m["status"]["state"] == "COMPROMISED"


def test_silent_bridge_reads_as_down(kind_file, monkeypatch):
    hub = GcsHub(Config.load())
    hub.command("set_payload_kind", {"kind": "medbox"})
    assert hub.build_payload()["medbox"]["link"] == "bridge_down"
    hub.command("medbox_status", {"link": "connected", "status": _status()})
    assert hub.build_payload()["medbox"]["link"] == "connected"
    real = hub_mod.time.time
    monkeypatch.setattr(hub_mod.time, "time", lambda: real() + 60)
    assert hub.build_payload()["medbox"]["link"] == "bridge_down"


def test_switching_away_from_medbox_drops_the_stale_box(kind_file):
    hub = GcsHub(Config.load())
    hub.command("set_payload_kind", {"kind": "medbox"})
    hub.command("medbox_status", {"link": "connected", "status": _status()})
    hub.command("set_payload_kind", {"kind": "parcel"})
    assert hub.build_payload()["medbox"]["status"] is None


def test_payload_kind_locked_while_armed_or_dropping(kind_file):
    from drone_stack.bus.topics import Topics
    from drone_stack.msg import ArmedStatus, MissionState
    hub = GcsHub(Config.load())
    hub.bus.publish(Topics.ARMED, ArmedStatus(armed=True))
    r = hub.command("set_payload_kind", {"kind": "medbox"})
    assert not r["ok"] and "armed" in r["message"]
    hub.bus.publish(Topics.ARMED, ArmedStatus(armed=False))
    hub.bus.publish(Topics.MISSION_STATE, MissionState(drop_stage="descend"))
    assert not hub.command("set_payload_kind", {"kind": "medbox"})["ok"]
    hub.bus.publish(Topics.MISSION_STATE, MissionState(drop_stage="done"))
    assert hub.command("set_payload_kind", {"kind": "medbox"})["ok"]
    assert hub.build_payload()["payload_kind"] == "medbox"


def _connected_hub(state="COMPROMISED"):
    hub = GcsHub(Config.load())
    hub.command("set_payload_kind", {"kind": "medbox"})
    hub.command("medbox_status", {"link": "connected", "key_ok": True,
                                  "status": _status(state=state, reason="impact/drop",
                                                    reason_value=3.23)})
    return hub


def test_reset_round_trip_through_the_bridge(kind_file):
    hub = _connected_hub()
    r = hub.command("medbox_reset", {})
    assert r["ok"]
    req = hub.build_payload()["medbox"]["reset"]
    assert req["state"] == "pending" and req["id"] == 1
    # the console keeps what the box is about to forget
    assert any("impact/drop" in c["msg"] and "RESET requested" in c["msg"]
               for c in hub.build_payload()["console"])
    assert not hub.command("medbox_reset", {})["ok"], "second reset while one is pending"
    # an ack for some other request id changes nothing
    hub.command("medbox_status", {"link": "connected", "key_ok": True, "status": _status(),
                                  "reset_ack": {"id": 99, "result": "OK"}})
    assert hub.build_payload()["medbox"]["reset"]["state"] == "pending"
    hub.command("medbox_status", {"link": "connected", "key_ok": True,
                                  "status": _status(state="UNARMED"),
                                  "reset_ack": {"id": 1, "result": "OK"}})
    assert hub.build_payload()["medbox"]["reset"] == {
        **req, "state": "done", "result": "OK"}


def test_reset_refused_in_the_air_or_without_a_box(kind_file):
    from drone_stack.bus.topics import Topics
    from drone_stack.msg import ArmedStatus, MissionState
    hub = _connected_hub()
    hub.bus.publish(Topics.ARMED, ArmedStatus(armed=True))
    r = hub.command("medbox_reset", {})
    assert not r["ok"] and "armed" in r["message"]
    hub.bus.publish(Topics.ARMED, ArmedStatus(armed=False))
    hub.bus.publish(Topics.MISSION_STATE, MissionState(drop_stage="settle"))
    assert not hub.command("medbox_reset", {})["ok"]
    hub.bus.publish(Topics.MISSION_STATE, MissionState(drop_stage=""))
    hub.command("medbox_status", {"link": "scanning", "key_ok": True, "status": None})
    r = hub.command("medbox_reset", {})
    assert not r["ok"] and "not connected" in r["message"]
    hub.command("medbox_status", {"link": "connected", "key_ok": False, "status": _status()})
    assert not hub.command("medbox_reset", {})["ok"]
    hub.command("set_payload_kind", {"kind": "parcel"})
    assert not hub.command("medbox_reset", {})["ok"]
    assert hub.build_payload()["medbox"]["reset"]["state"] == "idle"


def test_reset_with_no_answer_times_out(kind_file, monkeypatch):
    hub = _connected_hub()
    assert hub.command("medbox_reset", {})["ok"]
    real = hub_mod.time.time
    monkeypatch.setattr(hub_mod.time, "time", lambda: real() + 30)
    r = hub.build_payload()["medbox"]["reset"]
    assert r["state"] == "failed" and r["result"] == "TIMEOUT"
    # and a timed-out request no longer blocks the next one
    hub.command("medbox_status", {"link": "connected", "key_ok": True, "status": _status()})
    assert hub.command("medbox_reset", {})["ok"]


def test_bridge_sends_reset_once_per_request():
    import asyncio
    pytest.importorskip("bleak")            # lives in the payload venv only
    from drone_stack.payload.medbox_bridge import MedBoxBridge
    b = MedBoxBridge(b"k" * 32, "")
    sent, posts = [], []

    async def fake_send(cmd):
        sent.append(cmd)
        return "OK"

    async def fake_report(*a, **kw):
        posts.append(b._reset_ack)
    b.send, b.report = fake_send, fake_report
    req = {"id": 3, "state": "pending"}

    async def go():
        await b.follow_reset(req, {})
        await b.follow_reset(req, {})                       # same id: not again
        await b.follow_reset({"id": 4, "state": "done"}, {})  # not pending
        await b.follow_reset({"id": 5, "state": "pending"}, {"drop_stage": "descend"})
    asyncio.run(go())
    assert sent == ["RESET"]
    assert posts[0] == {"id": 3, "result": "OK"}
    assert b._reset_ack == {"id": 5, "result": "REFUSED_DROP_ACTIVE"}
