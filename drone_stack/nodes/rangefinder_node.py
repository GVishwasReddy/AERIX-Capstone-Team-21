"""RangefinderNode - downward TF02-Pro height above ground (2026-09-26).

Polls a :class:`~drone_stack.interfaces.rangefinder_interface.RangefinderInterface`
(real TF02-Pro or the sim mock) and publishes:

* ``/rangefinder`` - :class:`~drone_stack.msg.RangeReading`, the newest frame
  each tick (valid or flagged invalid). Nothing is published when nothing new
  arrived, so a consumer judges freshness by the message ``stamp`` - a dead
  sensor reads as stale, never as a frozen last value.
* ``/diagnostics/rangefinder`` - health for the GCS.

Used only by NavigationNode's precision drop. The RPLIDAR C1 remains the
avoidance sensor; this node does not touch it. Reconnects on its own if the
UART goes away, without ever blocking the rest of the stack.
"""
from __future__ import annotations

import time

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.interfaces.rangefinder_interface import RangefinderInterface
from drone_stack.msg import Diagnostic, DiagLevel
from drone_stack.utils.config import Config
from drone_stack.utils.node import NodeBase


class RangefinderNode(NodeBase):
    def __init__(self, bus: MessageBus, config: Config,
                 interface: RangefinderInterface) -> None:
        section = config.section("rangefinder")
        super().__init__("rangefinder", bus, config, rate_hz=section.get("rate_hz", 20))
        self.iface = interface
        self._port = str(section.get("port", ""))
        self._reconnect_s = float(section.get("reconnect_interval_s", 2.0))
        self._stale_s = float(section.get("stale_s", 0.5))
        self._last_try = 0.0
        self._last_rx = 0.0
        self._count = 0
        self._last_diag = 0.0
        self._last_level: DiagLevel | None = None
        #: Connected but silent this long -> close and reopen the port. A
        #: reopen resets the UART and flushes a half-frame; it cannot fix a
        #: power or wiring fault, and the diag says which one it is.
        self._reopen_s = float(section.get("reopen_after_s", 3.0))
        self._reopens = 0
        self._last_open = 0.0
        # Per-second wire counters (see _link_window).
        self._link_prev: dict[str, int] = {}
        self._link_t = 0.0
        self._link: dict[str, float] = {}

    def on_start(self) -> None:
        if self.iface.connect():
            self.log.info("TF02-Pro rangefinder open (%s)", self._port or "sim")
        self._last_try = self._last_open = time.monotonic()

    def step(self) -> None:
        now = time.monotonic()
        if not self.iface.connected:
            if now - self._last_try >= self._reconnect_s:
                self._last_try = now
                if self.iface.connect():
                    self._last_open = now
                    self.log.info("TF02-Pro rangefinder reconnected")
            self._diag(now, DiagLevel.ERROR, f"rangefinder not connected ({self._port})")
            return
        reading = self.iface.read()
        link = self._link_window(now)
        if reading is None:
            silent = now - self._last_rx
            if silent > self._stale_s:
                self._diag(now, DiagLevel.WARN, self._why_no_frames(link), **link)
            # Silent since the last frame OR the last open, whichever is later:
            # a sensor dead from boot has no last frame and must still be retried.
            quiet = now - max(self._last_rx, self._last_open)
            if quiet > self._reopen_s:
                self._reopens += 1
                self.log.warning("TF02-Pro silent %.1fs - reopening %s (%s)",
                                 quiet, self._port, self._why_no_frames(link))
                self._last_try = self._last_open = now
                self.iface.connect()
            return
        self._last_rx = now
        self._count += 1
        self.publish(Topics.RANGEFINDER, reading)
        if reading.valid:
            self._diag(now, DiagLevel.OK, "ok", agl_m=reading.distance_m,
                       strength=reading.strength, frames=self._count, **link)
        else:
            self._diag(now, DiagLevel.WARN, "no valid target (weak signal / out of range)",
                       strength=reading.strength, frames=self._count, **link)

    def _link_window(self, now: float) -> dict[str, float]:
        """Wire counters differenced over the last second, plus reopens.

        Recomputed at most once a second so every diag reports a full window
        rather than one 50 ms poll's worth of bytes.
        """
        if now - self._link_t < 1.0:
            return self._link
        stats = self.iface.link_stats()
        dt = now - self._link_t if self._link_t else 0.0
        link: dict[str, float] = {"reopens": self._reopens}
        if stats and self._link_prev and dt > 0:
            d = {k: stats[k] - self._link_prev[k] for k in stats if k in self._link_prev}
            link.update(rx_bps=round(d.get("rx_bytes", 0) / dt),
                        bad_frames=d.get("bad_frames", 0),
                        uart_frame=d.get("uart_frame", 0),
                        uart_overrun=d.get("uart_overrun", 0))
        self._link_prev, self._link_t, self._link = stats, now, link
        return link

    @staticmethod
    def _why_no_frames(link: dict[str, float]) -> str:
        """Silent and garbled are different faults with different fixes."""
        if "rx_bps" not in link:
            return "no frames from the rangefinder"
        if link["rx_bps"] == 0:
            return "TF02 silent - no bytes on the UART (power / TX wire / connector)"
        return (f"TF02 bytes garbled - {link.get('bad_frames', 0)} bad checksums, "
                f"{link.get('uart_frame', 0)} framing errors/s "
                "(noise / ground / loose pin)")

    def _diag(self, now: float, level: DiagLevel, message: str, **values) -> None:
        # Once a second while nothing changes; a change of level goes out at once.
        if now - self._last_diag < 1.0 and level == self._last_level:
            return
        self._last_diag, self._last_level = now, level
        self.publish(Topics.DIAG_RANGEFINDER, Diagnostic(
            name="rangefinder", level=level, message=message, values=values))

    def on_stop(self) -> None:
        self.iface.close()
