"""TF02-Pro downward rangefinder: frame parsing, the node, the sim mock.

Bench facts (2026-09-26): ~100 frames/s on /dev/ttyAMA0 and, with nothing in
range, distance 4500 cm at strength ~40 - the sensor's "no target" code.
"""
from __future__ import annotations

import fcntl
import struct
import time
from types import SimpleNamespace

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.gcs.hub import GcsHub
from drone_stack.interfaces import rangefinder_interface as rfi
from drone_stack.interfaces.rangefinder_interface import (
    FrameParser,
    RangefinderInterface,
    RealTF02,
    parse_frame,
)
from drone_stack.msg import DiagLevel, RangeReading
from drone_stack.nodes.rangefinder_node import RangefinderNode
from drone_stack.sim.mock_rangefinder import MockRangefinder
from drone_stack.utils.config import Config


def frame(dist_cm: int, strength: int = 900, temp_raw: int = 2248) -> bytes:
    b = bytes([0x59, 0x59, dist_cm & 0xFF, dist_cm >> 8, strength & 0xFF,
               strength >> 8, temp_raw & 0xFF, temp_raw >> 8])
    return b + bytes([sum(b) & 0xFF])


class TestParse:
    def test_valid_frame(self):
        r = parse_frame(frame(123, 900))
        assert r.valid and abs(r.distance_m - 1.23) < 1e-9 and r.strength == 900
        assert abs(r.temperature_c - (2248 / 8 - 256)) < 0.05

    def test_no_target_code_is_invalid_not_45_m(self):
        r = parse_frame(frame(4500, 40))
        assert r is not None and not r.valid

    def test_4500_is_invalid_even_at_high_strength(self):
        assert not parse_frame(frame(4500, 900)).valid

    def test_weak_and_saturated_are_invalid(self):
        assert not parse_frame(frame(200, 99)).valid
        assert not parse_frame(frame(200, 65535)).valid

    def test_zero_distance_invalid(self):
        assert not parse_frame(frame(0, 900)).valid

    def test_bad_checksum_is_not_a_frame(self):
        f = bytearray(frame(200))
        f[8] ^= 0xFF
        assert parse_frame(bytes(f)) is None


class TestStream:
    def test_resyncs_mid_frame_and_across_reads(self):
        p = FrameParser()
        stream = frame(150)[4:] + frame(151) + frame(152)
        got = p.feed(stream[:7]) + p.feed(stream[7:13]) + p.feed(stream[13:])
        assert [round(r.distance_m, 2) for r in got] == [1.51, 1.52]

    def test_distance_byte_equal_to_the_header(self):
        # 0x59 0x59 inside the distance (22873 cm is out of range but still a
        # checksummed frame) must not derail the stream.
        p = FrameParser()
        got = p.feed(b"\x59" + frame(0x5959, 900) + frame(89))
        assert [round(r.distance_m, 2) for r in got][-1] == 0.89

    def test_garbage_does_not_grow_the_buffer(self):
        p = FrameParser()
        for _ in range(100):
            p.feed(b"\x00" * 64)
        assert len(p._buf) <= 4 * 9


class _FakeIface(RangefinderInterface):
    def __init__(self, readings):
        self._readings = list(readings)
        self._open = False
        self.connects = 0
        self.stats: dict = {}

    @property
    def connected(self):
        return self._open

    def connect(self):
        self.connects += 1
        self._open = True
        return True

    def close(self):
        self._open = False

    def read(self):
        return self._readings.pop(0) if self._readings else None

    def link_stats(self):
        return dict(self.stats)


class TestNode:
    def _node(self, readings):
        bus = MessageBus()
        out, diag = [], []
        bus.subscribe(Topics.RANGEFINDER, out.append)
        bus.subscribe(Topics.DIAG_RANGEFINDER, diag.append)
        iface = _FakeIface(readings)
        node = RangefinderNode(bus, Config.load(), iface)
        node.on_start()
        return node, iface, out, diag

    def test_publishes_each_new_reading(self):
        node, _, out, diag = self._node([RangeReading(distance_m=2.0, valid=True),
                                         RangeReading(distance_m=45.0, valid=False)])
        node.step(); node.step(); node.step()
        assert [r.distance_m for r in out] == [2.0, 45.0]
        assert diag[0].level == DiagLevel.OK
        assert diag[-1].level == DiagLevel.WARN

    def test_reconnects_when_the_port_goes_away(self):
        node, iface, _, diag = self._node([])
        iface._open = False
        node._last_try = 0.0
        node.step()
        assert iface.connects == 2
        assert diag and diag[-1].level == DiagLevel.ERROR


class TestMock:
    def test_reads_world_height_plus_mount(self):
        world = SimpleNamespace(state=SimpleNamespace(z=3.0))
        rf = MockRangefinder(world, {"mount_height_m": 0.15, "sim_noise_m": 0.0})
        rf.connect()
        r = rf.read()
        assert r.valid and abs(r.distance_m - 3.15) < 1e-9


# --------------------------------------------------------------------------- #
# wire diagnosis (27 Sep: "no frames" in flight could not say silent vs garbled)
# --------------------------------------------------------------------------- #
class TestLinkDiagnosis:
    def _node(self, **cfg):
        bus = MessageBus()
        diag = []
        bus.subscribe(Topics.DIAG_RANGEFINDER, diag.append)
        iface = _FakeIface([])
        node = RangefinderNode(bus, Config.load(), iface)
        node.on_start()
        return node, iface, diag

    def _window(self, node, iface, **stats):
        """Advance one diag window with these cumulative counters."""
        iface.stats = stats
        node._link_t -= 1.1
        node._last_diag = 0.0
        node.step()

    def test_no_bytes_says_silent(self):
        node, iface, diag = self._node()
        iface.stats = {"rx_bytes": 500, "bad_frames": 0, "uart_frame": 0}
        node.step()
        self._window(node, iface, rx_bytes=500, bad_frames=0, uart_frame=0)
        assert diag[-1].level == DiagLevel.WARN
        assert "silent" in diag[-1].message
        assert diag[-1].values["rx_bps"] == 0

    def test_bytes_without_frames_says_garbled(self):
        node, iface, diag = self._node()
        iface.stats = {"rx_bytes": 0, "bad_frames": 0, "uart_frame": 0}
        node.step()
        self._window(node, iface, rx_bytes=900, bad_frames=50, uart_frame=12)
        m = diag[-1].message
        assert "garbled" in m and "50 bad checksums" in m and "12 framing" in m
        assert diag[-1].values["rx_bps"] > 0

    def test_a_silent_port_is_reopened_even_if_dead_from_boot(self):
        node, iface, _ = self._node()
        assert node._last_rx == 0.0            # never saw a frame
        node._last_open -= 3.5
        node.step()
        assert iface.connects == 2 and node._reopens == 1
        node.step()
        assert iface.connects == 2, "reopened again without waiting"

    def test_counters_ride_on_a_healthy_diag(self):
        node, iface, diag = self._node()
        iface.stats = {"rx_bytes": 0, "bad_frames": 0, "uart_frame": 0}
        node.step()
        iface._readings = [RangeReading(distance_m=2.0, valid=True)]
        self._window(node, iface, rx_bytes=900, bad_frames=0, uart_frame=0)
        assert diag[-1].level == DiagLevel.OK
        assert diag[-1].values["rx_bps"] > 0 and diag[-1].values["reopens"] == 0


class TestRealTF02Counters:
    def test_counts_bytes_and_bad_checksums(self):
        good = frame(250)
        bad = bytearray(frame(260)); bad[8] ^= 0xFF
        data = bytes(bad) + good

        class _Ser:
            in_waiting = len(data)
            def read(self, n): return data[:n]
            def fileno(self): raise OSError("not a tty")
            def close(self): pass

        tf = RealTF02({})
        tf._ser = _Ser()
        r = tf.read()
        assert r is not None and abs(r.distance_m - 2.5) < 1e-9
        stats = tf.link_stats()
        assert stats["rx_bytes"] == len(data) and stats["bad_frames"] == 1
        assert "uart_frame" not in stats      # no tty: counters omitted, no crash

    def test_uart_icount_layout(self, monkeypatch):
        monkeypatch.setattr(fcntl, "ioctl",
                            lambda fd, req, buf: struct.pack("20i", *range(20)))
        c = rfi.uart_icount(3)
        # cts dsr rng dcd rx tx FRAME OVERRUN PARITY BRK BUF_OVERRUN
        assert c == {"uart_frame": 6, "uart_overrun": 7 + 10,
                     "uart_parity": 8, "uart_brk": 9}


def test_hub_ships_the_wire_counters_to_the_flight_log():
    hub = SimpleNamespace(_rangefinder_enabled=True, config=Config.load())
    diag = SimpleNamespace(level=DiagLevel.WARN, message="TF02 silent",
                           values={"rx_bps": 0, "bad_frames": 0, "uart_frame": 0,
                                   "uart_overrun": 0, "reopens": 2})
    out = GcsHub._rangefinder_payload(hub, None, diag)
    assert out["rx_bps"] == 0 and out["reopens"] == 2 and out["message"] == "TF02 silent"
