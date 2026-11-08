"""Telemetry-as-probe: the video yields when the GCS link starts queueing.

2026-09-27, "the GCS lags when flying or the Wi-Fi is a little slow". Measured
with scripts/link_probe.py: button round trip p50 20 ms with no video, 180 ms
(max ~1 s) with the camera open, because the MJPEG stream kept the extender's
queue full even on its leanest JPEG rung. These pin the pieces of the fix:

* ws_flow.LinkDelay     - queueing delay from send->ack round trips
* FrameGate.on_ack      - the page's hold time is not charged to the link
* cameras._choose_rung  - delay drives the ladder; fps-capped rungs below q20
* ws_flow.mark_interactive - the WS socket gets the interactive class
"""
from __future__ import annotations

import socket
import time

from drone_stack.gcs import cameras
from drone_stack.gcs.ws_flow import FrameGate, LinkDelay, mark_interactive


# ---------------------------------------------------------------- LinkDelay
def test_no_samples_means_no_delay():
    assert LinkDelay().queue_delay() == 0.0


def test_delay_is_the_round_trip_above_the_empty_queue_minimum():
    d = LinkDelay(alpha=1.0)             # no smoothing: read the last sample
    d.add(0.020, now=0.0)
    d.add(0.180, now=2.0)
    assert abs(d.queue_delay() - 0.160) < 1e-9


def test_a_radio_hiccup_is_not_a_queue():
    """Measured on the bench after first deploy: 150-300 ms spikes with the
    video already at 0.35 Mbit/s. Retries, not a queue - they must not count."""
    d = LinkDelay()
    for i in range(30):
        d.add(0.020, now=i * 0.07)
    for i in range(3):
        d.add(0.300, now=2.1 + i * 0.07)
    assert d.queue_delay() < 0.005


def test_a_standing_queue_is_seen_through_the_minimum():
    """Video filling the extender raises EVERY round trip."""
    d = LinkDelay()
    d.add(0.020, now=0.0)
    for i in range(20):
        d.add(0.180 + 0.02 * (i % 3), now=2.0 + i * 0.07)
    assert d.queue_delay() >= 0.155


def test_the_minimum_is_forgotten_after_the_window():
    """A route change that genuinely lengthens the path must not read as a
    permanent queue."""
    d = LinkDelay(window_s=60.0, buckets=6, alpha=1.0, base_cap_s=1.0)
    d.add(0.020, now=0.0)
    for t in range(10, 80, 10):
        d.add(0.060, now=float(t))
    assert abs(d.queue_delay()) < 1e-9


def test_a_latecomer_cannot_learn_a_full_queue_as_its_baseline():
    """Connect while the queue is already full: every sample is ~180 ms, and
    without the cap the base would be 180 and the delay zero forever."""
    d = LinkDelay(alpha=1.0, base_cap_s=0.08)
    for i in range(20):
        d.add(0.180, now=i * 0.1)
    assert d.queue_delay() >= 0.099


def test_nonsense_samples_are_ignored():
    d = LinkDelay(alpha=1.0)
    d.add(0.020, now=0.0)
    d.add(-1.0, now=0.1)
    d.add(99.0, now=0.2)
    assert d.srtt == 0.020


# ---------------------------------------------------------- FrameGate + ack
def test_ack_measures_the_round_trip_minus_the_pages_hold():
    g = FrameGate()
    seq = g.next_seq()
    g._sent_at[seq] -= 0.300                  # sent 300 ms ago
    g.on_ack(seq, hold_ms=250)                # the page sat on it for 250
    assert 0.03 < g.delay.srtt < 0.08


def test_old_pages_without_hold_still_ack():
    g = FrameGate()
    seq = g.next_seq()
    g.on_ack(seq)
    assert g.acked == seq and g.delay.srtt is not None


def test_send_stamps_are_bounded_for_a_page_that_never_acks():
    g = FrameGate()
    for _ in range(500):
        g.next_seq()
    assert len(g._sent_at) <= 64


# --------------------------------------------------------------- controller
LADDER = [[1.0, 55], [1.0, 34], [1.0, 20], [1.0, 20, 10], [1.0, 20, 3]]
TARGET = 30.0


def _cam(rung: int = 0):
    cam = cameras._BaseCamera(1, "T", adaptive=True, ladder=LADDER,
                              target_fps=TARGET, delay_high_ms=100,
                              delay_low_ms=40)
    cam._rung = rung
    return cam


def _evaluate(cam, delivered, qdelay=None):
    rung, good = cam._choose_rung(cam._rung, delivered, TARGET, len(LADDER),
                                  cam._good_ticks, qdelay)
    cam._rung, cam._good_ticks = max(0, min(int(rung), len(LADDER) - 1)), good
    return cam._rung


def test_two_element_rungs_still_parse_and_carry_no_cap():
    cam = _cam()
    assert cam._ladder[0] == (1.0, 55)
    assert cam.stream_fps_cap() is None


def test_third_element_is_a_per_stream_fps_cap():
    cam = _cam(rung=3)
    assert cam._ladder[3] == (1.0, 20)
    assert cam.stream_fps_cap() == 10.0
    assert cam.info()["adapt"]["fps_cap"] == 10.0


def test_telemetry_queueing_steps_down_even_at_full_fps():
    """The whole bug: 30 fps delivered and still every button waits."""
    cam = _cam(rung=0)
    assert _evaluate(cam, delivered=30.0, qdelay=0.150) == 1


def test_a_collapsed_link_drops_two_rungs_at_once():
    cam = _cam(rung=0)
    assert _evaluate(cam, delivered=30.0, qdelay=0.250) == 2


def test_it_does_not_climb_while_telemetry_is_still_queueing():
    cam = _cam(rung=2)
    cam._probe_floor = 0
    assert [_evaluate(cam, 30.0, qdelay=0.060) for _ in range(6)] == [2] * 6


def test_it_climbs_once_the_queue_is_empty():
    cam = _cam(rung=2)
    cam._probe_floor = 0
    assert [_evaluate(cam, 30.0, qdelay=0.010) for _ in range(4)] == [2, 2, 2, 1]


def test_a_capped_rung_is_judged_against_its_own_cap():
    """10 fps on a 10 fps rung is the link keeping up, not falling behind."""
    cam = _cam(rung=3)
    cam._probe_floor = 0
    assert _evaluate(cam, delivered=9.8, qdelay=0.010) == 3
    assert [_evaluate(cam, 9.8, qdelay=0.010) for _ in range(3)] == [3, 3, 2]


def test_no_delay_signal_behaves_exactly_as_before():
    cam = _cam(rung=0)
    assert _evaluate(cam, delivered=30.0, qdelay=None) == 0
    assert _evaluate(cam, delivered=4.0, qdelay=None) == 1


def test_link_delay_goes_stale_and_is_dropped():
    cam = _cam()
    cam.report_link_delay("a", 0.2)
    cam.report_link_delay("b", 0.05)
    assert cam._worst_link_delay() == 0.2
    cam.drop_link_delay("a")
    assert cam._worst_link_delay() == 0.05
    cam._link_delay["b"] = (0.05, time.monotonic() - 10.0)
    assert cam._worst_link_delay() is None


# ------------------------------------------------------------ socket class
def test_mark_interactive_sets_ef_and_priority_on_a_real_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        assert mark_interactive(s) is True
        assert s.getsockopt(socket.IPPROTO_IP, socket.IP_TOS) == 0xB8
        if hasattr(socket, "SO_PRIORITY"):
            assert s.getsockopt(socket.SOL_SOCKET, socket.SO_PRIORITY) == 6
    finally:
        s.close()


def test_mark_interactive_fails_open():
    assert mark_interactive(None) is False
