"""Per-connection flow control for the GCS WebSocket  (added 2026-09-23).

Symptom: "the longer I am in the GCS the laggier the camera and every button
get; a page refresh fixes it". Measured on the Pi, two queues that only ever
grew for the life of a connection - which is exactly why a refresh (a new
connection) cleared them:

1. The WebSocket. uvicorn's websockets-sansio protocol never pauses a sender:
   ``send()`` waits on a ``writable`` Event that is set once and never cleared,
   and ``transport.write`` buffers without limit. ``/ws`` pushed 15 frames a
   second whether or not the browser was keeping up, so a slow tab accumulated
   frames in the Pi's asyncio buffer, and every button's result came back from
   the far end of that queue.

2. The camera. Linux autotunes a socket's send buffer UP (87 KB -> 818 KB ->
   1.3 MB observed, 4 MB ceiling) and never back down. Once full, 661 KB of
   JPEG sat unsent behind a 1.83 Mbit/s link: 2.9 s of latency, and growing
   with session length. Fixed at the listener in ``server.py``
   (``TCP_NOTSENT_LOWAT``), not here.

This module holds the WebSocket half: a credit window, and a thinner that
strips what the browser already has.

2026-09-27 - "the GCS lags when flying or when the Wi-Fi is a little slow".
Measured on the bench link (2.4 GHz extender) with scripts/link_probe.py:
button round trip p50 20 ms with no video, 176-190 ms (max ~1 s) with the
camera open, telemetry 13.8 Hz -> 8.9 Hz. The camera had already walked to
its last rung and still pushed 4-5.5 Mbit/s: the MJPEG stream is a greedy TCP
flow, so whenever the link is thinner than the leanest rung it keeps the
router's queue full, and every telemetry frame and button reply waits behind
video. TCP_NOTSENT_LOWAT bounds what the Pi holds back; it cannot bound what
is already in flight in the extender.

So the credit window now also MEASURES: every frame is stamped when sent and
the page's ack closes the loop. ``LinkDelay`` turns that into queueing delay
(smoothed RTT above the recent minimum), which the camera's adaptive
controller uses to back the video off BEFORE telemetry suffers - the
telemetry flow is the probe, the video is the thing that yields.
"""
from __future__ import annotations

import asyncio
import collections
import time


class LinkDelay:
    """Queueing delay on one connection, from send->ack round trips.

    ``base`` is the minimum RTT seen over the last ``window_s`` (kept as
    per-bucket minima, LEDBAT-style, so one idle moment is remembered without
    storing every sample). Delay = smoothed RTT - base: the part of the round
    trip that is queue rather than distance or rendering.

    ``base_cap_s`` guards LEDBAT's "latecomer" failure: a page that connects
    while the queue is ALREADY full never sees an empty-queue RTT, learns the
    congested RTT as its base, and reports zero delay forever. A GCS on local
    Wi-Fi has a true round trip of ~20 ms (measured), so any base above the cap
    is queue by definition.

    The CURRENT round trip is the minimum over the last ``standing_s``, not the
    smoothed mean (Copa's "standing RTT"). Measured after first deploy: with
    the video already down at 0.35 Mbit/s the bench link still threw 150-300 ms
    spikes - radio retries, not a queue - and a mean let them pin the camera to
    its 2 fps floor. A queue the video builds raises EVERY sample, so the
    minimum still sees it; a radio hiccup raises only some, and it does not.
    """

    def __init__(self, window_s: float = 60.0, buckets: int = 6,
                 alpha: float = 0.25, base_cap_s: float = 0.08,
                 standing_s: float = 1.0) -> None:
        self._bucket_s = window_s / buckets
        self._mins: collections.deque = collections.deque(maxlen=buckets)
        self._bucket_t = None
        self._recent: collections.deque = collections.deque()
        self.standing_s = standing_s
        self.base_cap_s = base_cap_s
        self.alpha = alpha
        self.srtt = None
        self.last_t = None

    def add(self, rtt_s: float, now: float | None = None) -> None:
        if rtt_s is None or rtt_s < 0 or rtt_s > 30.0:
            return
        now = time.monotonic() if now is None else now
        if self._bucket_t is None or now - self._bucket_t >= self._bucket_s:
            self._mins.append(rtt_s)
            self._bucket_t = now
        elif rtt_s < self._mins[-1]:
            self._mins[-1] = rtt_s
        self.srtt = rtt_s if self.srtt is None else \
            (1 - self.alpha) * self.srtt + self.alpha * rtt_s
        self.last_t = now
        self._recent.append((now, rtt_s))
        while self._recent and now - self._recent[0][0] > self.standing_s:
            self._recent.popleft()

    @property
    def standing(self):
        """Lowest round trip in the last ``standing_s`` (None if no data)."""
        return min(r for _, r in self._recent) if self._recent else None

    @property
    def base(self):
        return min(self._mins) if self._mins else None

    def queue_delay(self) -> float:
        if not self._recent or not self._mins:
            return 0.0
        return max(0.0, self.standing - min(self.base, self.base_cap_s))


class FrameGate:
    """At most ``window`` telemetry frames un-acknowledged by the browser.

    The page acks each frame's ``seq`` AFTER it has rendered it (from a
    requestAnimationFrame callback), so the window covers the network and the
    browser's own main thread alike. A page too slow to render 15 Hz simply
    receives fewer, always-fresh frames instead of a growing backlog.

    A page that has never acked (an old cached app.js) is left free-running,
    exactly as before, so a stale tab cannot be wedged by a server upgrade.
    """

    def __init__(self, window: int = 2, stall_s: float = 2.0) -> None:
        self.window = max(1, int(window))
        self.stall_s = float(stall_s)
        self.seq = 0
        self.acked = 0
        self.enabled = False
        self._acked_evt = asyncio.Event()
        # seq -> monotonic send time, for the round trip. Bounded: a page that
        # stops acking must not grow it.
        self._sent_at: collections.OrderedDict = collections.OrderedDict()
        self.delay = LinkDelay()

    def next_seq(self) -> int:
        self.seq += 1
        self._sent_at[self.seq] = time.monotonic()
        while len(self._sent_at) > 64:
            self._sent_at.popitem(last=False)
        return self.seq

    def in_flight(self) -> int:
        return self.seq - self.acked

    def on_ack(self, seq, hold_ms=None) -> None:
        """``hold_ms`` is how long the page sat on the frame before acking
        (waiting for an animation frame, rendering, or a hidden tab whose rAF
        is paused). It is subtracted so the sample is the NETWORK round trip:
        a backgrounded tab must not read as a congested link and starve the
        video for every other viewer. Old pages send none: counted as 0."""
        try:
            seq = int(seq)
        except (TypeError, ValueError):
            return
        self.enabled = True
        # Never trust an ack for a frame not yet sent; never move backwards.
        seq = min(seq, self.seq)
        sent = self._sent_at.pop(seq, None)
        if sent is not None:
            try:
                hold = max(0.0, float(hold_ms or 0.0)) / 1000.0
            except (TypeError, ValueError):
                hold = 0.0
            self.delay.add(time.monotonic() - sent - hold)
        if seq > self.acked:
            self.acked = seq
        self._acked_evt.set()

    async def wait_turn(self) -> None:
        """Return when the next frame may be sent.

        A browser that stops acking entirely - a hidden tab, whose rAF is
        paused - is not waited on forever: after ``stall_s`` one frame goes out
        anyway. That bounds a silent page to one frame per ``stall_s`` (it
        cannot back up) and guarantees a connection is never wedged by a lost
        ack; the next real ack restores the full rate.
        """
        if not self.enabled:
            return
        deadline = time.monotonic() + self.stall_s
        while self.in_flight() >= self.window:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            self._acked_evt.clear()
            try:
                await asyncio.wait_for(self._acked_evt.wait(), remaining)
            except asyncio.TimeoutError:
                return


# DSCP EF (46 << 2). Wi-Fi WMM maps it to the VOICE access category, which the
# radio contends for ahead of best-effort video; routers that honour DSCP do
# the same on the far hop.
_TOS_EF = 0xB8
# SO_PRIORITY 6 = TC_PRIO_INTERACTIVE: band 0 of the Pi's pfifo_fast root
# qdisc on wlan0 (priomap[6] = 0, measured 2026-09-27), strictly ahead of the
# band-1 default the camera stream uses. The highest value an unprivileged
# process may set.
_PRIO_INTERACTIVE = 6


def connection_socket(receive):
    """The kernel socket under an ASGI connection, or None.

    uvicorn does not expose it in the scope. The ``receive`` callable Starlette
    hands a WebSocket is the protocol's bound method, and the protocol owns the
    transport. Private, so every step is guarded: on any other server or
    version this returns None and the caller carries on unmarked.
    """
    proto = getattr(receive, "__self__", None)
    transport = getattr(proto, "transport", None)
    if transport is None:
        return None
    try:
        return transport.get_extra_info("socket")
    except Exception:  # noqa: BLE001
        return None


def mark_interactive(sock) -> bool:
    """Put a connection in the interactive class. True if the kernel took it."""
    import socket as _s
    if sock is None:
        return False
    ok = False
    try:
        sock.setsockopt(_s.IPPROTO_IP, _s.IP_TOS, _TOS_EF)
        ok = True
    except (OSError, AttributeError):
        pass
    so_priority = getattr(_s, "SO_PRIORITY", None)  # Linux only
    if so_priority is not None:
        try:
            # After IP_TOS, which overwrites sk_priority with its own mapping
            # (EF -> 4, the video class - not what we want).
            sock.setsockopt(_s.SOL_SOCKET, so_priority, _PRIO_INTERACTIVE)
            ok = True
        except OSError:
            pass
    return ok


class PayloadDelta:
    """Remove from a frame what this connection has already delivered.

    ``console`` was the largest key in every frame (7.3 KB of 17 KB): the same
    ~60 log lines re-sent 15 times a second, which the page then discarded by
    id. ``trail`` can reach 800 points (~30 KB) and only changes when the
    aircraft moves 0.7 m. Both compete with the camera for the same Wi-Fi.

    The page keeps the last trail it was sent, and merges console lines across
    frames it coalesces, so neither is lost by being sent once.
    """

    def __init__(self) -> None:
        self.console_id = 0
        self._trail_sig = None

    def apply(self, payload: dict) -> dict:
        con = payload.get("console")
        if con is not None:
            fresh = [e for e in con if e.get("id", 0) > self.console_id]
            if fresh:
                self.console_id = max(e.get("id", 0) for e in fresh)
            payload["console"] = fresh

        trail = payload.get("trail")
        if trail is not None:
            # The trail is a bounded deque: once full its length stops
            # changing, so the ends are part of the signature too.
            sig = (len(trail),
                   tuple(trail[0]) if trail else None,
                   tuple(trail[-1]) if trail else None)
            if sig == self._trail_sig:
                del payload["trail"]
            else:
                self._trail_sig = sig
        return payload
