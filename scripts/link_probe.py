#!/usr/bin/env python3
"""Measure what the operator feels: telemetry cadence and button latency.

Run from the OPERATOR'S machine (the laptop on the Wi-Fi), not the Pi - the
point is to measure through the radio link. It behaves like one GCS page:

* opens /ws and acks every telemetry frame (as app.js does after rendering),
* sends a ``ping`` every 250 ms; the ``pong`` returns on the SAME socket as
  the telemetry, so its round trip is exactly the delay a button press sees,
* optionally pulls the camera MJPEG like an <img> tag would, so the video
  competes for the link the way it does in the field.

No JavaScript in a browser is involved - scanning MJPEG in a page's main
thread starves the page and measures the probe, not the link (SKILL §6).

    python scripts/link_probe.py --host pi.local --seconds 40
    python scripts/link_probe.py --host pi.local --no-video

Prints one line per second, then percentiles over the whole run.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import statistics
import time

try:
    import websockets
except ImportError as exc:  # pragma: no cover - operator tool
    raise SystemExit("pip install websockets") from exc


def _pct(xs, p):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(p / 100.0 * (len(xs) - 1))))]


class Probe:
    def __init__(self) -> None:
        self.frames = 0
        self.frame_gaps: list[float] = []
        self.pings: list[float] = []
        self.video_frames = 0
        self.video_bytes = 0
        self.second = {"f": 0, "p": [], "vf": 0, "vb": 0}
        self.link = None
        self.cam = None
        self.net = None

    async def ws_loop(self, url: str, stop: float) -> None:
        async with websockets.connect(url, max_size=None,
                                      compression="deflate") as ws:
            last = None

            async def pinger():
                while time.monotonic() < stop:
                    await ws.send(json.dumps(
                        {"cmd": "ping", "t": time.monotonic() * 1000.0}))
                    await asyncio.sleep(0.25)

            ptask = asyncio.create_task(pinger())
            try:
                while time.monotonic() < stop:
                    try:
                        raw = await asyncio.wait_for(ws.recv(), 1.0)
                    except asyncio.TimeoutError:
                        continue
                    now = time.monotonic()
                    d = json.loads(raw)
                    if "pong" in d:
                        rtt = now * 1000.0 - float(d["pong"])
                        self.pings.append(rtt)
                        self.second["p"].append(rtt)
                        continue
                    if "seq" not in d:
                        continue
                    self.frames += 1
                    self.second["f"] += 1
                    if last is not None:
                        self.frame_gaps.append(now - last)
                    last = now
                    self.link = d.get("link", self.link)
                    self.net = d.get("net", self.net)   # the Pi's own view
                    cams = d.get("cameras") or []
                    if cams:
                        self.cam = cams[0].get("adapt")
                    # Ack as a rendering page would. hold = 0: nothing queued.
                    await ws.send(json.dumps(
                        {"cmd": "frame_ack", "seq": d["seq"], "hold": 0}))
            finally:
                ptask.cancel()

    async def video_loop(self, host: str, port: int, stop: float) -> None:
        reader, writer = await asyncio.open_connection(host, port)
        writer.write(f"GET /api/camera/1/stream HTTP/1.1\r\nHost: {host}\r\n"
                     "Connection: keep-alive\r\n\r\n".encode())
        await writer.drain()
        buf = b""
        try:
            while time.monotonic() < stop:
                try:
                    chunk = await asyncio.wait_for(reader.read(65536), 1.0)
                except asyncio.TimeoutError:
                    continue
                if not chunk:
                    break
                self.video_bytes += len(chunk)
                self.second["vb"] += len(chunk)
                buf += chunk
                n = buf.count(b"--frame\r\n")
                if n:
                    self.video_frames += n
                    self.second["vf"] += n
                    buf = buf[buf.rfind(b"--frame\r\n") + 9:]
                else:
                    buf = buf[-16:]
        finally:
            writer.close()

    async def reporter(self, stop: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() < stop:
            await asyncio.sleep(1.0)
            s, self.second = self.second, {"f": 0, "p": [], "vf": 0, "vb": 0}
            p = s["p"]
            print(f"t={time.monotonic() - t0:5.1f}s  telem {s['f']:3d} Hz  "
                  f"button rtt p50 {_pct(p, 50):6.0f} max {max(p) if p else float('nan'):6.0f} ms  "
                  f"video {s['vf']:3d} fps {s['vb'] * 8 / 1e6:5.2f} Mbit/s  "
                  f"pi-net {self.net}  cam {self.cam}", flush=True)


async def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="pi.local")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--no-video", action="store_true")
    a = ap.parse_args()
    probe = Probe()
    stop = time.monotonic() + a.seconds
    tasks = [probe.ws_loop(f"ws://{a.host}:{a.port}/ws", stop),
             probe.reporter(stop)]
    if not a.no_video:
        tasks.append(probe.video_loop(a.host, a.port, stop))
    await asyncio.gather(*tasks)

    gaps = probe.frame_gaps
    print("\n== summary ==")
    print(f"telemetry frames {probe.frames} over {a.seconds:.0f}s "
          f"= {probe.frames / a.seconds:.1f} Hz; worst gap {max(gaps) if gaps else 0:.2f}s; "
          f"gaps >1s: {sum(g > 1.0 for g in gaps)}")
    if probe.pings:
        print(f"button rtt ms  p50 {_pct(probe.pings, 50):.0f}  p90 {_pct(probe.pings, 90):.0f}  "
              f"p99 {_pct(probe.pings, 99):.0f}  max {max(probe.pings):.0f}  "
              f"mean {statistics.mean(probe.pings):.0f}")
    if not a.no_video:
        print(f"video {probe.video_frames / a.seconds:.1f} fps, "
              f"{probe.video_bytes * 8 / a.seconds / 1e6:.2f} Mbit/s")


if __name__ == "__main__":
    asyncio.run(main())
