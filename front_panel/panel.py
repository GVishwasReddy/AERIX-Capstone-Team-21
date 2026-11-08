#!/usr/bin/env python3
"""AERIX front panel: 1.3" SH1106 OLED + 3 buttons + buzzer on the Pi 5.

Viewer of the GCS (/api/state on localhost:8090). It never opens the MAVLink or
TF02 ports, so a crash here cannot touch flight. It takes two actions: a guarded
Pi shutdown, and payload servo LOCK/RELEASE sent through the GCS's own
/api/command set_servo (so the GCS envelope clamp and console log still apply).

Wiring (verified on the PCB 2026-10-04; B1/B3 labels confirmed with btn_id.py):
    OLED SDA  pin 3  (GPIO2)       OLED SCK pin 5 (GPIO3)    I2C 0x3C
    B1 UP     pin 15 (GPIO22)      B2 DOWN  pin 13 (GPIO27)
    B3 SELECT pin 11 (GPIO17)      BUZZER   pin 12 (GPIO18, passive)

Controls:
    B1 UP       next page                B2 DOWN     previous page
    B3 SELECT   page action: PRE-FLIGHT runs every check (press again for the
                full list), LOG opens the message viewer, OVERVIEW jumps to
                PRE-FLIGHT and runs it. Inside a list: UP/DOWN scroll, SELECT exits.
                PAYLOAD opens the servo menu: UP/DOWN pick LOCK / RELEASE / BACK,
                SELECT sends it. Refused while armed, mid-drop, or with no FC link.
                ALTITUDE opens the height editor: UP +0.5 m, DOWN -0.5 m (2 m to the
                ceiling), SELECT sends via set_flight_altitude (unchanged = exit).
                Refused while armed.
    B1 hold 1 s  mute / unmute alerts    B2 hold 1 s  jump to OVERVIEW
    B3 hold 3 s  shutdown menu (B3 = confirm, B1/B2 = cancel; refused if armed)
    The screen sleeps (fully off) after 15 s idle; any press only wakes it.
    It never sleeps while armed or during a pre-flight run, and an alert wakes it.

Fonts are Quicksand (apt: fonts-quicksand), rendered 1-bit; see QFont for
how each face is aligned to the layout grid.
"""
import json
import math
import os
import queue
import shutil
import socket
import subprocess
import threading
import time
import urllib.request

import lgpio
import numpy as np
import yaml
from gpiozero import Button, Device, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory, LGPIOPin
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from PIL import Image, ImageChops, ImageDraw, ImageFont

# ---------------------------------------------------------------- config
GCS_URL = "http://127.0.0.1:8090/api/state"
CMD_URL = "http://127.0.0.1:8090/api/command"
SERVO_ITEMS = ("LOCK", "RELEASE", "KIND", "BACK")
# Same set and order as hub.PAYLOAD_KINDS; KIND on the PAYLOAD menu cycles it.
PAYLOAD_KINDS = ("parcel", "medbox", "none")
# Same bounds and step as the GCS slider and NavigationNode._FLIGHT_ALT_MIN/MAX_M.
ALT_MIN, ALT_MAX, ALT_STEP = 2.0, 10.0, 0.5
# Drop stages in which the navigator owns the payload servo (see drone_stack
# navigation_node _drop_step). The panel must not fight it.
DROP_ACTIVE = {"approach", "descend", "settle", "release", "climb", "abort"}
REAL_YAML = "/home/pi/drone_stack/config/real.yaml"
PIN_UP, PIN_DOWN, PIN_SELECT, PIN_BUZZER = 22, 27, 17, 18
POLL_S = 0.5
SLEEP_AFTER_S = 15           # screen fully off after this much idle
SLIDE_S = 0.2                # page-change slide duration
SLEEP_POLL_S = 2.0           # slower polling while asleep (alerts still work)
BRIGHT = 255
SHUTDOWN_HOLD_S = 3.0
CONFIRM_WINDOW_S = 6.0
# Passive buzzer: the 2026-10-04 sweep was loudest at 2.0 kHz and 3.5 kHz
# (its two resonances), so every sound is built from those two notes.
LO, HI = 2000, 3500

W, H = 128, 64
HDR_H = 9                    # header bar height
BODY_Y, ROW_H = 10, 9        # 6 body rows of 5x8 text
# Fonts: Quicksand (apt: fonts-quicksand), the user's pick 2026-10-05 from a
# 28-font sheet rendered 1-bit at 128x64 (italics/cursive were rejected).
# Every y coordinate in the page code was laid out for the bitmap fonts below,
# so each Quicksand face is nudged vertically until it lands exactly where the
# bitmap face it replaces put it. The bitmaps are loaded only to measure that;
# nothing draws with them. No bold anywhere (user, 2026-10-05).
SP = "/usr/share/fonts/opentype/spleen/"
X11 = "/usr/share/fonts/X11/"
QS = "/usr/share/fonts/truetype/quicksand/Quicksand-"


class QFont(ImageFont.FreeTypeFont):
    """Quicksand drawn one cached glyph at a time, on whole pixels.

    FreeType lays a string out at fractional pen positions, so in 1-bit mode a
    decimal point shows or vanishes depending on where it lands, and one glyph
    ("A") lifts the whole line by 1 px. Here each character is rasterised once,
    its baseline snapped, and pasted at integer advances: identical every frame
    and cheaper than re-rasterising strings.
    `dy` shifts the face to where the bitmap font `like` sat: align="base"
    matches baselines, "top" matches cap tops (a taller face grows down)."""

    PAD = 2                                   # room for glyphs that overhang the pen

    def __init__(self, weight, size, like, align="base"):
        super().__init__(QS + weight + ".ttf", size)
        self.ft = ImageFont.truetype(QS + weight + ".ttf", size)
        self.asc, self.desc = self.ft.getmetrics()
        self.cache, self.dy = {}, 0
        k = 3 if align == "base" else 1
        self.dy = like.getbbox("H")[k] - self.getbbox("H")[k]

    def glyph(self, ch):
        g = self.cache.get(ch)
        if g is None:
            adv = round(self.ft.getlength(ch))
            im = Image.new("1", (adv + 3 * self.PAD, self.asc + self.desc + 2))
            d = ImageDraw.Draw(im)
            d.fontmode = "1"
            d.text((self.PAD, self.asc), ch, font=self.ft, fill=1, anchor="ls")
            box = im.getbbox()
            # hinting leaves some glyphs ("A") floating 1 px above the baseline
            if box and ch.isalnum() and ch not in "gjpqy" and box[3] < self.asc:
                im = ImageChops.offset(im, 0, self.asc - box[3])
            ink = (box[0] - self.PAD, box[2] - self.PAD) if box else None   # x span vs pen
            g = self.cache[ch] = (im, adv, ink)
        return g

    def layout(self, text):
        """Pen x per glyph and total advance. Neighbouring glyphs always keep
        one blank column between their ink, else a "." melts into a 2's foot
        ("192.168" read as "192168")."""
        xs, x, edge = [], 0, None              # edge: first free column after the last ink
        for c in str(text):
            _, adv, ink = self.glyph(c)
            if ink and edge is not None and x + ink[0] < edge + 1:
                x = edge + 1 - ink[0]
            xs.append(x)
            if ink:
                edge = x + ink[1]
            x += adv
        return xs, x

    def render(self, text):
        text = str(text)
        xs, w = self.layout(text)
        line = Image.new("1", (w + 3 * self.PAD, self.asc + self.desc + 2))
        for c, x in zip(text, xs):
            line.paste(1, (x, 0), self.glyph(c)[0])
        return line

    def getmask2(self, text, mode="", *a, **k):
        return self.render(text).convert("L").im, (-self.PAD, self.dy)

    def getlength(self, text, *a, **k):
        return self.layout(text)[1]

    def getbbox(self, text, *a, **k):
        box = self.render(text).getbbox() or (self.PAD, 0, self.PAD, 0)
        return (box[0] - self.PAD, box[1] + self.dy, box[2] - self.PAD, box[3] + self.dy)


def fit(weight, width, like, text="AERIX"):
    """Largest size whose `text` is no wider than `width` px."""
    size = 40
    while size > 8 and ImageFont.truetype(QS + weight + ".ttf", size).getlength(text) > width:
        size -= 1
    return QFont(weight, size, like)


_T = ImageFont.truetype(X11 + "misc/spleen-5x8.pcf.gz", 8)
# Body rows sit on a 9 px pitch. Regular 9.5 has 7 px caps, so lines no longer
# touch; Medium below 10 turns 8 into B, Regular keeps it open.
F_T = QFont("Regular", 9.5, _T)
F_S = QFont("Regular", 11, ImageFont.truetype(SP + "spleen-6x12.otf", 12))
F_M = QFont("Medium", 16, ImageFont.truetype(SP + "spleen-8x16.otf", 16))
F_X = fit("Medium", 80, ImageFont.truetype(SP + "spleen-16x32.otf", 32))   # splash word, 80 px slot
# Home page slots (were bitmap Helvetica + 4x6 captions)
F_CAP = QFont("Medium", 8, ImageFont.truetype(X11 + "misc/4x6.pcf.gz", 6))
F_HR = QFont("Medium", 10, ImageFont.truetype(X11 + "75dpi/helvR08.pcf.gz", 8), "top")
F_HB = QFont("Medium", 10, ImageFont.truetype(X11 + "75dpi/helvB08.pcf.gz", 8), "top")
# Black-on-white text inside 9 px bars/chips/highlights. Caps land on rows 2-7
# below the draw y, so a box spanning y-1..y+7 draws its text at y-1.
F_INV = QFont("Regular", 8.5, _T)
F_HERO = QFont("Medium", 12, ImageFont.truetype(X11 + "75dpi/helvB12.pcf.gz", 12))
# Status-chip letters sit in 7 px boxes; no Quicksand size fits, so they keep
# the 4x6 bitmap they were drawn around.
F_CHIP = ImageFont.truetype(X11 + "misc/4x6.pcf.gz", 6)


# ---------------------------------------------------------------- gpio
def rp1_chip():
    """The Pi 5 header lives on the RP1 chip, whose number moves between
    kernels (gpiochip4 on older images, gpiochip15 on this one)."""
    for n in range(32):
        try:
            h = lgpio.gpiochip_open(n)
        except lgpio.error:
            continue
        try:
            label = lgpio.gpio_get_chip_info(h)[3]
        finally:
            lgpio.gpiochip_close(h)
        if "rp1" in label:
            return n
    return 0


class RP1Factory(LGPIOFactory):
    """gpiozero 2.0's LGPIOFactory.__init__ overwrites its own `chip` argument
    with 4-or-0, so the chip has to be opened here instead."""

    def __init__(self, chip):
        super(LGPIOFactory, self).__init__()
        self._handle = lgpio.gpiochip_open(chip)
        self._chip = chip
        self.pin_class = LGPIOPin


Device.pin_factory = RP1Factory(rp1_chip())


def sweep(f0, f1, n, dt):
    """n steps from f0 to f1, dt seconds each: a pitch glide."""
    return [(int(f0 + (f1 - f0) * i / (n - 1)), dt) for i in range(n)]


class Buzzer:
    """Non-blocking tone player. A pattern is a list of (freq_hz, seconds);
    freq 0 is a rest. 50 % duty is the loudest a square wave gets."""

    def __init__(self, pin):
        self.dev = PWMOutputDevice(pin, frequency=HI, initial_value=0)
        self.q = queue.Queue()
        threading.Thread(target=self._run, daemon=True).start()

    def play(self, pattern):
        self.q.put(pattern)

    def _run(self):
        while True:
            for f, dur in self.q.get():
                if f:
                    self.dev.frequency = f
                    self.dev.value = 0.5
                else:
                    self.dev.value = 0
                time.sleep(dur)
            self.dev.value = 0

    # "Glide" style (user's pick, 2026-10-04): pitch sweeps between the two
    # resonances read as one smooth, loud gesture.
    # v5 (2026-10-05, user: "longer and more audible, same tone"): same notes
    # and shapes, ~2.5x longer, and every gesture now ENDS on a held resonance.
    # A sweep spends most of its time between 2.0 and 3.5 kHz, where this
    # buzzer is quiet; the hold sits on a peak, and the ear integrates loudness
    # over ~200 ms, so a held note sounds much louder than a 15 ms blip.
    def click(self):    self.play([(HI, 0.035)])
    def wake(self):     self.play(sweep(LO, HI, 8, 0.025) + [(HI, 0.12)])
    def ok(self):       self.play(sweep(LO, HI, 16, 0.03) + [(HI, 0.35)])
    def warn(self):     self.play((sweep(HI, LO, 14, 0.035) + [(LO, 0.2), (0, 0.1)]) * 2)
    def alarm(self):    self.play((sweep(LO, HI, 14, 0.03) + [(HI, 0.12)]
                                   + sweep(HI, LO, 14, 0.03) + [(LO, 0.12)]) * 4)
    def armed(self):    self.play(sweep(LO, HI, 20, 0.035) + [(HI, 0.3), (0, 0.1)]
                                  + sweep(LO, HI, 20, 0.035) + [(HI, 0.6)])
    def goodbye(self):  self.play(sweep(HI, LO, 30, 0.04) + [(LO, 0.4)])


# ---------------------------------------------------------------- data
def load_limits():
    """Battery and altitude limits come from the flight config, never from a
    copy here (config is the single source of truth)."""
    lim = {"bat_low": 21.6, "bat_crit": 20.4, "alt_max": 5.0}
    try:
        with open(REAL_YAML) as f:
            safety = (yaml.safe_load(f) or {}).get("safety", {})
        lim["bat_low"] = float(safety.get("battery_low_voltage", lim["bat_low"]))
        lim["bat_crit"] = float(safety.get("battery_critical_voltage", lim["bat_crit"]))
        lim["alt_max"] = float(safety.get("max_altitude_m", lim["alt_max"]))
    except Exception:
        pass
    lim["cells"] = max(1, round(lim["bat_low"] / 3.6))
    return lim


def run(cmd, timeout=2.0):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout.strip()
    except Exception:
        return ""


def ago(t):
    try:
        s = int(time.time() - float(t))
    except (TypeError, ValueError):
        return "?"
    if s < 60:
        return f"{s}s"
    if s < 3600:
        return f"{s // 60}m"
    return f"{s // 3600}h"


def home_vector(lat, lon, hlat, hlon):
    """Distance (m) and bearing (deg) from the aircraft back to home."""
    if None in (lat, lon, hlat, hlon):
        return None, None
    p1, p2 = math.radians(lat), math.radians(hlat)
    dp, dl = p2 - p1, math.radians(hlon - lon)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    dist = 2 * 6371000 * math.asin(math.sqrt(a))
    brg = math.degrees(math.atan2(math.sin(dl) * math.cos(p2),
                                  math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)))
    return dist, brg % 360


def post_cmd(cmd, params):
    """POST one GCS command; any transport failure comes back as ok=False."""
    body = json.dumps({"cmd": cmd, "params": params}).encode()
    try:
        req = urllib.request.Request(CMD_URL, data=body, method="POST",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=2.0) as r:
            return json.load(r)
    except Exception as e:
        return {"ok": False, "message": f"GCS error {type(e).__name__}"}


def clip_plain(s, n=24):
    s = str(s)
    return s if len(s) <= n else s[:n - 1] + "~"


def compass(deg):
    return "N NE E SE S SW W NW".split()[int(((deg or 0) + 22.5) // 45) % 8]


class State:
    def __init__(self):
        self.gcs = None
        self.gcs_ok = False
        self.sys = {}

    def poll_gcs(self):
        try:
            with urllib.request.urlopen(GCS_URL, timeout=1.0) as r:
                self.gcs = json.load(r)
            self.gcs_ok = True
        except Exception:
            self.gcs_ok = False
        return self.gcs_ok

    def poll_sys(self):
        s = {}
        ips = run(["hostname", "-I"]).split()
        s["ip"] = ips[0] if ips else "no IP"
        out = run(["nmcli", "-t", "-f", "active,ssid,signal,chan", "dev", "wifi"], 3)
        s["ssid"], s["signal"], s["chan"] = "-", 0, "-"
        for line in out.splitlines():
            if line.startswith("yes:"):
                parts = line.split(":")
                s["ssid"] = ":".join(parts[1:-2]) or "-"
                s["chan"] = parts[-1]
                try:
                    s["signal"] = int(parts[-2])
                except ValueError:
                    pass
        t = run(["vcgencmd", "get_throttled"]).split("=")[-1]
        s["throttled"] = int(t, 16) if t.startswith("0x") else None
        volt = run(["vcgencmd", "pmic_read_adc", "EXT5V_V"]).split("=")[-1].rstrip("V")
        try:
            s["ext5v"] = float(volt)
        except ValueError:
            s["ext5v"] = None
        s["disk_free_gb"] = shutil.disk_usage("/").free / 1e9
        s["load"] = os.getloadavg()[0]
        s["host"] = socket.gethostname()
        for svc in ("aerix-gcs", "aerix-ble", "aerix-payload"):
            s[svc] = run(["systemctl", "is-active", svc]) or "unknown"
        self.sys = s

    def g(self, *path, default=None):
        o = self.gcs if self.gcs_ok else None
        for k in path:
            if isinstance(o, list):
                o = o[k] if isinstance(k, int) and len(o) > k else None
            elif isinstance(o, dict):
                o = o.get(k)
            else:
                return default
        return default if o is None else o

    def armed(self):
        return self.g("telemetry", "armed") if self.gcs_ok else None

    def log_groups(self):
        """Console entries, newest first, with identical messages folded into
        one entry and a repeat count (the LiDAR retry alone posts 30/min)."""
        groups = {}
        for c in self.g("console", default=[]) or []:
            key = (str(c.get("level", "")).upper(), str(c.get("src") or ""), str(c.get("msg") or ""))
            t = c.get("t") or 0
            g = groups.get(key)
            if g is None:
                groups[key] = {"level": key[0], "src": key[1], "msg": key[2], "t": t, "n": 1}
            else:
                g["n"] += 1
                g["t"] = max(g["t"], t)
        return sorted(groups.values(), key=lambda g: -g["t"])


# ---------------------------------------------------------------- pre-flight
PASS, WARN, FAIL = "OK", "WN", "NO"


def preflight_checks(st, lim):
    """Each check returns (status, short detail). Order = what to fix first."""
    g, s = st.g, st.sys
    tel = lambda k, d=None: g("telemetry", k, default=d)
    cam = g("cameras", 0, default={}) or {}
    vision = cam.get("vision") or {}

    def chk_gcs():
        return (PASS, "API up") if st.gcs_ok else (FAIL, "no API")

    def chk_fc():
        fps = g("health", "mavlink_fps", default=0.0)
        if not g("link", "connected"):
            return FAIL, "no link"
        return (PASS, f"{fps:.0f}/s") if fps >= 5 else (WARN, f"{fps:.0f}/s")

    def chk_disarmed():
        a = st.armed()
        return (FAIL, "unknown") if a is None else ((FAIL, "ARMED") if a else (PASS, "safe"))

    def chk_mode():
        m = tel("flight_mode", "UNKNOWN")
        return (WARN, "unknown") if m in ("UNKNOWN", "") else (PASS, str(m)[:9])

    def chk_prearm():
        r = g("mission", "arm_refusal", default="")
        return (FAIL, str(r)[:12]) if r else (PASS, "clear")

    def chk_gps():
        fix, sats = tel("gps_fix", "none"), tel("satellites", 0)
        if fix in ("3D", "RTK") and sats >= 8:
            return PASS, f"{fix} {sats}sat"
        if fix in ("3D", "RTK") and sats >= 6:
            return WARN, f"{fix} {sats}sat"
        return FAIL, f"{fix} {sats}sat"

    def chk_home():
        return (PASS, "set") if g("position", "home_lat") is not None else (WARN, "not set")

    def chk_battery():
        v = tel("battery_v", 0.0)
        if v <= 0.1:
            return FAIL, "no data"
        cell = v / lim["cells"]
        if v >= lim["cells"] * 3.95:
            return PASS, f"{v:.1f}V {cell:.2f}/c"
        if v >= lim["bat_low"] + 0.6:
            return WARN, f"{v:.1f}V {cell:.2f}/c"
        return FAIL, f"{v:.1f}V {cell:.2f}/c"

    def chk_rc():
        return (PASS, f"{g('rc', 'count', default=0)}ch") if g("rc", "connected") \
            else (FAIL, "no rx")

    def chk_lidar():
        fps = g("health", "lidar_fps", default=0.0)
        return (PASS, f"{fps:.0f}fps") if fps >= 5 else (FAIL, f"{fps:.0f}fps")

    def chk_tf02():
        if not g("rangefinder", "enabled"):
            return WARN, "off"
        if not g("rangefinder", "valid"):
            return FAIL, "silent"
        agl = g("rangefinder", "agl_m", default=0.0)
        # on the ground it must read below the drop abort height
        limit = g("rangefinder", "abort_agl_m", default=0.6)
        return (PASS, f"{agl:.2f}m") if agl < limit else (WARN, f"{agl:.2f}m")

    def chk_camera():
        fps = cam.get("fps") or 0
        if not cam.get("connected"):
            return FAIL, "offline"
        return (PASS, f"{fps:.0f}fps") if fps >= 20 else (WARN, f"{fps:.0f}fps")

    def chk_npu():
        npu = vision.get("npu", "-")
        return (PASS, f"{vision.get('det_hz') or 0:.0f}Hz") if npu == "ready" else (FAIL, str(npu)[:9])

    def chk_avoid():
        return (PASS, "on") if g("avoidance", "enabled") else (FAIL, "off")

    def chk_ceiling():
        c = g("mission", "alt_ceiling_m", default=0.0)
        return (PASS, f"{c:.1f}m") if 0 < c <= lim["alt_max"] + 0.01 else (FAIL, f"{c:.1f}m")

    def chk_payload():
        ch = g("payload", "channel", default=0)
        if ch == 9:
            return FAIL, "AUX1 dead"
        return (PASS, f"SERVO{ch}") if ch else (FAIL, "none")

    def chk_medbox():
        if g("payload_kind") != "medbox":
            return PASS, f"{g('payload_kind', default='?')}"
        m = g("medbox", default={}) or {}
        box = m.get("status") or {}
        state = box.get("state")
        if m.get("link") != "connected":
            return FAIL, str(m.get("link", "no link"))[:9]
        if state == "COMPROMISED":
            return FAIL, "COMPROM."
        if state == "ARMED":
            return PASS, "sealed"
        return WARN, "unsealed" if state == "UNARMED" else str(state or "?")[:9].lower()

    def chk_ble():
        a = s.get("aerix-ble", "unknown")
        return (PASS, a) if a == "active" else (FAIL, a[:9])

    def chk_rec():
        if not g("recording", "enabled"):
            return WARN, "off"
        mb = g("recording", "free_mb", default=0.0)
        return (PASS, f"{mb / 1000:.0f}GB free") if mb > 2000 else (WARN, f"{mb:.0f}MB")

    def chk_temp():
        t = g("health", "temp", default=0.0)
        return (PASS, f"{t:.0f}C") if t < 70 else ((WARN, f"{t:.0f}C") if t < 80 else (FAIL, f"{t:.0f}C"))

    def chk_throttle():
        t = s.get("throttled")
        if t is None:
            return WARN, "?"
        if t & 0xF:                          # bits 0-3 = throttled right now
            return FAIL, f"now 0x{t:x}"
        return (WARN, f"was 0x{t:x}") if t else (PASS, "none")

    def chk_disk():
        gb = s.get("disk_free_gb", 0)
        return (PASS, f"{gb:.0f}GB") if gb > 2 else (FAIL, f"{gb:.1f}GB")

    def chk_wifi():
        sig = s.get("signal", 0)
        if s.get("ssid", "-") == "-":
            return WARN, "none"
        return (PASS, f"{sig}%") if sig >= 50 else (WARN, f"{sig}% weak")

    return [("GCS", chk_gcs), ("FC link", chk_fc), ("Disarmed", chk_disarmed),
            ("Mode", chk_mode), ("Pre-arm", chk_prearm), ("GPS", chk_gps),
            ("Home", chk_home), ("Battery", chk_battery), ("RC rx", chk_rc),
            ("LiDAR", chk_lidar), ("TF02", chk_tf02), ("Camera", chk_camera),
            ("NPU", chk_npu), ("Avoidance", chk_avoid), ("Ceiling", chk_ceiling),
            ("Payload", chk_payload), ("MedBox", chk_medbox), ("BLE svc", chk_ble), ("Recorder", chk_rec),
            ("Pi temp", chk_temp), ("Throttle", chk_throttle), ("Disk", chk_disk),
            ("WiFi", chk_wifi)]


# ---------------------------------------------------------------- drawing
def text_w(d, s, font=F_T):
    return d.textlength(str(s), font=font)


def clip(d, s, width, font=F_T):
    s = str(s)
    if text_w(d, s, font) <= width:
        return s
    while s and text_w(d, s + "~", font) > width:
        s = s[:-1]
    return s + "~"


def wrap(d, s, width, font=F_T):
    out, line = [], ""
    for word in str(s).split():
        trial = (line + " " + word).strip()
        if text_w(d, trial, font) <= width:
            line = trial
            continue
        if line:
            out.append(line)
        while text_w(d, word, font) > width:          # hard-split long tokens
            n = max(1, int(width // max(1, text_w(d, "M", font))))
            out.append(word[:n])
            word = word[n:]
        line = word
    if line:
        out.append(line)
    return out


def rtext(d, y, s, font=F_T, fill="white", right=127):
    d.text((right - text_w(d, s, font), y), str(s), font=font, fill=fill)


def chip(d, x, y, s, filled):
    """Status chip, 9 px tall. Filled = white box/black text (needs attention)."""
    w = int(text_w(d, s, F_INV)) + 3
    if filled:
        d.rectangle((x, y, x + w, y + 8), fill="white")
    else:
        d.rectangle((x, y, x + w, y + 8), outline="white")
    d.text((x + 2, y), s, font=F_INV, fill="black" if filled else "white")
    return w


def kv(d, rows, y0=BODY_Y):
    """Six rows of label-left / value-right. A truthy third element inverts
    the value so a problem stands out at a glance."""
    for i, row in enumerate(rows[:6]):
        label, value = row[0], str(row[1])
        bad = len(row) > 2 and bool(row[2])
        y = y0 + i * ROW_H
        d.text((0, y), label, font=F_T, fill="white")
        value = clip(d, value, 125 - text_w(d, label))
        w = text_w(d, value, F_INV if bad else F_T)
        if bad:
            d.rectangle((126 - w - 2, y - 1, 127, y + 7), fill="white")
        rtext(d, y - bad, value, font=F_INV if bad else F_T, fill="black" if bad else "white")


def bar(d, x, y, w, h, frac):
    frac = max(0.0, min(1.0, frac))
    d.rectangle((x, y, x + w, y + h), outline="white")
    if frac > 0:
        d.rectangle((x + 2, y + 2, x + 2 + int((w - 4) * frac), y + h - 2), fill="white")


def battery_icon(d, x, y, w, h, frac):
    d.rectangle((x, y, x + w, y + h), outline="white")
    d.rectangle((x + w + 1, y + 2, x + w + 2, y + h - 2), fill="white")      # terminal nub
    frac = max(0.0, min(1.0, frac))
    if frac > 0:
        d.rectangle((x + 2, y + 2, x + 2 + int((w - 4) * frac), y + h - 2), fill="white")


def quad_icon(d, cx, cy, r=7):
    """Tiny X-frame quad: four rotor rings on diagonal arms."""
    for sx in (-1, 1):
        for sy in (-1, 1):
            px, py = cx + sx * r, cy + sy * r
            d.line((cx, cy, px, py), fill="white")
            d.ellipse((px - 3, py - 3, px + 3, py + 3), outline="white")
    d.rectangle((cx - 1, cy - 1, cx + 1, cy + 1), fill="white")


def header(d, title, idx, total, st, muted):
    d.rectangle((0, 0, 127, HDR_H - 1), fill="white")
    d.text((2, 0), title, font=F_INV, fill="black")
    x = 126 - text_w(d, f"{idx}/{total}", F_INV)
    d.text((x, 0), f"{idx}/{total}", font=F_INV, fill="black")
    # status chips, right to left: mute (only if muted), FC link, GCS.
    # Filled black = up, hollow = down.
    for label, on in (("M", muted), ("F", bool(st.g("link", "connected"))), ("G", st.gcs_ok)):
        if label == "M" and not on:
            continue
        x -= 9
        if on:
            d.rectangle((x, 0, x + 6, 8), fill="black")
            d.text((x + 2, 2), label, font=F_CHIP, fill="white")
        else:
            d.rectangle((x, 0, x + 6, 8), outline="black")
            d.text((x + 2, 2), label, font=F_CHIP, fill="black")


# ---------------------------------------------------------------- pages
def ctext(d, cx, y, s, font, fill="white"):
    d.text((round(cx - d.textlength(str(s), font=font) / 2), y), str(s), font=font, fill=fill)


def p_overview(d, st, ui):
    """Home: a strict grid. Header | hero row (state pill, mode, pre-flight) |
    battery row | four equal tiles. Edges align to x=0/127 or tile dividers."""
    tel = lambda k, dflt=0.0: st.g("telemetry", k, default=dflt)
    armed = st.armed()
    d.rectangle((0, 0, 127, 8), fill="white")
    d.text((2, 0), "AERIX", font=F_INV, fill="black")
    pg = f"1/{len(PAGES)}"
    x = 127 - text_w(d, pg, F_CAP) - 1
    d.text((x, 1), pg, font=F_CAP, fill="black")
    for lab, on in (("M", ui.muted), ("F", bool(st.g("link", "connected"))), ("G", st.gcs_ok)):
        if lab == "M" and not on:
            continue
        x -= 9
        d.rectangle((x, 1, x + 6, 7), fill="black" if on else None, outline="black")
        d.text((x + 2, 1), lab, font=F_CHIP, fill="white" if on else "black")

    # hero row (y 11-25)
    state = "NO GCS" if armed is None else ("ARMED" if armed else "SAFE")
    d.rounded_rectangle((0, 11, 58, 25), radius=3, fill="white" if armed else None, outline="white")
    ctext(d, 29, 11, state, F_HERO, "black" if armed else "white")
    mode = clip(d, tel("flight_mode", "-"), 64, F_HB)
    d.text((127 - text_w(d, mode, F_HB), 9), mode, font=F_HB, fill="white")
    pf = ui.pf_summary()
    pf_bad = "NO" in pf
    w = text_w(d, pf, F_HR)
    if pf_bad:
        d.rectangle((127 - w - 3, 19, 127, 27), fill="white")
    d.text(((126 if pf_bad else 127) - w, 18), pf, font=F_HR, fill="black" if pf_bad else "white")

    # battery row (y 30-37)
    lim = ui.limits
    v, pct = tel("battery_v"), tel("battery_pct", None)
    full = lim["cells"] * 4.2
    frac = (v - lim["bat_crit"]) / (full - lim["bat_crit"]) if v > 0 else 0
    battery_icon(d, 0, 30, 56, 7, frac)
    low = 0 < v < lim["bat_low"]
    vs = f"{v:.1f}V" if v > 0 else "--.-V"
    if low:
        d.rectangle((62, 29, 64 + text_w(d, vs, F_HB), 37), fill="white")
    d.text((64, 28), vs, font=F_HB, fill="black" if low else "white")
    ps = f"{pct:.0f}%" if v > 0 and pct is not None and pct >= 0 else "--%"
    d.text((127 - text_w(d, ps, F_HB), 28), ps, font=F_HB, fill="white")

    # tiles (y 40-63)
    d.line((0, 40, 127, 40), fill="white")
    fix = tel("gps_fix", "none")
    agl = st.g("rangefinder", "agl_m") if st.g("rangefinder", "valid") else None
    tiles = [("GPS", f"{fix} {tel('satellites', 0)}" if fix not in ("none", "-", None) else "NO FIX"),
             ("AGL", f"{agl:.2f}" if agl is not None else "--"),
             ("ALT", f"{tel('altitude'):.1f}"),
             ("HDG", f"{tel('heading'):.0f}\u00b0")]
    for i, (cap, val) in enumerate(tiles):
        if i:
            d.line((i * 32 - 1, 43, i * 32 - 1, 61), fill="white")
        ctext(d, i * 32 + 16, 44, cap, F_CAP)
        vf = F_HB if text_w(d, val, F_HB) <= 30 else F_INV
        ctext(d, i * 32 + 16, 50, clip(d, val, 30, vf), vf)


def p_preflight(d, st, ui):
    if ui.pf_running:
        done, total = ui.pf_progress
        d.text((0, 12), "Running checks...", font=F_S, fill="white")
        bar(d, 0, 28, 127, 10, done / max(1, total))
        d.text((0, 42), f"{done}/{total}", font=F_T, fill="white")
        rtext(d, 42, ui.pf_current)
        if ui.pf_partial:
            name, status, detail = ui.pf_partial[-1]
            d.text((0, 52), clip(d, f"{status} {name} {detail}", 127), font=F_T, fill="white")
        return
    if not ui.pf_results:
        d.text((0, 12), "Not run yet", font=F_S, fill="white")
        d.text((0, 28), "Press SELECT to run", font=F_T, fill="white")
        d.text((0, 37), f"all {len(preflight_checks(st, ui.limits))} checks:", font=F_T, fill="white")
        d.text((0, 46), "link GPS batt RC sensors", font=F_T, fill="white")
        d.text((0, 55), "camera NPU svc Pi wifi", font=F_T, fill="white")
        return
    res = ui.pf_results
    n_fail = sum(r[1] == FAIL for r in res)
    n_warn = sum(r[1] == WARN for r in res)
    n_pass = len(res) - n_fail - n_warn
    verdict = "NOT READY" if n_fail else ("READY*" if n_warn else "READY")
    ink = "black" if n_fail else "white"
    if n_fail:
        d.rectangle((0, 10, 127, 22), fill="white")
    d.text((1, 10), verdict, font=F_S, fill=ink)
    rtext(d, 13, f"{n_pass}/{len(res)} {ago(ui.pf_time)}", fill=ink, right=126)
    # problems only, worst first: the summary screen is the fix-it list
    issues = sorted((r for r in res if r[1] != PASS), key=lambda r: r[1] != FAIL)
    if not issues:
        d.text((0, 28), f"All {len(res)} checks passed", font=F_T, fill="white")
        d.text((0, 46), "SELECT = full list", font=F_T, fill="white")
        return
    for i, (name, status, detail) in enumerate(issues[:4]):
        y = 24 + i * ROW_H
        chip(d, 0, y, status, status == FAIL)
        d.text((17, y + 1), name, font=F_T, fill="white")
        right = 127 if not (i == 3 and len(issues) > 4) else 110
        rtext(d, y + 1, clip(d, detail, right - 20 - text_w(d, name)), right=right)
    if len(issues) > 4:
        rtext(d, 52, f"+{len(issues) - 4}")


def p_flight(d, st, ui):
    t = lambda k, dflt=0.0: st.g("telemetry", k, default=dflt)
    armed = st.armed()
    kv(d, [("Mode", f"{t('flight_mode', '-')} {'ARMED' if armed else 'safe'}", armed),
           ("Alt rel / AMSL", f"{t('altitude'):.1f} / {t('altitude_amsl'):.0f}m"),
           ("Ground speed", f"{t('ground_speed'):.1f} m/s"),
           ("Vertical speed", f"{t('vert_speed'):+.1f} m/s"),
           ("Heading", f"{t('heading'):.0f} {compass(t('heading'))}"),
           ("Pitch / Roll", f"{t('pitch'):+.1f} / {t('roll'):+.1f}")])


def p_gps(d, st, ui):
    t = lambda k, dflt=None: st.g("telemetry", k, default=dflt)
    p = lambda k: st.g("position", k)
    lat, lon = p("lat"), p("lon")
    dist, brg = home_vector(lat, lon, p("home_lat"), p("home_lon"))
    fix, sats = t("gps_fix", "none"), t("satellites", 0)
    kv(d, [("Fix / sats", f"{fix} / {sats}", fix not in ("3D", "RTK") or sats < 8),
           ("Latitude", f"{lat:.6f}" if lat is not None else "--"),
           ("Longitude", f"{lon:.6f}" if lon is not None else "--"),
           ("Home", "set" if p("home_lat") is not None else "not set", p("home_lat") is None),
           ("To home", f"{dist:.0f}m {brg:.0f} {compass(brg)}" if dist is not None else "--"),
           ("Alt AMSL", f"{t('altitude_amsl', 0.0):.1f}m")])


def p_battery(d, st, ui):
    lim = ui.limits
    v = st.g("telemetry", "battery_v", default=0.0)
    pct = st.g("telemetry", "battery_pct", default=None)
    cells = lim["cells"]
    full = cells * 4.2
    frac = (v - lim["bat_crit"]) / (full - lim["bat_crit"]) if v > 0 else 0
    battery_icon(d, 0, 11, 120, 12, frac)
    label = f"{v:.2f}V" if v > 0 else "NO DATA"
    w = text_w(d, label, F_T)
    # label sits on the bar; draw it in a black box so it reads over the fill
    d.rectangle((60 - w / 2 - 2, 13, 60 + w / 2 + 1, 21), fill="black")
    d.text((60 - w / 2, 13), label, font=F_T, fill="white")
    state = ("--" if v <= 0 else "CRITICAL" if v < lim["bat_crit"] else
             "LOW" if v < lim["bat_low"] else "OK")
    kv(d, [("Per cell", f"{v / cells:.2f}V x{cells}S" if v > 0 else "--"),
           ("Remaining", f"{pct:.0f}%" if v > 0 and pct is not None and pct >= 0 else "--"),
           ("State", state, state in ("LOW", "CRITICAL", "--")),
           ("Low / crit", f"{lim['bat_low']:.1f} / {lim['bat_crit']:.1f}V")], y0=27)


def p_sensors(d, st, ui):
    h = lambda k: st.g("health", k, default=0.0)
    rf = lambda k, dflt=0: st.g("rangefinder", k, default=dflt)
    rf_ok = rf("valid", False)
    rc = st.g("rc", "connected")
    kv(d, [("TF02 AGL / str", f"{rf('agl_m', 0.0):.2f}m / {rf('strength')}" if rf_ok else "silent",
            not rf_ok),
           ("TF02 rx / bad", f"{rf('rx_bps')}B/s / {rf('bad_frames')}"),
           ("TF02 reopens", rf("reopens")),
           ("LiDAR", f"{h('lidar_fps'):.1f} fps", h("lidar_fps") < 5),
           ("MAVLink", f"{h('mavlink_fps'):.0f} msg/s", h("mavlink_fps") < 5),
           ("RC rx", f"{st.g('rc', 'count', default=0)}ch rssi {st.g('rc', 'rssi', default=0)}"
            if rc else "none", not rc)])


def p_avoid(d, st, ui):
    a = lambda k, dflt=0.0: st.g("avoidance", k, default=dflt)
    m = lambda v: f"{v:.1f}" if v else "--"
    kv(d, [("Status", a("status", "-"), not a("enabled", False)),
           ("Front / L / R", f"{m(a('front_m'))} {m(a('left_m'))} {m(a('right_m'))}"),
           ("Closest / TTC", f"{m(a('closest_m'))}m {a('ttc_s'):.1f}s"),
           ("CPA / stop", f"{a('cpa_m'):.1f} / {a('stop_distance_m'):.1f}m"),
           ("Cmd / dodge", f"{a('command', '-')} / {a('dodge', '-')}"),
           ("Obstacles", f"{a('count', 0)} ({a('dynamic_count', 0)} moving)")])
    reason = st.g("avoidance", "reason", default="")
    if reason and ui.tick % 6 >= 3:             # alternate last row with reason
        d.rectangle((0, BODY_Y + 5 * ROW_H - 1, 127, 63), fill="black")
        d.text((0, BODY_Y + 5 * ROW_H), clip(d, reason, 127), font=F_T, fill="white")


def p_camera(d, st, ui):
    cam = st.g("cameras", 0, default={}) or {}
    v, ad, stab = cam.get("vision") or {}, cam.get("adapt") or {}, cam.get("stab") or {}
    fps = cam.get("fps") or 0
    rec = st.g("recording", "state", default="-")
    kv(d, [("Camera", f"{fps:.0f}fps {cam.get('latency_ms') or 0:.0f}ms",
            not cam.get("connected") or fps < 20),
           ("Bitrate rung", f"{ad.get('rung', 0)} of {ad.get('rungs', 0)}"),
           ("Stabiliser", f"{stab.get('ok_pct') or 0:.0f}% ok"),
           ("NPU", f"{v.get('npu', '-')} {v.get('det_hz') or 0:.0f}Hz", v.get("npu") != "ready"),
           ("Target lock", f"{v.get('lock', '-')} {v.get('score') or 0:.2f}"),
           ("Recorder", f"{rec} {st.g('recording', 'free_mb', default=0) / 1000:.0f}GB")])


def p_mission(d, st, ui):
    oid = str(st.g("delivery", "order_id", default="") or "-")
    kv(d, [("Mission", st.g("mission", "phase", default="-")),
           ("Delivery", st.g("delivery", "phase", default="-")),
           ("Order", oid),
           ("Scan / drop", f"{st.g('mission', 'scan_stage', default='') or '-'} / "
                           f"{st.g('mission', 'drop_stage', default='') or '-'}"),
           ("BLE handshake", "OPEN" if st.g("mission", "handshake_open") else "closed"),
           ("Ceiling / pay", f"{st.g('mission', 'alt_ceiling_m', default=0):.1f}m "
                             f"S{st.g('payload', 'channel', default='?')}")])
    msg = st.g("mission", "message", default="") or st.g("delivery", "message", default="")
    if msg and ui.tick % 6 >= 3:
        d.rectangle((0, BODY_Y + 5 * ROW_H - 1, 127, 63), fill="black")
        d.text((0, BODY_Y + 5 * ROW_H), clip(d, msg, 127), font=F_T, fill="white")


def p_network(d, st, ui):
    s = st.sys
    kv(d, [("IP", s.get("ip", "-")),
           ("SSID", s.get("ssid", "-"), s.get("ssid", "-") == "-"),
           ("Signal / ch", f"{s.get('signal', 0)}% ch{s.get('chan', '-')}", s.get("signal", 0) < 50),
           ("GCS :8090", "UP" if st.gcs_ok else "DOWN", not st.gcs_ok),
           ("BLE svc", s.get("aerix-ble", "?"), s.get("aerix-ble") != "active"),
           ("Host", f"{s.get('host', '-')}.local")])


def p_system(d, st, ui):
    h = lambda k: st.g("health", k, default=0.0)
    s = st.sys
    thr = s.get("throttled")
    up = h("uptime")
    v5 = s.get("ext5v")
    svc = lambda n: "ok" if s.get(n) == "active" else "DOWN"
    kv(d, [("CPU ld RAM", f"{h('cpu'):.0f}% {s.get('load', 0):.1f} {h('ram'):.0f}%", h("cpu") > 90),
           ("SoC temp", f"{h('temp'):.1f}C", h("temp") >= 75),
           ("5V in / throttle", (f"{v5:.2f}V " if v5 else "") +
            ("ok" if thr == 0 else (f"0x{thr:x}" if thr else "?")), bool(thr) or bool(v5 and v5 < 4.8)),
           ("gcs/ble/pay", f"{svc('aerix-gcs')}/{svc('aerix-ble')}/{svc('aerix-payload')}",
            s.get("aerix-gcs") != "active" or s.get("aerix-ble") != "active"
            or s.get("aerix-payload") != "active"),
           ("Disk free", f"{s.get('disk_free_gb', 0):.1f}GB", s.get("disk_free_gb", 99) < 2),
           ("Uptime", f"{int(up // 3600)}h {int(up % 3600 // 60):02d}m")])


LEVEL_TAG = {"ERROR": "E", "CRITICAL": "E", "WARNING": "W", "WARN": "W", "INFO": "I", "DEBUG": "D"}


def p_log(d, st, ui):
    groups = st.log_groups()
    if not groups:
        d.text((0, 14), "(no messages)", font=F_T, fill="white")
        return
    for i, g in enumerate(groups[:6]):
        y = BODY_Y + i * ROW_H
        lv = LEVEL_TAG.get(g["level"], "?")
        if lv == "E":
            d.rectangle((0, y - 1, 6, y + 7), fill="white")
            d.text((1, y - 1), lv, font=F_INV, fill="black")
        else:
            d.text((1, y), lv, font=F_T, fill="white")
        cnt = f"x{g['n']}" if g["n"] > 1 else ago(g["t"])
        cw = text_w(d, cnt)
        rtext(d, y, cnt)
        d.text((9, y), clip(d, f"{g['src']}: {g['msg']}", 116 - cw), font=F_T, fill="white")


def draw_log_viewer(d, st, ui):
    """Focus mode on LOG: one message per screen, fully wrapped."""
    groups = st.log_groups()
    d.rectangle((0, 0, 127, HDR_H - 1), fill="white")
    if not groups:
        d.text((2, 0), "LOG", font=F_INV, fill="black")
        d.text((0, 14), "(no messages)", font=F_T, fill="white")
        return
    ui.cursor = max(0, min(ui.cursor, len(groups) - 1))
    g = groups[ui.cursor]
    d.text((2, 0), f"LOG {ui.cursor + 1}/{len(groups)}", font=F_INV, fill="black")
    rtext(d, 0, f"x{g['n']}  {ago(g['t'])} ago", font=F_INV, fill="black", right=126)
    lv = LEVEL_TAG.get(g["level"], "?")
    w = chip(d, 0, 10, {"E": "ERROR", "W": "WARN", "I": "INFO", "D": "DEBUG"}.get(lv, g["level"][:5]),
             lv == "E")
    d.text((w + 4, 11), clip(d, g["src"], 122 - w), font=F_T, fill="white")
    for i, line in enumerate(wrap(d, g["msg"], 127)[:5]):
        d.text((0, 21 + i * 9), line, font=F_T, fill="white")


def draw_pf_list(d, st, ui):
    """Focus mode on PRE-FLIGHT: every check, scrollable, six at a time."""
    res = ui.pf_results
    ui.cursor = max(0, min(ui.cursor, len(res) - 6))
    n_fail = sum(r[1] == FAIL for r in res)
    d.rectangle((0, 0, 127, HDR_H - 1), fill="white")
    d.text((2, 0), f"CHECKS {ui.cursor + 1}-{min(len(res), ui.cursor + 6)}/{len(res)}",
           font=F_INV, fill="black")
    rtext(d, 0, f"{n_fail} failed", font=F_INV, fill="black", right=126)
    for i, (name, status, detail) in enumerate(res[ui.cursor:ui.cursor + 6]):
        y = BODY_Y + i * ROW_H
        chip(d, 0, y - 1, status, status == FAIL)
        d.text((17, y), name, font=F_T, fill="white")
        rtext(d, y, clip(d, detail, 107 - text_w(d, name)))
    # scrollbar
    top = BODY_Y + int((64 - BODY_Y) * ui.cursor / len(res))
    d.line((127, top, 127, top + int((64 - BODY_Y) * 6 / len(res))), fill="white")


def servo_block(st):
    """Why the panel may NOT move the payload servo right now, or None.
    Someone at the panel is standing next to the props, and a RELEASE while a
    parcel is loaded drops it - so arming, or the navigator mid-drop, wins."""
    if not st.gcs_ok:
        return "GCS down"
    if not st.g("link", "connected"):
        return "no FC link"
    if st.armed() is not False:
        return "drone ARMED"
    if (st.g("mission", "drop_stage", default="") or "") in DROP_ACTIVE:
        return "drop running"
    if not st.g("payload", "channel"):
        return "no payload cfg"
    return None


def servo_aux(ch):
    return f"AUX{ch - 8}" if ch and ch >= 9 else f"MAIN{ch}"


def p_payload(d, st, ui):
    ch = st.g("payload", "channel", default=0)
    last = ui.servo_last
    block = servo_block(st)
    kv(d, [("Kind", str(st.g("payload_kind", default="?")).upper()),
           ("Servo", f"{servo_aux(ch)} (ch{ch})" if ch else "-", not ch),
           ("Lock / rel", f"{st.g('payload', 'lock_us', default='-')}/"
                          f"{st.g('payload', 'release_us', default='-')} us"),
           ("Last", f"{last[0]} {ago(last[1])} ago" if last else "- (this boot)"),
           ("Panel ctl", block or "ready", bool(block))])
    d.text((0, BODY_Y + 5 * ROW_H), "SELECT = servo / kind", font=F_T, fill="white")


MB_SHORT = {"bridge_down": "bridge down", "no_adapter": "no USB BT", "no_key": "no key",
            "scanning": "scanning", "connected": "ok", "error": "error", "off": "off"}


def p_medbox(d, st, ui):
    """The MedBox payload: what the box itself says about the medicine."""
    if st.g("payload_kind") != "medbox":
        kv(d, [("Payload", str(st.g("payload_kind", default="?")).upper())])
        for i, line in enumerate(wrap(d, "MedBox not selected. PAYLOAD page > KIND to switch.", 127)[:4]):
            d.text((0, BODY_Y + (2 + i) * ROW_H), line, font=F_T, fill="white")
        return
    m = st.g("medbox", default={}) or {}
    b = m.get("status") or {}
    link = m.get("link", "bridge_down")
    if link != "connected" or not b:
        rssi = f" {m['rssi']}dBm" if m.get("rssi") is not None else ""
        kv(d, [("Link", MB_SHORT.get(link, link) + rssi, True), ("Adapter", m.get("adapter") or "-")])
        for i, line in enumerate(wrap(d, m.get("note") or "", 127)[:4]):
            d.text((0, BODY_Y + (2 + i) * ROW_H), line, font=F_T, fill="white")
        return
    state = b.get("state", "?")
    if state == "COMPROMISED":
        st_txt = "COMPROMISED"
    elif state == "ARMED" and b.get("drop_phase") not in (None, "none"):
        st_txt = f"ARMED drop:{b['drop_phase']}"
    else:
        st_txt = state
    t, h = b.get("temp_c"), b.get("humidity")
    bv = b.get("batt_v")
    kv(d, [("Box", st_txt, state == "COMPROMISED"),
           ("Temp / RH", f"{t:.1f}C {h:.0f}%" if t is not None and h is not None else "n/a",
            bool(b.get("dht_fault"))),
           ("Lid", "closed" if b.get("lid_closed") else "OPEN",
            state == "ARMED" and not b.get("lid_closed")),
           ("Shake/shock", f"{b.get('max_shake_g') or 0:.2f}/{b.get('max_shock_g') or 0:.1f}g",
            bool(b.get("imu_fault"))),
           ("Why", str(b.get("reason", "?")), True) if state == "COMPROMISED" else
           ("Battery", f"{bv:.2f}V {b.get('batt_level', '')}" if bv else "n/a",
            b.get("batt_level") in ("LOW", "CRITICAL")),
           ("Link", (f"{m['rssi']}dBm " if m.get("rssi") is not None else "") + f"{m.get('adapter', '')}"
                    + ("" if m.get("key_ok") else " nokey"), not m.get("key_ok"))])


def draw_servo_ctl(d, st, ui):
    """Focus mode on PAYLOAD: pick LOCK / RELEASE / BACK, SELECT sends it."""
    ui.cursor = max(0, min(ui.cursor, len(SERVO_ITEMS) - 1))
    d.rectangle((0, 0, 127, HDR_H - 1), fill="white")
    d.text((2, 0), "SERVO", font=F_INV, fill="black")
    rtext(d, 0, servo_aux(st.g("payload", "channel", default=0)), font=F_INV,
          fill="black", right=126)
    us = {"LOCK": st.g("payload", "lock_us"), "RELEASE": st.g("payload", "release_us")}
    for i, item in enumerate(SERVO_ITEMS):
        y = 11 + i * 11
        sel = i == ui.cursor
        if sel:
            d.rounded_rectangle((0, y - 1, 127, y + 9), radius=2, fill="white")
        ink = "black" if sel else "white"
        d.text((4, y), ("> " if sel else "  ") + item, font=F_T, fill=ink)
        if us.get(item) is not None:
            rtext(d, y, f"{us[item]} us", fill=ink, right=123)
        elif item == "KIND":
            rtext(d, y, str(st.g("payload_kind", default="?")).upper(), fill=ink, right=123)
    block = servo_block(st)
    msg = f"BLOCKED: {block}" if block else (ui.servo_msg or "UP/DN pick  SEL send")
    d.text((0, 55), clip(d, msg, 127), font=F_T, fill="white")


def alt_block(st):
    """Why the panel may NOT change the flight altitude now, or None."""
    if not st.gcs_ok:
        return "GCS down"
    if st.armed() is not False:
        return "drone ARMED"
    return None


def alt_top(st):
    """Highest height worth offering: the navigator clamps to the ceiling anyway."""
    ceil = st.g("mission", "alt_ceiling_m", default=0.0) or 0.0
    return min(ALT_MAX, ceil) if ceil >= ALT_MIN else ALT_MAX


def p_altitude(d, st, ui):
    m = lambda k: st.g("mission", k, default=0.0) or 0.0
    block = alt_block(st)
    kv(d, [("Flight alt", f"{m('flight_alt_m'):.1f} m"),
           ("Ceiling", f"{m('alt_ceiling_m'):.1f} m"),
           ("Now (rel)", f"{st.g('telemetry', 'altitude', default=0.0):.1f} m"),
           ("Applies to", "takeoff + legs"),
           ("Panel ctl", block or "ready", bool(block))])
    d.text((0, BODY_Y + 5 * ROW_H), "SELECT = adjust height", font=F_T, fill="white")


def draw_alt_ctl(d, st, ui):
    """Focus mode on ALTITUDE: UP/DOWN edit a pending value, SELECT sends it."""
    cur = st.g("mission", "flight_alt_m", default=0.0) or 0.0
    d.rectangle((0, 0, 127, HDR_H - 1), fill="white")
    d.text((2, 0), "HEIGHT", font=F_INV, fill="black")
    rtext(d, 0, f"{cur:.1f}m", font=F_INV, fill="black", right=126)
    v = ui.alt_edit
    ctext(d, 64, 13, f"{v:.1f} m", F_M)
    lo, hi = ALT_MIN, alt_top(st)
    bar(d, 0, 38, 127, 7, (v - lo) / max(0.1, hi - lo))
    d.text((0, 46), f"{lo:.0f}", font=F_T, fill="white")
    rtext(d, 46, f"{hi:.1f} ceil")
    block = alt_block(st)
    if block:
        msg = f"BLOCKED: {block}"
    elif ui.alt_msg:
        msg = ui.alt_msg
    elif abs(v - cur) < 0.01:
        msg = "UP/DN change  SEL exit"
    else:
        msg = "SELECT = send"
    d.text((0, 55), clip(d, msg, 127), font=F_T, fill="white")


ROTORS = [(-11, -11), (11, -11), (-11, 11), (11, 11)]


def draw_drone(d, c, arm_t, rotor_t, angle):
    cx, cy = c
    for i, (dx, dy) in enumerate(ROTORS):
        px, py = cx + dx * arm_t, cy + dy * arm_t
        d.line((cx, cy, px, py), fill="white")
        if rotor_t > 0:
            r = 6 * rotor_t
            d.ellipse((px - r, py - r, px + r, py + r), outline="white")
            # diagonal pairs spin opposite ways, like a real quad
            a = angle if i in (0, 3) else -angle
            bx, by = 4.5 * rotor_t * math.cos(a), 4.5 * rotor_t * math.sin(a)
            d.line((px - bx, py - by, px + bx, py + by), fill="white")
    d.rectangle((cx - 3, cy - 3, cx + 3, cy + 3), fill="white")
    d.point((cx, cy), fill="black")


def ease(t):
    return t * t * (3 - 2 * t)


def startup_animation(push, buz, fps=30):
    """~5 s boot: spin-up on the pad, lift-off, fly to the corner, then the
    name wipes in while it hovers. Only the drone, wipe edge and bar change
    per frame, so page-diff pushes keep it smooth on 100 kHz I2C."""
    word, wx = "AERIX", 42
    sub = "AUTONOMOUS DELIVERY"
    pad, home = (64, 42), (19, 21)
    names = ["arms", "spin", "lift", "fly", "wipe", "sub", "bar", "hold"]
    lens = [10, 18, 14, 20, 16, 10, 18, 10]
    angle, dt = 0.0, 1 / fps
    letters = 0
    img = None
    for pi, (phase, n) in enumerate(zip(names, lens)):
        if phase == "arms":
            buz.play(sweep(LO, 2600, 6, .03))
        elif phase == "spin":
            buz.play(sweep(2600, HI, 12, .035))             # motors winding up
        elif phase == "lift":
            buz.play([(HI, .05), (0, .03), (HI, .05)])
        for k in range(n):
            t0 = time.monotonic()
            f = (k + 1) / n
            done = set(names[:pi])
            angle += 0.15 + 0.6 * (f if phase == "spin" else (1.0 if "spin" in done else 0))
            img = Image.new("1", (W, H))
            d = ImageDraw.Draw(img)
            arm_t = f if phase == "arms" else 1.0
            rotor_t = 1.0 if phase != "arms" else min(1.0, f * 1.5)
            if phase in ("arms", "spin"):
                c = pad
                d.line((40, 58, 88, 58), fill="white")            # landing pad
            elif phase == "lift":
                c = (pad[0], pad[1] - 14 * ease(f))
                d.line((40, 58, 88, 58), fill="white")
                if k % 2 == 0:                                   # rotor wash
                    for x in (52, 64, 76):
                        d.line((x, 55, x + (x - 64) // 6, 57), fill="white")
            elif phase == "fly":
                e = ease(f)
                c = (pad[0] + (home[0] - pad[0]) * e, pad[1] - 14 + (home[1] - pad[1] + 14) * e)
            else:
                c = (home[0], home[1] + round(math.sin(time.monotonic() * 5)))   # hover bob
            draw_drone(d, (round(c[0]), round(c[1])), arm_t, rotor_t, angle)
            if phase == "wipe" or "wipe" in done:
                d.text((wx, -1), word, font=F_X, fill="white")
                if phase == "wipe":
                    edge = wx + int(80 * f)
                    d.rectangle((edge, 0, 127, 34), fill="black")
                    if f < 1:
                        d.line((edge, 2, edge, 32), fill="white")      # scan edge
                    shown = min(5, int(f * 5 + 0.2))
                    if shown > letters:
                        buz.play([(HI, .012)])                        # tick per letter
                        letters = shown
            if phase == "sub" or "sub" in done:
                n_ch = len(sub) if phase != "sub" else int(len(sub) * f)
                x = (W - text_w(d, sub)) / 2
                d.text((x, 41), sub[:n_ch], font=F_T, fill="white")
            if phase in ("bar", "hold"):
                bar(d, 14, 53, 99, 7, f if phase == "bar" else 1.0)
            push(img)
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))
    buz.play([(LO, .08), (0, .02), (HI, .08), (0, .02), (HI, .2)])  # ready chord
    inv = Image.eval(img.convert("L"), lambda v: 255 - v).convert("1")
    push(inv)                                                       # flash
    time.sleep(0.08)
    push(img)
    time.sleep(0.6)


PAGES = [("AERIX", p_overview), ("PRE-FLIGHT", p_preflight), ("FLIGHT", p_flight),
         ("GPS", p_gps), ("BATTERY", p_battery), ("SENSORS", p_sensors),
         ("AVOIDANCE", p_avoid), ("CAMERA", p_camera), ("MISSION", p_mission),
         ("PAYLOAD", p_payload), ("MEDBOX", p_medbox), ("ALTITUDE", p_altitude), ("NETWORK", p_network), ("SYSTEM", p_system), ("LOG", p_log)]
PAGE_IDX = {name: i for i, (name, _) in enumerate(PAGES)}


# ---------------------------------------------------------------- app
class Panel:
    def __init__(self):
        self.oled = sh1106(i2c(port=1, address=0x3C))
        # Contrast only: pre-charge/VCOMH tweaks (0xD9=0xF1, 0xDB=0x40) blanked
        # this SH1106 clone on 2026-10-04.
        self.oled.contrast(BRIGHT)
        self.buz = Buzzer(PIN_BUZZER)
        self.st = State()
        self.limits = load_limits()
        self.events = queue.Queue()
        self.page = 0
        self.focus = None            # None | "log" | "pf" | "servo" | "alt"  (focus modes)
        self.cursor = 0
        self.muted = False
        self.confirm_until = 0.0
        self.last_press = time.monotonic()
        self.asleep = False
        self.prev = {}
        self.next_warn = {}
        self.tick = 0
        self.last_frame = None       # bytes of the frame on the glass
        self.glass = None            # packed SH1106 pages last sent
        self.frame_s = 0.1           # measured I2C push time (EMA)
        self.held = {"up": False, "down": False, "select": False}
        self.pf_running = False
        self.pf_progress = (0, 0)
        self.pf_current = ""
        self.pf_partial = []
        self.pf_results = []
        self.pf_time = 0.0
        self.servo_last = None       # (action, wall time) last sent from the panel
        self.servo_msg = ""
        self.servo_busy = False
        self.alt_edit = 0.0          # pending height in the ALTITUDE editor
        self.alt_msg = ""
        self.alt_busy = False

        for name, pin, hold in (("up", PIN_UP, 1.0), ("down", PIN_DOWN, 1.0),
                                ("select", PIN_SELECT, SHUTDOWN_HOLD_S)):
            # gpiozero devices refuse new attributes, so the "this press
            # already fired as a long press" flag lives in self.held.
            b = Button(pin, pull_up=True, bounce_time=0.03, hold_time=hold)
            b.when_pressed = lambda n=name: self.held.__setitem__(n, False)
            b.when_held = lambda n=name: (self.held.__setitem__(n, True),
                                          self.events.put((n, "long")))
            b.when_released = lambda n=name: (None if self.held[n]
                                              else self.events.put((n, "short")))
            setattr(self, "btn_" + name, b)

    # ---- frames
    def push(self, img):
        """Send only the changed span of each 8-px page row. luma always
        resends all 1024 bytes (~95 ms on 100 kHz I2C); a spinning prop or a
        ticking value changes a few dozen. Unchanged frames cost nothing."""
        b = img.tobytes()
        if b == self.last_frame:
            return
        a = np.asarray(img, dtype=bool).astype(np.uint8)
        pages = np.packbits(a.reshape(8, 8, W), axis=1, bitorder="little").reshape(8, W)
        t0, sent = time.monotonic(), 0
        for pg in range(8):
            if self.glass is None:
                x0, x1 = 0, W
            else:
                diff = np.flatnonzero(pages[pg] != self.glass[pg])
                if diff.size == 0:
                    continue
                x0, x1 = int(diff[0]), int(diff[-1]) + 1
            col = x0 + 2                    # SH1106 RAM is 132 wide; glass starts at col 2
            self.oled.command(0xB0 | pg, col & 0x0F, 0x10 | (col >> 4))
            self.oled.data(pages[pg, x0:x1].tolist())
            sent += x1 - x0
        if sent >= 256:                     # estimate full-frame time from big pushes
            full = (time.monotonic() - t0) * 1024 / sent
            self.frame_s = 0.8 * self.frame_s + 0.2 * full
        self.glass, self.last_frame = pages, b

    def render(self):
        img = Image.new("1", (W, H))
        d = ImageDraw.Draw(img)
        if self.confirm_until > time.monotonic():
            ok = self.shutdown_allowed()
            d.rectangle((0, 0, 127, 63), outline="white")
            d.text((6, 4), "SHUT DOWN PI?", font=F_S, fill="white")
            d.text((6, 22), "B3 = yes  B1/B2 = no" if ok else "BLOCKED: drone ARMED", font=F_T, fill="white")
            left = self.confirm_until - time.monotonic()
            bar(d, 6, 40, 115, 7, left / CONFIRM_WINDOW_S)
            d.text((6, 51), f"cancels in {left:.0f}s", font=F_T, fill="white")
        elif self.focus == "log":
            draw_log_viewer(d, self.st, self)
        elif self.focus == "pf" and self.pf_results:
            draw_pf_list(d, self.st, self)
        elif self.focus == "servo":
            draw_servo_ctl(d, self.st, self)
        elif self.focus == "alt":
            draw_alt_ctl(d, self.st, self)
        else:
            self.focus = None
            title, fn = PAGES[self.page]
            if fn is not p_overview:              # home draws its own header
                header(d, title, self.page + 1, len(PAGES), self.st, self.muted)
            try:
                fn(d, self.st, self)
            except Exception as e:  # one bad field must not kill the panel
                d.text((0, 20), f"page error: {type(e).__name__}", font=F_T, fill="white")
                print(f"[draw] {title}: {e!r}", flush=True)
        return img

    def glass_img(self):
        return Image.frombytes("1", (W, H), self.last_frame) if self.last_frame else None

    def move(self, old, new, axis, sign, dist, top=0, dur=SLIDE_S):
        """Eased transition of rows top..63 from `old` to `new`: the old content
        travels `dist` px along `axis` ("x"/"y", sign +1 = left/up) while the
        new content follows it in. Rows above `top` (a header) cut straight to
        `new`. dist = full width/height swaps screens; dist = ROW_H scrolls a
        list by one row, since the new list is then the old one shifted a row.
        Time-based, so a slow bus drops frames instead of stretching the move,
        and a press mid-move cuts to the target. Every frame rewrites every
        row, so it is off when a full push takes > 45 ms (100 kHz I2C). CPU per
        frame: a few pastes and a bit-pack, well under 1 ms."""
        if old is None or self.frame_s > 0.045 or old.tobytes() == new.tobytes():
            self.push(new)
            return
        box = (0, top, W, H)
        a, b = old.crop(box), new.crop(box)
        t0 = time.monotonic()
        while self.events.empty():
            f = (time.monotonic() - t0) / dur
            if f >= 1.0:
                break
            off = round(dist * (1 - (1 - f) ** 3))   # ease-out cubic: fast start, soft landing
            body = Image.new("1", a.size)
            if axis == "y":
                body.paste(a, (0, -off * sign))
                body.paste(b, (0, (dist - off) * sign))
            else:
                body.paste(a, (-off * sign, 0))
                body.paste(b, ((dist - off) * sign, 0))
            img = new.copy()
            img.paste(body, (0, top))
            self.push(img)
        self.push(new)

    def slide(self, old, new, direction):
        """Page change: B1 UP pushes the page up and the next rises from below."""
        self.move(old, new, "y", direction, H)

    def splash(self):
        startup_animation(self.push, self.buz)

    # ---- pre-flight
    def pf_summary(self):
        if self.pf_running:
            return "PF ..."
        if not self.pf_results:
            return "PF not run"
        n_fail = sum(r[1] == FAIL for r in self.pf_results)
        return f"PF {n_fail} NO" if n_fail else "PF READY"

    def run_preflight(self):
        if self.pf_running:
            return
        self.pf_running = True
        self.pf_progress = (0, 1)
        self.pf_partial = []
        threading.Thread(target=self._preflight_worker, daemon=True).start()

    def _preflight_worker(self):
        try:
            self.pf_current = "refresh"
            self.st.poll_sys()          # fresh services / Wi-Fi / throttle
            self.st.poll_gcs()          # fresh telemetry snapshot
            checks = preflight_checks(self.st, self.limits)
            results = []
            for i, (name, fn) in enumerate(checks):
                self.pf_current, self.pf_progress = name, (i, len(checks))
                try:
                    status, detail = fn()
                except Exception as e:  # a broken check must read as a failure
                    status, detail = FAIL, type(e).__name__
                results.append((name, status, str(detail)))
                self.pf_partial = list(results)
                time.sleep(0.06)        # visible progress
            self.pf_progress = (len(checks), len(checks))
            self.pf_results, self.pf_time = results, time.time()
            self.last_press = time.monotonic()   # keep the verdict on screen
            if any(r[1] == FAIL for r in results):
                self.buz.alarm()
            elif any(r[1] == WARN for r in results):
                self.buz.warn()
            else:
                self.buz.ok()
            for name, status, detail in results:
                print(f"[preflight] {status:2} {name:10} {detail}", flush=True)
        finally:
            self.pf_running = False

    # ---- input
    def handle(self, name, kind):
        now = time.monotonic()
        print(f"[btn] {name} {kind} page={PAGES[self.page][0]} focus={self.focus}", flush=True)
        self.last_press = now
        if self.asleep:
            self.wake()
            self.buz.wake()
            return
        self.buz.click()
        if self.confirm_until > now:
            if name == "select" and kind == "short":
                self.shutdown()
            else:
                self.confirm_until = 0.0
            return
        if name == "select" and kind == "long":
            self.focus = None
            self.confirm_until = now + CONFIRM_WINDOW_S
            self.buz.warn()
            return
        if self.focus == "servo":
            if kind != "short":
                return
            if name == "up":
                self.cursor = max(0, self.cursor - 1)
            elif name == "down":
                self.cursor = min(len(SERVO_ITEMS) - 1, self.cursor + 1)
            elif SERVO_ITEMS[self.cursor] == "BACK":
                self.focus = None
                self.move(self.glass_img(), self.render(), "x", -1, W)
            elif SERVO_ITEMS[self.cursor] == "KIND":
                self.send_kind()
            else:
                self.send_servo(SERVO_ITEMS[self.cursor])
            return
        if self.focus == "alt":
            if kind != "short":
                return
            cur = self.st.g("mission", "flight_alt_m", default=0.0) or 0.0
            if name in ("up", "down"):
                step = ALT_STEP if name == "up" else -ALT_STEP
                self.alt_edit = max(ALT_MIN, min(alt_top(self.st), self.alt_edit + step))
                self.alt_msg = ""
            elif abs(self.alt_edit - cur) < 0.01 and not self.alt_busy:
                self.focus = None
                self.move(self.glass_img(), self.render(), "x", -1, W)
            else:
                self.send_alt(self.alt_edit)
            return
        if self.focus:                       # inside a list viewer
            old, focus, prev = self.glass_img(), self.focus, self.cursor
            if name == "up" and kind == "short":
                self.cursor = max(0, self.cursor - 1)
            elif name == "down" and kind == "short":
                self.cursor += 1
            elif name == "select" and kind == "short":
                self.focus = None
            new = self.render()              # clamps the cursor at the list ends
            if self.focus is None:           # back out: the page returns from the left
                self.move(old, new, "x", -1, W)
            elif self.cursor != prev:
                sign = 1 if self.cursor > prev else -1
                if focus == "pf":            # six-row list: scroll by one row
                    self.move(old, new, "y", sign, ROW_H, top=HDR_H, dur=0.12)
                else:                        # one message per screen: swap the body
                    self.move(old, new, "y", sign, H - HDR_H, top=HDR_H)
            return
        title = PAGES[self.page][0]
        old_page, old_focus = self.page, self.focus
        if name == "up" and kind == "short":
            self.page = (self.page + 1) % len(PAGES)
        elif name == "down" and kind == "short":
            self.page = (self.page - 1) % len(PAGES)
        elif name == "up" and kind == "long":
            self.muted = not self.muted
        elif name == "down" and kind == "long":
            self.page = PAGE_IDX["AERIX"]
        elif name == "select" and kind == "short":
            if title == "PRE-FLIGHT":
                if self.pf_results and not self.pf_running:
                    self.focus, self.cursor = "pf", 0
                else:
                    self.run_preflight()
            elif title == "LOG":
                self.focus, self.cursor = "log", 0
            elif title == "ALTITUDE":
                cur = self.st.g("mission", "flight_alt_m", default=0.0) or 5.0
                self.focus, self.alt_msg = "alt", ""
                self.alt_edit = max(ALT_MIN, min(alt_top(self.st), round(cur * 2) / 2))
            elif title == "PAYLOAD":
                self.focus, self.cursor, self.servo_msg = "servo", 0, ""
            elif title == "AERIX":
                self.page = PAGE_IDX["PRE-FLIGHT"]
                self.run_preflight()
        if self.page != old_page:
            fwd = (self.page - old_page) % len(PAGES) == 1
            self.slide(self.glass_img(), self.render(), 1 if fwd else -1)
        elif self.focus and not old_focus:   # opening a list: it comes in from the right
            self.move(self.glass_img(), self.render(), "x", 1, W)

    # ---- payload servo
    def send_servo(self, action):
        if self.servo_busy:
            return
        self.st.poll_gcs()                   # gate on a fresh snapshot, not a 0.5 s old one
        block = servo_block(self.st)
        if block:
            self.servo_msg = f"BLOCKED: {block}"
            self.buz.alarm()
            return
        ch = int(self.st.g("payload", "channel"))
        us = self.st.g("payload", "lock_us" if action == "LOCK" else "release_us")
        if us is None:
            self.servo_msg = f"no {action.lower()}_us in GCS"
            self.buz.alarm()
            return
        self.servo_busy, self.servo_msg = True, f"sending {action}..."
        threading.Thread(target=self._servo_worker, args=(action, ch, int(us)),
                         daemon=True).start()

    def _servo_worker(self, action, ch, us):
        res = post_cmd("set_servo", {"channel": ch, "pwm": us})
        ok = bool(res.get("ok"))
        self.servo_msg = f"{action} {us}us sent" if ok else clip_plain(res.get("message", "refused"))
        print(f"[servo] {action} ch{ch} {us}us ok={ok} {self.servo_msg}", flush=True)
        if ok:
            self.servo_last = (action, time.time())
            self.buz.ok()
        else:
            self.buz.alarm()
        self.last_press = time.monotonic()
        self.servo_busy = False

    def send_kind(self):
        """Cycle parcel -> medbox -> none. The hub refuses while armed or
        mid-drop, so the panel just reports its answer."""
        if self.servo_busy:
            return
        self.st.poll_gcs()
        if not self.st.gcs_ok:
            self.servo_msg = "BLOCKED: GCS down"
            self.buz.alarm()
            return
        cur = self.st.g("payload_kind", default="parcel")
        nxt = PAYLOAD_KINDS[(PAYLOAD_KINDS.index(cur) + 1) % len(PAYLOAD_KINDS)
                            if cur in PAYLOAD_KINDS else 0]
        self.servo_busy, self.servo_msg = True, f"kind -> {nxt}..."

        def work():
            res = post_cmd("set_payload_kind", {"kind": nxt})
            ok = bool(res.get("ok"))
            self.servo_msg = f"kind {nxt}" if ok else clip_plain(res.get("message", "refused"))
            print(f"[payload] kind {cur} -> {nxt} ok={ok} {self.servo_msg}", flush=True)
            (self.buz.ok if ok else self.buz.alarm)()
            self.last_press = time.monotonic()
            self.servo_busy = False
        threading.Thread(target=work, daemon=True).start()

    # ---- flight altitude
    def send_alt(self, alt):
        if self.alt_busy:
            return
        self.st.poll_gcs()
        block = alt_block(self.st)
        if block:
            self.alt_msg = f"BLOCKED: {block}"
            self.buz.alarm()
            return
        self.alt_busy, self.alt_msg = True, f"sending {alt:.1f} m..."
        threading.Thread(target=self._alt_worker, args=(alt,), daemon=True).start()

    def _alt_worker(self, alt):
        res = post_cmd("set_flight_altitude", {"alt_m": alt})
        ok = bool(res.get("ok"))
        applied = (res.get("data") or {}).get("alt_m")
        if ok:
            self.alt_msg = f"set {float(applied if applied is not None else alt):.1f} m"
        else:
            self.alt_msg = clip_plain(res.get("message", "refused"))
        print(f"[alt] {alt:.1f} m ok={ok} {res.get('message', '')}", flush=True)
        self.st.poll_gcs()                   # show the new setpoint right away
        (self.buz.ok if ok else self.buz.alarm)()
        self.last_press = time.monotonic()
        self.alt_busy = False

    def shutdown_allowed(self):
        """Never power the Pi off under a flying aircraft. If the GCS is down
        the Pi is not flying anything, so that is allowed too."""
        return self.st.armed() is not True

    def shutdown(self):
        self.confirm_until = 0.0
        if not self.shutdown_allowed():
            self.buz.alarm()
            return
        self.buz.goodbye()
        img = Image.new("1", (W, H))
        d = ImageDraw.Draw(img)
        d.text(((128 - text_w(d, "AERIX", F_M)) / 2, 6), "AERIX", font=F_M, fill="white")
        d.text((8, 30), "Shutting down...", font=F_T, fill="white")
        d.text((8, 42), "wait for green LED", font=F_T, fill="white")
        d.text((8, 51), "to stop blinking", font=F_T, fill="white")
        self.push(img)
        time.sleep(1.0)
        subprocess.run(["sudo", "-n", "systemctl", "poweroff"])

    # ---- sleep
    def fade(self, levels):
        for v in levels:
            self.oled.contrast(v)
            time.sleep(0.03)

    def wake(self):
        if self.asleep:
            self.push(self.render())         # refresh RAM while still dark
            self.oled.contrast(0)
            self.oled.show()
            self.fade((30, 80, 150, BRIGHT))
            self.asleep = False
            print("[sleep] wake", flush=True)
        self.last_press = time.monotonic()

    def manage_sleep(self):
        busy = (self.st.armed() is True or self.pf_running
                or self.confirm_until > time.monotonic())
        if busy:
            if self.asleep:
                self.wake()
            return
        if not self.asleep and time.monotonic() - self.last_press > SLEEP_AFTER_S:
            # hide() sends DISPLAYOFF: the panel's charge pump stops and the
            # OLED drops to microamps. GPIO/I2C stay up, so a press wakes it.
            self.fade((170, 100, 50, 15, 0))
            self.oled.hide()
            self.oled.contrast(BRIGHT)       # display is off; ready for the next wake
            self.asleep = True
            self.focus = None
            print("[sleep] screen off", flush=True)

    # ---- alerts (edge-triggered so they do not nag)
    def alert(self, sound):
        sound()
        self.wake()                 # show why it beeped

    def alerts(self):
        st, now = self.st, time.monotonic()
        mb = (st.g("medbox", default={}) or {}) if st.g("payload_kind") == "medbox" else {}
        cur = {"gcs": st.gcs_ok, "armed": st.armed(), "fc": st.g("link", "connected"),
               "box": (mb.get("status") or {}).get("state"), "mb_link": mb.get("link")}
        p = self.prev
        if p:
            if cur["armed"] is True and p.get("armed") is not True:
                self.alert(self.buz.armed)
            elif cur["armed"] is False and p.get("armed") is True:
                self.alert(self.buz.ok)
            if not self.muted:
                if p.get("gcs") and not cur["gcs"]:
                    self.alert(self.buz.alarm)
                if p.get("fc") and not cur["fc"]:
                    self.alert(self.buz.warn)
                if cur["box"] == "COMPROMISED" and p.get("box") != "COMPROMISED":
                    self.alert(self.buz.alarm)      # medicine no longer trustworthy
                elif p.get("mb_link") == "connected" and cur["mb_link"] not in (None, "connected"):
                    self.alert(self.buz.warn)       # lost the box
        self.prev = cur
        if self.muted:
            return
        v = st.g("telemetry", "battery_v", default=0.0)
        if cur["armed"] and 0 < v < self.limits["bat_low"] and now > self.next_warn.get("bat", 0):
            self.alert(self.buz.warn)
            self.next_warn["bat"] = now + 30
        if st.g("health", "temp", default=0.0) > 80 and now > self.next_warn.get("tmp", 0):
            self.alert(self.buz.warn)
            self.next_warn["tmp"] = now + 60

    # ---- main loop
    def run(self):
        self.splash()
        print(f"[oled] frame push {self.frame_s * 1000:.0f} ms -> page slide "
              f"{'on' if self.frame_s <= 0.045 else 'off (I2C too slow)'}", flush=True)
        next_sys = 0.0
        while True:
            t0 = time.monotonic()
            self.tick += 1
            if not self.pf_running:      # the worker does its own fresh polls
                if t0 >= next_sys:
                    self.st.poll_sys()
                    next_sys = t0 + (30.0 if self.asleep else 5.0)
                self.st.poll_gcs()
            while not self.events.empty():
                self.handle(*self.events.get())
            self.alerts()
            self.manage_sleep()
            if not self.asleep:
                self.push(self.render())
            period = 0.12 if self.pf_running else (SLEEP_POLL_S if self.asleep else POLL_S)
            end = t0 + period
            while time.monotonic() < end:
                try:
                    self.handle(*self.events.get(timeout=max(0.0, end - time.monotonic())))
                    if not self.asleep:
                        self.push(self.render())
                except queue.Empty:
                    break


if __name__ == "__main__":
    panel = Panel()
    try:
        panel.run()
    except KeyboardInterrupt:
        pass
    finally:
        panel.buz.dev.value = 0
        panel.oled.clear()
