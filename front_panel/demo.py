#!/usr/bin/env python3
"""Front-panel style demo: fonts 1-4, buzzer styles A-D, startup animation.
Run with the panel stopped (it owns the buzzer pin):
    sudo systemctl stop aerix-panel; venv/bin/python demo.py; sudo systemctl start aerix-panel
"""
import math
import sys
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, "/home/pi/front_panel")
import panel as P                                   # noqa: E402  (GPIO factory, Buzzer, fonts)
from luma.core.interface.serial import i2c          # noqa: E402
from luma.oled.device import sh1106                 # noqa: E402

X11 = "/usr/share/fonts/X11/"
FONTS = {
    "1 SPLEEN 5x8":  (P.F_T, P.F_T, 9, 0),
    "2 HELVETICA":   (ImageFont.truetype(X11 + "75dpi/helvR08.pcf.gz", 8),
                      ImageFont.truetype(X11 + "75dpi/helvB08.pcf.gz", 8), 9, -1),
    "3 CLASSIC 6x10": (ImageFont.truetype(X11 + "misc/6x10.pcf.gz", 10),
                       ImageFont.truetype(X11 + "misc/6x10.pcf.gz", 10), 11, 0),
    "4 SPLEEN 6x12": (P.F_S, P.F_S, 11, -1),
}
SAMPLE = [("Mode", "LOITER ARMED"), ("Alt rel/AMSL", "3.2 / 914m"), ("Ground speed", "1.4 m/s"),
          ("Vertical", "+0.3 m/s"), ("Heading", "271 W"), ("Pitch/Roll", "+2.1 / -0.8")]


class Glass:
    """Sends only the changed span of each 8-px page. luma always resends all
    1024 bytes (~95 ms at 100 kHz I2C); a spinning prop changes ~150."""

    def __init__(self, oled):
        self.oled, self.pages = oled, None

    def push(self, img):
        a = np.asarray(img, dtype=bool).astype(np.uint8)
        pages = np.packbits(a.reshape(8, 8, 128), axis=1, bitorder="little").reshape(8, 128)
        for p in range(8):
            if self.pages is None:
                x0, x1 = 0, 128
            else:
                diff = np.flatnonzero(pages[p] != self.pages[p])
                if diff.size == 0:
                    continue
                x0, x1 = int(diff[0]), int(diff[-1]) + 1
            col = x0 + 2                    # SH1106 RAM is 132 wide; glass starts at col 2
            self.oled.command(0xB0 | p, col & 0x0F, 0x10 | (col >> 4))
            self.oled.data(pages[p, x0:x1].tolist())
        self.pages = pages


def font_page(name, spec):
    lab_f, val_f, row_h, dy = spec
    img = Image.new("1", (128, 64))
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, 127, 8), fill="white")
    d.text((2, 0), "FLIGHT", font=P.F_T, fill="black")
    d.text((126 - d.textlength(name, font=P.F_T), 0), name, font=P.F_T, fill="black")
    y = 10
    for label, value in SAMPLE:
        if y + row_h - 2 > 64:
            break
        d.text((0, y + dy), label, font=lab_f, fill="white")
        d.text((127 - d.textlength(value, font=val_f), y + dy), value, font=val_f, fill="white")
        y += row_h
    return img


def sweep(f0, f1, n, dt):
    return [(int(f0 + (f1 - f0) * i / (n - 1)), dt) for i in range(n)]


LO, HI = P.LO, P.HI
STYLES = {
    "A TWO-TONE": {"ok": [(LO, .07), (0, .02), (HI, .14)],
                   "warn": [(HI, .16), (0, .05), (LO, .28)],
                   "alarm": [(HI, .12), (LO, .12)] * 3},
    "B GLIDE": {"ok": sweep(LO, HI, 12, .014) + [(HI, .09)],
                "warn": sweep(HI, LO, 10, .02) + [(0, .05)] + sweep(HI, LO, 10, .02),
                "alarm": (sweep(LO, HI, 10, .015) + sweep(HI, LO, 10, .015)) * 3},
    "C WARBLE": {"ok": [(LO, .02), (HI, .02)] * 3 + [(HI, .1)],
                 "warn": [(LO, .03), (HI, .03)] * 7,
                 "alarm": [(LO, .015), (HI, .015)] * 30},
    "D MELODY": {"ok": [(2000, .06), (2500, .06), (3000, .06), (3500, .14)],
                 "warn": [(3500, .12), (3000, .12), (2000, .25)],
                 "alarm": ([(3500, .07), (0, .03)] * 3 + [(2000, .2), (0, .1)]) * 2},
}


def title(text, sub):
    img = Image.new("1", (128, 64))
    d = ImageDraw.Draw(img)
    d.text(((128 - d.textlength(text, font=P.F_M)) / 2, 14), text, font=P.F_M, fill="white")
    d.text(((128 - d.textlength(sub, font=P.F_T)) / 2, 40), sub, font=P.F_T, fill="white")
    return img


# ---------------------------------------------------------------- startup
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


def startup_animation(glass, buz, fps=30):
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
            spin_rate = 0.15 + 0.6 * (f if phase == "spin" else (1.0 if "spin" in done else 0))
            angle += spin_rate
            img = Image.new("1", (128, 64))
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
                d.text((wx, -1), word, font=P.F_X, fill="white")
                if phase == "wipe":
                    edge = wx + int(80 * f)
                    d.rectangle((edge, 0, 127, 34), fill="black")
                    if f < 1:
                        d.line((edge, 2, edge, 32), fill="white")
                    shown = min(5, int(f * 5 + 0.2))
                    if shown > letters:
                        buz.play([(HI, .012)])
                        letters = shown
            if phase == "sub" or "sub" in done:
                n_ch = len(sub) if phase != "sub" else int(len(sub) * f)
                x = (128 - d.textlength(sub, font=P.F_T)) / 2
                d.text((x, 41), sub[:n_ch], font=P.F_T, fill="white")
            if phase in ("bar", "hold"):
                P.bar(d, 14, 53, 99, 7, f if phase == "bar" else 1.0)
            glass.push(img)
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))
    buz.play([(LO, .08), (0, .02), (HI, .08), (0, .02), (HI, .2)])  # ready chord
    inv = Image.eval(img.convert("L"), lambda v: 255 - v).convert("1")
    glass.push(inv)
    time.sleep(0.08)
    glass.push(img)
    time.sleep(0.6)


def picker(glass, buz, events, title_txt, options, show, on_change=None, hold_picks=False):
    """UP/DOWN cycle, SELECT picks (or replays when hold_picks; hold = pick)."""
    i = 0
    show(i)
    if on_change:
        on_change(i)
    while True:
        name, kind = events.get()
        if name == "up" and kind == "short":
            i = (i + 1) % len(options)
        elif name == "down" and kind == "short":
            i = (i - 1) % len(options)
        elif name == "select" and (kind == "long" or not hold_picks):
            buz.play([(HI, .03), (0, .03), (HI, .03)])
            return options[i]
        elif name == "select":
            if on_change:
                on_change(i)
            continue
        else:
            continue
        show(i)
        if on_change:
            on_change(i)


if __name__ == "__main__":
    if "--sheet" in sys.argv:                 # comparison image only, no hardware
        imgs = [font_page(n, s) for n, s in FONTS.items()]
        sheet = Image.new("L", (2 * 136 + 8, 2 * 72 + 8), 40)
        for i, im in enumerate(imgs):
            sheet.paste(im.convert("L"), (8 + (i % 2) * 136, 8 + (i // 2) * 72))
        sheet.resize((sheet.width * 3, sheet.height * 3), Image.NEAREST).save("/tmp/fonts.png")
        sys.exit()
    import queue
    from gpiozero import Button
    oled = sh1106(i2c(port=1, address=0x3C))
    oled.contrast(255)
    glass, buz = Glass(oled), P.Buzzer(P.PIN_BUZZER)
    if "--anim" in sys.argv:
        startup_animation(glass, buz)
        sys.exit()
    events = queue.Queue()
    held = {}
    btns = []
    for n, pin in (("up", P.PIN_UP), ("down", P.PIN_DOWN), ("select", P.PIN_SELECT)):
        b = Button(pin, pull_up=True, bounce_time=0.03, hold_time=1.0)
        b.when_pressed = lambda n=n: held.__setitem__(n, False)
        b.when_held = lambda n=n: (held.__setitem__(n, True), events.put((n, "long")))
        b.when_released = lambda n=n: None if held.get(n) else events.put((n, "short"))
        btns.append(b)

    glass.push(title("PICK A FONT", "UP/DOWN browse  SEL pick"))
    time.sleep(2.5)
    names = list(FONTS)
    font = picker(glass, buz, events, "FONT", names,
                  lambda i: (glass.push(font_page(names[i], FONTS[names[i]])), buz.click()))

    glass.push(title("PICK A SOUND", "UP/DN browse SEL replay"))
    time.sleep(1.5)
    glass.push(title("", "HOLD SEL 1s to pick"))
    time.sleep(1.5)
    snames = list(STYLES)

    def play_style(i):
        pats = STYLES[snames[i]]
        buz.play(pats["ok"] + [(0, .5)] + pats["warn"] + [(0, .5)] + pats["alarm"])

    style = picker(glass, buz, events, "SOUND", snames,
                   lambda i: glass.push(title(snames[i].split()[0], snames[i].split(None, 1)[1]
                                              + ": ok warn alarm")),
                   on_change=play_style, hold_picks=True)

    glass.push(title("STARTUP", "animation"))
    time.sleep(1.5)
    startup_animation(glass, buz)
    open("/tmp/picks.txt", "w").write(f"font={font}\nsound={style}\n")
    glass.push(title("PICKED", f"{font[:13]} / {style}"))
    print(f"font={font} sound={style}", flush=True)
    time.sleep(3)
    oled.clear()
