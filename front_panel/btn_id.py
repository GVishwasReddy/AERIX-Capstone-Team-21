#!/usr/bin/env python3
"""Identify which physical button is B1 / B2 / B3.

Press a button: the OLED shows its name in big letters, the terminal prints it,
and the buzzer beeps 1, 2 or 3 times. Ctrl+C to quit.
Stop the panel first (it owns these pins):  sudo systemctl stop aerix-panel
"""
import time

import lgpio
from gpiozero import Button, Device, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory, LGPIOPin
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from PIL import ImageFont

BUTTONS = {"B1": 22, "B2": 27, "B3": 17}   # name -> GPIO (as labelled on the PCB)
BUZZER = 18
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


class RP1Factory(LGPIOFactory):
    """gpiozero ignores LGPIOFactory(chip=...); the Pi 5 header is gpiochip15 here."""
    def __init__(self, chip=15):
        super(LGPIOFactory, self).__init__()
        self._handle = lgpio.gpiochip_open(chip)
        self._chip = chip
        self.pin_class = LGPIOPin


Device.pin_factory = RP1Factory()
oled = sh1106(i2c(port=1, address=0x3C))
oled.contrast(255)            # max (luma default is 207)
buz = PWMOutputDevice(BUZZER, frequency=2700, initial_value=0)
big = ImageFont.truetype(FONT, 40)
small = ImageFont.truetype(FONT, 11)


def show(name, gpio_pin):
    with canvas(oled) as d:
        d.text((30, 2), name, font=big, fill="white")
        d.text((14, 50), f"GPIO{gpio_pin}  pin {PHYS[gpio_pin]}", font=small, fill="white")


def beep(n):
    for _ in range(n):
        buz.value = 0.5; time.sleep(0.12)
        buz.value = 0;   time.sleep(0.12)


PHYS = {17: 11, 27: 13, 22: 15}
buttons = []
for i, (name, pin) in enumerate(BUTTONS.items(), start=1):
    b = Button(pin, pull_up=True, bounce_time=0.05)
    b.when_pressed = lambda n=name, p=pin, k=i: (print(f"{n} pressed  (GPIO{p}, pin {PHYS[p]})", flush=True),
                                                 show(n, p), beep(k))
    buttons.append(b)

with canvas(oled) as d:
    d.text((4, 10), "Press a button", font=small, fill="white")
    d.text((4, 30), "B1=1 beep B2=2 B3=3", font=small, fill="white")
print("Press each button. Ctrl+C to quit.", flush=True)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    buz.value = 0
    oled.clear()
