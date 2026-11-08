"""Downward rangefinder - Benewake TF02-Pro over the Pi's UART (2026-09-26).

What it is for, and what it is not
----------------------------------
The TF02-Pro points straight DOWN and gives height above ground. It exists for
one job: the precision drop, where the navigator brings the aircraft down to
~1 m above the ground next to the recipient before releasing the parcel. The
barometer cannot do that - its altitude is relative to HOME, not to the ground
under the drop point, and it drifts by about a metre over a flight.

It is not an obstacle sensor. The RPLIDAR C1 is the avoidance LiDAR and nothing
here reads or changes it.

Wiring on this airframe: TF02 TX -> GPIO15 (RXD0), TF02 RX -> GPIO14 (TXD0),
5 V, GND. ``dtparam=uart0=on`` puts that UART on ``/dev/ttyAMA0`` on the Pi 5
(``/dev/serial0`` is the debug header's ttyAMA10 - not this one).

Frame (9 bytes, 100 Hz by default)::

    0x59 0x59 Dist_L Dist_H Str_L Str_H Temp_L Temp_H Checksum
    distance  = Dist_L | Dist_H << 8          (cm)
    strength  = Str_L  | Str_H  << 8
    temp_c    = (Temp_L | Temp_H << 8) / 8 - 256
    checksum  = sum(bytes[0:8]) & 0xFF

Measured on the bench, 2026-09-26: the sensor streams ~100 frames/s and, with
nothing in range, reports ``4500`` cm at strength ~40. That is the TF02-Pro's
"no valid target" code, NOT a 45 m reading - which is why validity is judged
here, once, instead of by every consumer.
"""
from __future__ import annotations

import abc
import struct
from typing import Any

from drone_stack.msg import RangeReading
from drone_stack.utils.logging_setup import get_logger

HEADER = 0x59
FRAME_LEN = 9

#: The TF02-Pro's distance code for "no target / signal too weak" (cm).
NO_TARGET_CM = 4500
#: Strength the TF02-Pro reports when the receiver is saturated.
SATURATED = 65535


def checksum_ok(frame: bytes | bytearray) -> bool:
    return len(frame) == FRAME_LEN and (sum(frame[:8]) & 0xFF) == frame[8]


def parse_frame(
    frame: bytes | bytearray,
    min_strength: int = 100,
    min_range_m: float = 0.1,
    max_range_m: float = 22.0,
) -> RangeReading | None:
    """One 9-byte frame -> RangeReading, or None if it is not a frame at all.

    A frame that parses but that the sensor flags as unusable comes back with
    ``valid=False`` rather than None: "the sensor is talking but sees nothing
    it trusts" and "the sensor is not talking" are different faults and the
    GCS shows them differently.
    """
    if len(frame) != FRAME_LEN or frame[0] != HEADER or frame[1] != HEADER:
        return None
    if not checksum_ok(frame):
        return None
    dist_cm = frame[2] | (frame[3] << 8)
    strength = frame[4] | (frame[5] << 8)
    temp_c = (frame[6] | (frame[7] << 8)) / 8.0 - 256.0
    dist_m = dist_cm / 100.0
    valid = (
        min_strength <= strength < SATURATED
        and 0 < dist_cm < NO_TARGET_CM
        and min_range_m <= dist_m <= max_range_m
    )
    return RangeReading(
        distance_m=dist_m, strength=strength,
        temperature_c=round(temp_c, 1), valid=valid,
    )


class FrameParser:
    """Byte stream -> frames. Resynchronises on the 0x59 0x59 header.

    A UART opened mid-stream starts part-way through a frame, and a byte can be
    lost at any time; both are handled by dropping one byte and looking for the
    next header whenever a candidate frame fails its checksum. Distance bytes
    can legitimately be 0x59 (89 cm), so the checksum - not the header alone -
    is what decides where a frame starts.
    """

    def __init__(self, **limits: Any) -> None:
        self._buf = bytearray()
        self._limits = limits
        self.bad_frames = 0

    def feed(self, data: bytes) -> list[RangeReading]:
        self._buf.extend(data)
        out: list[RangeReading] = []
        buf = self._buf
        i = 0
        while len(buf) - i >= FRAME_LEN:
            if buf[i] != HEADER or buf[i + 1] != HEADER:
                i += 1
                continue
            reading = parse_frame(bytes(buf[i:i + FRAME_LEN]), **self._limits)
            if reading is None:
                self.bad_frames += 1
                i += 1
                continue
            out.append(reading)
            i += FRAME_LEN
        del buf[:i]
        if len(buf) > 4 * FRAME_LEN:          # never grow without bound
            del buf[:-FRAME_LEN]
        return out


class RangefinderInterface(abc.ABC):
    """Abstract downward rangefinder. Must survive unplug/replug via connect()."""

    @property
    @abc.abstractmethod
    def connected(self) -> bool:
        ...

    @abc.abstractmethod
    def connect(self) -> bool:
        """Attempt to (re)open the sensor. Never raises."""

    @abc.abstractmethod
    def close(self) -> None:
        ...

    @abc.abstractmethod
    def read(self) -> RangeReading | None:
        """Newest reading since the last call, or None if nothing arrived."""

    def link_stats(self) -> dict[str, int]:
        """Cumulative wire counters (bytes, bad checksums, UART errors).

        Empty when the link has nothing below the frame level to report (the
        sim mock). The node differences these per second to tell a SILENT
        sensor (power, TX wire) from a GARBLED one (noise, ground, a loose
        pin) - "no frames" alone cannot.
        """
        return {}


class RealTF02(RangefinderInterface):
    """TF02-Pro on a serial port (default /dev/ttyAMA0, 115200 8N1)."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.log = get_logger("rangefinder")
        self._port = str(config.get("port", "/dev/ttyAMA0"))
        self._baud = int(config.get("baud", 115200))
        self._parser = FrameParser(
            min_strength=int(config.get("min_strength", 100)),
            min_range_m=float(config.get("min_range_m", 0.1)),
            max_range_m=float(config.get("max_range_m", 22.0)),
        )
        self._ser = None
        self._rx_bytes = 0

    @property
    def connected(self) -> bool:
        return self._ser is not None

    def connect(self) -> bool:
        self.close()
        try:
            import serial  # pyserial; already a dependency of the RPLIDAR driver

            self._ser = serial.Serial(self._port, self._baud, timeout=0)
            self._ser.reset_input_buffer()
            return True
        except Exception as exc:                 # noqa: BLE001 - never raise
            self.log.warning("TF02-Pro open failed on %s: %s", self._port, exc)
            self._ser = None
            return False

    def close(self) -> None:
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:                    # noqa: BLE001
                pass

    def read(self) -> RangeReading | None:
        if self._ser is None:
            return None
        try:
            n = self._ser.in_waiting
            data = self._ser.read(n) if n else b""
        except Exception as exc:                 # noqa: BLE001 - unplugged
            self.log.warning("TF02-Pro read failed: %s", exc)
            self.close()
            return None
        if not data:
            return None
        self._rx_bytes += len(data)
        readings = self._parser.feed(data)
        return readings[-1] if readings else None

    @property
    def bad_frames(self) -> int:
        return self._parser.bad_frames

    def link_stats(self) -> dict[str, int]:
        out = {"rx_bytes": self._rx_bytes, "bad_frames": self._parser.bad_frames}
        ser = self._ser
        if ser is not None:
            try:
                out.update(uart_icount(ser.fileno()))
            except Exception:                    # noqa: BLE001 - not a tty / not Linux
                pass
        return out


#: Linux TIOCGICOUNT - the per-port error counters /proc/tty/driver/ttyAMA
#: shows, but readable by the unprivileged fd owner (that file is root-only).
TIOCGICOUNT = 0x545D
#: struct serial_icounter_struct: cts dsr rng dcd rx tx frame overrun parity
#: brk buf_overrun reserved[9] - twenty ints.
_ICOUNT = struct.Struct("20i")


def uart_icount(fd: int) -> dict[str, int]:
    """Kernel UART counters for ``fd``: framing / overrun / parity / break.

    A framing error is the UART sampling a stop bit that is not there - a
    corrupted byte on the WIRE (noise coupled from the ESCs, a floating
    ground, a marginal connector). An overrun is the Pi not draining the FIFO
    in time. Neither is visible above the checksum, which is why they are read
    here.
    """
    import fcntl

    raw = fcntl.ioctl(fd, TIOCGICOUNT, bytes(_ICOUNT.size))
    v = _ICOUNT.unpack(raw)
    return {"uart_frame": v[6], "uart_overrun": v[7] + v[10],
            "uart_parity": v[8], "uart_brk": v[9]}
