#!/usr/bin/env python3
"""Benewake TF02-Pro LiDAR Test Script

Reads raw 9-byte frames from a TF02-Pro ToF LiDAR over UART/Serial:
  - Header:      0x59 0x59
  - Distance:    Dist_L + Dist_H * 256 (cm)
  - Strength:    Strength_L + Strength_H * 256 (0-65535, <100 unreliable)
  - Temperature: (Temp_L + Temp_H * 256) / 8 - 256 (°C)
  - Checksum:    (sum(bytes[0:8]) & 0xFF) == bytes[8]

Usage:
  python3 test_tf02_pro.py                     # auto-detects or defaults to /dev/ttyUSB0
  python3 test_tf02_pro.py --port /dev/serial0 # Pi onboard UART
  python3 test_tf02_pro.py --baud 115200
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
import serial


HEADER = b"\x59\x59"
FRAME_SIZE = 9


def find_default_port() -> str:
    """Finds the most likely serial port on Linux/Pi, or falls back to /dev/ttyUSB0."""
    candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + ["/dev/serial0"]
    for port in candidates:
        if os.path.exists(port):
            return port
    return "/dev/ttyUSB0"


def read_tf02_frame(ser: serial.Serial, timeout_sec: float = 1.0) -> dict | None:
    """Synchronizes on 0x59 0x59 header and validates a 9-byte TF02-Pro frame."""
    start_time = time.monotonic()

    while time.monotonic() - start_time < timeout_sec:
        # Search for first header byte 0x59
        b1 = ser.read(1)
        if not b1 or b1 != b"\x59":
            continue

        # Check second header byte 0x59
        b2 = ser.read(1)
        if not b2 or b2 != b"\x59":
            continue

        # Read remaining 7 bytes (data + checksum)
        payload = ser.read(7)
        if len(payload) != 7:
            continue

        frame = HEADER + payload

        # Checksum validation: low 8 bits of sum(frame[0..7]) == frame[8]
        expected_checksum = sum(frame[:8]) & 0xFF
        actual_checksum = frame[8]
        if expected_checksum != actual_checksum:
            continue

        # Extract values
        dist_cm = frame[2] + (frame[3] << 8)
        strength = frame[4] + (frame[5] << 8)
        raw_temp = frame[6] + (frame[7] << 8)
        temp_c = (raw_temp / 8.0) - 256.0

        return {
            "distance_cm": dist_cm,
            "distance_m": dist_cm / 100.0,
            "strength": strength,
            "temperature_c": temp_c,
            "valid": strength >= 100 and dist_cm > 0,
        }

    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Test Benewake TF02-Pro LiDAR via UART")
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port (e.g. /dev/ttyUSB0, /dev/serial0). Default: auto-detect",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw values per line instead of live dashboard",
    )
    args = parser.parse_args()

    port = args.port or find_default_port()
    print("=" * 60)
    print(" Benewake TF02-Pro LiDAR Test")
    print(f" Port: {port}  |  Baud: {args.baud}")
    print("=" * 60)

    try:
        ser = serial.Serial(
            port=port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"\n[ERROR] Failed to open serial port {port}: {e}")
        print("Check:")
        print("  1. Is the sensor connected?")
        print("  2. Do you have permissions? Run: sudo usermod -a -G dialout $USER")
        print("  3. List available ports: ls -l /dev/ttyUSB* /dev/serial0")
        sys.exit(1)

    print("[INFO] Serial connection opened. Reading data (Press Ctrl+C to stop)...\n")

    frame_count = 0
    fps_start = time.monotonic()
    fps = 0.0

    try:
        while True:
            data = read_tf02_frame(ser, timeout_sec=0.5)

            if data is None:
                if args.raw:
                    print("[WARN] No valid frame received (timeout / bad checksum)")
                time.sleep(0.01)
                continue

            frame_count += 1
            now = time.monotonic()
            elapsed = now - fps_start
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_start = now

            dist_m = data["distance_m"]
            dist_cm = data["distance_cm"]
            strength = data["strength"]
            temp_c = data["temperature_c"]
            valid = data["valid"]

            # Visual bar for distance (0 - 20 m range visualization)
            bar_len = min(30, int(dist_m * 1.5))
            bar = "█" * bar_len + "░" * (30 - bar_len)
            status_str = "OK" if valid else ("LOW SIGNAL" if strength < 100 else "OUT OF RANGE")

            if args.raw:
                print(
                    f"Dist: {dist_cm:5d} cm ({dist_m:5.2f} m) | "
                    f"Strength: {strength:5d} | Temp: {temp_c:5.1f} °C | Status: {status_str}"
                )
            else:
                sys.stdout.write(
                    f"\rDist: \033[1;32m{dist_m:6.2f} m\033[0m ({dist_cm:4d} cm) "
                    f"[{bar}] | Strength: {strength:5d} | "
                    f"Temp: {temp_c:4.1f}°C | Rate: {fps:4.1f} Hz | Status: {status_str:<12}"
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\n[INFO] Stopped by user.")
    finally:
        ser.close()
        print("[INFO] Serial port closed.")


if __name__ == "__main__":
    main()
