"""MockRangefinder - a simulated downward TF02-Pro backed by the SimWorld.

The sim ground is flat at home elevation, so height above ground is the
vehicle's altitude above home plus the sensor's mounting height. Readings go
through the same validity limits as the real parser, so a sim aircraft sitting
on the pad reports what the real one does (valid, ~mount height) and one above
max_range_m reports invalid, like the real sensor's 45 m "no target" code.
"""
from __future__ import annotations

import random
from typing import Any

from drone_stack.interfaces.rangefinder_interface import RangefinderInterface
from drone_stack.msg import RangeReading
from drone_stack.sim.world import SimWorld


class MockRangefinder(RangefinderInterface):
    def __init__(self, world: SimWorld, config: dict[str, Any]) -> None:
        self._world = world
        self._mount = float(config.get("mount_height_m", 0.15))
        self._min = float(config.get("min_range_m", 0.1))
        self._max = float(config.get("max_range_m", 22.0))
        self._noise = float(config.get("sim_noise_m", 0.02))
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        self._connected = True
        return True

    def close(self) -> None:
        self._connected = False

    def read(self) -> RangeReading | None:
        if not self._connected:
            return None
        d = max(0.0, self._world.state.z) + self._mount + random.gauss(0.0, self._noise)
        valid = self._min <= d <= self._max
        return RangeReading(distance_m=round(d, 2), strength=1500 if valid else 40,
                            temperature_c=30.0, valid=valid, sensor="sim")
