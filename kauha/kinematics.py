"""Machine kinematics shared by the twin and the autonomy computer."""
from __future__ import annotations

import math

from kauha.config import MachineConfig


def bucket_tip_local(c: MachineConfig, boom: float, bucket: float) -> tuple[float, float]:
    """Cutting-edge position in the front frame: (ahead of front axle, height)."""
    bx = c.boom_pivot_forward + c.boom_length * math.cos(boom)
    bz = c.boom_pivot_height + c.boom_length * math.sin(boom)
    tx = bx + c.bucket_length * math.cos(-0.35 * bucket)
    tz = bz - 0.45 - 0.5 * c.bucket_length * math.sin(bucket)
    return tx, max(0.0, tz)
