"""Stopping-distance budget for the protective-field design.

    d_stop = v * t_react + v^2 / (2 a) + margin

t_react = scanner response + PLC cycles + brake actuator latency + brake
pressure build-up. The protective field must be at least d_stop long at the
top speed of its speed band. With a 4 m protective-field range (typical of
outdoor safety scanners; confirm for the selected part) the scanner range,
not the drivetrain, caps the autonomous speed. That cap falls further on a
low-friction winter surface.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class StopBudget:
    scanner_response_s: float = 0.10
    plc_cycle_s: float = 0.025
    plc_cycles: int = 2
    actuator_latency_s: float = 0.12
    brake_buildup_s: float = 0.20
    decel_m_s2: float = 2.5
    margin_m: float = 0.5

    @property
    def reaction_s(self) -> float:
        return (self.scanner_response_s + self.plc_cycle_s * self.plc_cycles
                + self.actuator_latency_s + self.brake_buildup_s)

    def winter(self, decel_m_s2: float = 1.5) -> "StopBudget":
        return replace(self, decel_m_s2=decel_m_s2)


def stopping_distance(v: float, b: StopBudget) -> float:
    v = abs(v)
    return v * b.reaction_s + v * v / (2 * b.decel_m_s2) + b.margin_m


def max_speed_for_field(length_m: float, b: StopBudget) -> float:
    """Largest speed whose stopping distance fits in a field of given length."""
    room = length_m - b.margin_m
    if room <= 0:
        return 0.0
    a2 = 1.0 / (2 * b.decel_m_s2)
    t = b.reaction_s
    return (-t + math.sqrt(t * t + 4 * a2 * room)) / (2 * a2)
