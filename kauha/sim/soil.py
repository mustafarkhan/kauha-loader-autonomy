"""Bucket-pile interaction for bucket filling.

A reduced model in the spirit of the Fundamental Equation of Earthmoving (FEE),
as used to train excavation policies in simulation (e.g. Egli et al., ETH,
"Reinforcement learning-based bucket filling for autonomous excavation").
It is deliberately simple and fast so that policies can be trained on a CPU
with heavy domain randomisation:

  * the pile face rises at the material's angle of repose from the toe;
  * penetration resistance grows with the square of cut depth (passive
    soil wedge), with material stiffness, and with bucket fill;
  * material enters the bucket from forward crowding, boom lift through
    the face, and bucket curl;
  * bucket capacity depends on bucket attitude; excess spills.

This is not a validated soil model. Its job is to make the trade-offs a real
operator faces (stall vs. shallow cut vs. lift timing) present in training.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.81
N_GAMMA = 3.5          # passive wedge factor (dimensionless, order of magnitude)
COHESION_TERM = 4000.0  # N/m per m depth, lumps cohesion + surcharge
BUCKET_MOUTH_M = 1.0   # vertical opening of the bucket


@dataclass
class Pile:
    material: str
    density_t_m3: float
    k_resist: float
    repose_deg: float
    height_m: float = 3.0

    def surface_height(self, penetration_m: float) -> float:
        if penetration_m <= 0:
            return 0.0
        return min(self.height_m, penetration_m * math.tan(math.radians(self.repose_deg)))


def bucket_capacity(rated_m3: float, attitude_rad: float) -> float:
    """Volume a bucket holds at a given attitude (heaped when racked)."""
    frac = min(1.0, max(0.0, (attitude_rad + 0.1) / 0.7))
    return rated_m3 * (0.45 + 0.70 * frac)   # 0.45 V flat ... 1.15 V racked


@dataclass
class DigResult:
    resist_force_n: float
    normal_boost_n: float
    cut_depth_m: float
    added_m3: float
    spilled_m3: float


class BucketPileInteraction:
    """Tracks penetration of the bucket into one pile face."""

    def __init__(self, pile: Pile, bucket_width_m: float, rated_volume_m3: float):
        self.pile = pile
        self.w = bucket_width_m
        self.v_rated = rated_volume_m3
        self.penetration = 0.0
        self.implement_load_n = 0.0

    def step(self, *, dx_forward: float, tip_z: float, tip_dz: float,
             attitude: float, attitude_rate: float, load_m3: float, dt: float) -> DigResult:
        p = self.pile
        self.penetration = max(0.0, self.penetration + dx_forward)
        surface = p.surface_height(self.penetration)
        depth = max(0.0, surface - tip_z)
        rho = p.density_t_m3 * 1000.0

        fill_ratio = min(1.5, load_m3 / self.v_rated)
        # Edge digs best near flat; a steep attitude ploughs.
        attitude_factor = 1.0 + 0.8 * abs(min(attitude, 0.6) - 0.05)
        resist = 0.0
        if self.penetration > 0 and depth > 0:
            resist = p.k_resist * self.w * (0.5 * rho * G * depth ** 2 * N_GAMMA
                                            + COHESION_TERM * depth)
            resist *= attitude_factor * (1.0 + 1.2 * fill_ratio)

        # Material inflow. `inside` is how far the cutting edge sits behind the
        # pile face at its current height (limited by bucket floor length).
        d_eff = min(depth, BUCKET_MOUTH_M)
        tan_r = math.tan(math.radians(p.repose_deg))
        inside = min(1.2, max(0.0, self.penetration - tip_z / tan_r))
        flow = 1.0 / (0.6 + 0.4 * p.k_resist)            # stiff material flows less readily
        crowd = max(dx_forward, 0.0) * d_eff * self.w * 0.6 * flow
        lift = max(tip_dz, 0.0) * dt * self.w * inside * 0.9 * flow if depth > 0 else 0.0
        curl = max(attitude_rate, 0.0) * dt * inside * self.w * 0.8 * flow if depth > 0 else 0.0
        room = max(0.0, 1.0 - load_m3 / (1.2 * self.v_rated))
        added = (crowd + lift + curl) * room
        # Vertical load on the lift cylinders while lifting through material.
        self.implement_load_n = 0.55 * resist if tip_dz > 0 else 0.25 * resist

        # Lifting in the pile loads the front axle (raises traction limit).
        normal_boost = 0.35 * resist if tip_dz > 0 else 0.0

        new_load = load_m3 + added
        cap = bucket_capacity(self.v_rated, attitude)
        # Inside the face the pile supports a heap above the bucket's own
        # capacity; tipping the bucket forward still pushes material out.
        support = 0.0 if depth < 0.05 else 0.25 * self.v_rated
        spilled = max(0.0, new_load - cap - support)
        return DigResult(resist, normal_boost, depth, added, spilled)
