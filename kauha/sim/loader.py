"""Digital twin of an articulated wheel loader.

Planar articulated-frame kinematics with actuator latency, rate limits and a
traction limit, plus a side-view boom/bucket model. Reference point is the
centre of the front axle. Heading `theta` is the front-frame heading;
`gamma` is the articulation angle (front relative to rear, positive = left).

Kinematics follow the standard articulated-vehicle model:
    theta_dot = (v * sin(gamma) + lr * gamma_dot) / (lf * cos(gamma) + lr)
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from kauha.config import MachineConfig
from kauha.kinematics import bucket_tip_local

G = 9.81
DIESEL_L_PER_KWH = 0.27   # ~230 g/kWh specific consumption, 0.84 kg/L
IDLE_POWER_KW = 12.0


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class LoaderCommand:
    """Actuator-level command, as decoded by the drive-by-wire gateway."""
    speed: float = 0.0          # requested front-axle speed, m/s (signed)
    articulation: float = 0.0   # requested articulation angle, rad
    boom: float = 0.0           # boom rate request, -1..1 of max rate
    bucket: float = 0.0         # bucket rate request, -1..1 (+ = roll back)
    brake: bool = True          # service brake request


@dataclass
class LoaderState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    gamma: float = 0.0
    v: float = 0.0
    boom: float = -0.62
    bucket: float = 0.0
    load_m3: float = 0.0
    load_t: float = 0.0
    stalled: bool = False
    fuel_l: float = 0.0
    odometer_m: float = 0.0


@dataclass
class Loader:
    cfg: MachineConfig
    state: LoaderState = field(default_factory=LoaderState)
    dt: float = 0.05

    def __post_init__(self) -> None:
        n = max(1, round(self.cfg.actuator_latency / self.dt))
        self._pipe: deque[LoaderCommand] = deque([LoaderCommand()] * n, maxlen=n)
        self.state.boom = self.cfg.boom_min

    # ------------------------------------------------------------------ geometry
    def rear_axle(self) -> tuple[float, float, float]:
        s, c = self.state, self.cfg
        jx = s.x - c.lf * math.cos(s.theta)
        jy = s.y - c.lf * math.sin(s.theta)
        th_r = s.theta - s.gamma
        return jx - c.lr * math.cos(th_r), jy - c.lr * math.sin(th_r), th_r

    def bucket_tip_local(self) -> tuple[float, float]:
        """Cutting-edge position in the front frame: (forward of front axle, height)."""
        return bucket_tip_local(self.cfg, self.state.boom, self.state.bucket)

    def bucket_tip_world(self) -> tuple[float, float, float]:
        fx, z = self.bucket_tip_local()
        s = self.state
        return s.x + fx * math.cos(s.theta), s.y + fx * math.sin(s.theta), z

    def footprint(self) -> list[tuple[float, float]]:
        """Approximate outline as two rectangles (front and rear frame)."""
        s, c = self.state, self.cfg
        w = c.body_width / 2
        pts = []
        tx, _ = self.bucket_tip_local()
        for (ox, oy, th, back, fwd) in (
            (s.x, s.y, s.theta, -c.lf, tx),
            (*self.rear_axle()[:2], self.rear_axle()[2], -1.6, c.lr),
        ):
            ct, st = math.cos(th), math.sin(th)
            for lx, ly in ((fwd, w), (fwd, -w), (back, -w), (back, w)):
                pts.append((ox + lx * ct - ly * st, oy + lx * st + ly * ct))
        return pts

    def bucket_bottom_height(self) -> float:
        c, s = self.cfg, self.state
        return c.boom_pivot_height + c.boom_length * math.sin(s.boom) - 1.0

    def body_polygons(self) -> tuple[list, list | None]:
        """Ground-level body outline (rear frame, front frame, low bucket) and the
        plan-view outline of the bucket when it is raised above head height."""
        s, c = self.state, self.cfg
        w = c.body_width / 2
        tx, _ = self.bucket_tip_local()

        def rect(ox, oy, th, back, fwd, hw):
            ct, st = math.cos(th), math.sin(th)
            return [(ox + lx * ct - ly * st, oy + lx * st + ly * ct)
                    for lx, ly in ((fwd, hw), (fwd, -hw), (back, -hw), (back, hw))]

        rx, ry, rth = self.rear_axle()
        polys = [rect(rx, ry, rth, -1.6, c.lr, w), rect(s.x, s.y, s.theta, -c.lf, 1.9, w)]
        bucket = rect(s.x, s.y, s.theta, tx - c.bucket_length, tx, c.bucket_width / 2)
        if self.bucket_bottom_height() < 2.0:
            polys.append(bucket)
            return polys, None
        return polys, bucket

    # ------------------------------------------------------------------ dynamics
    def step(self, cmd: LoaderCommand, resist_force_n: float = 0.0,
             normal_boost_n: float = 0.0, implement_load_n: float = 0.0) -> None:
        """Advance one time step.

        resist_force_n: force opposing forward motion (from soil while digging).
        normal_boost_n: extra normal load on the front axle (boom lifting in pile),
                        which raises the traction limit.
        """
        c, s, dt = self.cfg, self.state, self.dt
        self._pipe.append(cmd)
        act = self._pipe[0]

        # Articulation: rate-limited servo toward target.
        g_target = clamp(act.articulation, -c.max_articulation, c.max_articulation)
        g_dot = clamp((g_target - s.gamma) / dt, -c.articulation_rate, c.articulation_rate)
        if abs(s.v) < 0.02:
            g_dot *= 0.6   # steering a stationary loader is slower
        s.gamma = clamp(s.gamma + g_dot * dt, -c.max_articulation, c.max_articulation)

        # Longitudinal: traction-limited acceleration.
        mass = c.operating_mass_kg + s.load_t * 1000.0
        traction_max = c.traction_coefficient * (mass * G + normal_boost_n)
        if act.brake:
            a = -math.copysign(min(c.brake_decel, abs(s.v) / dt), s.v) if s.v else 0.0
            s.stalled = False
        else:
            v_target = clamp(act.speed, -c.max_speed, c.max_speed)
            a_wanted = clamp((v_target - s.v) / max(dt, 0.25), -c.max_accel, c.max_accel)
            moving_fwd = s.v > 0.01 or v_target > 0
            f_res = resist_force_n if moving_fwd else 0.0
            # The drivetrain supplies whatever force tracks the speed request,
            # up to the tyre traction limit.
            f_drive = clamp(mass * a_wanted + f_res, -traction_max, traction_max)
            s.stalled = moving_fwd and v_target > 0.05 and f_res >= traction_max * 0.98
            a = (f_drive - f_res) / mass
            if f_res > 0 and s.v + a * dt < 0 and v_target >= 0:
                a = -s.v / dt  # resistance can stop, not reverse, the machine
        s.v += a * dt

        # Front-frame kinematics.
        denom = c.lf * math.cos(s.gamma) + c.lr
        th_dot = (s.v * math.sin(s.gamma) + c.lr * g_dot) / denom
        s.theta = wrap_angle(s.theta + th_dot * dt)
        s.x += s.v * math.cos(s.theta) * dt
        s.y += s.v * math.sin(s.theta) * dt
        s.odometer_m += abs(s.v) * dt

        # Boom and bucket.
        # Lift and tilt slow down as the implement load approaches breakout force.
        payload_n = s.load_t * 1000.0 * G
        lift_margin = clamp(1.0 - (implement_load_n + 0.5 * payload_n) / c.breakout_force, 0.1, 1.0)
        boom_req = clamp(act.boom, -1, 1)
        boom_rate = boom_req * c.boom_rate * (lift_margin if boom_req > 0 else 1.0)
        tilt_req = clamp(act.bucket, -1, 1)
        tilt_rate = tilt_req * c.bucket_rate * (max(0.25, lift_margin) if tilt_req > 0 else 1.0)
        s.boom = clamp(s.boom + boom_rate * dt, c.boom_min, c.boom_max)
        s.bucket = clamp(s.bucket + tilt_rate * dt, c.bucket_min, c.bucket_max)

        # Energy proxy: traction + hydraulics + idle.
        p_trac = abs((mass * a + resist_force_n) * s.v) / 0.75
        p_hyd = (abs(act.boom) * 55e3 * (1 + s.load_t / 5) + abs(act.bucket) * 30e3) / 0.8
        p_kw = IDLE_POWER_KW + (p_trac + p_hyd) / 1000.0
        s.fuel_l += p_kw * DIESEL_L_PER_KWH * dt / 3600.0

    def speed_abs(self) -> float:
        return abs(self.state.v)

    def stopping_distance(self, reaction_s: float) -> float:
        v = abs(self.state.v)
        return v * (reaction_s + self.cfg.actuator_latency) + v * v / (2 * self.cfg.brake_decel)
