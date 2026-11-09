"""Actuator linkages of the 1:8 testbed: lift, tilt and steering.

The full-size loader moves its boom with two lift cylinders, its bucket with a
Z-bar tilt linkage and its frames with two steering cylinders. The testbed uses
electric linear actuators (Progressive Automations PA-14P, potentiometer
feedback) in the same places. This module finds anchor points so that each
actuator's length stays inside its stroke over the machine's whole range of
motion, with the largest smallest moment arm, and writes the result (points,
strokes, length tables) for the CAD model and the gateway firmware.

Why the bucket does not use a direct actuator: the testbed needs about 143 deg of
bucket travel relative to the boom (rolled back while digging to dumped at full
lift). A single actuator acting on a lever gives well under 120 deg with usable
force. The testbed therefore uses the four-bar "side link + H-link" arrangement of
excavator buckets, mounted on the boom, which gives the full range from one stroke.
It keeps the twin's convention of a bucket angle relative to the boom; the
full-size Z-bar couples bucket and boom angles and is not reproduced.

Frames: side view x forward from the front axle, z up from the ground (m);
plan view for steering, origin at the articulation joint.

    python -m kauha.scale.linkage          # search, check and write the design
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass

import numpy as np

from kauha.config import REPO_ROOT, load_machine
from kauha.scale.profile import F, scale_machine

OUT = REPO_ROOT / "hardware" / "scale" / "generated" / "linkage.json"
IN = 0.0254
PA14_DEAD = 5.51 * IN          # PA-14P: retracted length = stroke + 5.51 in, extended = 2 * stroke + 5.51 in
PA14_FORCE_N = {150: 667.0, 100: 445.0, 50: 222.0}    # rated push/pull, lbf -> N
MARGIN = 0.003                 # keep 3 mm off each end stop

M = scale_machine(load_machine())
WHEEL_R = 0.0825               # 6.5 in hub-motor wheel (165 mm); 1:8 of the 0.78 m tyre would be 97.5 mm
P = (M.boom_pivot_forward, M.boom_pivot_height)          # boom pivot
LB = M.boom_length                                         # boom pivot to bucket pivot
BOOM_RANGE = (M.boom_min, M.boom_max)
# bucket angle relative to the boom, rad (twin convention: pitch = -boom + bucket)
BUCKET_REL_RANGE = (-1.55, 0.95)       # dumped at full lift .. rolled back while digging
DIG_REL_RANGE = (0.40, 0.95)           # where breakout torque is needed
GAMMA_RANGE = (-0.70, 0.70)            # articulation, covers the field sectors


def pa14(stroke_in: float) -> tuple[float, float]:
    s = stroke_in * IN
    return s + PA14_DEAD, 2 * s + PA14_DEAD


@dataclass
class Actuator:
    name: str
    stroke_in: float
    force_lbf: int
    base: tuple                  # anchor on the fixed body (frame or boom), m
    rod: tuple                   # rod-end anchor in the moving body at angle 0 (or as stated), m
    angle_range: tuple
    lengths: list                # actuator length at each table angle, m
    angles: list
    min_arm_m: float             # smallest |dL/dangle| over the range = effective moment arm
    note: str = ""

    @property
    def force_n(self) -> float:
        return PA14_FORCE_N[self.force_lbf]


def _arm(lengths: np.ndarray, angles: np.ndarray) -> np.ndarray:
    return np.abs(np.gradient(lengths, np.ravel(angles), axis=-1))


# ------------------------------------------------------------------ lift
def lift_lengths(ax, az, s, n, beta):
    """Lift actuator from frame point (ax, az) to a boom point s along and n normal to the boom."""
    cb, sb = np.cos(beta), np.sin(beta)
    cx = P[0] + s * cb - n * sb
    cz = P[1] + s * sb + n * cb
    return np.hypot(cx - ax, cz - az)


def design_lift(rng: np.random.Generator, n_samples: int = 200_000) -> Actuator:
    beta = np.linspace(*BOOM_RANGE, 41)
    best = None
    for stroke in (2, 3, 4, 6):
        lo, hi = pa14(stroke)
        ax = rng.uniform(-0.15, 0.10, n_samples)[:, None]      # on the front frame, clear of the hinge block
        az = rng.uniform(0.06, 0.30, n_samples)[:, None]      # above the scaled ground clearance
        s = rng.uniform(0.03, 0.30, n_samples)[:, None]
        n = rng.uniform(-0.08, 0.04, n_samples)[:, None]
        L = lift_lengths(ax, az, s, n, beta[None, :])
        ok = (L.min(1) >= lo + MARGIN) & (L.max(1) <= hi - MARGIN)
        # base must stay clear of the front wheel (actuators run inboard, but keep the
        # anchor out of the tyre envelope so the bracket can be welded to the frame)
        ok &= np.hypot(ax[:, 0], az[:, 0] - WHEEL_R) > WHEEL_R + 0.02
        ok &= np.all(np.diff(L, axis=1) > 0, axis=1)        # monotonic: longer = boom up
        if not ok.any():
            continue
        arm = _arm(L, beta[None, :]).min(1)
        arm[~ok] = -1
        i = int(arm.argmax())
        if best is None or arm[i] > best[0]:
            best = (arm[i], stroke, float(ax[i, 0]), float(az[i, 0]), float(s[i, 0]), float(n[i, 0]), L[i])
    if best is None:
        raise RuntimeError("no lift geometry fits the actuator strokes")
    arm, stroke, ax, az, s, n, L = best
    # local refinement of the best sample (Nelder-Mead on the same constraints, as penalties)
    from scipy.optimize import minimize
    lo, hi = pa14(stroke)

    def cost(x):
        L = lift_lengths(x[0], x[1], x[2], x[3], beta)
        pen = max(0, lo + MARGIN - L.min()) + max(0, L.max() - hi + MARGIN) + max(0, 0.06 - x[1]) \
            + max(0, -0.15 - x[0]) + max(0, WHEEL_R + 0.02 - math.hypot(x[0], x[1] - WHEEL_R)) \
            + max(0, -np.diff(L).min())
        return -_arm(L, beta).min() + 100 * pen

    r = minimize(cost, [ax, az, s, n], method="Nelder-Mead", options=dict(xatol=1e-5, fatol=1e-7, maxiter=4000))
    if cost(r.x) < cost([ax, az, s, n]) and cost(r.x) < 0:
        ax, az, s, n = (float(v) for v in r.x)
        L = lift_lengths(ax, az, s, n, beta)
        arm = _arm(L, beta).min()
    return Actuator("lift", stroke, 150, (ax, az), (s, n), BOOM_RANGE, [float(x) for x in L],
                    [float(b) for b in beta], float(arm),
                    "two actuators, one beside each boom arm; rod point given as (along, normal) on the boom")


# ------------------------------------------------------------------ tilt (side link + H-link on the boom)
def _circle_x(c1, r1, c2, r2, branch):
    """Intersection of circles (vectorised); returns points and a validity mask."""
    dx, dz = c2[0] - c1[0], c2[1] - c1[1]
    d = np.hypot(dx, dz)
    ok = (d < r1 + r2) & (d > np.abs(r1 - r2)) & (d > 1e-9)
    a = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * np.where(ok, d, 1))
    h = np.sqrt(np.clip(r1 ** 2 - a ** 2, 0, None))
    mx, mz = c1[0] + a * dx / np.where(ok, d, 1), c1[1] + a * dz / np.where(ok, d, 1)
    ex = mx + branch * h * (-dz) / np.where(ok, d, 1)
    ez = mz + branch * h * dx / np.where(ok, d, 1)
    return (ex, ez), ok


def tilt_solve(p: dict, psi: np.ndarray):
    """Boom frame: u along the boom from its pivot, w normal (up). Returns E, D, F, R, lengths, ok."""
    bu, bw = LB, 0.0
    du = bu + p["a"] * np.cos(psi + p["phi"])
    dw = bw + p["a"] * np.sin(psi + p["phi"])
    R = (bu - p["g"], p["h"])
    E, ok = _circle_x(R, p["b"], (du, dw), p["c"], p["branch"])
    Fp = (p["fu"], p["fw"])
    L = np.hypot(E[0] - Fp[0], E[1] - Fp[1])
    # transmission angles: coupler vs bucket lever at D, actuator vs side link at E
    v1 = np.stack([E[0] - du, E[1] - dw])
    v2 = np.stack([bu - du, bw - dw])
    cos_d = (v1 * v2).sum(0) / (np.linalg.norm(v1, axis=0) * np.linalg.norm(v2, axis=0) + 1e-12)
    w1 = np.stack([E[0] - Fp[0], E[1] - Fp[1]])
    w2 = np.stack([E[0] - R[0], E[1] - R[1]])
    cos_e = (w1 * w2).sum(0) / (np.linalg.norm(w1, axis=0) * np.linalg.norm(w2, axis=0) + 1e-12)
    return E, (du, dw), Fp, R, L, ok, np.degrees(np.arccos(np.clip(cos_d, -1, 1))), \
        np.degrees(np.arccos(np.clip(cos_e, -1, 1)))


def design_tilt(rng: np.random.Generator, n_samples: int = 300_000) -> tuple[Actuator, dict]:
    psi = np.linspace(*BUCKET_REL_RANGE, 51)
    dig = (psi >= DIG_REL_RANGE[0]) & (psi <= DIG_REL_RANGE[1])
    best = None
    for stroke in (4, 6):
        lo, hi = pa14(stroke)
        for branch in (1, -1):
            k = n_samples
            p = dict(a=rng.uniform(0.035, 0.07, k), phi=rng.uniform(1.75, 2.8, k), g=rng.uniform(0.03, 0.09, k),
                     h=rng.uniform(0.0, 0.05, k), b=rng.uniform(0.03, 0.08, k), c=rng.uniform(0.03, 0.09, k),
                     fu=rng.uniform(0.02, 0.15, k), fw=rng.uniform(0.0, 0.05, k), branch=branch)
            pc = {kk: (v[:, None] if isinstance(v, np.ndarray) else v) for kk, v in p.items()}
            E, D, Fp, R, L, ok, td, te = tilt_solve(pc, psi[None, :])
            good = ok.all(1) & (L.min(1) >= lo + MARGIN) & (L.max(1) <= hi - MARGIN)
            good &= (td.min(1) > 25) & (td.max(1) < 155) & (te.min(1) > 20) & (te.max(1) < 160)
            dL = np.diff(L, axis=1)
            good &= np.all(dL > 0, axis=1) | np.all(dL < 0, axis=1)
            if not good.any():
                continue
            arm = _arm(L, psi[None, :])
            score = np.where(good, np.minimum(arm[:, dig].min(1), 4 * arm.min(1)), -1)
            i = int(score.argmax())
            if best is None or score[i] > best[0]:
                sel = {kk: (float(v[i]) if isinstance(v, np.ndarray) else v) for kk, v in p.items()}
                best = (float(score[i]), stroke, sel, L[i], float(arm[i].min()), float(arm[i][dig].min()))
    if best is None:
        raise RuntimeError("no tilt linkage fits the actuator strokes")
    _, stroke, sel, L, arm_min, arm_dig = best
    act = Actuator("tilt", stroke, 150, (sel["fu"], sel["fw"]), (LB - sel["g"], sel["h"]), BUCKET_REL_RANGE,
                   [float(x) for x in L], [float(x) for x in psi], arm_min,
                   f"on the boom; side link pivot R, H-link to the bucket; moment arm in the dig range "
                   f"{arm_dig * 1000:.1f} mm")
    return act, sel


# ------------------------------------------------------------------ steering (plan view, origin at the joint)
def steer_lengths(fx, fy, rx, ry, gamma):
    """Actuator from front-frame point (fx, fy) to rear-frame point (rx, ry), rear frame turned by -gamma."""
    c, s = np.cos(-gamma), np.sin(-gamma)
    qx, qy = rx * c - ry * s, rx * s + ry * c
    return np.hypot(qx - fx, qy - fy)


def design_steer(rng: np.random.Generator, n_samples: int = 200_000) -> Actuator:
    gam = np.linspace(*GAMMA_RANGE, 41)
    best = None
    for stroke in (2, 3, 4, 6):
        lo, hi = pa14(stroke)
        fx = rng.uniform(0.03, 0.19, n_samples)[:, None]
        fy = rng.uniform(0.075, 0.09, n_samples)[:, None]      # outboard of the 60 mm frame plates
        rx = rng.uniform(-0.25, -0.04, n_samples)[:, None]
        ry = rng.uniform(0.075, 0.09, n_samples)[:, None]
        L = steer_lengths(fx, fy, rx, ry, gam[None, :])
        ok = (L.min(1) >= lo + MARGIN) & (L.max(1) <= hi - MARGIN)
        dL = np.diff(L, axis=1)
        ok &= np.all(dL > 0, axis=1) | np.all(dL < 0, axis=1)
        if not ok.any():
            continue
        arm = _arm(L, gam[None, :]).min(1)
        arm[~ok] = -1
        i = int(arm.argmax())
        if best is None or arm[i] > best[0]:
            best = (arm[i], stroke, float(fx[i, 0]), float(fy[i, 0]), float(rx[i, 0]), float(ry[i, 0]), L[i])
    if best is None:
        raise RuntimeError("no steering geometry fits the actuator strokes")
    arm, stroke, fx, fy, rx, ry, L = best
    return Actuator("steer", stroke, 150, (fx, fy), (rx, ry), GAMMA_RANGE, [float(x) for x in L],
                    [float(g) for g in gam], float(arm),
                    "plan view, origin at the articulation joint; two actuators, mirrored left and right "
                    "(as the two steering cylinders of the full machine); table for the left one")


# ------------------------------------------------------------------ loads
def load_checks(lift: Actuator, tilt: Actuator, steer: Actuator) -> dict:
    """Static force checks against the Froude-scaled loads (see docs/scale/design_basis.md)."""
    g = 9.81
    payload = M.bucket_volume * 1600.0                     # sand, kg
    bucket_mass = 1.2                                      # bent 1.5 mm steel shell, kg (CAD gives the exact value)
    boom_mass = 1.0
    reach = LB + 0.08                                      # load centre ahead of the boom pivot, horizontal boom
    lift_torque = (payload + bucket_mass) * g * reach + boom_mass * g * LB / 2
    tilt_torque = (payload + bucket_mass) * g * 0.08
    breakout_torque = F.torque(140_000 * 1.3)              # full-size breakout x bucket length, scaled
    steer_torque = F.torque(60_000)                        # full-size standstill steering moment (assumed), scaled
    return {
        "payload_kg": payload,
        "lift_torque_needed_Nm": lift_torque, "lift_torque_available_Nm": 2 * lift.force_n * lift.min_arm_m,
        "tilt_hold_torque_needed_Nm": tilt_torque, "tilt_torque_available_Nm": tilt.force_n * tilt.min_arm_m,
        "breakout_torque_froude_Nm": breakout_torque,
        "tilt_torque_dig_range_Nm": tilt.force_n * float(tilt.note.split("range ")[1].split(" mm")[0]) / 1000,
        "steer_torque_needed_Nm": steer_torque, "steer_torque_available_Nm": 2 * steer.force_n * steer.min_arm_m,
    }


def design(seed: int = 8) -> dict:
    rng = np.random.default_rng(seed)
    lift = design_lift(rng)
    tilt, tilt_params = design_tilt(rng)
    steer = design_steer(rng)
    loads = load_checks(lift, tilt, steer)
    return {"seed": seed, "boom_pivot": P, "boom_length": LB, "wheel_radius": WHEEL_R,
            "actuators": {a.name: {**asdict(a), "force_n": a.force_n,
                                   "length_range_m": pa14(a.stroke_in)} for a in (lift, tilt, steer)},
            "tilt_linkage": tilt_params, "loads": loads}


def check(d: dict) -> list[str]:
    out = []
    for name, a in d["actuators"].items():
        lo, hi = a["length_range_m"]
        if min(a["lengths"]) < lo or max(a["lengths"]) > hi:
            out.append(f"{name}: length outside the stroke")
    ld = d["loads"]
    if ld["lift_torque_available_Nm"] < 1.3 * ld["lift_torque_needed_Nm"]:
        out.append("lift: less than 1.3 x the full-bucket torque")
    if ld["tilt_torque_dig_range_Nm"] < 1.5 * ld["tilt_hold_torque_needed_Nm"]:
        out.append("tilt: cannot hold a full bucket with margin in the dig and carry range")
    if ld["tilt_torque_available_Nm"] < 2.0 * 1.2 * 9.81 * 0.08:
        out.append("tilt: cannot move the empty bucket at the dump end")
    if ld["steer_torque_available_Nm"] < ld["steer_torque_needed_Nm"]:
        out.append("steer: below the scaled standstill steering moment")
    return out


def main() -> None:
    d = design()
    problems = check(d)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({**d, "checks": problems or "pass"}, indent=1))
    for name, a in d["actuators"].items():
        print(f"{name:6s} PA-14P {a['stroke_in']}\" {a['force_lbf']} lbf  L {min(a['lengths'])*1000:.0f}-"
              f"{max(a['lengths'])*1000:.0f} mm of {a['length_range_m'][0]*1000:.0f}-{a['length_range_m'][1]*1000:.0f}"
              f"  min arm {a['min_arm_m']*1000:.1f} mm")
    for k, v in d["loads"].items():
        print(f"  {k:30s} {v:8.2f}")
    print("checks:", problems or "pass")
    if problems:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

