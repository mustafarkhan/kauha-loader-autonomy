"""Sensor models for the digital twin.

Fidelity is chosen per sensor by what the test needs:

  * Safety scanners: real 2D ray casting against people, the truck, stockpile
    faces and walls, with dust returns, so protective-field behaviour, false
    stops and stop distances are measured, not assumed. Each scanner
    evaluates the field set the PLC selected (as the real devices do) and
    needs a hit on >= 2 adjacent beams in 2 consecutive scans (object-size
    and multiple-sampling filters).
  * 3D lidar: object level (landmarks, pile face, truck pose, people) with
    noise, range limits and dust dropout. Point clouds are not simulated.
  * Camera person detector: detection probability and false-positive rate
    taken from the detector's measured test-set performance.
  * GNSS/IMU/odometry: noise, bias, wheel slip and outages.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kauha.geometry import points_in_polygon  # noqa: F401  (re-exported)
from kauha.safety.fields import FieldSet


# ----------------------------------------------------------------- ray casting
def raycast(ox: float, oy: float, angles: np.ndarray, max_range: float,
            segments: np.ndarray, circles: np.ndarray) -> np.ndarray:
    """Ranges along each ray. segments: (M,4) x0,y0,x1,y1. circles: (C,3) x,y,r."""
    dx, dy = np.cos(angles), np.sin(angles)
    rng = np.full(angles.shape, max_range)
    if len(segments):
        sx, sy = segments[:, 0], segments[:, 1]
        ex, ey = segments[:, 2] - sx, segments[:, 3] - sy
        den = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
        qx, qy = sx[None, :] - ox, sy[None, :] - oy
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qx * ey[None, :] - qy * ex[None, :]) / den
            u = (qx * dy[:, None] - qy * dx[:, None]) / den
        ok = (np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
        t = np.where(ok, t, np.inf)
        rng = np.minimum(rng, t.min(axis=1))
    if len(circles):
        cx, cy, r = circles[:, 0] - ox, circles[:, 1] - oy, circles[:, 2]
        b = dx[:, None] * cx[None, :] + dy[:, None] * cy[None, :]
        c = cx ** 2 + cy ** 2 - r ** 2
        disc = b ** 2 - c[None, :]
        with np.errstate(invalid="ignore"):
            t = b - np.sqrt(disc)
        t = np.where((disc >= 0) & (t > 0), t, np.inf)
        rng = np.minimum(rng, t.min(axis=1))
    return rng


@dataclass
class ScannerMount:
    name: str
    body: str            # "front" or "rear"
    x: float             # in body frame (rear frame: x points backwards)
    y: float
    heading: float       # in body frame
    fov: float = math.radians(275)
    resolution: float = math.radians(1.0)
    max_range: float = 15.0


# Four outdoor safety scanners (275 deg field of view), one at each corner,
# turned 45 deg outward so each also covers its side of the frame it is on.
FRONT_MOUNTS = (ScannerMount("front_left", "front", 2.0, 1.35, math.radians(45)),
                ScannerMount("front_right", "front", 2.0, -1.35, math.radians(-45)))
REAR_MOUNTS = (ScannerMount("rear_left", "rear", 1.7, -1.2, math.radians(-45)),
               ScannerMount("rear_right", "rear", 1.7, 1.2, math.radians(45)))


@dataclass
class ScanResult:
    front_prot: bool
    front_warn: bool
    rear_prot: bool
    rear_warn: bool
    side_prot: bool
    side_contact: bool            # SR-12 contact band beside either frame (always evaluated)
    front_side: bool              # SR-16 front-frame side fields, all sets (always evaluated)
    under_near: bool              # SR-16 under-bucket fields (always evaluated)
    under_far: bool
    under_mid: bool               # SR-16 gap ahead of the near field, up to the low bucket's back edge
    points_front: np.ndarray      # (N,2) in front frame, for logging/visualisation
    points_rear: np.ndarray       # (N,2) in rear frame


class SafetyScanners:
    def __init__(self, rng: np.random.Generator):
        self.rng = rng
        self.last_ranges: dict[str, np.ndarray] = {}     # per scanner, for ROS LaserScan output
        self._prev = {"fp": False, "fw": False, "rp": False, "rw": False, "sp": False, "sc": False, "un": False,
                      "uf": False, "fs": False}

    @staticmethod
    def _adjacent_hits(mask: np.ndarray) -> bool:
        return bool(np.any(mask[1:] & mask[:-1]))

    def scan(self, body_poses: dict[str, tuple[float, float, float]], fs: FieldSet,
             segments: np.ndarray, circles: np.ndarray, dust: float = 0.0) -> ScanResult:
        hits = {"front": [], "rear": []}
        for m in FRONT_MOUNTS + REAR_MOUNTS:
            bx, by, bth = body_poses[m.body]   # rear body heading already points backwards
            c, s = math.cos(bth), math.sin(bth)
            ox, oy = bx + c * m.x - s * m.y, by + s * m.x + c * m.y
            n = int(m.fov / m.resolution) + 1
            local = m.heading + np.linspace(-m.fov / 2, m.fov / 2, n)
            r = raycast(ox, oy, bth + local, m.max_range, segments, circles)
            if dust > 0:
                # Airborne dust returns short, spurious echoes.
                d = self.rng.random(n) < dust
                r = np.where(d, np.minimum(r, self.rng.uniform(0.3, 3.5, n)), r)
            valid = r < m.max_range
            # beam endpoints in the body frame (`local` angles are body-frame)
            hx = m.x + r * np.cos(local)
            hy = m.y + r * np.sin(local)
            hits[m.body].append((hx, hy, valid))
            self.last_ranges[m.name] = r

        def eval_fields(body: str, polys: list) -> list[bool]:
            out = []
            for poly in polys:
                if not poly:
                    out.append(False)
                    continue
                v = False
                for hx, hy, valid in hits[body]:
                    inside = valid & points_in_polygon(hx, hy, poly)
                    if self._adjacent_hits(inside):
                        v = True
                        break
                out.append(v)
            return out

        side_polys = [fs.side, [(x, -y) for x, y in fs.side]] if fs.side else []
        front_p, front_w = (eval_fields("front", [fs.protective, fs.warning])
                            if fs.direction >= 0 else (False, False))
        rear_p, rear_w = (eval_fields("rear", [fs.protective_for("rear"), fs.warning_for("rear")])
                          if fs.direction <= 0 else (False, False))
        side = any(eval_fields("front", side_polys)) if side_polys else False
        if fs.side_rear:
            side = side or any(eval_fields("rear", [fs.side_rear, [(x, -y) for x, y in fs.side_rear]]))
        contact = any(eval_fields("front", [fs.side_contact, [(x, -y) for x, y in fs.side_contact]])) or \
            any(eval_fields("rear", [fs.side_contact_rear, [(x, -y) for x, y in fs.side_contact_rear]]))
        # Multiple sampling: a field counts as violated in two consecutive scans.
        un, uf, um = eval_fields("front", [fs.under_near, fs.under_far, fs.under_mid])
        fsd = any(eval_fields("front", [fs.front_side, [(x, -y) for x, y in fs.front_side]]))
        now = {"fp": front_p, "fw": front_w, "rp": rear_p, "rw": rear_w, "sp": side, "sc": contact, "un": un, "uf": uf,
               "um": um, "fs": fsd}
        out = {k: now[k] and self._prev[k] for k in now}
        self._prev = now
        pf = np.column_stack([np.concatenate([h[0][h[2]] for h in hits["front"]]),
                              np.concatenate([h[1][h[2]] for h in hits["front"]])])
        pr = np.column_stack([np.concatenate([h[0][h[2]] for h in hits["rear"]]),
                              np.concatenate([h[1][h[2]] for h in hits["rear"]])])
        return ScanResult(out["fp"], out["fw"], out["rp"], out["rw"], out["sp"], out["sc"], out["fs"], out["un"],
                          out["uf"], out["um"], pf, pr)


# ------------------------------------------------------------------- GNSS etc
@dataclass
class GnssFix:
    x: float
    y: float
    heading: float
    sigma_xy: float
    sigma_h: float
    rtk: bool


class Gnss:
    """Dual-antenna RTK receiver. Float solution when RTK is lost; none in outage."""

    def __init__(self, rng: np.random.Generator, sigma_rtk: float = 0.02, sigma_float: float = 0.6):
        self.rng, self.s_rtk, self.s_float = rng, sigma_rtk, sigma_float
        self._bias = np.zeros(2)

    def read(self, x: float, y: float, th: float, outage: bool, degraded: bool = False) -> GnssFix | None:
        if outage:
            return None
        r = self.rng
        if degraded:
            self._bias = 0.98 * self._bias + r.normal(0, 0.05, 2)
            s = self.s_float
            return GnssFix(x + self._bias[0] + r.normal(0, s), y + self._bias[1] + r.normal(0, s),
                           th + r.normal(0, 0.03), s * 1.5, 0.03, False)
        return GnssFix(x + r.normal(0, self.s_rtk), y + r.normal(0, self.s_rtk), th + r.normal(0, 0.004),
                       self.s_rtk, 0.004, True)


@dataclass
class Odometry:
    """Wheel speed (with slip in the pile) and front-frame gyro."""
    rng: np.random.Generator
    speed_scale: float = 1.01
    gyro_bias: float = 0.002

    def speed(self, v: float, slipping: bool) -> float:
        slip = self.rng.uniform(0.05, 0.25) if slipping and v > 0 else 0.0
        return v * self.speed_scale * (1 + slip) + self.rng.normal(0, 0.02)

    def yaw_rate(self, w: float) -> float:
        return w + self.gyro_bias + self.rng.normal(0, 0.003)


# ------------------------------------------------------------- lidar & camera
@dataclass
class LidarFrame:
    landmarks: list[tuple[int, float, float]]           # (id, range, bearing) front frame
    face: list[tuple[str, int, float, float]]           # (pile, slot, toe_x est, slot y)
    truck: tuple[float, float, float] | None            # world pose estimate of truck
    people: list[tuple[float, float]]                   # world positions (non-safety)


@dataclass
class Lidar3D:
    rng: np.random.Generator
    max_range: float = 60.0
    face_range: float = 25.0

    def read(self, pose: tuple[float, float, float], site, people, dust: float) -> LidarFrame:
        x, y, th = pose
        r = self.rng
        keep = 1.0 - 0.7 * min(1.0, dust * 20)    # heavy dust hides most returns
        lms = []
        for i, (lx, ly) in enumerate(site.landmarks):
            d = math.hypot(lx - x, ly - y)
            if d < self.max_range and r.random() < keep:
                b = math.atan2(ly - y, lx - x) - th
                lms.append((i, d + r.normal(0, 0.03), math.atan2(math.sin(b), math.cos(b)) + r.normal(0, 0.003)))
        face = []
        for pile in site.piles.values():
            for j, s in enumerate(pile.slots):
                d = math.hypot(s.toe_x - x, s.y_c - y)
                b = math.atan2(s.y_c - y, s.toe_x - x) - th
                if d < self.face_range and abs(math.atan2(math.sin(b), math.cos(b))) < math.radians(70) \
                        and r.random() < keep:
                    face.append((pile.name, j, s.toe_x + r.normal(0, 0.06), s.y_c))
        truck = None
        tr = site.truck
        if tr.present and math.hypot(tr.cx - x, tr.cy - y) < 35 and r.random() < keep:
            truck = (tr.cx + r.normal(0, 0.05), tr.cy + r.normal(0, 0.05), tr.heading + r.normal(0, 0.01))
        ppl = [(px + r.normal(0, 0.1), py + r.normal(0, 0.1)) for _, px, py in people
               if math.hypot(px - x, py - y) < 30 and r.random() < 0.9 * keep]
        return LidarFrame(lms, face, truck, ppl)


@dataclass
class CameraDetector:
    """Front/rear camera person detection, calibrated from the trained model's test metrics."""
    rng: np.random.Generator
    recall: float = 0.85
    fp_per_frame: float = 0.01
    max_range: float = 25.0
    fov: float = math.radians(100)

    def detect(self, pose: tuple[float, float, float], people, reverse: bool, dust: float) -> list[tuple[float, float]]:
        x, y, th = pose
        look = th + (math.pi if reverse else 0.0)
        out = []
        for _, px, py in people:
            d = math.hypot(px - x, py - y)
            b = math.atan2(py - y, px - x) - look
            if d < self.max_range and abs(math.atan2(math.sin(b), math.cos(b))) < self.fov / 2:
                p = self.recall * max(0.2, min(1.0, 1.25 - d / self.max_range)) * (1 - 0.5 * min(1, dust * 20))
                if self.rng.random() < p:
                    out.append((px + self.rng.normal(0, 0.05 * d), py + self.rng.normal(0, 0.05 * d)))
        if self.rng.random() < self.fp_per_frame:
            d = self.rng.uniform(3, self.max_range)
            a = look + self.rng.uniform(-self.fov / 2, self.fov / 2)
            out.append((x + d * math.cos(a), y + d * math.sin(a)))
        return out
