"""Site model for the digital twin: stockpiles, truck bay, people, environment.

Stockpiles are split into lateral slots along the face; each slot's toe
recedes as material is removed, so the dig point has to be re-found every
cycle. Trucks arrive, are loaded to capacity and leave. People follow
scripted walks. Environment events (dust, GNSS outage, radio link loss,
autonomy-computer freeze) are scheduled by the scenario.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from kauha.config import REPO_ROOT, MachineConfig, Material
from kauha.geometry import dist_to_poly_edge, point_in_poly  # noqa: F401  (re-exported)
from kauha.sim.soil import Pile as SoilPile

DEFAULT_SITE_FILE = REPO_ROOT / "hardware" / "config" / "site_demo.yaml"
Polygon = list[tuple[float, float]]


@dataclass
class Slot:
    y_c: float
    width: float
    toe_x: float
    removed_m3: float = 0.0


@dataclass
class Stockpile:
    name: str
    material: Material
    slots: list[Slot]
    height: float
    back_x: float

    def slot_at(self, y: float) -> Slot | None:
        for s in self.slots:
            if abs(y - s.y_c) <= s.width / 2:
                return s
        return None

    def soil(self, rng: np.random.Generator | None = None) -> SoilPile:
        m = self.material
        return SoilPile(m.name, m.density_t_m3, m.k_resist, m.friction_deg, self.height)

    def remove(self, slot: Slot, m3: float) -> None:
        slot.removed_m3 += m3
        # The face recedes by volume / (slot width x face height).
        slot.toe_x = min(self.back_x - 2.0, slot.toe_x + m3 / (slot.width * self.height * 0.85))

    def footprint_segments(self, scan_height: float = 0.5) -> list[tuple[float, float, float, float]]:
        """Where a horizontal scan plane at `scan_height` meets each slot's face."""
        segs = []
        off = scan_height / math.tan(math.radians(self.material.friction_deg))
        for s in self.slots:
            x = s.toe_x + off
            y0, y1 = s.y_c - s.width / 2, s.y_c + s.width / 2
            segs += [(x, y0, x, y1), (x, y0, self.back_x, y0), (x, y1, self.back_x, y1)]
        return segs


@dataclass
class Truck:
    cx: float
    cy: float
    heading: float
    length: float
    width: float
    capacity_t: float
    payload_t: float = 0.0
    present: bool = True
    buckets: int = 0
    arrived_t: float = 0.0
    serial: int = 1

    def corners(self) -> Polygon:
        c, s = math.cos(self.heading), math.sin(self.heading)
        hl, hw = self.length / 2, self.width / 2
        return [(self.cx + c * dx - s * dy, self.cy + s * dx + c * dy)
                for dx, dy in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]

    def segments(self) -> list[tuple[float, float, float, float]]:
        if not self.present:
            return []
        p = self.corners()
        return [(*p[i], *p[(i + 1) % 4]) for i in range(4)]

    def side_y(self) -> float:
        """y of the long side facing the loading lane (+y side, heading 0)."""
        return self.cy + self.width / 2


@dataclass
class Person:
    name: str
    route: list[tuple[float, float, float]]   # (t_arrive, x, y) waypoints; linear between
    radius: float = 0.25
    delay: float = 0.0          # time spent waiting (people do not walk into the machine)

    def position(self, t: float) -> tuple[float, float] | None:
        r = self.route
        t = t - self.delay
        if t < r[0][0] or t > r[-1][0]:
            return None
        for (t0, x0, y0), (t1, x1, y1) in zip(r, r[1:]):
            if t0 <= t <= t1:
                u = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return x0 + u * (x1 - x0), y0 + u * (y1 - y0)
        return r[-1][1], r[-1][2]


@dataclass
class Event:
    kind: str          # dust | gnss_outage | link_loss | autonomy_freeze | remote_estop
    t0: float
    t1: float
    level: float = 1.0


@dataclass
class Site:
    name: str
    operating_area: Polygon
    walls: list[tuple[float, float, float, float]]
    piles: dict[str, Stockpile]
    dig_zones: dict[str, Polygon]
    dock_zone: Polygon
    truck: Truck
    dock_offset: float
    swap_time_s: float
    landmarks: list[tuple[float, float]]
    start_pose: tuple[float, float, float]
    people: list[Person] = field(default_factory=list)
    events: list[Event] = field(default_factory=list)
    trucks_completed: int = 0
    tonnes_loaded: float = 0.0
    _truck_gone_at: float | None = None

    # ------------------------------------------------------------- dynamics
    def update(self, t: float) -> None:
        tr = self.truck
        if not tr.present and self._truck_gone_at is not None and t - self._truck_gone_at >= self.swap_time_s:
            tr.present, tr.payload_t, tr.buckets, tr.arrived_t = True, 0.0, 0, t
            tr.serial += 1
            self._truck_gone_at = None

    def dump_into_truck(self, t: float, tonnes: float) -> float:
        tr = self.truck
        if not tr.present:
            return 0.0
        accepted = min(tonnes, tr.capacity_t * 1.05 - tr.payload_t)
        tr.payload_t += accepted
        tr.buckets += 1
        self.tonnes_loaded += accepted
        return accepted

    def dispatch_truck_if_full(self, t: float, next_bucket_t: float) -> bool:
        tr = self.truck
        if tr.present and tr.payload_t + 0.7 * next_bucket_t > tr.capacity_t:
            tr.present = False
            self._truck_gone_at = t
            self.trucks_completed += 1
            return True
        return False

    def active(self, kind: str, t: float) -> Event | None:
        for e in self.events:
            if e.kind == kind and e.t0 <= t < e.t1:
                return e
        return None

    def people_at(self, t: float) -> list[tuple[str, float, float]]:
        out = []
        for p in self.people:
            pos = p.position(t)
            if pos is not None:
                out.append((p.name, *pos))
        return out

    def static_segments(self) -> list[tuple[float, float, float, float]]:
        segs = list(self.walls)
        for pile in self.piles.values():
            segs += pile.footprint_segments()
        return segs

    def zone_of(self, x: float, y: float) -> str | None:
        for name, poly in self.dig_zones.items():
            if point_in_poly(x, y, poly):
                return name
        return None


def load_site(cfg: MachineConfig, path: str | Path = DEFAULT_SITE_FILE) -> Site:
    d = yaml.safe_load(Path(path).read_text())
    piles = {}
    for p in d["piles"]:
        n = int(round((p["y_max"] - p["y_min"]) / p["slot_width"]))
        slots = [Slot(p["y_min"] + (i + 0.5) * p["slot_width"], p["slot_width"], p["toe_x"]) for i in range(n)]
        piles[p["name"]] = Stockpile(p["name"], cfg.materials[p["material"]], slots, p["height"],
                                     p["toe_x"] + p["depth"])
    tb = d["truck_bay"]
    truck = Truck(tb["center"][0], tb["center"][1], tb["heading"], tb["length"], tb["width"], tb["capacity_t"])
    walls = [(a[0], a[1], b[0], b[1]) for a, b in d["site"]["walls"]]
    return Site(
        name=d["site"]["name"],
        operating_area=[tuple(p) for p in d["site"]["operating_area"]],
        walls=walls, piles=piles,
        dig_zones={k: [tuple(p) for p in v] for k, v in d["dig_zones"].items()},
        dock_zone=[tuple(p) for p in tb["dock_zone"]],
        truck=truck, dock_offset=tb["dock_offset"], swap_time_s=tb["swap_time_s"],
        landmarks=[tuple(p) for p in d["landmarks"]],
        start_pose=tuple(d["start_pose"]),
    )
