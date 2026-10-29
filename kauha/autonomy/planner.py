"""Truck-loading V-cycle planner and motion control.

    GOTO_PILE -> APPROACH -> DIG -> BACKOUT -> (WAIT_TRUCK) -> GOTO_TRUCK
        -> DOCK -> DUMP -> BACKOFF -> GOTO_PILE ...

The planner asks the safety PLC for the dig and dock field sets at the
right moments and waits for the PLC to confirm them before letting the bucket
near the face or the truck. It never assumes a request was granted.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from kauha.autonomy.paths import Path, PurePursuit, bezier, from_points, kappa_from_gamma, reverse_arc, straight
from kauha.autonomy.perception import PeopleTracker, PileMap, TruckTracker
from kauha.config import MachineConfig
from kauha.geometry import dist_to_poly_edge, point_in_poly
from kauha.kinematics import bucket_tip_local
from kauha.safety.fields import DIG_SPEED, DOCK_SPEED

DIG_FIELD_ID = 100
DOCK_FIELD_ID = 101

BOOM_CARRY, BUCKET_CARRY = -0.2, 0.55
BOOM_DIG, BUCKET_DIG = -0.62, 0.0
BOOM_DUMP, BUCKET_DUMP = 0.85, -0.7
DIG_TIMEOUT_S = 14.0
LINE_UP_M = 4.0         # straight before the dig entry point
RUN_IN_M = 5.0          # straight approach to the face (room to settle lateral error)
PRE_DOCK_M = 2.0        # straight final approach to the truck
SQUARE_ON_M = 8.0       # straight run lined up with the truck before the pre-dock point


class Phase(str, Enum):
    GOTO_PILE = "GOTO_PILE"
    APPROACH = "APPROACH"
    DIG = "DIG"
    BACKOUT = "BACKOUT"
    WAIT_TRUCK = "WAIT_TRUCK"
    GOTO_TRUCK = "GOTO_TRUCK"
    DOCK = "DOCK"
    DUMP = "DUMP"
    BACKOFF = "BACKOFF"


PHASE_TIMEOUT_S = {Phase.GOTO_PILE: 60.0, Phase.APPROACH: 40.0, Phase.BACKOUT: 40.0, Phase.GOTO_TRUCK: 70.0,
                   Phase.DOCK: 40.0, Phase.DUMP: 20.0, Phase.BACKOFF: 40.0}


@dataclass
class Percept:
    t: float
    x: float
    y: float
    th: float
    rear: tuple[float, float, float]
    v: float
    gamma: float
    boom: float
    bucket: float
    load_frac: float          # payload estimate / rated bucket volume
    force_ratio: float
    stalled: bool
    safety_state: int
    speed_limit: float
    field_set: int
    truck_present: bool
    loc_sigma: float


@dataclass
class Command:
    speed: float = 0.0
    articulation: float = 0.0
    boom: float = 0.0
    bucket: float = 0.0
    brake: bool = True
    dig_req: bool = False
    dock_req: bool = False


@dataclass
class CycleRecord:
    t_start: float
    t_dig_start: float = 0.0
    t_dig_end: float = 0.0
    t_dump: float = 0.0
    fill_est: float = 0.0
    pile: str = ""
    slot: int = -1


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def servo(target: float, value: float, gain: float = 4.0) -> float:
    return float(np.clip(gain * (target - value), -1.0, 1.0))


@dataclass
class CyclePlanner:
    cfg: MachineConfig
    piles: PileMap
    truck: TruckTracker
    people: PeopleTracker
    bucket_controller: object                  # callable(obs) -> action[3]
    dock_offset: float
    dig_zone_entry_x: float = 41.0
    pile_schedule: list[tuple[float, str]] = field(default_factory=lambda: [(0.0, "A")])
    area: list | None = None                   # operating area polygon from the site map
    boundary_clearance: float = 6.5            # rear contour + reverse field + margin
    phase: Phase = Phase.GOTO_PILE
    path: Path | None = None
    hold_reason: str = ""
    cycles: list[CycleRecord] = field(default_factory=list)

    def __post_init__(self) -> None:
        c = self.cfg
        self.pp = PurePursuit(c.lf, c.lr, c.max_articulation)
        self.kmax = 0.85 * kappa_from_gamma(c.max_articulation, c.lf, c.lr)
        self.W = c.lf + c.lr
        self.slot = 0
        self.pile = self.pile_schedule[0][1]
        self.t_phase = 0.0
        self.dig_t0 = 0.0
        self.dock_k = 0
        self.dump_t0 = None
        self.cur = CycleRecord(0.0)
        self.hold_since: float | None = None
        self.assist_request: str | None = None
        self.dig_fallback = False
        self.next_slot: int | None = None
        self.repositions = 0
        self._timeout_flag = None

    # ---------------------------------------------------------------- helpers
    def _enter(self, phase: Phase, t: float) -> None:
        self.phase, self.t_phase = phase, t
        self.pp.reset()

    def current_pile(self, t: float) -> str:
        name = self.pile_schedule[0][1]
        for t0, n in self.pile_schedule:
            if t >= t0:
                name = n
        return name

    def _dig_entry(self) -> tuple[float, float, float]:
        s = self.piles.slots[self.pile][self.slot]
        x_face_reach = s.toe_x - 5.1 - RUN_IN_M
        return (x_face_reach, s.y_c, 0.0)

    def _dock_pose(self) -> tuple[float, float, float]:
        tx, ty, th = self.truck.best()
        along = (-2.5, 0.0, 2.5, 0.0)[self.dock_k % 4]
        side_y = ty + 1.25
        return (tx + along, side_y + self.dock_offset, -math.pi / 2)

    def _speed_profile(self, p: Percept, v_max: float, v_end: float = 0.0, decel: float = 0.7,
                       end_margin: float = 0.0) -> float:
        rem = self.path.length - float(self.path.s[self.pp.idx]) if self.path is not None else 0.0
        j = min(len(self.path.kappa) - 1, self.pp.idx + 10) if self.path is not None else 0
        k_ahead = float(np.abs(self.path.kappa[self.pp.idx:j + 1]).max()) if self.path is not None else 0.0
        v_curve = math.sqrt(0.9 / max(k_ahead, 1e-3))
        v_stop = math.sqrt(v_end ** 2 + 2 * decel * max(0.0, rem - 0.1 - end_margin))
        return max(0.0, min(v_max, v_curve, v_stop, p.speed_limit))

    def _follow(self, p: Percept) -> tuple[float, float]:
        if self.path.reverse:
            rx, ry, rth = p.rear
            return self.pp.steer(self.path, (rx, ry), rth + math.pi, p.v)
        return self.pp.steer(self.path, (p.x, p.y), p.th, p.v)

    def _people_gate(self, p: Percept, v: float) -> float:
        """Autonomy-level slowdown/hold for detected people on the path ahead."""
        if self.path is None:
            return v
        d = self.people.nearest_in_corridor(self.path.pts, self.pp.idx)
        if d < 8.0:
            if self.hold_since is None:
                self.hold_since = p.t
            self.hold_reason = "person_on_path"
            if p.t - self.hold_since > 45.0:
                self.assist_request = "person_on_path"
                self.hold_since = p.t
            return 0.0
        self.hold_since = None
        self.hold_reason = ""
        if d < 16.0:
            return min(v, 1.0)
        return v

    # ------------------------------------------------------------------- tick
    def tick(self, p: Percept, dig_obs_fn) -> Command:
        ph = self.phase
        cmd = Command(brake=False)
        limit = PHASE_TIMEOUT_S.get(ph)
        if limit and p.t - self.t_phase > limit and self._timeout_flag != (ph, self.t_phase):
            self._timeout_flag = (ph, self.t_phase)
            self.assist_request = f"phase_timeout_{ph.value.lower()}"

        if ph == Phase.GOTO_PILE:
            if self.path is None:
                self.pile = self.current_pile(p.t)
                self.slot = self.next_slot if self.next_slot is not None else self.piles.choose(self.pile)
                self.next_slot = None
                ex, ey, eth = self._dig_entry()
                # curve to a lead point, then line up square to the face
                lead = (ex - LINE_UP_M, ey)
                curve = bezier((p.x, p.y), p.th, lead, eth, self.kmax, reverse=False)
                ahead = (lead[0] - p.x) * math.cos(p.th) + (lead[1] - p.y) * math.sin(p.th)
                if not curve.feasible or ahead < 4.0:
                    # No drivable forward route from here: back up and re-plan.
                    self.next_slot = self.slot
                    self._reposition(p, "no_route_to_face")
                    return Command(brake=True)
                line = straight(lead, eth, LINE_UP_M, reverse=False)
                self.path = from_points(np.vstack([curve.pts, line.pts[1:]]), reverse=False)
                self.cur = CycleRecord(p.t, pile=self.pile, slot=self.slot)
            g, rem = self._follow(p)
            v = self._people_gate(p, self._speed_profile(p, 3.0, v_end=DIG_SPEED))
            near = rem < 8.0
            cmd = Command(v, g, servo(BOOM_DIG if near else BOOM_CARRY, p.boom),
                          servo(BUCKET_DIG if near else BUCKET_CARRY, p.bucket), False)
            if rem < 0.3:
                ex, ey, _ = self._dig_entry()
                if abs(p.y - ey) > 0.8 or abs(wrap(p.th)) > 0.3:
                    self.next_slot = self.slot
                    self._reposition(p, "missed_line_up")
                    return Command(brake=True)
                self.path = straight((p.x, ey), 0.0, RUN_IN_M + 2.5, reverse=False)
                self._enter(Phase.APPROACH, p.t)

        elif ph == Phase.APPROACH:
            g, rem = self._follow(p)
            s = self.piles.slots[self.pile][self.slot]
            tip_x = p.x + bucket_tip_local(self.cfg, p.boom, p.bucket)[0] * math.cos(p.th)
            pen = tip_x - s.toe_x
            granted = p.field_set == DIG_FIELD_ID
            v = DIG_SPEED * 0.95
            if not granted and s.toe_x - tip_x < 1.0:
                v = 0.0     # never touch the face without the dig field set
                if p.t - self.t_phase > 12.0:
                    self.next_slot = self.slot
                    self._reposition(p, "dig_field_not_granted")
                    return Command(brake=True)
            cmd = Command(v, g, servo(BOOM_DIG, p.boom), servo(BUCKET_DIG, p.bucket), False,
                          dig_req=p.x > self.dig_zone_entry_x)
            if granted and pen >= 0.0:
                self.repositions = 0
                self.dig_t0 = p.t
                self.cur.t_dig_start = p.t
                self.dig_fallback = False
                self._enter(Phase.DIG, p.t)

        elif ph == Phase.DIG:
            obs = dig_obs_fn(p, self.pile, self.slot, p.t - self.dig_t0)
            pen, tip_z = obs[0], obs[1]
            if p.t - self.dig_t0 > DIG_TIMEOUT_S:
                self.dig_fallback = True
            if self.dig_fallback:
                a = np.array([-0.6, 0.5, 1.0])
            else:
                a = np.asarray(self.bucket_controller(obs), dtype=float)
            speed = -1.0 + (a[0] + 1) / 2 * (DIG_SPEED + 1.0)
            # Hold the heading straight into the face.
            g = float(np.clip(-1.5 * math.atan2(math.sin(p.th), math.cos(p.th)), -0.1, 0.1))
            cmd = Command(speed, g, float(a[1]), float(a[2]), False, dig_req=True)
            surface = self.piles.surface(self.pile, pen)
            out = pen <= 0.0 or tip_z > surface + 0.25
            loaded = p.load_frac * self.cfg.bucket_volume > 0.2
            if (p.bucket >= 0.55 and out and loaded) or (pen < -0.5 and loaded):
                self.cur.t_dig_end = p.t
                self.cur.fill_est = p.load_frac
                self._plan_backout(p)
                self._enter(Phase.BACKOUT, p.t)
            elif pen < -0.5 and not loaded:
                # Came out of the face empty: back away and try again.
                rx, ry, rth = p.rear
                self.path = straight((rx, ry), rth + math.pi, 5.0, reverse=True)
                self._enter(Phase.BACKOFF, p.t)

        elif ph == Phase.BACKOUT:
            g, rem = self._follow(p)
            v = -self._people_gate(p, self._speed_profile(p, 2.5))
            travelled = self.path.s[self.pp.idx]
            # Keep the dig field set until moving backwards (the PLC then uses the
            # rear fields), so the pile flanks do not trip the standstill set.
            cmd = Command(v, g, servo(BOOM_CARRY, p.boom), servo(BUCKET_CARRY, p.bucket), False,
                          dig_req=travelled < 1.0)
            if rem < 0.15 and abs(p.v) < 0.1:
                self.path = None
                self._enter(Phase.GOTO_TRUCK if p.truck_present else Phase.WAIT_TRUCK, p.t)

        elif ph == Phase.WAIT_TRUCK:
            cmd = Command(0.0, 0.0, servo(BOOM_CARRY, p.boom), servo(BUCKET_CARRY, p.bucket), True)
            if p.truck_present:
                self._enter(Phase.GOTO_TRUCK, p.t)

        elif ph == Phase.GOTO_TRUCK:
            dx, dy, dth = self._dock_pose()
            if self.path is None:
                pre = (dx - PRE_DOCK_M * math.cos(dth), dy - PRE_DOCK_M * math.sin(dth))
                # Finish with a straight, square-on run so no corner of the truck
                # swings into a curved protective field before the dock set is granted.
                lead = (pre[0] - SQUARE_ON_M * math.cos(dth), pre[1] - SQUARE_ON_M * math.sin(dth))
                curve = bezier((p.x, p.y), p.th, lead, dth, self.kmax, reverse=False)
                line = straight(lead, dth, SQUARE_ON_M, reverse=False)
                self.path = from_points(np.vstack([curve.pts, line.pts[1:]]), reverse=False)
            g, rem = self._follow(p)
            # Be at dock speed before the truck can reach the normal protective
            # field, i.e. before entering the dock zone (~3.5 m before pre-dock).
            v = self._people_gate(p, self._speed_profile(p, 3.0, v_end=DOCK_SPEED * 0.8, end_margin=4.5))
            raise_boom = rem < 10.0
            cmd = Command(v, g, servo(BOOM_DUMP if raise_boom else BOOM_CARRY, p.boom),
                          servo(BUCKET_CARRY, p.bucket), False, dock_req=rem < 6.0)
            if rem < 0.3:
                self.path = straight((p.x, p.y), dth, PRE_DOCK_M + 0.3, reverse=False)
                self._enter(Phase.DOCK, p.t)

        elif ph == Phase.DOCK:
            dx, dy, dth = self._dock_pose()
            g, _ = self._follow(p)
            granted = p.field_set == DOCK_FIELD_ID
            # distance from front axle to the dock pose, along the heading
            rem = (dx - p.x) * math.cos(dth) + (dy - p.y) * math.sin(dth)
            v = min(DOCK_SPEED * 0.92, math.sqrt(2 * 0.5 * max(0.0, rem))) if granted else 0.0
            if not granted and p.t - self.t_phase > 20.0:
                self.assist_request = "dock_field_not_granted"
            cmd = Command(v, g, servo(BOOM_DUMP, p.boom), servo(BUCKET_CARRY, p.bucket), False,
                          dock_req=True)
            if rem < 0.05 and abs(p.v) < 0.05 and p.boom > BOOM_DUMP - 0.05:
                self.dump_t0 = p.t
                self._enter(Phase.DUMP, p.t)

        elif ph == Phase.DUMP:
            cmd = Command(0.0, 0.0, servo(BOOM_DUMP, p.boom), -1.0, True, dock_req=True)
            if p.bucket <= BUCKET_DUMP + 0.05 and p.load_frac < 0.05 and p.t - self.dump_t0 > 1.0:
                self.cur.t_dump = p.t
                self.cycles.append(self.cur)
                self.dock_k += 1
                self._plan_backoff(p)
                self._enter(Phase.BACKOFF, p.t)

        elif ph == Phase.BACKOFF:
            g, rem = self._follow(p)
            v = -self._people_gate(p, self._speed_profile(p, 2.5))
            travelled = self.path.s[self.pp.idx]
            boom_t = BOOM_DUMP if (travelled < 1.5 and p.boom > 0.5) else BOOM_CARRY
            # Keep the dock field set until moving backwards; the PLC then
            # switches to the rear fields on its own speed measurement.
            cmd = Command(v, g, servo(boom_t, p.boom), servo(BUCKET_CARRY, p.bucket), False,
                          dock_req=travelled < 1.0)
            if rem < 0.15 and abs(p.v) < 0.1:
                self.path = None
                self._enter(Phase.GOTO_PILE, p.t)

        return cmd

    def _reposition(self, p: Percept, reason: str) -> None:
        """Back straight up 6 m and try the approach again; ask for help after two tries."""
        self.repositions += 1
        if self.repositions > 2:
            self.assist_request = reason
            self.repositions = 0
        rx, ry, rth = p.rear
        self.path = straight((rx, ry), rth + math.pi, 6.0, reverse=True)
        self._enter(Phase.BACKOFF, p.t)

    def resume(self, p: Percept) -> None:
        """Re-plan the current phase from wherever the machine now is (after teleop)."""
        self.pp.reset()
        if self.phase in (Phase.APPROACH, Phase.DIG, Phase.BACKOUT):
            if p.load_frac * self.cfg.bucket_volume > 0.3:
                self._plan_backout(p)
                self._enter(Phase.BACKOUT, p.t)
            else:
                self.path = None
                self._enter(Phase.GOTO_PILE, p.t)
        elif self.phase in (Phase.DOCK, Phase.DUMP, Phase.BACKOFF):
            if p.load_frac * self.cfg.bucket_volume > 0.3:
                self.path = None
                self._enter(Phase.GOTO_TRUCK, p.t)
            else:
                self._plan_backoff(p)
                self._enter(Phase.BACKOFF, p.t)
        else:
            self.path = None

    # ------------------------------------------------------------ path setup
    def _fit_reverse(self, p: Percept, dh: float, straight_m: float = 2.0) -> Path:
        """Reverse arc that keeps the rear clear of the site boundary.

        Shrinks the swing (and then the straight part) until every point of the
        rear-axle path is at least `boundary_clearance` inside the operating
        area, so the rear fields never reach the perimeter wall.
        """
        best = None
        for scale in (1.0, 0.7, 0.45, 0.2, 0.0):
            for runout in (3.0, 1.5):
                path, _ = reverse_arc((p.x, p.y, p.th), dh * scale, self.W, straight_m=straight_m, runout_m=runout)
                if self.area is None or all(
                        point_in_poly(x, y, self.area) and dist_to_poly_edge(x, y, self.area) > self.boundary_clearance
                        for x, y in path.pts[::3]):
                    return path
                best = best or path
        return best

    def _plan_backout(self, p: Percept) -> None:
        dx, dy, _ = self._dock_pose()
        # V-pattern: back out of the face swinging the rear away from the truck
        # (clockwise), so the front ends up pointing at the truck.
        dh = -1.0
        for _ in range(3):
            _, end = reverse_arc((p.x, p.y, p.th), dh, self.W)
            want = math.atan2(dy + PRE_DOCK_M - end[1], dx - end[0])
            dh = float(np.clip(-((p.th - want) % (2 * math.pi)), -1.5, -0.3))   # clockwise
        self.path = self._fit_reverse(p, dh)

    def _plan_backoff(self, p: Percept) -> None:
        self.pile = self.current_pile(p.t)
        nxt = self.piles.choose(self.pile)
        self.next_slot = nxt
        s = self.piles.slots[self.pile][nxt]
        # Back away from the truck swinging the rear away from the stockpiles
        # (counter-clockwise), so the front turns toward the face.
        dh = 1.0
        for _ in range(3):
            _, end = reverse_arc((p.x, p.y, p.th), dh, self.W)
            want = math.atan2(s.y_c - end[1], (s.toe_x - 18.0) - end[0])
            dh = float(np.clip((want - p.th) % (2 * math.pi), 0.3, 1.5))        # counter-clockwise
        self.path = self._fit_reverse(p, dh, straight_m=2.5)
