"""Run a full simulated shift of the autonomous loader and measure it.

Everything a real machine would have is in the loop: the loader dynamics and
soil, the safety scanners (ray-cast), the safety PLC with its geofence and
hardwired stop outputs, the CAN bus with end-to-end protection, the
drive-by-wire gateway, the autonomy computer, and a remote supervisor with
teleoperation latency. The scenario injects the situations a site actually
produces: people walking into the work area, a radio link drop, an unmapped
obstacle, dust, a GNSS outage, a material change, an autonomy-computer
freeze and a remote e-stop.

    python -m kauha.demo.shift --controller learned --minutes 60 --out results/shift_learned
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from kauha.autonomy.bucket_policy import BucketPolicy
from kauha.autonomy.planner import Command, Phase
from kauha.autonomy.stack import AutonomyComputer
from kauha.can.codec import Codec
from kauha.can.gateway import MachineGateway, VirtualBus
from kauha.config import REPO_ROOT, load_machine
from kauha.geometry import point_in_poly
from kauha.safety.fields import BOOM_RAISED_M, Geometry
from kauha.safety.geofence import Geofence, SafetyPositioner
from kauha.safety.plc import FIELD_CAUSES, Cause, Mode, SafetyInputs, SafetyParams, SafetyPLC, State
from kauha.safety.stopping import StopBudget
from kauha.sim.loader import Loader, LoaderCommand, LoaderState
from kauha.sim.remote import RemoteStation
from kauha.sim.sensors import CameraDetector, Gnss, Lidar3D, Odometry, SafetyScanners, points_in_polygon
from kauha.sim.soil import BucketPileInteraction, DigResult
from kauha.sim.world import Event, Person, load_site
from kauha.train.bucket_env import ScriptedOperator

SIM_DT = 0.025


def field_polygons(fs, under: bool = False) -> list[tuple[str, list]]:
    """(body, polygon) for every protective area of a field set, including the side fields
    (and, with under=True, the SR-16 under-bucket fields)."""
    out = [("front", fs.protective if fs.direction >= 0 else []),
           ("rear", fs.protective_for("rear") if fs.direction <= 0 else [])]
    if under:
        out += [("front", fs.under_near), ("front", fs.under_far)]
    if fs.direction < 0 and fs.front_side:           # SR-16 front-side fields while reversing
        out += [("front", fs.front_side), ("front", [(x, -y) for x, y in fs.front_side])]
    for body, side in (("front", fs.side), ("rear", fs.side_rear)):
        if side:
            out += [(body, side), (body, [(x, -y) for x, y in side])]
    return out
RECORD_EVERY = 8          # 5 Hz replay frames
WALK_AROUND_AFTER_S = 15.0


# ------------------------------------------------------------------ scenario
def demo_scenario(minutes: float) -> tuple[list[Person], list[Event], list[tuple[float, str]], list[dict]]:
    m = 60.0
    people = [
        # Site worker crosses the loading lane between the pile and the truck.
        Person("worker", [(6 * m, 22, -14), (6 * m + 14, 37, -8), (6 * m + 34, 37, -8), (6 * m + 60, 22, 8)]),
        # Truck driver gets out and walks along the truck side, then back.
        Person("driver", [(42 * m, 45.5, -16.2), (42 * m + 12, 37, -16.0), (42 * m + 30, 37, -16.0),
                          (42 * m + 42, 45.5, -16.2)]),
        # Visitor walking the perimeter path, outside the operating area.
        Person("visitor", [(2 * m, -4, -31), (2 * m + 90, 68, -31.5)]),
    ]
    events = [
        Event("link_loss", 12 * m, 12 * m + 2.0),
        Event("dust", 24 * m, 29 * m, 0.01),
        Event("gnss_outage", 36 * m, 37 * m),
        Event("autonomy_freeze", 48 * m, 48 * m + 0.8),
        Event("remote_estop", 54 * m, 54 * m + 5.0),
    ]
    obstacles = [{"t0": 18 * m, "x": 37.0, "y": -7.0, "r": 0.6, "name": "fallen_block"}]
    schedule = [(0.0, "A"), (30 * m, "B")]
    keep = lambda t0: t0 < minutes * m
    return ([p for p in people if keep(p.route[0][0])], [e for e in events if keep(e.t0)], schedule,
            [o for o in obstacles if keep(o["t0"])])


@dataclass
class Kpis:
    sim_minutes: float = 0.0
    controller: str = ""
    cycles: int = 0
    tonnes: float = 0.0
    trucks: int = 0
    buckets_per_truck: float = 0.0
    tonnes_per_hour: float = 0.0
    cycles_per_hour: float = 0.0
    cycle_time_median_s: float = 0.0
    cycle_time_p90_s: float = 0.0
    clean_cycle_time_median_s: float = 0.0
    dig_time_mean_s: float = 0.0
    fill_factor_mean: float = 0.0
    fill_factor_by_pile: dict = field(default_factory=dict)
    truck_wait_s: float = 0.0
    interventions: int = 0
    interventions_per_hour: float = 0.0
    interventions_by_reason: dict = field(default_factory=dict)
    protective_stops: int = 0
    stops_by_cause: dict = field(default_factory=dict)
    false_stops: int = 0
    false_stops_per_hour: float = 0.0
    person_stop_response_s: list = field(default_factory=list)
    min_person_distance_m: float = 99.0
    min_person_distance_moving_m: float = 99.0
    person_field_entries_while_moving: int = 0
    person_under_raised_bucket_moving_s: float = 0.0
    loc_rms_m: float = 0.0
    loc_rms_gnss_outage_m: float = 0.0
    loc_max_m: float = 0.0
    loc_filter_resets: int = 0
    teleop_latency_p50_ms: float = 0.0
    teleop_latency_p95_ms: float = 0.0
    teleop_budget_ms: float = 150.0
    fuel_l: float = 0.0
    fuel_l_per_t: float = 0.0
    spill_t: float = 0.0
    e2e_errors: int = 0
    gateway_timeouts: int = 0
    log_chain_ok: bool = False
    log_records: int = 0
    wall_time_s: float = 0.0


class ShiftSim:
    def __init__(self, controller: str = "learned", minutes: float = 60.0, seed: int = 1,
                 winter: bool = False, policy_path: Path | None = None, guard: bool = False,
                 careless: bool = False):
        self.rng = np.random.default_rng(seed)
        # Pedestrian clearance (m): (from a moving machine, from a stopped one). The
        # default assumes site-trained people keep about a metre away; "careless"
        # walkers come within 0.3 m and only hold back from a moving machine at 1 m.
        self.yield_m = (1.0, 0.3) if careless else (1.5, 1.0)
        self.cfg = load_machine()
        self.site = load_site(self.cfg)          # ground truth
        site_map = load_site(self.cfg)           # the autonomy's survey map
        people, events, schedule, obstacles = demo_scenario(minutes)
        self.site.people, self.site.events = people, events
        self.obstacles = obstacles
        self.minutes = minutes
        self.controller_name = controller
        if controller == "learned":
            ctrl = BucketPolicy.load(policy_path or REPO_ROOT / "models" / "bucket_policy.npz").act
        else:
            ctrl = ScriptedOperator()
        x, y, th = self.site.start_pose
        self.loader = Loader(self.cfg, LoaderState(x=x, y=y, theta=th), dt=SIM_DT)
        self.loader.state.boom = -0.2
        self.loader.state.bucket = 0.55
        budget = StopBudget().winter() if winter else StopBudget()
        self.plc = SafetyPLC(SafetyParams(), budget)
        self.scanners = SafetyScanners(self.rng)
        self.bus = VirtualBus()
        self.gw = MachineGateway()
        self.auto = AutonomyComputer(self.cfg, site_map, ctrl, schedule, budget=budget,
                                     guard_enabled=guard)
        self.remote = RemoteStation(self.rng)
        self.mcodec = Codec()
        self.plc_codec = Codec()
        self.plc_rx_drive = self.plc_codec.receiver("DriveCmd", 0.15)
        self.plc_rx_hb = self.plc_codec.receiver("RemoteHeartbeat", 0.5)
        self.plc_drive: dict = {}
        self.plc_hb: dict = {}
        self.gnss = Gnss(self.rng)
        self.safety_gnss = Gnss(self.rng, sigma_rtk=0.3)
        self.odo = Odometry(self.rng)
        self.lidar = Lidar3D(self.rng)
        cam = json.loads((REPO_ROOT / "models" / "ppe_detector_metrics.json").read_text()) \
            if (REPO_ROOT / "models" / "ppe_detector_metrics.json").exists() else {}
        self.camera = CameraDetector(self.rng, recall=cam.get("person_recall", 0.85),
                                     fp_per_frame=cam.get("person_fp_per_image", 0.01))
        self.camera_source = "measured" if cam else "assumed"
        self.pos = SafetyPositioner(self.cfg.lf, self.cfg.lr)
        self.pos.fix(x, y, th)
        self.fence = Geofence(self.site.operating_area, self.site.dig_zones, self.site.dock_zone, self.pos)
        self.geom = Geometry()
        self.posts = np.array([(lx, ly, 0.1) for lx, ly in self.site.landmarks], dtype=float)
        self.interaction: BucketPileInteraction | None = None
        self.dig_slot = None
        self.dig_removed = 0.0
        self.last_dig = DigResult(0.0, 0.0, 0.0, 0.0, 0.0)
        self.scan = None
        self.plc_out = None
        self.t = 0.0
        self.k = 0
        self.frames: list[dict] = []
        self.k_out = Kpis(controller=controller)
        self.true_fill: list[tuple[str, float]] = []
        self.spill_t = 0.0
        self.loc_err: list[tuple[float, bool]] = []
        self._prev_gamma = 0.0
        self._phase_prev = None
        self._stop_events: list[dict] = []
        self._person_in_field_since: dict[str, float] = {}
        self._blocked_since: dict[str, float] = {}
        self._prev_state = State.RUN
        self._wait_start = None
        self.truck_wait = 0.0
        self._obstacle_removed: set[str] = set()
        self._teleop_plan = None

    # ----------------------------------------------------------- utilities
    def body_poses(self):
        s = self.loader.state
        rx, ry, rth = self.loader.rear_axle()
        return {"front": (s.x, s.y, s.theta), "rear": (rx, ry, rth + math.pi)}

    def active_obstacles(self):
        return [o for o in self.obstacles if self.t >= o["t0"] and o["name"] not in self._obstacle_removed]

    def circles(self):
        c = [(px, py, 0.25) for _, px, py in self.site.people_at(self.t)]
        c += [(o["x"], o["y"], o["r"]) for o in self.active_obstacles()]
        arr = np.array(c, dtype=float) if c else np.zeros((0, 3))
        return np.vstack([arr, self.posts]) if len(self.posts) else arr

    def segments(self):
        # walls, stockpile faces (which move as material is removed) and the truck
        segs = [tuple(s) for s in self.site.walls]
        for pile in self.site.piles.values():
            segs += pile.footprint_segments()
        segs += self.site.truck.segments()
        return np.array(segs, dtype=float)

    def _to_body(self, body: str, px: float, py: float):
        bx, by, bth = self.body_poses()[body]
        dx, dy = px - bx, py - by
        c, s = math.cos(bth), math.sin(bth)
        return dx * c + dy * s, -dx * s + dy * c

    def _objects_in_protective(self, fs, under: bool = False) -> list[str]:
        """Ground truth: which real objects are inside the active protective/side fields."""
        names = []
        objs = [(n, px, py) for n, px, py in self.site.people_at(self.t)]
        objs += [(o["name"], o["x"], o["y"]) for o in self.active_obstacles()]
        radius = {o["name"]: o["r"] for o in self.active_obstacles()}
        objs += [(f"post{i}", lx, ly) for i, (lx, ly) in enumerate(self.site.landmarks)]
        for wx0, wy0, wx1, wy1 in self.site.walls:
            for u in np.linspace(0, 1, 60):
                objs.append(("wall", wx0 + u * (wx1 - wx0), wy0 + u * (wy1 - wy0)))
        tr = self.site.truck
        if tr.present:
            cs = tr.corners()
            for u in np.linspace(0, 1, 12):
                for a, b in zip(cs, cs[1:] + cs[:1]):
                    objs.append(("truck", a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])))
        for n, px, py in objs:
            for body, poly in field_polygons(fs, under):
                if not poly:
                    continue
                lx, ly = self._to_body(body, px, py)
                # objects have width: test the centre and points on the circle
                rad = radius.get(n, 0.25)
                ring = [(0.0, 0.0)] + [(rad * math.cos(a), rad * math.sin(a)) for a in np.linspace(0, 6.28, 16)]
                for ox, oy in ring:
                    if points_in_polygon(np.array([lx + ox]), np.array([ly + oy]), poly)[0]:
                        names.append(n)
                        break
        # the stockpile faces and flanks at scanner height
        for pile in self.site.piles.values():
            for x0, y0, x1, y1 in pile.footprint_segments():
                for u in np.linspace(0, 1, 25):
                    px, py = x0 + u * (x1 - x0), y0 + u * (y1 - y0)
                    for body, poly in field_polygons(fs):
                        if poly:
                            lx, ly = self._to_body(body, px, py)
                            if points_in_polygon(np.array([lx]), np.array([ly]), poly)[0]:
                                names.append(f"pile_{pile.name}")
        return sorted(set(names))

    def _pedestrians_yield(self, t: float) -> None:
        """People walk their route but stop rather than step into the machine body.

        They can still walk into the safety fields ahead of a moving machine,
        which is the case the safety channel has to handle.
        """
        polys, _ = self.loader.body_polygons()   # same ground-level outline as the contact metric
        moving = abs(self.loader.state.v) > 0.05

        def clearance(pos):
            return min(self._poly_distance(pos[0], pos[1], poly) for poly in polys)

        for p in self.site.people:
            nxt = p.position(t + SIM_DT)
            if nxt is None:
                continue
            d = clearance(nxt)
            if (moving and d < self.yield_m[0]) or d < self.yield_m[1]:
                p.delay += SIM_DT
                since = self._blocked_since.setdefault(p.name, t)
                if not moving and t - since > WALK_AROUND_AFTER_S:
                    # Nobody waits forever beside a parked machine: they walk round it.
                    # Modelled as a skip along the route to the first point clear of the
                    # machine; the walk itself is not simulated.
                    for skip in np.arange(1.0, 90.0, 0.5):
                        q = p.position(t + skip)
                        if q is None or clearance(q) >= self.yield_m[0]:
                            p.delay -= skip
                            break
                    self._blocked_since.pop(p.name, None)
            else:
                self._blocked_since.pop(p.name, None)

    @staticmethod
    def _poly_distance(px: float, py: float, poly) -> float:
        if point_in_poly(px, py, poly):
            return 0.0
        best = 99.0
        for i in range(len(poly)):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % len(poly)]
            dx, dy = bx - ax, by - ay
            u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
            best = min(best, math.hypot(px - ax - u * dx, py - ay - u * dy))
        return best

    def _person_min_distance(self) -> float:
        """Clearance between any person (0.25 m radius) and the machine at ground level."""
        polys, _ = self.loader.body_polygons()
        best = 99.0
        for _, px, py in self.site.people_at(self.t):
            for poly in polys:
                best = min(best, self._poly_distance(px, py, poly) - 0.25)
        return best

    def _person_under_raised_bucket(self) -> bool:
        _, raised = self.loader.body_polygons()
        return raised is not None and any(point_in_poly(px, py, raised) for _, px, py in self.site.people_at(self.t))

    # ------------------------------------------------------------------ step
    def step(self) -> None:
        t, k, s = self.t, self.k, self.loader.state
        site = self.site
        site.update(t)
        self._pedestrians_yield(t)
        dust_ev = site.active("dust", t)
        dust = dust_ev.level if dust_ev else 0.0
        link_ok = site.active("link_loss", t) is None
        frozen = site.active("autonomy_freeze", t) is not None
        self.remote.estop = site.active("remote_estop", t) is not None

        # --- remote station
        action = self.remote.update(t, link_ok)
        if action == "teleop_start":
            self.auto.mode = Mode.TELEOP
            self._start_teleop()
        if k % 2 == 0:
            for fid, data in self.remote.heartbeat(t, link_ok):
                self.bus.send(t, fid, data)

        # --- wheel speed (shared by gateway encoder and PLC sensor, with slip)
        slipping = self.interaction is not None and s.stalled
        v_wheel = s.v * (1 + (self.rng.uniform(0.05, 0.25) if slipping else 0.0))
        speed_a = v_wheel + self.rng.normal(0, 0.02)
        speed_b = v_wheel + self.rng.normal(0, 0.02)

        # --- safety positioner + geofence (PLC's own sensors)
        g_rate = (s.gamma - self._prev_gamma) / SIM_DT
        self._prev_gamma = s.gamma
        self.pos.predict(speed_b, s.gamma, g_rate, SIM_DT)
        outage = site.active("gnss_outage", t) is not None
        if k % 8 == 0 and not outage:
            f = self.safety_gnss.read(s.x, s.y, s.theta, False)
            self.pos.fix(f.x, f.y, f.heading)
        fence = self.fence.evaluate()

        # --- safety scanners (20 Hz)
        fs = self.plc.sets[self.plc.field_set]
        if k % 2 == 0:
            self.scan = self.scanners.scan(self.body_poses(), fs, self.segments(), self.circles(), dust)
        sc = self.scan

        # --- safety PLC (40 Hz): reads its own CAN receivers
        for (_, fid, data) in list(self.bus.frames):
            name = self.plc_codec.db.get_message_by_frame_id(fid).name
            if name == "DriveCmd" and self.plc_rx_drive.check(data, t).value.startswith("ok"):
                self.plc_drive = self.plc_codec.decode(fid, data)[1]
            elif name == "RemoteHeartbeat" and self.plc_rx_hb.check(data, t).value.startswith("ok"):
                self.plc_hb = self.plc_codec.decode(fid, data)[1]
        hb_age = t - self.plc_rx_hb.last_ok_time if self.plc_rx_hb.last_ok_time is not None else 99.0
        cmd_age = t - self.plc_rx_drive.last_ok_time if self.plc_rx_drive.last_ok_time is not None else 99.0
        mode = Mode(int(self.plc_drive.get("Mode", 0)))   # IDLE until the first command
        hb_fresh = hb_age <= 0.5
        inp = SafetyInputs(
            t=t, mode=mode, speed_a=speed_a, speed_b=speed_b, articulation=s.gamma,
            front_prot=sc.front_prot, front_warn=sc.front_warn, rear_prot=sc.rear_prot, rear_warn=sc.rear_warn,
            side_prot=sc.side_prot, side_contact=sc.side_contact, remote_hb_age=hb_age,
            boom_raised=self.loader.bucket_bottom_height() > BOOM_RAISED_M, under_near=sc.under_near,
            front_side=sc.front_side,
            under_far=sc.under_far,
            remote_estop=bool(self.plc_hb.get("EStop", 0)) and hb_fresh,
            reset_req=bool(self.plc_hb.get("ResetReq", 0)) and hb_fresh,
            autonomy_cmd_age=cmd_age, dig_req=bool(self.plc_drive.get("DigFieldReq", 0)),
            dock_req=bool(self.plc_drive.get("DockFieldReq", 0)),
            truck_door_open=any(n == "driver" for n, _, _ in site.people_at(t)), **fence)
        out = self.plc.step(inp)
        self.plc_out = out
        self.bus.send(t, *self.plc_codec.encode("SafetyStatus", {
            "State": int(out.state), "ActiveFieldSet": out.field_set if out.field_set < 256 else 255,
            "Cause": int(out.cause) & 0xFFFF, "SpeedLimit": max(0.0, min(6.0, out.speed_limit)),
            "ResetPending": int(out.reset_pending), "DockBlocked": int(out.dock_blocked)}))
        self._track_stops(out, fs)

        # --- autonomy computer (20 Hz)
        frames = self.bus.drain()
        for (_, fid, data) in frames:
            self.gw.receive(t, fid, data)
            self.auto.receive(fid, data)
        if k % 2 == 1 and not frozen:
            odo_v = self.odo.speed(s.v, slipping)
            th_dot = (s.v * math.sin(s.gamma) + self.cfg.lr * g_rate) / (self.cfg.lf * math.cos(s.gamma) + self.cfg.lr)
            gyro = self.odo.yaw_rate(th_dot)
            gnss = self.gnss.read(s.x, s.y, s.theta, outage) if k % 4 == 1 else None
            lidar = self.lidar.read((s.x, s.y, s.theta), site, site.people_at(t), dust) if k % 8 == 1 else None
            camera = (self.camera.detect((s.x, s.y, s.theta), site.people_at(t), s.v < -0.05, dust)
                      if k % 4 == 1 else None)
            if self.remote.teleop_active:
                op = self._teleop_operator()
                due = self.remote.delay_command(t, op)
                if due is not None:
                    self.auto.teleop_cmd = due
            force_ratio = self.last_dig.resist_force_n / max(1.0, self.cfg.traction_coefficient * (
                (self.cfg.operating_mass_kg + s.load_t * 1000) * 9.81 + self.last_dig.normal_boost_n))
            signals = {"force_ratio": force_ratio + self.rng.normal(0, 0.03), "stalled": s.stalled,
                       "truck_present": site.truck.present}
            for fid, data in self.auto.tick(t, odo_v, gyro, gnss, lidar, camera, signals):
                self.bus.send(t, fid, data)
            # assistance requests
            if self.auto.assist:
                kind = "ack" if self.auto.assist == "person_on_path" else "teleop"
                if kind == "ack":
                    self.remote.request(t, "reset", self.auto.assist)
                else:
                    self.remote.request(t, "teleop", self.auto.assist)
                self.auto.assist = None
            self.loc_err.append((math.hypot(self.auto.ekf.x[0] - s.x, self.auto.ekf.x[1] - s.y), outage))
        # latched stop that needs a supervisor (anything but a clearing field stop)
        if out.state >= State.PROTECTIVE_STOP and out.reset_pending and not self.remote.busy:
            needs = (out.cause & ~FIELD_CAUSES) != Cause.NONE or self.remote.teleop_active
            long_field = out.cause & FIELD_CAUSES and self._stop_age(t) > 30.0
            if needs or long_field:
                self.remote.request(t, "reset", (out.cause.name or "").lower())

        # --- gateway -> machine, with the PLC's hardwired outputs
        cmd = self.gw.command(t)
        if not out.brake_release:
            cmd = LoaderCommand(speed=0.0, articulation=s.gamma, brake=True)
        if not out.implement_enable:
            cmd.boom = cmd.bucket = 0.0
        self.loader.step(cmd, self.last_dig.resist_force_n, self.last_dig.normal_boost_n,
                         self.interaction.implement_load_n if self.interaction else 0.0)
        self._soil()
        self._dump()
        # --- machine state frames
        self.bus.send(t, *self.mcodec.encode("MachineState1", {
            "Speed": float(np.clip(speed_a, -5.9, 5.9)), "Articulation": float(np.clip(s.gamma, -0.79, 0.79)),
            "Boom": float(np.clip(s.boom, -0.99, 0.99))}))
        if k % 2 == 0:
            payload = max(0.0, s.load_t * 1000 + self.rng.normal(0, 60))
            self.bus.send(t, *self.mcodec.encode("MachineState2", {
                "Bucket": float(np.clip(s.bucket, -0.99, 0.99)), "Payload": min(20000.0, payload),
                "OperatorPresent": 0, "ParkBrake": int(cmd.brake)}))
        self._bookkeeping(out)
        if k % RECORD_EVERY == 0:
            self._record(out, dust, outage, link_ok)
        self.k += 1
        self.t = self.k * SIM_DT

    # -------------------------------------------------------------- physics
    def _soil(self) -> None:
        s = self.loader.state
        tx, ty, tz = self.loader.bucket_tip_world()
        hit = None
        for pile in self.site.piles.values():
            sl = pile.slot_at(ty)
            if sl is not None and tx > sl.toe_x - 0.02:
                hit = (pile, sl)
                break
        if hit is None:
            if self.interaction is not None:
                self._end_dig()
            return
        pile, sl = hit
        pen = tx - sl.toe_x
        if self.interaction is None:
            self.interaction = BucketPileInteraction(pile.soil(), self.cfg.bucket_width, self.cfg.bucket_volume)
            self.interaction.penetration = 0.0
            self.dig_slot = (pile, sl)
            self._dig_z = tz
            self._dig_att = s.bucket
        it = self.interaction
        res = it.step(dx_forward=pen - it.penetration, tip_z=tz, tip_dz=(tz - self._dig_z) / SIM_DT,
                      attitude=s.bucket, attitude_rate=(s.bucket - self._dig_att) / SIM_DT,
                      load_m3=s.load_m3, dt=SIM_DT)
        self._dig_z, self._dig_att = tz, s.bucket
        s.load_m3 = max(0.0, s.load_m3 + res.added_m3 - res.spilled_m3)
        s.load_t = s.load_m3 * pile.material.density_t_m3
        self.dig_removed += res.added_m3 - res.spilled_m3
        self.last_dig = res
        if pen <= 0.0:
            self._end_dig()

    def _end_dig(self) -> None:
        if self.dig_slot is not None and self.dig_removed > 0:
            pile, sl = self.dig_slot
            pile.remove(sl, self.dig_removed)
            self.true_fill.append((pile.name, self.loader.state.load_m3 / self.cfg.bucket_volume))
        self.interaction, self.dig_slot, self.dig_removed = None, None, 0.0
        self.last_dig = DigResult(0.0, 0.0, 0.0, 0.0, 0.0)

    def _dump(self) -> None:
        s = self.loader.state
        if s.bucket > -0.3 or s.load_m3 <= 0.0:
            return
        dv = min(s.load_m3, 2.5 * SIM_DT)
        dens = s.load_t / s.load_m3 if s.load_m3 > 1e-6 else 1.6
        tx, ty, _ = self.loader.bucket_tip_world()
        tr = self.site.truck
        pivot_h = self.cfg.boom_pivot_height + self.cfg.boom_length * math.sin(s.boom)
        over = tr.present and point_in_poly(tx, ty, tr.corners()) and pivot_h > 3.2
        if over:
            self.site.dump_into_truck(self.t, dv * dens)
        else:
            self.spill_t += dv * dens
        s.load_m3 -= dv
        s.load_t = s.load_m3 * dens
        if s.load_m3 <= 1e-6:
            s.load_m3 = s.load_t = 0.0
            self.site.dispatch_truck_if_full(self.t, self.cfg.bucket_volume * 1.7 * 0.95)

    # -------------------------------------------------------------- teleop
    def _start_teleop(self) -> None:
        s = self.loader.state
        # Move away from whatever is blocking: forward if only the rear field is occupied.
        sc = self.scan
        away = 1.0 if (sc is not None and sc.rear_prot and not sc.front_prot) else -1.0
        self._teleop_plan = {"t0": self.t, "x0": s.x, "y0": s.y, "phase": "reset", "dir": away}

    def _teleop_operator(self) -> Command:
        """Operator moves the machine 4 m away from the obstruction; the crew clears it."""
        pl, s = self._teleop_plan, self.loader.state
        out = self.plc_out
        if pl["phase"] == "reset":
            if out.state >= State.PROTECTIVE_STOP:
                self.remote._reset_pulse_until = self.t + 0.2   # operator presses reset
            pl["phase"] = "reverse"
            pl["t_reset"] = self.t
            return Command(brake=True)
        if pl["phase"] == "reverse":
            if out.state >= State.PROTECTIVE_STOP:
                if self.t - pl["t_reset"] > 3.0:          # press reset again
                    self.remote._reset_pulse_until = self.t + 0.2
                    pl["t_reset"] = self.t
                    pl["retries"] = pl.get("retries", 0) + 1
                if pl.get("retries", 0) > 10:              # give up: hand back, log unresolved
                    pl["phase"] = "clear"
                    pl["t_clear"] = self.t - 40.0
                return Command(brake=True)
            d = math.hypot(s.x - pl["x0"], s.y - pl["y0"])
            if d >= 4.0:
                pl["phase"] = "clear"
                pl["t_clear"] = self.t
                return Command(brake=True)
            return Command(speed=0.45 * pl["dir"], articulation=0.0, brake=False,
                           boom=float(np.clip(4 * (-0.2 - s.boom), -1, 1)))
        if pl["phase"] == "clear":
            # Ground crew removes the block while the machine waits.
            wait = 40.0 if self.active_obstacles() else 3.0
            if self.t - pl["t_clear"] > wait:
                for o in self.active_obstacles():
                    self._obstacle_removed.add(o["name"])
                self.remote.end_teleop(self.t)
                self.auto.mode = Mode.AUTO
                self.auto.teleop_cmd = None
                self.auto.planner.resume(self.auto.last_percept)   # replan from where the machine is
            return Command(brake=True)
        return Command(brake=True)

    # ------------------------------------------------------------ bookkeeping
    def _stop_age(self, t: float) -> float:
        return t - self._stop_events[-1]["t"] if self._stop_events else 0.0

    def _track_stops(self, out, fs) -> None:
        s = self.loader.state
        if out.state >= State.PROTECTIVE_STOP and self._prev_state < State.PROTECTIVE_STOP:
            truth = self._objects_in_protective(self.plc.sets[self.plc.field_set],
                                                under=bool(out.cause & Cause.UNDER_BUCKET)) \
                if out.cause & FIELD_CAUSES else []
            ev = {"t": round(self.t, 2), "cause": out.cause.name, "v": round(s.v, 2), "objects": truth,
                  "field": self.plc.sets[self.plc.field_set].name}
            if out.cause & FIELD_CAUSES:
                ev["false"] = not truth
            self._stop_events.append(ev)
        self._prev_state = out.state
        # person-in-field response: from first true entry (while moving) to the stop
        fsx = self.plc.sets[self.plc.field_set]
        moving = abs(s.v) > 0.1
        for n, px, py in self.site.people_at(self.t):
            inside = False
            for body, poly in field_polygons(fsx):
                if poly:
                    lx, ly = self._to_body(body, px, py)
                    if any(points_in_polygon(np.array([lx + ox]), np.array([ly + oy]), poly)[0]
                           for ox, oy in ((0, 0), (0.25, 0), (-0.25, 0), (0, 0.25), (0, -0.25))):
                        inside = True
            if inside and out.state < State.PROTECTIVE_STOP and moving and n not in self._person_in_field_since:
                self._person_in_field_since[n] = self.t
                self.k_out.person_field_entries_while_moving += 1
            if n in self._person_in_field_since and out.state >= State.PROTECTIVE_STOP:
                self.k_out.person_stop_response_s.append(round(self.t - self._person_in_field_since.pop(n), 3))
            if not inside and n in self._person_in_field_since:
                self._person_in_field_since.pop(n)

    def _bookkeeping(self, out) -> None:
        d = self._person_min_distance()
        self.k_out.min_person_distance_m = min(self.k_out.min_person_distance_m, d)
        if abs(self.loader.state.v) > 0.05:
            self.k_out.min_person_distance_moving_m = min(self.k_out.min_person_distance_moving_m, d)
            if self._person_under_raised_bucket():
                self.k_out.person_under_raised_bucket_moving_s += SIM_DT
        if self.auto.planner.phase == Phase.WAIT_TRUCK:
            self.truck_wait += SIM_DT

    def _record(self, out, dust, outage, link_ok) -> None:
        s = self.loader.state
        rx, ry, rth = self.loader.rear_axle()
        tr = self.site.truck
        fr = {
            "t": round(self.t, 2), "x": round(s.x, 2), "y": round(s.y, 2), "th": round(s.theta, 3),
            "g": round(s.gamma, 3), "v": round(s.v, 2), "boom": round(s.boom, 2), "bkt": round(s.bucket, 2),
            "load": round(s.load_m3, 2), "st": int(out.state), "fs": out.field_set,
            "ph": self.auto.planner.phase.value, "mode": int(self.auto.mode),
            "ex": round(float(self.auto.ekf.x[0]), 2), "ey": round(float(self.auto.ekf.x[1]), 2),
            "ppl": [[n, round(x, 2), round(y, 2)] for n, x, y in self.site.people_at(self.t)],
            "tr": [int(tr.present), round(tr.payload_t, 1), tr.serial],
            "dust": dust > 0, "gnss": not outage, "link": link_ok,
            "obs": [[o["x"], o["y"], o["r"]] for o in self.active_obstacles()],
            "lim": round(out.speed_limit, 2),
        }
        if self.k % (RECORD_EVERY * 25) == 0:
            fr["toes"] = {n: [round(sl.toe_x, 2) for sl in p.slots] for n, p in self.site.piles.items()}
        self.frames.append(fr)

    # ------------------------------------------------------------------- run
    def run(self) -> Kpis:
        t0 = time.time()
        n = int(self.minutes * 60 / SIM_DT)
        for _ in range(n):
            self.step()
        return self.kpis(time.time() - t0)

    def kpis(self, wall: float) -> Kpis:
        K = self.k_out
        hours = self.minutes / 60.0
        cyc = self.auto.planner.cycles
        K.sim_minutes = self.minutes
        K.cycles = len(cyc)
        K.tonnes = round(self.site.tonnes_loaded, 1)
        K.trucks = self.site.trucks_completed
        K.tonnes_per_hour = round(K.tonnes / hours, 1)
        K.cycles_per_hour = round(K.cycles / hours, 1)
        dumps = np.array([c.t_dump for c in cyc])
        if len(dumps) > 2:
            ct = np.diff(dumps)
            K.cycle_time_median_s = round(float(np.median(ct)), 1)
            K.cycle_time_p90_s = round(float(np.percentile(ct, 90)), 1)
            stops = [e["t"] for e in self._stop_events]
            clean = [c for c, a, b in zip(ct, dumps[:-1], dumps[1:])
                     if not any(a <= st <= b for st in stops)]
            K.clean_cycle_time_median_s = round(float(np.median(clean)), 1) if clean else 0.0
        digs = [c.t_dig_end - c.t_dig_start for c in cyc if c.t_dig_end > c.t_dig_start > 0]
        K.dig_time_mean_s = round(float(np.mean(digs)), 2) if digs else 0.0
        if self.true_fill:
            K.fill_factor_mean = round(float(np.mean([f for _, f in self.true_fill])), 3)
            for name in sorted({p for p, _ in self.true_fill}):
                K.fill_factor_by_pile[name] = round(float(np.mean([f for p, f in self.true_fill if p == name])), 3)
        K.buckets_per_truck = round(K.cycles / max(1, K.trucks + (1 if self.site.truck.buckets else 0)), 2)
        K.truck_wait_s = round(self.truck_wait, 1)
        iv = self.remote.interventions
        K.interventions = len(iv)
        K.interventions_per_hour = round(len(iv) / hours, 2)
        for i in iv:
            K.interventions_by_reason[i.reason] = K.interventions_by_reason.get(i.reason, 0) + 1
        K.protective_stops = len(self._stop_events)
        for e in self._stop_events:
            K.stops_by_cause[e["cause"]] = K.stops_by_cause.get(e["cause"], 0) + 1
        K.false_stops = sum(1 for e in self._stop_events if e.get("false"))
        K.false_stops_per_hour = round(K.false_stops / hours, 2)
        err = np.array([e for e, _ in self.loc_err])
        out_err = np.array([e for e, o in self.loc_err if o])
        K.loc_rms_m = round(float(np.sqrt(np.mean(err ** 2))), 3)
        K.loc_max_m = round(float(err.max()), 3)
        K.loc_filter_resets = self.auto.ekf.gnss_resets
        K.loc_rms_gnss_outage_m = round(float(np.sqrt(np.mean(out_err ** 2))), 3) if len(out_err) else 0.0
        lat = np.array(self.remote.latency_samples_ms)
        if len(lat):
            K.teleop_latency_p50_ms = round(float(np.percentile(lat, 50)), 1)
            K.teleop_latency_p95_ms = round(float(np.percentile(lat, 95)), 1)
        K.fuel_l = round(self.loader.state.fuel_l, 2)
        K.fuel_l_per_t = round(K.fuel_l / max(1e-6, K.tonnes), 3)
        K.spill_t = round(self.spill_t, 2)
        K.e2e_errors = self.gw.stats.e2e_errors
        K.gateway_timeouts = self.gw.stats.timeouts
        K.log_chain_ok = self.plc.log.verify() is None
        K.log_records = len(self.plc.log.records)
        K.min_person_distance_m = round(K.min_person_distance_m, 2)
        K.min_person_distance_moving_m = round(K.min_person_distance_moving_m, 2)
        K.person_under_raised_bucket_moving_s = round(K.person_under_raised_bucket_moving_s, 2)
        K.wall_time_s = round(wall, 1)
        return K


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", choices=("learned", "scripted"), default="learned")
    ap.add_argument("--minutes", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--winter", action="store_true")
    ap.add_argument("--guard", action="store_true", help="enable field-aware speed limiting (AR-04)")
    ap.add_argument("--careless", action="store_true", help="pedestrians come within 0.3 m (stress case)")
    ap.add_argument("--out", type=Path, default=Path("results/shift"))
    args = ap.parse_args()
    sim = ShiftSim(args.controller, args.minutes, args.seed, args.winter, guard=args.guard,
                   careless=args.careless)
    k = sim.run()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "kpis.json").write_text(json.dumps(asdict(k), indent=1))
    (args.out / "stops.json").write_text(json.dumps(sim._stop_events, indent=1))
    (args.out / "safety_log.json").write_text(json.dumps(sim.plc.log.records, indent=1))
    (args.out / "replay.json").write_text(json.dumps({
        "frames": sim.frames, "site": {
            "area": sim.site.operating_area, "walls": sim.site.walls,
            "piles": {n: {"slots": [[sl.y_c, sl.width] for sl in p.slots], "back_x": p.back_x,
                          "material": p.material.name} for n, p in sim.site.piles.items()},
            "dig_zones": sim.site.dig_zones, "dock_zone": sim.site.dock_zone,
            "truck": [sim.site.truck.cx, sim.site.truck.cy, sim.site.truck.length, sim.site.truck.width],
            "landmarks": sim.site.landmarks},
        "fields": {fid: {"name": f.name, "dir": f.direction, "prot": f.protective, "warn": f.warning,
                         "rprot": f.protective_for("rear"), "rwarn": f.warning_for("rear"), "rside": f.side_rear,
                         "side": f.side} for fid, f in sim.plc.sets.items()},
        "machine": {"lf": sim.cfg.lf, "lr": sim.cfg.lr, "width": sim.cfg.body_width},
        "stops": sim._stop_events,
        "interventions": [asdict(i) for i in sim.remote.interventions],
    }, separators=(",", ":")))
    print(json.dumps(asdict(k), indent=1))


if __name__ == "__main__":
    main()
