"""The autonomy computer: localisation, perception, planning, supervision.

Runs at 20 Hz. Talks to the machine only through CAN frames (DriveCmd,
ImplementCmd out; SafetyStatus, MachineState in) and reads its own sensors
(GNSS, IMU, lidar, cameras) directly. Nothing here is safety-rated; the
supervisor's job is to degrade gracefully and ask for help early, so the
safety channel is the last line rather than the first.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kauha.autonomy.field_guard import FieldGuard
from kauha.autonomy.localization import EKF
from kauha.autonomy.perception import PeopleTracker, PileMap, TruckTracker
from kauha.autonomy.planner import Command, CyclePlanner, Percept
from kauha.can.codec import Codec
from kauha.config import MachineConfig
from kauha.kinematics import bucket_tip_local
from kauha.safety.plc import Mode
from kauha.safety.stopping import StopBudget
from kauha.train.bucket_env import MAX_T

DT = 0.05


@dataclass
class SupervisorState:
    speed_cap: float = 3.0
    reason: str = ""


@dataclass
class AutonomyComputer:
    cfg: MachineConfig
    site: object                 # the site *map* (survey); not ground truth during the run
    bucket_controller: object
    pile_schedule: list[tuple[float, str]]
    codec: Codec = field(default_factory=Codec)
    mode: Mode = Mode.AUTO
    teleop_cmd: Command | None = None
    budget: StopBudget = field(default_factory=StopBudget)   # same stop budget the PLC uses
    guard_enabled: bool = False   # measured: costs throughput once fields and zones are right

    def __post_init__(self) -> None:
        s = self.site
        self.ekf = EKF(landmarks=list(s.landmarks))
        self.ekf.init(*s.start_pose)
        self.piles = PileMap.from_site(s)
        tr = s.truck
        self.truck = TruckTracker((tr.cx, tr.cy, tr.heading))
        self.people = PeopleTracker()
        self.planner = CyclePlanner(self.cfg, self.piles, self.truck, self.people, self.bucket_controller,
                                    dock_offset=s.dock_offset, pile_schedule=self.pile_schedule,
                                    area=list(s.operating_area),
                                    guard=FieldGuard(self.budget) if self.guard_enabled else None)
        self._map_t = -1e9
        self.sup = SupervisorState()
        self.machine: dict = {}
        self.safety: dict = {}
        self.assist: str | None = None
        self.stop_sites: list[tuple[float, float, float]] = []
        self._prev_state = 0
        self._stopped_since = 0.0
        self._asked_for_this_stop = False
        self.densities = {n: p.material.density_t_m3 for n, p in s.piles.items()}

    def _update_guard_map(self, truck_present: bool) -> None:
        """Static objects the safety fields can see, from the autonomy's own map."""
        s = self.site
        segs = list(s.walls)
        for name, slots in self.piles.slots.items():
            back = s.piles[name].back_x
            off = 0.5 / math.tan(math.radians(self.piles.repose_deg[name]))
            for sl in slots:
                x = sl.toe_x + off
                y0, y1 = sl.y_c - sl.width / 2, sl.y_c + sl.width / 2
                segs += [(x, y0, x, y1), (x, y0, back, y0), (x, y1, back, y1)]
        if truck_present:
            tx, ty, th = self.truck.best()
            tr = s.truck
            c, sn = math.cos(th), math.sin(th)
            hl, hw = tr.length / 2, tr.width / 2
            corners = ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))
            pts = [(tx + c * dx - sn * dy, ty + sn * dx + c * dy) for dx, dy in corners]
            segs += [(*pts[i], *pts[(i + 1) % 4]) for i in range(4)]
        self.planner.guard.set_map(segs, list(s.landmarks))

    # ---------------------------------------------------------------- inputs
    def receive(self, frame_id: int, data: bytes) -> None:
        try:
            name, sig = self.codec.decode(frame_id, data)
        except KeyError:
            return
        if name == "SafetyStatus":
            self.safety = sig
        elif name in ("MachineState1", "MachineState2"):
            self.machine.update(sig)

    def dig_obs(self, p: Percept, pile: str, slot: int, t_dig: float) -> np.ndarray:
        s = self.piles.slots[pile][slot]
        tx, tz = bucket_tip_local(self.cfg, p.boom, p.bucket)
        pen = p.x + tx * math.cos(p.th) - s.toe_x
        depth = self.piles.surface(pile, pen) - tz
        return np.array([pen, tz, p.boom, p.bucket, p.v, p.force_ratio, p.load_frac,
                         t_dig / MAX_T, float(p.stalled), depth], dtype=np.float32)

    # ------------------------------------------------------------------ step
    def tick(self, t: float, odo_v: float, gyro: float, gnss, lidar, camera, signals: dict) -> list[tuple[int, bytes]]:
        ekf = self.ekf
        slipping = bool(signals.get("stalled", False)) or signals.get("force_ratio", 0.0) > 0.6
        ekf.predict(odo_v, gyro, DT, slipping)
        if gnss is not None:
            ekf.update_gnss(gnss.x, gnss.y, gnss.heading, gnss.sigma_xy, gnss.sigma_h, gnss.rtk)
        if lidar is not None:
            for lid, r, b in lidar.landmarks:
                ekf.update_landmark(lid, r, b)
            self.piles.observe(t, lidar.face)
            self.truck.observe(t, lidar.truck)
            self.people.observe(t, lidar.people)
        if camera is not None:
            self.people.observe(t, camera, source="camera")

        x, y, th = ekf.x
        gamma = self.machine.get("Articulation", 0.0)
        W = self.cfg.lf
        jx, jy = x - W * math.cos(th), y - W * math.sin(th)
        thr = th - gamma
        rear = (jx - self.cfg.lr * math.cos(thr), jy - self.cfg.lr * math.sin(thr), thr)
        pile = self.planner.pile
        dens = self.densities.get(pile, 1.6)
        payload_m3 = self.machine.get("Payload", 0.0) / 1000.0 / dens
        state = int(self.safety.get("State", 3))
        p = Percept(t, x, y, th, rear, odo_v, gamma, self.machine.get("Boom", -0.62),
                    self.machine.get("Bucket", 0.0), payload_m3 / self.cfg.bucket_volume,
                    signals.get("force_ratio", 0.0), signals.get("stalled", False), state,
                    min(self.safety.get("SpeedLimit", 0.0), self.sup.speed_cap),
                    int(self.safety.get("ActiveFieldSet", 0)), signals.get("truck_present", False),
                    ekf.sigma_xy, bool(self.safety.get("DockBlocked", 0)))

        # Supervisor: localisation quality -> speed cap or hold.
        sig = ekf.sigma_xy
        if sig > 0.6:
            self.sup = SupervisorState(0.0, "localisation_lost")
            self.assist = "localisation_lost"
        elif sig > 0.25:
            self.sup = SupervisorState(1.5, "localisation_degraded")
        else:
            self.sup = SupervisorState(3.0, "")

        # A stop that does not clear, or repeated stops at one place, means
        # something unmapped is in the way: ask the remote operator.
        if state >= 2:
            self._stopped_since = t if self._prev_state < 2 else self._stopped_since
            if t - self._stopped_since > 20.0 and not self._asked_for_this_stop and self.mode == Mode.AUTO:
                self.assist = "blocked_path"
                self._asked_for_this_stop = True
        else:
            self._asked_for_this_stop = False
        if state >= 2 and self._prev_state < 2:
            self.stop_sites.append((t, x, y))
            recent = [s for s in self.stop_sites if t - s[0] < 180 and math.hypot(s[1] - x, s[2] - y) < 6]
            if len(recent) >= 3:
                self.assist = "blocked_path"
        self._prev_state = state

        if self.mode == Mode.TELEOP and self.teleop_cmd is not None:
            cmd = self.teleop_cmd
        elif state >= 2:
            cmd = Command(brake=True)
        else:
            if self.planner.guard is not None and t - self._map_t > 1.0:
                self._update_guard_map(signals.get("truck_present", False))
                self._map_t = t
            cmd = self.planner.tick(p, self.dig_obs)
            if self.planner.assist_request:
                self.assist = self.planner.assist_request
                self.planner.assist_request = None
            if self.sup.speed_cap <= 0.0:
                cmd = Command(brake=True, dig_req=cmd.dig_req, dock_req=cmd.dock_req,
                              boom=cmd.boom, bucket=cmd.bucket)
        self.last_percept = p
        self.last_cmd = cmd
        frames = [
            self.codec.encode("DriveCmd", {
                "SpeedReq": float(np.clip(cmd.speed, -5.9, 5.9)),
                "ArticulationReq": float(np.clip(cmd.articulation, -0.79, 0.79)),
                "Mode": int(self.mode), "BrakeReq": int(cmd.brake),
                "DigFieldReq": int(cmd.dig_req), "DockFieldReq": int(cmd.dock_req)}),
            self.codec.encode("ImplementCmd", {
                "BoomRate": float(np.clip(cmd.boom, -1, 1)), "BucketRate": float(np.clip(cmd.bucket, -1, 1)),
                "Hold": 0}),
        ]
        return frames
