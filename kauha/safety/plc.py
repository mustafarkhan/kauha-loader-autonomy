"""Safety PLC logic: the independent, deterministic safety channel.

Runs every 25 ms. It contains no learned components and does not depend on
the autonomy computer for any decision: it reads the scanners' field outputs,
its own speed and articulation sensors, the remote station heartbeat, the
safety GNSS geofence result and the hardwired e-stops. Its stop outputs are
hardwired to the brake and hydraulic pilot cut-off (see docs/hardware), so a
stop does not need the CAN bus or the autonomy computer to work.

The autonomy computer appears only as a *monitored* input: its command
heartbeat (watchdog) and its requests for the dig/dock field sets, which the
PLC grants only when its own zone and speed checks allow.

Each requirement it implements is tagged SR-xx; tests/safety trace them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

from kauha.safety.fields import (
    DIG_SPEED,
    DOCK_SPEED,
    STANDSTILL,
    FieldSet,
    build_field_sets,
    field_speed_cap,
    select_field_set,
)
from kauha.safety.logger import EventLog
from kauha.safety.stopping import StopBudget

CYCLE_S = 0.025


class State(IntEnum):
    RUN = 0
    WARNING = 1
    PROTECTIVE_STOP = 2
    EMERGENCY_STOP = 3


class Mode(IntEnum):
    IDLE = 0
    AUTO = 1
    TELEOP = 2


class Cause(IntFlag):
    NONE = 0
    FIELD_FRONT = 1 << 0
    FIELD_REAR = 1 << 1
    FIELD_SIDE = 1 << 2
    OVERSPEED = 1 << 3
    REMOTE_LINK = 1 << 4
    WATCHDOG = 1 << 5
    GEOFENCE = 1 << 6
    SPEED_DISCREPANCY = 1 << 7
    SCANNER_FAULT = 1 << 8
    MUTING_TIMEOUT = 1 << 9
    ESTOP_MACHINE = 1 << 10
    ESTOP_REMOTE = 1 << 11
    POSITION_INVALID = 1 << 12
    TRUCK_DOOR = 1 << 13


FIELD_CAUSES = Cause.FIELD_FRONT | Cause.FIELD_REAR | Cause.FIELD_SIDE
# Causes that clear by themselves (the object or person moves away); eligible for SR-11 auto restart.
CLEARING_CAUSES = FIELD_CAUSES | Cause.TRUCK_DOOR


@dataclass(frozen=True)
class SafetyParams:
    remote_timeout_s: float = 0.5        # SR-05
    watchdog_timeout_s: float = 0.2      # SR-06
    overspeed_tol: float = 0.2           # SR-04
    decel_ramp: float = 0.8              # SR-04: guaranteed deceleration after a limit drops, m/s^2
    ramp_reaction_s: float = 0.3         # actuator latency + gateway + PLC cycles before the ramp
    discrepancy_tol: float = 0.3         # SR-08
    discrepancy_time_s: float = 0.2
    position_invalid_s: float = 2.0      # SR-07
    max_mute_s: float = 25.0             # SR-10
    auto_restart_clear_s: float | None = 3.0   # SR-11; None = supervisor reset always
    warning_hold_s: float = 2.0
    auto_speed: float = 3.0
    teleop_speed: float = 3.0
    warning_speed: float = 1.0
    standstill: float = 0.05
    recovery_speed: float = 0.5          # SR-12


@dataclass
class SafetyInputs:
    t: float
    mode: Mode
    speed_a: float                      # drive-by-wire gateway (non-safety source)
    speed_b: float                      # PLC's own driveline speed sensor
    articulation: float                 # PLC's own articulation sensor
    front_prot: bool = False
    front_warn: bool = False
    rear_prot: bool = False
    rear_warn: bool = False
    side_prot: bool = False
    side_contact: bool = False           # SR-12: someone (or something) right beside the frame
    scanners_ok: bool = True
    remote_hb_age: float = 0.0
    remote_estop: bool = False
    reset_req: bool = False
    machine_estop: bool = False
    autonomy_cmd_age: float = 0.0
    dig_req: bool = False
    dock_req: bool = False
    in_operating_area: bool = True
    in_dig_zone: bool = False
    in_dock_zone: bool = False
    truck_door_open: bool = False        # SR-14: cab-door switch of the truck in the bay
    position_valid: bool = True


@dataclass
class SafetyOutputs:
    state: State
    cause: Cause
    speed_limit: float
    field_set: int
    brake_release: bool          # False => service + park brake applied, drive cut (hardwired)
    implement_enable: bool       # False => hydraulic pilot supply cut (hardwired)
    reset_pending: bool
    dock_blocked: bool = False           # SR-14: tells the autonomy to hold off docking


@dataclass
class SafetyPLC:
    params: SafetyParams = field(default_factory=SafetyParams)
    budget: StopBudget = field(default_factory=StopBudget)
    log: EventLog = field(default_factory=EventLog)

    def __post_init__(self) -> None:
        self.sets: dict[int, FieldSet] = build_field_sets(self.budget)
        self.field_cap = field_speed_cap(self.budget)
        self.state = State.RUN
        self.latched = Cause.NONE
        self.field_set = 0
        self._prev_reset = False
        self._over_since: float | None = None
        self._disc_since: float | None = None
        self._posinv_since: float | None = None
        self._mute_since: float | None = None
        self._clear_since: float | None = None
        self._warn_until = -1.0
        self._allowed = 0.0                  # monitored speed envelope (SR-04)
        self._prev_limit = 0.0
        self._env_t0 = -1e9
        self._env_v0 = 0.0
        self.recovery = Cause.NONE     # SR-12: occupied fields the operator is backing away from

    # ----------------------------------------------------------------- helpers
    def _trip(self, t: float, cause: Cause, emergency: bool = False, **data) -> None:
        new = cause & ~self.latched
        self.latched |= cause
        target = State.EMERGENCY_STOP if emergency else State.PROTECTIVE_STOP
        if target > self.state or new:
            if target > self.state:
                self.state = target
            self.log.append(t, "stop", state=self.state.name, cause=cause.name or str(int(cause)), **data)

    def _mode_limit(self, mode: Mode) -> float:
        p = self.params
        if mode == Mode.AUTO:
            return min(p.auto_speed, self.field_cap)
        if mode == Mode.TELEOP:
            return min(p.teleop_speed, self.field_cap)
        return 0.0

    # -------------------------------------------------------------------- step
    def step(self, i: SafetyInputs) -> SafetyOutputs:
        p, t = self.params, i.t
        v = i.speed_b
        rising_reset = i.reset_req and not self._prev_reset
        self._prev_reset = i.reset_req

        # SR-01 emergency stops (machine buttons, remote e-stop): latch, category 0/1.
        if i.machine_estop:
            self._trip(t, Cause.ESTOP_MACHINE, emergency=True)
        if i.remote_estop:
            self._trip(t, Cause.ESTOP_REMOTE, emergency=True)

        # Field-set selection (SR-02, SR-10). Dig/dock sets need zone + speed agreement.
        dig_ok = i.dig_req and i.in_dig_zone and abs(v) <= DIG_SPEED + p.overspeed_tol and i.position_valid
        dock_ok = (i.dock_req and i.in_dock_zone and abs(v) <= DOCK_SPEED + p.overspeed_tol and i.position_valid
                   and not i.truck_door_open)
        # SR-14: the driver leaving the cab while the loader is docking stops it; the
        # raised bucket would otherwise pass over a person standing by the truck.
        if i.truck_door_open and self.field_set == 101 and i.in_dock_zone:
            self._trip(t, Cause.TRUCK_DOOR)
        if dig_ok and v > -STANDSTILL:          # the front field is actually muted
            self._mute_since = t if self._mute_since is None else self._mute_since
            if t - self._mute_since > p.max_mute_s:
                self._trip(t, Cause.MUTING_TIMEOUT)
                dig_ok = False
        else:
            self._mute_since = None
        fs_id = select_field_set(self.sets, v, i.articulation, dig_ok, dock_ok)
        if fs_id != self.field_set:
            self.field_set = fs_id
        fs = self.sets[fs_id]

        # SR-12 recovery: after a supervisor reset in TELEOP, motion *away* from an
        # occupied field is allowed at creep speed; motion toward it trips again.
        if self.recovery:
            if i.mode != Mode.TELEOP:
                self.recovery = Cause.NONE
            else:
                # A field counts as clear only at standstill, where the standstill
                # set watches front, rear and sides together; while moving, the
                # scanners only evaluate the direction of travel.
                at_rest = abs(v) < p.standstill
                if at_rest and Cause.FIELD_FRONT in self.recovery and not i.front_prot:
                    self.recovery &= ~Cause.FIELD_FRONT
                if at_rest and Cause.FIELD_REAR in self.recovery and not i.rear_prot:
                    self.recovery &= ~Cause.FIELD_REAR
                if at_rest and Cause.FIELD_SIDE in self.recovery and not i.side_prot:
                    self.recovery &= ~Cause.FIELD_SIDE
                # Beside the frame no direction is "away": the bucket is wider than the
                # front frame and slides past a person standing there either way.
                if Cause.FIELD_SIDE in self.recovery and i.side_contact and not self._band_exempt(i):
                    self._trip(t, Cause.FIELD_SIDE, field="recovery_contact", v=round(v, 2))
                if Cause.FIELD_FRONT in self.recovery and v > p.standstill:
                    self._trip(t, Cause.FIELD_FRONT, field="recovery", v=round(v, 2))
                if Cause.FIELD_REAR in self.recovery and v < -p.standstill:
                    self._trip(t, Cause.FIELD_REAR, field="recovery", v=round(v, 2))

        # SR-02 protective fields. The scanners evaluate the *selected* set.
        if i.front_prot and fs.direction >= 0 and Cause.FIELD_FRONT not in self.recovery:
            self._trip(t, Cause.FIELD_FRONT, field=fs.name, v=round(v, 2))
        if i.rear_prot and (fs.direction <= 0 or abs(v) < p.standstill) and Cause.FIELD_REAR not in self.recovery:
            self._trip(t, Cause.FIELD_REAR, field=fs.name, v=round(v, 2))
        if i.side_prot and abs(v) > p.standstill and Cause.FIELD_SIDE not in self.recovery:
            self._trip(t, Cause.FIELD_SIDE, field=fs.name, v=round(v, 2))
        if not i.scanners_ok:
            self._trip(t, Cause.SCANNER_FAULT)

        # SR-05 remote supervision link, SR-06 autonomy watchdog.
        if i.mode in (Mode.AUTO, Mode.TELEOP) and i.remote_hb_age > p.remote_timeout_s:
            self._trip(t, Cause.REMOTE_LINK, age=round(i.remote_hb_age, 3))
        if i.mode in (Mode.AUTO, Mode.TELEOP) and i.autonomy_cmd_age > p.watchdog_timeout_s:
            self._trip(t, Cause.WATCHDOG, age=round(i.autonomy_cmd_age, 3))

        # SR-07 geofence on the safety GNSS; invalid position tolerated briefly.
        if i.mode == Mode.AUTO:
            if not i.position_valid:
                self._posinv_since = t if self._posinv_since is None else self._posinv_since
                if t - self._posinv_since > p.position_invalid_s:
                    self._trip(t, Cause.POSITION_INVALID)
            else:
                self._posinv_since = None
                if not i.in_operating_area:
                    self._trip(t, Cause.GEOFENCE)

        # SR-08 two-channel speed plausibility.
        if abs(i.speed_a - i.speed_b) > p.discrepancy_tol:
            self._disc_since = t if self._disc_since is None else self._disc_since
            if t - self._disc_since > p.discrepancy_time_s:
                self._trip(t, Cause.SPEED_DISCREPANCY, a=round(i.speed_a, 2), b=round(v, 2))
        else:
            self._disc_since = None

        # SR-03 warning field -> speed limit (non-latching). Inside the access-
        # controlled dig and dock zones the warning field is not used: there it
        # only ever sees the stockpile or the truck, and the zones exclude people
        # on foot. Protective fields are unaffected.
        in_zone = i.in_dig_zone or i.in_dock_zone
        if not in_zone and ((i.front_warn and fs.direction >= 0) or (i.rear_warn and fs.direction <= 0)):
            self._warn_until = t + p.warning_hold_s
        warning = t < self._warn_until

        # SR-04 speed limit and overspeed monitoring.
        limit = self._mode_limit(i.mode)
        if warning:
            limit = min(limit, p.warning_speed)
        if self.recovery:
            limit = min(limit, p.recovery_speed if not (self.recovery & Cause.FIELD_SIDE) else 0.3)
        if fs_id == 100:
            limit = min(limit, DIG_SPEED)
        elif fs_id == 101:
            limit = min(limit, DOCK_SPEED)
        # The monitored envelope follows a limit increase at once. After a
        # decrease it holds the current speed for the stop reaction time, then
        # ramps down at the guaranteed deceleration (SS1-r style monitoring).
        if limit < self._prev_limit - 1e-9:
            self._env_t0 = t
            self._env_v0 = min(self._allowed, abs(v))
        self._prev_limit = limit
        dt_env = t - self._env_t0
        ramp = self._env_v0 - p.decel_ramp * max(0.0, dt_env - p.ramp_reaction_s)
        self._allowed = max(limit, ramp) if ramp > limit else limit
        if abs(v) > self._allowed + p.overspeed_tol:
            self._over_since = t if self._over_since is None else self._over_since
            if t - self._over_since >= CYCLE_S:
                self._trip(t, Cause.OVERSPEED, v=round(v, 2), limit=round(limit, 2),
                           envelope=round(self._allowed, 2))
        else:
            self._over_since = None

        # Reset handling (SR-09, SR-11).
        standstill = abs(v) < p.standstill
        active = self._active_causes(i, fs)
        if self.state >= State.PROTECTIVE_STOP:
            remaining = self.latched & active
            if (rising_reset and i.mode == Mode.TELEOP and self.state == State.PROTECTIVE_STOP
                    and remaining and not (remaining & ~FIELD_CAUSES)
                    and not (remaining & Cause.FIELD_SIDE and i.side_contact and not self._band_exempt(i))):
                # SR-12: enter recovery instead of refusing the reset.
                self.recovery = remaining
                self.log.append(t, "recovery", fields=remaining.name or str(int(remaining)))
                remaining = Cause.NONE
            if not remaining:
                self._clear_since = t if self._clear_since is None else self._clear_since
            else:
                self._clear_since = None
            auto_ok = (self.state == State.PROTECTIVE_STOP
                       and p.auto_restart_clear_s is not None
                       and self.latched & ~CLEARING_CAUSES == Cause.NONE
                       and self._clear_since is not None
                       and t - self._clear_since >= p.auto_restart_clear_s
                       and not warning)
            if standstill and not remaining and (rising_reset or auto_ok):
                self.log.append(t, "reset", by="auto" if auto_ok and not rising_reset else "supervisor",
                                cleared=self.latched.name or str(int(self.latched)))
                self.state = State.RUN
                self.latched = Cause.NONE
                self._clear_since = None
        if self.state < State.PROTECTIVE_STOP:
            self.state = State.WARNING if warning else State.RUN

        stopped = self.state >= State.PROTECTIVE_STOP
        return SafetyOutputs(
            state=self.state, cause=self.latched, speed_limit=0.0 if stopped else limit,
            field_set=fs_id, brake_release=not stopped,
            # SR-12: no boom/bucket motion while recovering from an occupied field,
            # so a load is never lowered over the person who caused the stop.
            implement_enable=not stopped and not self.recovery,
            reset_pending=stopped and not (self.latched & self._active_causes(i, fs)),
            dock_blocked=i.truck_door_open,
        )

    @staticmethod
    def _band_exempt(i: SafetyInputs) -> bool:
        """Inside the access-controlled zones the object in the contact band is taken
        to be the pile flank (dig zone) or the truck (dock zone, driver in the cab).
        Same organisational measure the dig-field muting already relies on (SR-10)."""
        return i.in_dig_zone or (i.in_dock_zone and not i.truck_door_open)

    def _active_causes(self, i: SafetyInputs, fs: FieldSet) -> Cause:
        """Causes whose condition is still present (blocks reset)."""
        p = self.params
        c = Cause.NONE
        if i.machine_estop:
            c |= Cause.ESTOP_MACHINE
        if i.remote_estop:
            c |= Cause.ESTOP_REMOTE
        if i.front_prot:
            c |= Cause.FIELD_FRONT
        if i.rear_prot:
            c |= Cause.FIELD_REAR
        if i.side_prot:
            c |= Cause.FIELD_SIDE
        if not i.scanners_ok:
            c |= Cause.SCANNER_FAULT
        if i.mode != Mode.IDLE and i.remote_hb_age > p.remote_timeout_s:
            c |= Cause.REMOTE_LINK
        if i.mode != Mode.IDLE and i.autonomy_cmd_age > p.watchdog_timeout_s:
            c |= Cause.WATCHDOG
        if i.mode == Mode.AUTO and i.position_valid and not i.in_operating_area:
            c |= Cause.GEOFENCE
        if abs(i.speed_a - i.speed_b) > p.discrepancy_tol:
            c |= Cause.SPEED_DISCREPANCY
        if i.mode == Mode.AUTO and not i.position_valid:
            c |= Cause.POSITION_INVALID
        if i.truck_door_open and i.in_dock_zone:
            c |= Cause.TRUCK_DOOR
        return c
