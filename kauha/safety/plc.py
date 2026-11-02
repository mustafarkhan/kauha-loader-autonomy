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

from kauha.safety import core
from kauha.safety.fields import FieldSet, build_field_sets, field_table
from kauha.safety.logger import EventLog
from kauha.safety.stopping import StopBudget

CYCLE_S = core.CYCLE_S


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
    UNDER_BUCKET = 1 << 14


FIELD_CAUSES = Cause.FIELD_FRONT | Cause.FIELD_REAR | Cause.FIELD_SIDE | Cause.UNDER_BUCKET
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
    boom_raised: bool = False            # SR-16: PLC's own boom-angle sensor: bucket bottom above 1.0 m
    front_side: bool = False             # SR-16: front-frame side fields (every set, own output)
    under_near: bool = False             # SR-16: under-bucket field 2.0-2.8 m ahead of the front axle
    under_far: bool = False              # SR-16: under-bucket field 2.8-5.1 m
    under_mid: bool = False              # SR-16: gap field 2.8-3.35 m, short of the lowered bucket
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
    """The safety PLC: memory plus the pure step function in ``core.py``.

    ``core.step`` is the single source of the safety logic. This class runs it
    on concrete values and turns its outputs and log events into the public
    types; ``kauha.verify`` runs the same function symbolically.
    """
    params: SafetyParams = field(default_factory=SafetyParams)
    budget: StopBudget = field(default_factory=StopBudget)
    log: EventLog = field(default_factory=EventLog)

    def __post_init__(self) -> None:
        self.sets: dict[int, FieldSet] = build_field_sets(self.budget)
        self.table = field_table(self.budget)
        self.field_cap = self.table.field_cap
        self.mem = core.Memory()

    # read-only views used by the simulation and tests
    @property
    def state(self) -> State:
        return State(self.mem.state)

    @property
    def latched(self) -> Cause:
        return Cause(self.mem.latched)

    @property
    def field_set(self) -> int:
        return self.mem.field_set

    @property
    def recovery(self) -> Cause:
        return Cause(self.mem.recovery)

    def step(self, i: SafetyInputs) -> SafetyOutputs:
        self.mem, o = core.step(core.CONCRETE, self.params, self.table, self.mem, i)
        for cond, kind, data in o.events:
            if cond:
                self.log.append(i.t, kind, **self._log_data(kind, data))
        return SafetyOutputs(state=State(o.state), cause=Cause(o.cause), speed_limit=o.speed_limit,
                             field_set=o.field_set, brake_release=o.brake_release,
                             implement_enable=o.implement_enable, reset_pending=o.reset_pending,
                             dock_blocked=o.dock_blocked)

    def _log_data(self, kind: str, data: dict) -> dict:
        out = {}
        for k, val in data.items():
            if k == "state":
                val = State(val).name
            elif k in ("cause", "fields", "cleared"):
                c = Cause(val)
                val = c.name or str(int(c))
            elif k == "field" and isinstance(val, int):
                val = self.sets[val].name
            elif isinstance(val, float):
                val = round(val, 3)
            out[k] = val
        return out
