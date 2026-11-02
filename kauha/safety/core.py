"""The safety PLC's step function, written once and executed two ways.

``step`` is pure: it takes the PLC memory and one cycle's inputs and returns the
new memory, the outputs and the log events. All branching goes through an
``Ops`` object instead of Python ``if``/``and``/``or``:

* ``CONCRETE`` evaluates on floats, ints and bools. ``SafetyPLC`` in ``plc.py``
  wraps it; this is the code the simulation, the SIL processes and CI run.
* ``kauha.verify.symbolic.Z3Ops`` evaluates the *same function* on z3 terms.
  The result is a formula for one PLC cycle over every possible input and
  memory state, and ``kauha.verify.prove`` proves the safety requirements on it.

So the proofs are about this code, not about a separate model of it. The
remaining gaps are stated in ``docs/safety/verification.md``: real arithmetic
instead of IEEE floats, and a Python reference that a certified PLC
toolchain still has to re-implement.

Rules for editing: no Python ``if``/``and``/``or``/``not``/``min``/``max``/
``abs`` on values that depend on inputs or memory; use ``O.If``, ``O.And``,
``O.Or``, ``O.Not``, ``O.Min``, ``O.Max``, ``O.Abs``. Causes are bit masks;
use ``O.has`` and ``O.nz`` to test them.
"""
from __future__ import annotations

from dataclasses import dataclass, field

# ------------------------------------------------------------------ constants
RUN, WARNING, PROTECTIVE_STOP, EMERGENCY_STOP = 0, 1, 2, 3
IDLE, AUTO, TELEOP = 0, 1, 2

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
CAUSE_BITS = 16

FIELD_CAUSES = FIELD_FRONT | FIELD_REAR | FIELD_SIDE | UNDER_BUCKET
CLEARING_CAUSES = FIELD_CAUSES | TRUCK_DOOR

CYCLE_S = 0.025
DIG_SET, DOCK_SET, STANDSTILL_SET = 100, 101, 0


# ----------------------------------------------------------------------- ops
class ConcreteOps:
    """Plain Python evaluation."""

    @staticmethod
    def If(c, a, b):  # noqa: N802
        return a if c else b

    @staticmethod
    def And(*xs):  # noqa: N802
        return all(xs)

    @staticmethod
    def Or(*xs):  # noqa: N802
        return any(xs)

    @staticmethod
    def Not(x):  # noqa: N802
        return not x

    @staticmethod
    def Abs(x):  # noqa: N802
        return abs(x)

    @staticmethod
    def Min(a, b):  # noqa: N802
        return a if a <= b else b

    @staticmethod
    def Max(a, b):  # noqa: N802
        return a if a >= b else b

    @staticmethod
    def bv(x: int):
        return x

    @staticmethod
    def nz(x) -> bool:
        return x != 0

    @staticmethod
    def has(x, bits: int) -> bool:
        return (x & bits) != 0

    @staticmethod
    def clear(x, bits: int):
        return x & ~bits


CONCRETE = ConcreteOps()


# --------------------------------------------------------------------- types
@dataclass
class Memory:
    """Everything the PLC remembers between cycles. Timers are (running, start) pairs."""
    state: object = RUN
    latched: object = 0
    field_set: object = STANDSTILL_SET
    recovery: object = 0
    prev_reset: object = False
    over_on: object = False
    over_t0: object = 0.0
    disc_on: object = False
    disc_t0: object = 0.0
    posinv_on: object = False
    posinv_t0: object = 0.0
    mute_on: object = False
    mute_t0: object = 0.0
    clear_on: object = False
    clear_t0: object = 0.0
    warn_until: object = -1.0
    allowed: object = 0.0
    prev_limit: object = 0.0
    env_t0: object = -1e9
    env_v0: object = 0.0


@dataclass(frozen=True)
class FieldTable:
    """What the step function needs to know about the loaded field sets."""
    bands: tuple[float, ...]            # top speed of each band
    sectors: tuple[float, ...]          # articulation sector boundaries (rad)
    n_sectors: int
    standstill: float                   # below this the standstill set is selected
    dig_speed: float
    dock_speed: float
    v_protected: tuple[float, ...]      # highest speed each band's protective field stops from
    field_cap: float                    # highest speed the scanners can protect at all
    speed_cap: float                    # highest speed limit the PLC grants: field_cap less the
                                        # overspeed tolerance, so the tolerance band stays protected


@dataclass
class Out:
    state: object
    cause: object
    speed_limit: object
    field_set: object
    brake_release: object
    implement_enable: object
    reset_pending: object
    dock_blocked: object
    fs_direction: object = 0
    fs_v_protected: object = 0.0
    events: list = field(default_factory=list)


# ------------------------------------------------------------- field select
def select(O, T: FieldTable, v, gamma, dig, dock):
    """Field-set selection (SR-02, SR-10): returns (set id, direction, protected speed)."""
    dig_sel = O.And(dig, v > -T.standstill)          # reversing out of the pile uses the rear fields
    dock_sel = O.And(O.Not(dig_sel), dock, v > -T.standstill)
    still = O.Abs(v) < T.standstill
    a = O.Abs(v)
    lo = T.sectors
    sector = O.If(gamma < lo[0], 0, O.If(gamma < lo[1], 1, O.If(gamma < lo[2], 2, O.If(gamma < lo[3], 3, 4))))
    band = O.If(a <= T.bands[0], 0, O.If(a <= T.bands[1], 1, O.If(a <= T.bands[2], 2, 3)))
    base = O.If(v > 0, 1, 1 + len(T.bands) * T.n_sectors)
    moving_id = base + band * T.n_sectors + sector
    fs_id = O.If(dig_sel, DIG_SET, O.If(dock_sel, DOCK_SET, O.If(still, STANDSTILL_SET, moving_id)))
    direction = O.If(O.Or(dig_sel, dock_sel), 1, O.If(still, 0, O.If(v > 0, 1, -1)))
    vp = T.v_protected
    v_band = O.If(band == 0, vp[0], O.If(band == 1, vp[1], O.If(band == 2, vp[2], vp[3])))
    v_prot = O.If(dig_sel, T.dig_speed, O.If(dock_sel, T.dock_speed, O.If(still, T.standstill, v_band)))
    return fs_id, direction, v_prot


# ---------------------------------------------------------------------- step
def step(O, P, T: FieldTable, s: Memory, i):
    """One 25 ms PLC cycle. P: SafetyParams. i: SafetyInputs (or symbolic twin)."""
    ev: list = []
    t, v = i.t, i.speed_b
    st, lat = s.state, s.latched

    def trip(cond, cause: int, emergency: bool = False, **data):
        nonlocal st, lat
        target = EMERGENCY_STOP if emergency else PROTECTIVE_STOP
        raise_ = O.And(cond, target > st)
        log = O.And(cond, O.Or(target > st, O.nz(O.bv(cause) & ~lat)))
        lat = O.If(cond, lat | O.bv(cause), lat)
        st = O.If(raise_, target, st)
        ev.append((log, "stop", dict(state=st, cause=cause, **data)))

    rising_reset = O.And(i.reset_req, O.Not(s.prev_reset))

    # SR-01 emergency stops: latch.
    trip(i.machine_estop, ESTOP_MACHINE, emergency=True)
    trip(i.remote_estop, ESTOP_REMOTE, emergency=True)

    # Field-set selection (SR-02, SR-10, SR-14).
    dig_ok = O.And(i.dig_req, i.in_dig_zone, O.Abs(v) <= T.dig_speed + P.overspeed_tol, i.position_valid)
    dock_ok = O.And(i.dock_req, i.in_dock_zone, O.Abs(v) <= T.dock_speed + P.overspeed_tol, i.position_valid,
                    O.Not(i.truck_door_open))
    # SR-14: the driver leaving the cab while the loader is docking stops it.
    trip(O.And(i.truck_door_open, s.field_set == DOCK_SET, i.in_dock_zone), TRUCK_DOOR)
    muting = O.And(dig_ok, v > -T.standstill)          # the front field is actually muted
    mute_t0 = O.If(O.And(muting, s.mute_on), s.mute_t0, t)
    mute_timeout = O.And(muting, t - mute_t0 > P.max_mute_s)
    trip(mute_timeout, MUTING_TIMEOUT)
    dig_ok = O.And(dig_ok, O.Not(mute_timeout))
    fs_id, fs_dir, fs_vprot = select(O, T, v, i.articulation, dig_ok, dock_ok)

    # SR-12 recovery: after a supervisor reset in TELEOP, motion away from an
    # occupied field is allowed at creep speed; motion toward it trips again.
    # A field counts as clear only at standstill, where the standstill set
    # watches front, rear and sides together.
    rec = O.If(i.mode == TELEOP, s.recovery, O.bv(0))
    at_rest = O.Abs(v) < P.standstill
    rec = O.If(O.And(at_rest, O.Not(i.front_prot)), O.clear(rec, FIELD_FRONT), rec)
    rec = O.If(O.And(at_rest, O.Not(i.rear_prot)), O.clear(rec, FIELD_REAR), rec)
    rec = O.If(O.And(at_rest, O.Not(i.side_prot)), O.clear(rec, FIELD_SIDE), rec)
    # SR-16: someone between the front tyres and the bucket, or under the raised
    # bucket. The near field (2.0-2.8 m) is never reached by the bucket at scan
    # height, so it is watched at every boom position; the far field only once the
    # bucket is raised clear of the scan plane. Inside the access-controlled dig
    # zone the pile fills both while the bucket lifts out of the face; beside a
    # docked truck (driver in the cab) the truck fills the far field.
    exempt = O.Or(i.in_dig_zone, O.And(i.in_dock_zone, O.Not(i.truck_door_open)))
    # The mid field (2.8-3.35 m) reaches up to the lowered bucket's back edge at any
    # boom position, beyond where a docked truck's side may be (found by the
    # adversarial search on the carry site).
    under = O.And(O.Not(i.in_dig_zone),
                  O.Or(i.under_near, O.And(O.Not(exempt), O.Or(i.under_mid, O.And(i.boom_raised, i.under_far)))))
    rec = O.If(O.And(at_rest, O.Not(under)), O.clear(rec, UNDER_BUCKET), rec)
    # Beside the frame no direction is "away": the bucket is wider than the front
    # frame and slides past a person standing there either way. Inside the
    # access-controlled zones the object in the band is the pile flank or the truck.
    trip(O.And(O.has(rec, FIELD_SIDE), i.side_contact, O.Not(exempt)), FIELD_SIDE, field="recovery_contact", v=v)
    trip(O.And(O.has(rec, UNDER_BUCKET), v > P.standstill), UNDER_BUCKET, field="recovery", v=v)
    trip(O.And(O.has(rec, FIELD_FRONT), v > P.standstill), FIELD_FRONT, field="recovery", v=v)
    trip(O.And(O.has(rec, FIELD_REAR), v < -P.standstill), FIELD_REAR, field="recovery", v=v)

    # SR-02 protective fields. The scanners evaluate the *selected* set.
    trip(O.And(i.front_prot, fs_dir >= 0, O.Not(O.has(rec, FIELD_FRONT))), FIELD_FRONT, field=fs_id, v=v)
    trip(O.And(i.rear_prot, O.Or(fs_dir <= 0, O.Abs(v) < P.standstill), O.Not(O.has(rec, FIELD_REAR))),
         FIELD_REAR, field=fs_id, v=v)
    trip(O.And(i.side_prot, O.Abs(v) > P.standstill, O.Not(O.has(rec, FIELD_SIDE))), FIELD_SIDE, field=fs_id, v=v)
    trip(O.Not(i.scanners_ok), SCANNER_FAULT)
    # SR-16: a person under the raised bucket stops travel and holds the boom
    # (found by the adversarial search: the bucket was lowered onto a person while
    # the machine reversed away from the truck).
    trip(O.And(under, O.Not(O.has(rec, UNDER_BUCKET))), UNDER_BUCKET, v=v)
    # Reversing: someone stepping in beside the front frame is heading for the gap
    # between the front tyres and the bucket's back edge (outside the zones where
    # the pile flank or the docked truck sits there).
    trip(O.And(i.front_side, v < -P.standstill, O.Not(exempt), O.Not(O.has(rec, FIELD_SIDE))), FIELD_SIDE,
         field="front_side_reversing", v=v)

    # SR-05 remote supervision link, SR-06 autonomy watchdog.
    active_mode = O.Or(i.mode == AUTO, i.mode == TELEOP)
    trip(O.And(active_mode, i.remote_hb_age > P.remote_timeout_s), REMOTE_LINK, age=i.remote_hb_age)
    trip(O.And(active_mode, i.autonomy_cmd_age > P.watchdog_timeout_s), WATCHDOG, age=i.autonomy_cmd_age)

    # SR-07 geofence on the safety GNSS; an invalid position is tolerated briefly.
    auto = i.mode == AUTO
    posinv = O.And(auto, O.Not(i.position_valid))
    posinv_t0 = O.If(O.And(posinv, O.Not(s.posinv_on)), t, s.posinv_t0)
    posinv_on = O.If(auto, O.Not(i.position_valid), s.posinv_on)
    trip(O.And(posinv, t - posinv_t0 > P.position_invalid_s), POSITION_INVALID)
    trip(O.And(auto, i.position_valid, O.Not(i.in_operating_area)), GEOFENCE)

    # SR-08 two-channel speed plausibility.
    disc = O.Abs(i.speed_a - i.speed_b) > P.discrepancy_tol
    disc_t0 = O.If(O.And(disc, O.Not(s.disc_on)), t, s.disc_t0)
    trip(O.And(disc, t - disc_t0 > P.discrepancy_time_s), SPEED_DISCREPANCY, a=i.speed_a, b=v)

    # SR-03 warning field -> speed limit (non-latching); not used inside the
    # access-controlled dig and dock zones.
    in_zone = O.Or(i.in_dig_zone, i.in_dock_zone)
    warn_trig = O.And(O.Not(in_zone), O.Or(O.And(i.front_warn, fs_dir >= 0), O.And(i.rear_warn, fs_dir <= 0)))
    warn_until = O.If(warn_trig, t + P.warning_hold_s, s.warn_until)
    warning = t < warn_until

    # SR-04 speed limit and overspeed monitoring.
    limit = O.If(auto, min(P.auto_speed, T.speed_cap), O.If(i.mode == TELEOP, min(P.teleop_speed, T.speed_cap), 0.0))
    limit = O.If(warning, O.Min(limit, P.warning_speed), limit)
    limit = O.If(O.nz(rec), O.Min(limit, O.If(O.has(rec, FIELD_SIDE), 0.3, P.recovery_speed)), limit)
    limit = O.If(fs_id == DIG_SET, O.Min(limit, T.dig_speed),
                 O.If(fs_id == DOCK_SET, O.Min(limit, T.dock_speed), limit))
    # The monitored envelope follows a limit increase at once. After a decrease
    # it holds the current speed for the stop reaction time, then ramps down at
    # the guaranteed deceleration (SS1-r style monitoring).
    dec = limit < s.prev_limit - 1e-9
    env_t0 = O.If(dec, t, s.env_t0)
    env_v0 = O.If(dec, O.Min(s.allowed, O.Abs(v)), s.env_v0)
    ramp = env_v0 - P.decel_ramp * O.Max(0.0, t - env_t0 - P.ramp_reaction_s)
    allowed = O.Max(limit, ramp)
    over = O.Abs(v) > allowed + P.overspeed_tol
    over_t0 = O.If(O.And(over, O.Not(s.over_on)), t, s.over_t0)
    trip(O.And(over, t - over_t0 >= CYCLE_S), OVERSPEED, v=v, limit=limit, envelope=allowed)
    # SR-04: never travel faster than the selected protective field can stop from.
    # Found by the proof of SR-04b: the tolerance band above the top speed band
    # was not covered by any field.
    travel_set = O.And(fs_dir != 0, fs_id != DIG_SET, fs_id != DOCK_SET)
    trip(O.And(travel_set, O.Abs(v) > fs_vprot), OVERSPEED, v=v, field=fs_id, protected=fs_vprot)

    # Reset handling (SR-09, SR-11, SR-12).
    standstill = O.Abs(v) < P.standstill
    active = active_causes(O, P, i)
    active = O.If(under, active | O.bv(UNDER_BUCKET), active)
    # Muting past its limit is a standing condition: it blocks reset while the dig
    # request continues (found by the proof of SR-10b).
    active = O.If(mute_timeout, active | O.bv(MUTING_TIMEOUT), active)
    # A cause latched in this very cycle cannot be reset in the same cycle (found by
    # the proof of SR-10b: a muting timeout was cleared by a reset pressed with it).
    stopped_now = O.And(st >= PROTECTIVE_STOP, O.Not(O.nz(lat & ~s.latched)))
    remaining = lat & active
    enter_rec = O.And(stopped_now, rising_reset, i.mode == TELEOP, st == PROTECTIVE_STOP, O.nz(remaining),
                      O.Not(O.nz(remaining & ~O.bv(FIELD_CAUSES))),
                      O.Not(O.And(O.has(remaining, FIELD_SIDE), i.side_contact, O.Not(exempt))))
    # Fields already in recovery stay in it (found by the proof of SR-02a: replacing
    # the set dropped an occupied front field out of recovery and released the brakes).
    rec = O.If(enter_rec, rec | remaining, rec)
    ev.append((enter_rec, "recovery", dict(fields=remaining)))
    remaining = O.If(enter_rec, O.bv(0), remaining)
    clear_t0 = O.If(O.And(stopped_now, O.Not(O.nz(remaining)), O.Not(s.clear_on)), t, s.clear_t0)
    clear_on = O.If(stopped_now, O.Not(O.nz(remaining)), s.clear_on)
    if P.auto_restart_clear_s is None:
        auto_ok = False
    else:
        auto_ok = O.And(st == PROTECTIVE_STOP, O.Not(O.nz(lat & ~O.bv(CLEARING_CAUSES))), clear_on,
                        t - clear_t0 >= P.auto_restart_clear_s, O.Not(warning))
    do_reset = O.And(stopped_now, standstill, O.Not(O.nz(remaining)), O.Or(rising_reset, auto_ok))
    ev.append((do_reset, "reset", dict(by=O.If(O.And(auto_ok, O.Not(rising_reset)), "auto", "supervisor")
                                       if O is CONCRETE else "", cleared=lat)))
    st = O.If(do_reset, RUN, st)
    lat = O.If(do_reset, O.bv(0), lat)
    clear_on = O.If(do_reset, False, clear_on)
    st = O.If(st < PROTECTIVE_STOP, O.If(warning, WARNING, RUN), st)

    stopped = st >= PROTECTIVE_STOP
    new = Memory(state=st, latched=lat, field_set=fs_id, recovery=rec, prev_reset=i.reset_req,
                 over_on=over, over_t0=over_t0, disc_on=disc, disc_t0=disc_t0,
                 posinv_on=posinv_on, posinv_t0=posinv_t0, mute_on=muting, mute_t0=mute_t0,
                 clear_on=clear_on, clear_t0=clear_t0, warn_until=warn_until, allowed=allowed,
                 prev_limit=limit, env_t0=env_t0, env_v0=env_v0)
    # The recovery creep limit applies from the cycle recovery starts (found by the
    # proof of SR-12e).
    limit_out = O.If(O.nz(rec), O.Min(limit, O.If(O.has(rec, FIELD_SIDE), 0.3, P.recovery_speed)), limit)
    out = Out(state=st, cause=lat, speed_limit=O.If(stopped, 0.0, limit_out), field_set=fs_id,
              brake_release=O.Not(stopped),
              # SR-12: no boom/bucket motion while recovering from an occupied field.
              implement_enable=O.And(O.Not(stopped), O.Not(O.nz(rec))),
              reset_pending=O.And(stopped, O.Not(O.nz(lat & active))),
              dock_blocked=i.truck_door_open, fs_direction=fs_dir, fs_v_protected=fs_vprot, events=ev)
    return new, out


def active_causes(O, P, i):
    """Causes whose condition is still present (blocks reset)."""
    c = O.bv(0)

    def add(cond, bit):
        nonlocal c
        c = O.If(cond, c | O.bv(bit), c)

    add(i.machine_estop, ESTOP_MACHINE)
    add(i.remote_estop, ESTOP_REMOTE)
    add(i.front_prot, FIELD_FRONT)
    add(i.rear_prot, FIELD_REAR)
    add(i.side_prot, FIELD_SIDE)
    add(O.Not(i.scanners_ok), SCANNER_FAULT)
    add(O.And(i.mode != IDLE, i.remote_hb_age > P.remote_timeout_s), REMOTE_LINK)
    add(O.And(i.mode != IDLE, i.autonomy_cmd_age > P.watchdog_timeout_s), WATCHDOG)
    add(O.And(i.mode == AUTO, i.position_valid, O.Not(i.in_operating_area)), GEOFENCE)
    add(O.Abs(i.speed_a - i.speed_b) > P.discrepancy_tol, SPEED_DISCREPANCY)
    add(O.And(i.mode == AUTO, O.Not(i.position_valid)), POSITION_INVALID)
    add(O.And(i.truck_door_open, i.in_dock_zone), TRUCK_DOOR)
    return c
