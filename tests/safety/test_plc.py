"""Requirement-traced tests of the safety PLC logic (see docs/safety/requirements.md)."""
from __future__ import annotations

import pytest

from kauha.safety.plc import CYCLE_S, Cause, Mode, SafetyInputs, SafetyParams, SafetyPLC, State


def run(plc: SafetyPLC, n: int, t0: float = 0.0, **kw):
    out = None
    for i in range(n):
        out = plc.step(SafetyInputs(t=t0 + i * CYCLE_S, **kw))
    return out


def nominal(**over):
    base = dict(mode=Mode.AUTO, speed_a=1.0, speed_b=1.0, articulation=0.0, in_operating_area=True)
    base.update(over)
    return base


@pytest.fixture
def plc():
    return SafetyPLC()


def test_runs_nominally(plc):
    out = run(plc, 40, **nominal())
    assert out.state == State.RUN and out.brake_release and out.implement_enable


# ---------------------------------------------------------------- SR-01
@pytest.mark.req("SR-01")
def test_sr01_machine_estop_latches_until_reset(plc):
    out = run(plc, 2, **nominal(machine_estop=True))
    assert out.state == State.EMERGENCY_STOP and not out.brake_release and not out.implement_enable
    out = run(plc, 40, t0=1.0, **nominal(speed_a=0, speed_b=0))
    assert out.state == State.EMERGENCY_STOP, "e-stop must stay latched after release"
    out = run(plc, 2, t0=3.0, **nominal(speed_a=0, speed_b=0, reset_req=True))
    assert out.state == State.RUN


@pytest.mark.req("SR-01")
def test_sr01_remote_estop(plc):
    out = run(plc, 1, **nominal(remote_estop=True))
    assert out.state == State.EMERGENCY_STOP and Cause.ESTOP_REMOTE in out.cause


@pytest.mark.req("SR-01")
def test_sr01_reset_refused_while_estop_held(plc):
    run(plc, 1, **nominal(machine_estop=True))
    out = run(plc, 4, t0=1.0, **nominal(speed_a=0, speed_b=0, machine_estop=True, reset_req=True))
    assert out.state == State.EMERGENCY_STOP


# ---------------------------------------------------------------- SR-02
@pytest.mark.req("SR-02")
@pytest.mark.parametrize("field,speed", [("front_prot", 1.0), ("rear_prot", -1.0), ("side_prot", 1.0)])
def test_sr02_protective_field_stops_within_one_cycle(plc, field, speed):
    run(plc, 4, **nominal(speed_a=speed, speed_b=speed))
    out = plc.step(SafetyInputs(t=1.0, **nominal(speed_a=speed, speed_b=speed, **{field: True})))
    assert out.state == State.PROTECTIVE_STOP and not out.brake_release


@pytest.mark.req("SR-02")
def test_sr02_rear_field_ignored_when_driving_forward(plc):
    out = run(plc, 4, **nominal(rear_prot=True))
    assert out.state == State.RUN


# ---------------------------------------------------------------- SR-03
@pytest.mark.req("SR-03")
def test_sr03_warning_field_limits_speed_without_stopping(plc):
    out = run(plc, 2, **nominal(front_warn=True))
    assert out.state == State.WARNING and out.speed_limit == pytest.approx(1.0)
    out = run(plc, 100, t0=1.0, **nominal(speed_a=0.9, speed_b=0.9))
    assert out.state == State.RUN and out.speed_limit > 2.5


# ---------------------------------------------------------------- SR-04
@pytest.mark.req("SR-03")
def test_sr03_warning_ignored_inside_access_controlled_zones(plc):
    out = run(plc, 4, **nominal(front_warn=True, in_dock_zone=True))
    assert out.state == State.RUN and out.speed_limit > 2.5
    out = run(plc, 4, t0=1.0, **nominal(front_prot=True, in_dock_zone=True))
    assert out.state == State.PROTECTIVE_STOP, "protective fields still apply in zones"


@pytest.mark.req("SR-04")
def test_sr04_overspeed_trips(plc):
    out = run(plc, 4, **nominal(speed_a=3.6, speed_b=3.6))
    assert out.state == State.PROTECTIVE_STOP and Cause.OVERSPEED in out.cause


@pytest.mark.req("SR-04")
def test_sr04_deceleration_ramp_tolerated_after_limit_drop(plc):
    run(plc, 4, **nominal(speed_a=2.8, speed_b=2.8))
    v, t = 2.8, 0.2
    for i in range(120):  # warning appears; machine slows at 1.0 m/s^2 after 0.15 s latency
        if i * CYCLE_S > 0.15:
            v = max(0.9, v - 1.0 * CYCLE_S)
        out = plc.step(SafetyInputs(t=t + i * CYCLE_S, **nominal(speed_a=v, speed_b=v, front_warn=True)))
        assert out.state != State.PROTECTIVE_STOP


@pytest.mark.req("SR-04")
def test_sr04_too_slow_deceleration_trips(plc):
    run(plc, 4, **nominal(speed_a=2.8, speed_b=2.8))
    v, tripped = 2.8, False
    for i in range(120):  # only 0.3 m/s^2: below the guaranteed 0.8 m/s^2
        v = max(0.9, v - 0.3 * CYCLE_S)
        out = plc.step(SafetyInputs(t=0.2 + i * CYCLE_S, **nominal(speed_a=v, speed_b=v, front_warn=True)))
        tripped |= out.state == State.PROTECTIVE_STOP
    assert tripped


@pytest.mark.req("SR-04")
def test_sr04_speed_cap_follows_scanner_range():
    summer = SafetyPLC()
    winter = SafetyPLC(budget=summer.budget.winter())
    assert winter.field_cap < summer.field_cap
    assert run(winter, 2, **nominal()).speed_limit <= winter.field_cap + 1e-9


# ---------------------------------------------------------------- SR-05
@pytest.mark.req("SR-05")
def test_sr05_remote_link_loss_stops(plc):
    out = run(plc, 4, **nominal(remote_hb_age=0.45))
    assert out.state == State.RUN
    out = run(plc, 1, t0=1.0, **nominal(remote_hb_age=0.55))
    assert out.state == State.PROTECTIVE_STOP and Cause.REMOTE_LINK in out.cause


@pytest.mark.req("SR-05")
def test_sr05_link_stop_needs_supervisor_reset(plc):
    run(plc, 1, **nominal(remote_hb_age=0.6))
    out = run(plc, 400, t0=1.0, **nominal(speed_a=0, speed_b=0))
    assert out.state == State.PROTECTIVE_STOP and out.reset_pending
    out = run(plc, 2, t0=20.0, **nominal(speed_a=0, speed_b=0, reset_req=True))
    assert out.state == State.RUN


# ---------------------------------------------------------------- SR-06
@pytest.mark.req("SR-06")
def test_sr06_autonomy_watchdog(plc):
    out = run(plc, 1, **nominal(autonomy_cmd_age=0.25))
    assert out.state == State.PROTECTIVE_STOP and Cause.WATCHDOG in out.cause


@pytest.mark.req("SR-06")
def test_sr06_watchdog_not_applied_in_idle(plc):
    out = run(plc, 4, **nominal(mode=Mode.IDLE, speed_a=0, speed_b=0, autonomy_cmd_age=99))
    assert out.state == State.RUN


# ---------------------------------------------------------------- SR-07
@pytest.mark.req("SR-07")
def test_sr07_geofence(plc):
    out = run(plc, 1, **nominal(in_operating_area=False))
    assert Cause.GEOFENCE in out.cause


@pytest.mark.req("SR-07")
def test_sr07_position_invalid_tolerated_briefly(plc):
    out = run(plc, int(1.5 / CYCLE_S), **nominal(position_valid=False))
    assert out.state == State.RUN
    out = run(plc, int(1.0 / CYCLE_S), t0=1.5, **nominal(position_valid=False))
    assert Cause.POSITION_INVALID in out.cause


# ---------------------------------------------------------------- SR-08
@pytest.mark.req("SR-08")
def test_sr08_speed_sensor_discrepancy(plc):
    out = run(plc, 4, **nominal(speed_a=1.0, speed_b=0.5))
    assert out.state == State.RUN
    out = run(plc, 12, t0=1.0, **nominal(speed_a=1.0, speed_b=0.5))
    assert Cause.SPEED_DISCREPANCY in out.cause


# ---------------------------------------------------------------- SR-09
@pytest.mark.req("SR-09")
def test_sr09_no_reset_while_moving(plc):
    run(plc, 1, **nominal(remote_hb_age=0.6))
    out = run(plc, 4, t0=1.0, **nominal(speed_a=0.5, speed_b=0.5, reset_req=True))
    assert out.state == State.PROTECTIVE_STOP


# ---------------------------------------------------------------- SR-10
@pytest.mark.req("SR-10")
def test_sr10_dig_field_only_in_dig_zone(plc):
    out = run(plc, 4, **nominal(speed_a=0.8, speed_b=0.8, dig_req=True, in_dig_zone=False))
    assert out.field_set != 100
    out = run(plc, 4, t0=1.0, **nominal(speed_a=0.8, speed_b=0.8, dig_req=True, in_dig_zone=True))
    assert out.field_set == 100 and out.speed_limit <= 1.0


@pytest.mark.req("SR-10")
def test_sr10_dig_field_refused_above_dig_speed(plc):
    out = run(plc, 4, **nominal(speed_a=2.0, speed_b=2.0, dig_req=True, in_dig_zone=True))
    assert out.field_set != 100


@pytest.mark.req("SR-10")
def test_sr10_dig_field_not_used_when_reversing(plc):
    out = run(plc, 4, **nominal(speed_a=-0.5, speed_b=-0.5, dig_req=True, in_dig_zone=True))
    assert plc.sets[out.field_set].direction == -1


@pytest.mark.req("SR-10")
def test_sr10_muting_time_limited(plc):
    out = run(plc, int(26 / CYCLE_S), **nominal(speed_a=0.5, speed_b=0.5, dig_req=True, in_dig_zone=True))
    assert Cause.MUTING_TIMEOUT in out.cause


# ---------------------------------------------------------------- SR-11
@pytest.mark.req("SR-11")
def test_sr11_auto_restart_after_field_clears(plc):
    run(plc, 2, **nominal(front_prot=True))
    out = run(plc, int(2.0 / CYCLE_S), t0=1.0, **nominal(speed_a=0, speed_b=0))
    assert out.state == State.PROTECTIVE_STOP
    out = run(plc, int(2.0 / CYCLE_S), t0=3.0, **nominal(speed_a=0, speed_b=0))
    assert out.state == State.RUN


@pytest.mark.req("SR-11")
def test_sr11_no_auto_restart_when_disabled():
    plc = SafetyPLC(SafetyParams(auto_restart_clear_s=None))
    run(plc, 2, **nominal(front_prot=True))
    out = run(plc, int(10 / CYCLE_S), t0=1.0, **nominal(speed_a=0, speed_b=0))
    assert out.state == State.PROTECTIVE_STOP


@pytest.mark.req("SR-11")
def test_sr11_no_auto_restart_for_non_field_causes(plc):
    run(plc, 2, **nominal(front_prot=True, remote_hb_age=0.7))
    out = run(plc, int(10 / CYCLE_S), t0=1.0, **nominal(speed_a=0, speed_b=0))
    assert out.state == State.PROTECTIVE_STOP


# ---------------------------------------------------------------- SR-12
@pytest.mark.req("SR-12")
def test_sr12_recovery_allows_backing_away_only():
    plc = SafetyPLC()
    run(plc, 2, **nominal(mode=Mode.TELEOP, front_prot=True))
    out = run(plc, 2, t0=1.0, **nominal(mode=Mode.TELEOP, speed_a=0, speed_b=0, front_prot=True, reset_req=True))
    assert out.state == State.RUN and out.speed_limit <= 0.5
    assert not out.implement_enable, "no implement motion during recovery"
    out = run(plc, 4, t0=2.0, **nominal(mode=Mode.TELEOP, speed_a=-0.3, speed_b=-0.3, front_prot=True))
    assert out.state == State.RUN, "reversing away from the occupied front field is allowed"
    out = run(plc, 2, t0=3.0, **nominal(mode=Mode.TELEOP, speed_a=0.3, speed_b=0.3, front_prot=True))
    assert out.state == State.PROTECTIVE_STOP, "moving toward it trips again"


@pytest.mark.req("SR-12")
def test_sr12_side_recovery_refused_while_someone_is_right_beside_the_frame():
    plc = SafetyPLC()
    run(plc, 2, **nominal(mode=Mode.TELEOP, side_prot=True, side_contact=True))
    out = run(plc, 2, t0=1.0, **nominal(mode=Mode.TELEOP, speed_a=0, speed_b=0, side_prot=True,
                                         side_contact=True, reset_req=True))
    assert out.state == State.PROTECTIVE_STOP, "no direction is 'away' from a person beside the frame"
    # Object further out (truck side, pile face): recovery is allowed ...
    out = run(plc, 2, t0=2.0, **nominal(mode=Mode.TELEOP, speed_a=0, speed_b=0, side_prot=True))
    out = run(plc, 2, t0=3.0, **nominal(mode=Mode.TELEOP, speed_a=0, speed_b=0, side_prot=True, reset_req=True))
    assert out.state == State.RUN and not out.implement_enable
    # ... and stops again if anyone steps into the contact band while it creeps away.
    out = run(plc, 2, t0=4.0, **nominal(mode=Mode.TELEOP, speed_a=-0.3, speed_b=-0.3, side_prot=True,
                                         side_contact=True))
    assert out.state == State.PROTECTIVE_STOP and Cause.FIELD_SIDE in out.cause


@pytest.mark.req("SR-12")
def test_sr12_contact_band_does_not_block_recovery_in_access_controlled_zones():
    kw = dict(mode=Mode.TELEOP, side_prot=True, side_contact=True, in_dock_zone=True)
    plc = SafetyPLC()
    run(plc, 2, **nominal(**kw))
    out = run(plc, 2, t0=1.0, **nominal(speed_a=0, speed_b=0, reset_req=True, **kw))
    assert out.state == State.RUN, "driver in the cab: the object in the band is the truck"
    plc = SafetyPLC()
    run(plc, 2, **nominal(**kw))
    out = run(plc, 2, t0=1.0, **nominal(speed_a=0, speed_b=0, reset_req=True, truck_door_open=True, **kw))
    assert out.state == State.PROTECTIVE_STOP, "cab door open: it may be the driver"
    plc = SafetyPLC()
    kw = dict(mode=Mode.TELEOP, side_prot=True, side_contact=True, in_dig_zone=True)
    run(plc, 2, **nominal(**kw))
    out = run(plc, 2, t0=1.0, **nominal(speed_a=0, speed_b=0, reset_req=True, **kw))
    assert out.state == State.RUN, "dig zone: the pile flank must not lock the machine in the slot"


@pytest.mark.req("SR-12")
def test_sr12_no_recovery_in_auto(plc):
    run(plc, 2, **nominal(front_prot=True))
    out = run(plc, 2, t0=1.0, **nominal(speed_a=0, speed_b=0, front_prot=True, reset_req=True))
    assert out.state == State.PROTECTIVE_STOP


# ---------------------------------------------------------------- logging
@pytest.mark.req("SR-13")
def test_sr13_every_stop_and_reset_is_logged(plc):
    run(plc, 1, **nominal(remote_hb_age=0.6))
    run(plc, 2, t0=1.0, **nominal(speed_a=0, speed_b=0, reset_req=True))
    kinds = [r["kind"] for r in plc.log.records]
    assert kinds == ["stop", "reset"] and plc.log.verify() is None


# ---------------------------------------------------------------- SR-14
@pytest.mark.req("SR-14")
def test_sr14_dock_refused_while_truck_door_open(plc):
    kw = dict(speed_a=0.5, speed_b=0.5, dock_req=True, in_dock_zone=True)
    out = run(plc, 4, **nominal(truck_door_open=True, **kw))
    assert out.field_set != 101 and out.dock_blocked
    out = run(plc, 4, t0=1.0, **nominal(**kw))
    assert out.field_set == 101 and not out.dock_blocked


@pytest.mark.req("SR-14")
def test_sr14_door_opening_while_docking_stops_and_clears(plc):
    kw = dict(speed_a=0.5, speed_b=0.5, dock_req=True, in_dock_zone=True)
    run(plc, 4, **nominal(**kw))
    out = run(plc, 2, t0=1.0, **nominal(truck_door_open=True, **kw))
    assert out.state == State.PROTECTIVE_STOP and Cause.TRUCK_DOOR in out.cause
    out = run(plc, int(5 / CYCLE_S), t0=2.0, **nominal(speed_a=0, speed_b=0, truck_door_open=True,
                                                      dock_req=True, in_dock_zone=True))
    assert out.state == State.PROTECTIVE_STOP, "no restart while the door is open"
    out = run(plc, int(4 / CYCLE_S), t0=8.0, **nominal(speed_a=0, speed_b=0, dock_req=True, in_dock_zone=True))
    assert out.state == State.RUN, "auto restart once the door has been closed for 3 s"


# ------------------------------------------------- regressions found by the proofs
@pytest.mark.req("SR-04")
def test_found_by_proof_speed_above_the_top_band_is_never_unprotected():
    """SR-04b counterexample: 3.0-3.2 m/s ran on a field sized for 3.0 m/s."""
    from kauha.safety.fields import BANDS, protective_length
    from kauha.safety.stopping import stopping_distance
    for budget in (SafetyPLC().budget, SafetyPLC().budget.winter()):
        plc = SafetyPLC(budget=budget)
        top = plc.table.v_protected[-1]
        assert stopping_distance(top, budget) <= protective_length(BANDS[-1], budget) + 1e-9
        assert plc.table.speed_cap + plc.params.overspeed_tol <= top + 1e-9
        out = run(plc, 4, **nominal(speed_a=top + 0.01, speed_b=top + 0.01))
        assert out.state == State.PROTECTIVE_STOP and Cause.OVERSPEED in out.cause


@pytest.mark.req("SR-12")
def test_found_by_proof_reset_keeps_fields_already_in_recovery():
    """SR-02a counterexample: a reset replaced the recovery set and dropped the front field."""
    plc = SafetyPLC()
    run(plc, 2, **nominal(mode=Mode.TELEOP, front_prot=True))
    run(plc, 2, t0=1.0, **nominal(mode=Mode.TELEOP, speed_a=0, speed_b=0, front_prot=True, reset_req=True))
    assert Cause.FIELD_FRONT in plc.recovery
    # creeping back, a side stop happens; the front field is still occupied
    run(plc, 2, t0=2.0, **nominal(mode=Mode.TELEOP, speed_a=-0.2, speed_b=-0.2, front_prot=True, side_prot=True))
    out = run(plc, 2, t0=3.0, **nominal(mode=Mode.TELEOP, speed_a=0, speed_b=0, front_prot=True, side_prot=True,
                                         reset_req=True))
    assert Cause.FIELD_FRONT in plc.recovery and Cause.FIELD_SIDE in plc.recovery
    out = run(plc, 2, t0=4.0, **nominal(mode=Mode.TELEOP, speed_a=0.2, speed_b=0.2, front_prot=True))
    assert out.state == State.PROTECTIVE_STOP, "moving toward the occupied front field still trips"


@pytest.mark.req("SR-10")
def test_found_by_proof_muting_timeout_cannot_be_reset_while_muting_continues():
    """SR-10b counterexample: a reset cleared the muting timeout while digging continued."""
    plc = SafetyPLC()
    kw = dict(speed_a=0.0, speed_b=0.0, dig_req=True, in_dig_zone=True)
    out = run(plc, int(26 / CYCLE_S), **nominal(**kw))
    assert Cause.MUTING_TIMEOUT in out.cause
    out = run(plc, 3, t0=30.0, **nominal(reset_req=True, **kw))
    assert out.state == State.PROTECTIVE_STOP, "still muting: reset refused"
    out = run(plc, 3, t0=31.0, **nominal(speed_a=0.0, speed_b=0.0))
    out = run(plc, 3, t0=32.0, **nominal(speed_a=0.0, speed_b=0.0, reset_req=True))
    assert out.state == State.RUN, "dig request dropped: reset accepted"


@pytest.mark.req("SR-12")
def test_found_by_proof_creep_limit_applies_from_the_first_recovery_cycle():
    """SR-12e counterexample: the cycle that entered recovery still output the full speed limit."""
    plc = SafetyPLC()
    run(plc, 2, **nominal(mode=Mode.TELEOP, front_prot=True))
    plc.step(SafetyInputs(t=1.0, **nominal(mode=Mode.TELEOP, speed_a=0, speed_b=0, front_prot=True)))
    out = plc.step(SafetyInputs(t=1.025, **nominal(mode=Mode.TELEOP, speed_a=0, speed_b=0, front_prot=True,
                                                   reset_req=True)))
    assert out.state == State.RUN and out.speed_limit <= 0.5


# ---------------------------------------------------------------- SR-16
@pytest.mark.req("SR-16")
def test_sr16_person_under_raised_bucket_stops_travel_and_holds_the_boom(plc):
    out = run(plc, 4, **nominal(speed_a=-1.5, speed_b=-1.5, boom_raised=True, under_near=True))
    assert out.state == State.PROTECTIVE_STOP and Cause.UNDER_BUCKET in out.cause and not out.implement_enable


@pytest.mark.req("SR-16")
def test_sr16_ignored_with_the_bucket_down_and_far_field_exempt_at_a_docked_truck():
    plc = SafetyPLC()
    out = run(plc, 4, **nominal(boom_raised=False, under_far=True))
    assert out.state == State.RUN, "bucket low: the far field sees the bucket itself"
    back = dict(speed_a=-0.5, speed_b=-0.5, boom_raised=True, in_dock_zone=True)
    out = run(plc, 4, t0=1.0, **nominal(under_far=True, **back))
    assert out.state == State.RUN, "truck side in the far field while backing off the truck"
    out = run(plc, 4, t0=2.0, **nominal(under_near=True, **back))
    assert out.state == State.PROTECTIVE_STOP, "near field (tyres to truck side) is never exempt"


@pytest.mark.req("SR-16")
def test_sr16_gap_field_stops_at_any_boom_position_except_beside_a_docked_truck():
    """The mid field ends short of the lowered bucket's back edge, so it is watched with the boom down."""
    plc = SafetyPLC()
    out = run(plc, 4, **nominal(boom_raised=False, under_mid=True))
    assert out.state == State.PROTECTIVE_STOP and Cause.UNDER_BUCKET in out.cause and not out.implement_enable
    for zone in (dict(in_dig_zone=True), dict(in_dock_zone=True)):
        plc = SafetyPLC()
        out = run(plc, 4, **nominal(speed_a=-0.5, speed_b=-0.5, under_mid=True, **zone))
        assert out.state == State.RUN, zone


@pytest.mark.req("SR-16")
def test_sr16_beside_the_front_frame_while_reversing_stops_except_at_pile_or_docked_truck():
    plc = SafetyPLC()
    out = run(plc, 4, **nominal(speed_a=-1.0, speed_b=-1.0, front_side=True))
    assert out.state == State.PROTECTIVE_STOP and Cause.FIELD_SIDE in out.cause
    for zone in (dict(in_dig_zone=True), dict(in_dock_zone=True)):
        plc = SafetyPLC()
        out = run(plc, 4, **nominal(speed_a=-1.0, speed_b=-1.0, front_side=True, **zone))
        assert out.state == State.RUN, zone
