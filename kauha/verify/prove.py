"""Prove the safety requirements on the PLC's own step function.

    python -m kauha.verify.prove            # prove all, write results/verification.json

Each property is a statement about one PLC cycle, from any memory state that
satisfies the invariant ``INV`` and any input. ``INV`` is itself proved
inductive: it holds at power-up and every cycle preserves it. A property is
PROVEN when z3 shows its negation unsatisfiable; otherwise z3 returns a
counterexample: a memory state and inputs that break it.
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import z3

from kauha.safety import core
from kauha.safety.core import (
    AUTO,
    CLEARING_CAUSES,
    DIG_SET,
    DOCK_SET,
    EMERGENCY_STOP,
    ESTOP_MACHINE,
    ESTOP_REMOTE,
    FIELD_FRONT,
    FIELD_REAR,
    FIELD_SIDE,
    PROTECTIVE_STOP,
    TELEOP,
    UNDER_BUCKET,
)
from kauha.safety.fields import field_table
from kauha.safety.plc import SafetyParams
from kauha.safety.stopping import StopBudget
from kauha.verify.symbolic import BV, Z3, Symbols, step, symbols

REPO_ROOT = Path(__file__).resolve().parents[2]


def bv(x):
    return z3.BitVecVal(x, BV)


def has(x, bits):
    return Z3.has(x, bits)


def nz(x):
    return Z3.nz(x)


def as_bool(x):
    return z3.BoolVal(x) if isinstance(x, bool) else x


def invariant(m: core.Memory):
    """Facts true of every reachable PLC memory state."""
    stopped = m.state >= PROTECTIVE_STOP
    return z3.And(
        stopped == nz(m.latched),                                            # stopped iff a cause is latched
        (m.state == EMERGENCY_STOP) == has(m.latched, ESTOP_MACHINE | ESTOP_REMOTE),
        m.allowed >= m.prev_limit,                                           # envelope never below the limit
    )


@dataclass(frozen=True)
class Prop:
    id: str
    req: str
    text: str
    formula: Callable           # (pre: Symbols, post: Memory, out: core.Out, ctx) -> z3 Bool


def _ctx(P, T, s: Symbols):
    i, v = s.inp, s.inp.speed_b
    return dict(
        v=v, absv=z3.If(v >= 0, v, -v),
        rising=z3.And(i.reset_req, z3.Not(s.mem.prev_reset)),
        active_mode=z3.Or(i.mode == AUTO, i.mode == TELEOP),
        exempt=z3.Or(i.in_dig_zone, z3.And(i.in_dock_zone, z3.Not(i.truck_door_open))),
        P=P, T=T)


def _stop(out):
    return z3.Not(as_bool(out.brake_release))


PROPS = [
    # ---------------------------------------------------------------- outputs
    Prop("OUT-1", "SR-01", "A stopped state always applies the brakes, cuts the implement and zeroes the speed limit.",
         lambda s, m, o, c: z3.Implies(o.state >= PROTECTIVE_STOP,
                                       z3.And(_stop(o), z3.Not(as_bool(o.implement_enable)), o.speed_limit == 0))),
    # ---------------------------------------------------------------- SR-01
    Prop("SR-01a", "SR-01", "A pressed e-stop (machine or remote) gives an emergency stop in the same cycle.",
         lambda s, m, o, c: z3.Implies(z3.Or(s.inp.machine_estop, s.inp.remote_estop),
                                       z3.And(o.state == EMERGENCY_STOP, _stop(o)))),
    Prop("SR-01b", "SR-01", "An emergency stop is never cleared without a new supervisor reset (no automatic restart).",
         lambda s, m, o, c: z3.Implies(z3.And(s.mem.state == EMERGENCY_STOP, z3.Not(c["rising"])),
                                       o.state == EMERGENCY_STOP)),
    # ---------------------------------------------------------------- SR-02
    Prop("SR-02a", "SR-02", "A violated front protective field of the selected set stops the machine in the same "
         "cycle, unless the front field is in supervised teleop recovery.",
         lambda s, m, o, c: z3.Implies(z3.And(s.inp.front_prot, o.fs_direction >= 0,
                                              z3.Not(has(m.recovery, FIELD_FRONT))),
                                       _stop(o))),
    Prop("SR-02b", "SR-02", "Same for the rear protective field (reversing, or at standstill).",
         lambda s, m, o, c: z3.Implies(z3.And(s.inp.rear_prot,
                                              z3.Or(o.fs_direction <= 0, c["absv"] < c["P"].standstill),
                                              z3.Not(has(m.recovery, FIELD_REAR))), _stop(o))),
    Prop("SR-02c", "SR-02", "Same for the side fields while the machine moves.",
         lambda s, m, o, c: z3.Implies(z3.And(s.inp.side_prot, c["absv"] > c["P"].standstill,
                                              z3.Not(has(m.recovery, FIELD_SIDE))), _stop(o))),
    Prop("SR-02d", "SR-02", "A scanner fault stops the machine.",
         lambda s, m, o, c: z3.Implies(z3.Not(s.inp.scanners_ok), _stop(o))),
    # ---------------------------------------------------------------- SR-03
    Prop("SR-03", "SR-03", "While the warning hold is active the speed limit is at most the warning speed.",
         lambda s, m, o, c: z3.Implies(s.inp.t < m.warn_until, o.speed_limit <= c["P"].warning_speed)),
    # ---------------------------------------------------------------- SR-04
    Prop("SR-04a", "SR-04", "The commanded speed limit never exceeds the speed the scanners can protect.",
         lambda s, m, o, c: o.speed_limit <= c["T"].field_cap),
    Prop("SR-04b", "SR-04", "Whenever the brakes are released on a travel field set, the measured speed is within "
         "what that set's protective field can stop from.",
         lambda s, m, o, c: z3.Implies(z3.And(as_bool(o.brake_release), o.field_set >= 1, o.field_set <= 40),
                                       c["absv"] <= o.fs_v_protected)),
    # ---------------------------------------------------------------- SR-05/06
    Prop("SR-05", "SR-05", "Loss of the remote supervision heartbeat (> 0.5 s) in AUTO or TELEOP stops the machine.",
         lambda s, m, o, c: z3.Implies(z3.And(c["active_mode"], s.inp.remote_hb_age > c["P"].remote_timeout_s),
                                       _stop(o))),
    Prop("SR-06", "SR-06", "A stale autonomy command (> 0.2 s) in AUTO or TELEOP stops the machine.",
         lambda s, m, o, c: z3.Implies(z3.And(c["active_mode"], s.inp.autonomy_cmd_age > c["P"].watchdog_timeout_s),
                                       _stop(o))),
    # ---------------------------------------------------------------- SR-07
    Prop("SR-07a", "SR-07", "In AUTO with a valid safety position outside the operating area, the machine stops.",
         lambda s, m, o, c: z3.Implies(z3.And(s.inp.mode == AUTO, s.inp.position_valid,
                                              z3.Not(s.inp.in_operating_area)), _stop(o))),
    Prop("SR-07b", "SR-07", "In AUTO, a safety position invalid for more than 2 s stops the machine.",
         lambda s, m, o, c: z3.Implies(z3.And(s.inp.mode == AUTO, z3.Not(s.inp.position_valid), s.mem.posinv_on,
                                              s.inp.t - s.mem.posinv_t0 > c["P"].position_invalid_s), _stop(o))),
    # ---------------------------------------------------------------- SR-08
    Prop("SR-08", "SR-08", "The two speed channels disagreeing by more than 0.3 m/s for more than 0.2 s stops the "
         "machine.",
         lambda s, m, o, c: z3.Implies(z3.And(z3.If(s.inp.speed_a - s.inp.speed_b >= 0, s.inp.speed_a - s.inp.speed_b,
                                                    s.inp.speed_b - s.inp.speed_a) > c["P"].discrepancy_tol,
                                              s.mem.disc_on, s.inp.t - s.mem.disc_t0 > c["P"].discrepancy_time_s),
                                       _stop(o))),
    # ---------------------------------------------------------------- SR-09/11
    Prop("SR-09", "SR-09", "A stop is only ever cleared at standstill.",
         lambda s, m, o, c: z3.Implies(z3.And(s.mem.state >= PROTECTIVE_STOP, o.state < PROTECTIVE_STOP),
                                       c["absv"] < c["P"].standstill)),
    Prop("SR-11", "SR-11", "Without a supervisor reset, only self-clearing causes (fields, truck door) restart "
         "automatically.",
         lambda s, m, o, c: z3.Implies(z3.And(s.mem.state >= PROTECTIVE_STOP, z3.Not(c["rising"]),
                                              nz(s.mem.latched & ~bv(CLEARING_CAUSES))),
                                       o.state >= PROTECTIVE_STOP)),
    # ---------------------------------------------------------------- SR-10
    Prop("SR-10a", "SR-10", "The dig field set (front field muted) is only active when requested, inside the dig "
         "zone, with a valid position and at no more than the dig speed plus tolerance.",
         lambda s, m, o, c: z3.Implies(o.field_set == DIG_SET,
                                       z3.And(s.inp.dig_req, s.inp.in_dig_zone, s.inp.position_valid,
                                              c["absv"] <= c["T"].dig_speed + c["P"].overspeed_tol))),
    Prop("SR-10b", "SR-10", "Muting held for more than 25 s stops the machine.",
         lambda s, m, o, c: z3.Implies(z3.And(m.mute_on, s.mem.mute_on, s.inp.t - s.mem.mute_t0 > c["P"].max_mute_s),
                                       _stop(o))),
    # ---------------------------------------------------------------- SR-12
    Prop("SR-12a", "SR-12", "The implement (boom, bucket) is never enabled during recovery.",
         lambda s, m, o, c: z3.Implies(as_bool(o.implement_enable), z3.Not(nz(m.recovery)))),
    Prop("SR-12b", "SR-12", "During recovery from a front stop, moving forward stops the machine; from a rear "
         "stop, reversing stops it.",
         lambda s, m, o, c: z3.And(
             z3.Implies(z3.And(has(m.recovery, FIELD_FRONT), c["v"] > c["P"].standstill), _stop(o)),
             z3.Implies(z3.And(has(m.recovery, FIELD_REAR), c["v"] < -c["P"].standstill), _stop(o)))),
    Prop("SR-12c", "SR-12", "During recovery from a side stop, an occupied contact band stops the machine "
         "(outside the access-controlled zones).",
         lambda s, m, o, c: z3.Implies(z3.And(has(m.recovery, FIELD_SIDE), s.inp.side_contact, z3.Not(c["exempt"])),
                                       _stop(o))),
    Prop("SR-12d", "SR-12", "Recovery exists only in TELEOP.",
         lambda s, m, o, c: z3.Implies(nz(m.recovery), s.inp.mode == TELEOP)),
    Prop("SR-12e", "SR-12", "While recovering with the brakes released, the speed limit is creep speed (0.5 m/s, "
         "0.3 m/s for a side stop).",
         lambda s, m, o, c: z3.Implies(z3.And(nz(m.recovery), as_bool(o.brake_release)),
                                       o.speed_limit <= c["P"].recovery_speed)),
    # ---------------------------------------------------------------- SR-13
    Prop("SR-13", "SR-13", "Every transition into a stop and every reset is written to the safety log.",
         lambda s, m, o, c: z3.And(
             z3.Implies(z3.And(s.mem.state < PROTECTIVE_STOP, o.state >= PROTECTIVE_STOP),
                        z3.Or(*[as_bool(cond) for cond, kind, _ in o.events if kind == "stop"])),
             z3.Implies(z3.And(s.mem.state >= PROTECTIVE_STOP, o.state < PROTECTIVE_STOP),
                        z3.Or(*[as_bool(cond) for cond, kind, _ in o.events if kind == "reset"])))),
    # ---------------------------------------------------------------- SR-16
    Prop("SR-16a", "SR-16", "Outside the dig zone, anyone between the front tyres and the bucket (near field at any "
         "boom position; mid field at any boom position, except beside a docked truck) or under the raised bucket "
         "(far field, except beside a docked truck) stops travel and holds the boom, unless backing away in recovery.",
         lambda s, m, o, c: z3.Implies(z3.And(z3.Not(s.inp.in_dig_zone),
                                              z3.Or(s.inp.under_near,
                                                    z3.And(z3.Or(s.inp.under_mid,
                                                                 z3.And(s.inp.boom_raised, s.inp.under_far)),
                                                           z3.Not(c["exempt"]))),
                                              z3.Not(has(m.recovery, UNDER_BUCKET))),
                                       z3.And(_stop(o), z3.Not(as_bool(o.implement_enable))))),
    Prop("SR-16c", "SR-16", "While reversing, anyone beside the front frame stops the machine (outside the dig zone "
         "and a docked truck), unless backing away from a side stop in recovery.",
         lambda s, m, o, c: z3.Implies(z3.And(s.inp.front_side, c["v"] < -c["P"].standstill, z3.Not(c["exempt"]),
                                              z3.Not(has(m.recovery, FIELD_SIDE))), _stop(o))),
    Prop("SR-16b", "SR-16", "During recovery from an under-bucket stop, moving forward stops the machine, and the boom "
         "stays held.",
         lambda s, m, o, c: z3.And(
             z3.Implies(z3.And(has(m.recovery, UNDER_BUCKET), c["v"] > c["P"].standstill), _stop(o)),
             z3.Implies(has(m.recovery, UNDER_BUCKET), z3.Not(as_bool(o.implement_enable))))),
    # ---------------------------------------------------------------- SR-14
    Prop("SR-14a", "SR-14", "The dock field set is never active while the truck's cab door is open.",
         lambda s, m, o, c: z3.Implies(o.field_set == DOCK_SET, z3.Not(s.inp.truck_door_open))),
    Prop("SR-14b", "SR-14", "The cab door opening while docking in the dock zone stops the machine.",
         lambda s, m, o, c: z3.Implies(z3.And(s.inp.truck_door_open, s.mem.field_set == DOCK_SET, s.inp.in_dock_zone),
                                       _stop(o))),
]


def _model_dict(model, s: Symbols, extra: dict) -> dict:
    def val(x):
        v = model.eval(x, model_completion=True)
        if z3.is_true(v) or z3.is_false(v):
            return z3.is_true(v)
        if z3.is_bv_value(v):
            return v.as_long()
        if z3.is_int_value(v):
            return v.as_long()
        if z3.is_rational_value(v):
            return round(float(v.numerator_as_long()) / float(v.denominator_as_long()), 4)
        if z3.is_algebraic_value(v):
            return round(float(v.approx(6).as_fraction()), 4)
        return str(v)
    out = {k: val(getattr(s.inp, k)) for k in vars(s.inp)}
    out.update({"mem." + k: val(getattr(s.mem, k)) for k in vars(s.mem) if isinstance(getattr(s.mem, k), z3.ExprRef)})
    out.update({k: val(x) for k, x in extra.items()})
    return out


def check(P: SafetyParams, T, prop: Prop | None = None) -> dict:
    """Prove one property (or, with prop=None, that INV is inductive)."""
    s = symbols()
    m, o = step(P, T, s)
    solver = z3.Solver()
    solver.set("timeout", 60_000)
    solver.add(*s.domain)
    solver.add(invariant(s.mem))
    if prop is None:
        goal = invariant(m)
    else:
        goal = prop.formula(s, m, o, _ctx(P, T, s))
    solver.add(z3.Not(goal))
    t0 = time.perf_counter()
    r = solver.check()
    status = "proven" if r == z3.unsat else "counterexample" if r == z3.sat else "unknown"
    res = {"status": status,
           "seconds": round(time.perf_counter() - t0, 3)}
    if r == z3.sat:
        mdl = solver.model()
        res["counterexample"] = _model_dict(mdl, s, {
            "out.state": o.state, "out.brake_release": as_bool(o.brake_release), "out.speed_limit": o.speed_limit,
            "out.field_set": o.field_set, "out.fs_v_protected": o.fs_v_protected, "post.recovery": m.recovery})
    return res


def init_holds() -> bool:
    s = z3.Solver()
    s.add(z3.Not(invariant(core.Memory(latched=bv(0), recovery=bv(0)))))
    return s.check() == z3.unsat


def configs():
    for name, budget in (("dry", StopBudget()), ("winter", StopBudget().winter())):
        for restart, params in (("auto-restart", SafetyParams()),
                                ("supervisor-only", SafetyParams(auto_restart_clear_s=None))):
            yield f"{name}/{restart}", params, field_table(budget)


def prove_all(only: set[str] | None = None) -> dict:
    report = {"invariant": {"init": init_holds()}, "properties": [], "configs": []}
    for cname, P, T in configs():
        report["configs"].append(cname)
        report["invariant"][cname] = check(P, T, None)
        for prop in PROPS:
            if only and prop.id not in only:
                continue
            r = check(P, T, prop)
            report["properties"].append({"id": prop.id, "req": prop.req, "text": prop.text, "config": cname, **r})
    return report


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "results" / "verification.json")
    args = ap.parse_args()
    rep = prove_all(set(args.only) if args.only else None)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(rep, indent=1))
    print("invariant:", {k: (v if isinstance(v, bool) else v["status"]) for k, v in rep["invariant"].items()})
    for p in rep["properties"]:
        line = f"{p['id']:7s} {p['config']:24s} {p['status']:15s} {p['seconds']:6.2f}s"
        if p["status"] != "proven":
            line += "  " + json.dumps({k: v for k, v in p.get("counterexample", {}).items()
                                      if not k.startswith("mem.") or k in ("mem.state", "mem.latched", "mem.recovery")})
        print(line)


if __name__ == "__main__":
    main()
