"""Run ``kauha.safety.core.step`` on z3 terms.

``Z3Ops`` gives ``core.step`` the same interface as ``core.CONCRETE``, but every
operation builds a z3 term. Calling ``step`` with a symbolic memory and symbolic
inputs therefore returns a formula for one PLC cycle over all possible inputs
and all memory states. Arithmetic is over the reals, not IEEE floats.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from types import SimpleNamespace

import z3

from kauha.safety import core

BV = core.CAUSE_BITS
ALL_CAUSES = (1 << 15) - 1
MOVING_SETS = 40                     # ids 1..40: 2 directions x 4 bands x 5 sectors


def _lift(x):
    if isinstance(x, z3.ExprRef):
        return x
    if isinstance(x, bool):
        return z3.BoolVal(x)
    if isinstance(x, int):
        return z3.IntVal(x)
    if isinstance(x, float):
        return z3.RealVal(x)
    raise TypeError(f"cannot lift {x!r}")


class Z3Ops:
    @staticmethod
    def If(c, a, b):  # noqa: N802
        if isinstance(c, bool):
            return a if c else b
        if isinstance(a, z3.BitVecRef) or isinstance(b, z3.BitVecRef):
            return z3.If(c, a, b)
        a, b = _lift(a), _lift(b)
        if a.sort() != b.sort():          # Int vs Real
            a, b = z3.ToReal(a) if a.is_int() else a, z3.ToReal(b) if b.is_int() else b
        return z3.If(c, a, b)

    @staticmethod
    def And(*xs):  # noqa: N802
        if any(x is False for x in xs):
            return False
        xs = [x for x in xs if x is not True]
        return True if not xs else xs[0] if len(xs) == 1 else z3.And(*xs)

    @staticmethod
    def Or(*xs):  # noqa: N802
        if any(x is True for x in xs):
            return True
        xs = [x for x in xs if x is not False]
        return False if not xs else xs[0] if len(xs) == 1 else z3.Or(*xs)

    @staticmethod
    def Not(x):  # noqa: N802
        return (not x) if isinstance(x, bool) else z3.Not(x)

    @staticmethod
    def Abs(x):  # noqa: N802
        return abs(x) if isinstance(x, (int, float)) else z3.If(x >= 0, x, -x)

    @classmethod
    def Min(cls, a, b):  # noqa: N802
        return cls.If(_lift(a) <= _lift(b), a, b)

    @classmethod
    def Max(cls, a, b):  # noqa: N802
        return cls.If(_lift(a) >= _lift(b), a, b)

    @staticmethod
    def bv(x: int):
        return z3.BitVecVal(x, BV)

    @staticmethod
    def nz(x):
        return x != z3.BitVecVal(0, BV)

    @staticmethod
    def has(x, bits: int):
        return (x & z3.BitVecVal(bits, BV)) != z3.BitVecVal(0, BV)

    @staticmethod
    def clear(x, bits: int):
        return x & z3.BitVecVal(~bits & ((1 << BV) - 1), BV)


Z3 = Z3Ops()

INPUT_BOOLS = ("front_prot", "front_warn", "rear_prot", "rear_warn", "side_prot", "side_contact", "scanners_ok",
               "remote_estop", "reset_req", "machine_estop", "dig_req", "dock_req", "in_operating_area",
               "in_dig_zone", "in_dock_zone", "truck_door_open", "position_valid", "boom_raised", "under_near",
               "under_far", "under_mid", "front_side")
INPUT_REALS = ("t", "speed_a", "speed_b", "articulation", "remote_hb_age", "autonomy_cmd_age")


@dataclass
class Symbols:
    mem: core.Memory
    inp: SimpleNamespace
    domain: list            # type constraints on memory and inputs


def symbols(prefix: str = "") -> Symbols:
    """Fresh symbolic memory and inputs, with their type/range constraints."""
    p = prefix
    i = SimpleNamespace(**{n: z3.Bool(p + n) for n in INPUT_BOOLS}, **{n: z3.Real(p + n) for n in INPUT_REALS},
                        mode=z3.Int(p + "mode"))
    m = core.Memory(
        state=z3.Int(p + "m.state"), latched=z3.BitVec(p + "m.latched", BV), field_set=z3.Int(p + "m.field_set"),
        recovery=z3.BitVec(p + "m.recovery", BV), prev_reset=z3.Bool(p + "m.prev_reset"),
        **{f.name: (z3.Bool if f.name.endswith("_on") else z3.Real)(p + "m." + f.name)
           for f in fields(core.Memory) if f.name.endswith(("_on", "_t0", "_until", "allowed", "_limit", "_v0"))})
    dom = [
        z3.And(i.mode >= 0, i.mode <= 2),
        i.remote_hb_age >= 0, i.autonomy_cmd_age >= 0,
        z3.And(m.state >= 0, m.state <= 3),
        z3.Or(z3.And(m.field_set >= 0, m.field_set <= MOVING_SETS), m.field_set == 100, m.field_set == 101),
        m.latched & ~z3.BitVecVal(ALL_CAUSES, BV) == 0,
        m.recovery & ~z3.BitVecVal(core.FIELD_CAUSES, BV) == 0,
        # timers started in the past; the cycle clock only moves forward
        m.over_t0 <= i.t, m.disc_t0 <= i.t, m.posinv_t0 <= i.t, m.mute_t0 <= i.t, m.clear_t0 <= i.t,
        m.env_t0 <= i.t, m.allowed >= 0, m.prev_limit >= 0, m.env_v0 >= 0,
    ]
    return Symbols(m, i, dom)


def step(P, T, s: Symbols):
    return core.step(Z3, P, T, s.mem, s.inp)
