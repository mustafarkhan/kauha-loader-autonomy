"""The proof runs core.step on z3 terms; the machine runs it on floats.

This checks that both evaluations give the same answers: random concrete memory
and inputs are substituted into the symbolic formula and compared with the
concrete step.
"""
from __future__ import annotations

import random

import pytest
import z3

from kauha.safety import core
from kauha.safety.fields import field_table
from kauha.safety.plc import SafetyParams
from kauha.safety.stopping import StopBudget
from kauha.verify.symbolic import INPUT_BOOLS, INPUT_REALS, symbols
from kauha.verify.symbolic import step as sym_step


def _val(x):
    if isinstance(x, bool):
        return z3.BoolVal(x)
    if isinstance(x, int):
        return z3.IntVal(x)
    return z3.RealVal(repr(x))


def _py(e):
    e = z3.simplify(e)
    if z3.is_true(e) or z3.is_false(e):
        return z3.is_true(e)
    if z3.is_bv_value(e) or z3.is_int_value(e):
        return e.as_long()
    return float(e.numerator_as_long()) / float(e.denominator_as_long())


def _random_case(rng: random.Random):
    t = round(rng.uniform(1, 100), 3)
    v = rng.choice([0.0, 0.01, -0.01, round(rng.uniform(-3.4, 3.4), 3)])
    inp = {n: rng.random() < 0.3 for n in INPUT_BOOLS}
    inp.update(scanners_ok=rng.random() > 0.05, position_valid=rng.random() > 0.1, in_operating_area=rng.random() > 0.1)
    inp.update(t=t, speed_b=v, speed_a=v + rng.choice([0.0, 0.0, 0.4]), articulation=round(rng.uniform(-0.75, 0.75), 3),
               remote_hb_age=rng.choice([0.05, 0.6]), autonomy_cmd_age=rng.choice([0.05, 0.3]), mode=rng.randint(0, 2))
    st = rng.choice([0, 0, 1, 2, 3])
    lat = 0 if st < 2 else (rng.choice([1, 2, 4, 16, 512, 8192]) | (1024 if st == 3 else 0))
    mem = core.Memory(
        state=st, latched=lat, field_set=rng.choice([0, 3, 18, 27, 100, 101]), recovery=rng.choice([0, 0, 1, 2, 4, 5]),
        prev_reset=rng.random() < 0.5, over_on=rng.random() < 0.3, over_t0=t - rng.choice([0.0, 0.05]),
        disc_on=rng.random() < 0.3, disc_t0=t - rng.choice([0.1, 0.3]), posinv_on=rng.random() < 0.3,
        posinv_t0=t - rng.choice([1.0, 3.0]), mute_on=rng.random() < 0.3, mute_t0=t - rng.choice([5.0, 30.0]),
        clear_on=rng.random() < 0.5, clear_t0=t - rng.choice([1.0, 4.0]), warn_until=t + rng.choice([-1.0, 1.0]),
        allowed=3.0, prev_limit=rng.choice([0.0, 1.0, 2.97]), env_t0=t - 1.0, env_v0=rng.choice([0.0, 2.0]))
    return mem, inp


@pytest.mark.parametrize("budget", [StopBudget(), StopBudget().winter()], ids=["dry", "winter"])
def test_symbolic_and_concrete_step_agree(budget):
    P, T = SafetyParams(), field_table(budget)
    s = symbols()
    m_sym, o_sym = sym_step(P, T, s)
    names_in = list(INPUT_BOOLS) + list(INPUT_REALS) + ["mode"]
    mem_fields = [f for f in vars(s.mem) if isinstance(getattr(s.mem, f), z3.ExprRef)]
    rng = random.Random(7)
    for _ in range(150):
        mem, inp = _random_case(rng)
        subs = [(getattr(s.inp, n), _val(inp[n])) for n in names_in]
        subs += [(getattr(s.mem, f), z3.BitVecVal(getattr(mem, f), 16) if f in ("latched", "recovery")
                  else _val(getattr(mem, f))) for f in mem_fields]
        ns = type("I", (), inp)
        m_c, o_c = core.step(core.CONCRETE, P, T, mem, ns)
        for name in ("state", "cause", "speed_limit", "field_set", "brake_release", "implement_enable",
                     "reset_pending"):
            want = getattr(o_c, name)
            got = _py(z3.substitute(_lift_bool(getattr(o_sym, name)), *subs))
            assert got == pytest.approx(want, abs=1e-9), (name, mem, inp)
        for name in ("latched", "recovery", "field_set", "allowed", "warn_until"):
            want = getattr(m_c, name)
            got = _py(z3.substitute(_lift_bool(getattr(m_sym, name)), *subs))
            assert got == pytest.approx(want, abs=1e-9), (name, mem, inp)


def _lift_bool(x):
    if isinstance(x, bool):
        return z3.BoolVal(x)
    if isinstance(x, (int, float)):
        return z3.RealVal(x)
    return x
