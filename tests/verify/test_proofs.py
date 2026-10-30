"""Machine-checked proofs of the safety requirements on the PLC's own step function.

Each property is proved over every input and every memory state that satisfies
the inductive invariant, for dry and winter braking and with and without
automatic restart. A failure prints z3's counterexample.
"""
from __future__ import annotations

import json

import pytest

from kauha.verify.prove import PROPS, check, configs, init_holds

CONFIGS = list(configs())


def test_invariant_holds_at_power_up():
    assert init_holds()


@pytest.mark.parametrize("cfg", CONFIGS, ids=[c[0] for c in CONFIGS])
def test_invariant_is_inductive(cfg):
    _, P, T = cfg
    assert check(P, T, None)["status"] == "proven"


@pytest.mark.parametrize("prop", [pytest.param(p, marks=pytest.mark.req(p.req), id=p.id) for p in PROPS])
def test_property_proven_in_every_configuration(prop):
    for name, P, T in CONFIGS:
        r = check(P, T, prop)
        detail = json.dumps(r.get("counterexample"), indent=1)
        assert r["status"] == "proven", f"{prop.id} [{name}]: {prop.text}\n{detail}"
