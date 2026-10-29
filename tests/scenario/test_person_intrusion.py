"""Closed-loop safety-channel test: a person steps into the machine's path.

Only the safety channel is in the loop (loader dynamics, ray-cast scanners,
PLC, hardwired brake). The autonomy keeps requesting full speed, i.e. it is
assumed to have failed to see the person. The machine must stop before
reaching the person, at every speed band, both directions, dry and winter.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from kauha.config import load_machine
from kauha.safety.plc import CYCLE_S, Mode, SafetyInputs, SafetyPLC, State
from kauha.safety.stopping import StopBudget
from kauha.sim.loader import Loader, LoaderCommand, LoaderState
from kauha.sim.sensors import SafetyScanners

WALLS = np.array([(-100, -50, 100, -50), (-100, 50, 100, 50)], dtype=float)


def run_case(v: float, budget: StopBudget, decel: float, lateral: float = 0.0, seed: int = 0):
    cfg = load_machine()
    loader = Loader(cfg, LoaderState(v=v), dt=CYCLE_S)
    loader.state.boom, loader.state.bucket = -0.2, 0.55
    # give the twin the same braking capability the budget assumes
    object.__setattr__(cfg, "brake_decel", decel)
    plc = SafetyPLC(budget=budget)
    plc._allowed = plc._prev_limit = plc.field_cap
    scanners = SafetyScanners(np.random.default_rng(seed))
    # Person steps out 10 m ahead of the leading contour, after the machine is at speed.
    ahead = 5.1 + 10.0 if v > 0 else -(3.2 + 1.7 + 10.0)
    person = np.array([[ahead, lateral, 0.25]])
    min_gap, t_stop = math.inf, None
    for k in range(int(20 / CYCLE_S)):
        t = k * CYCLE_S
        s = loader.state
        rx, ry, rth = loader.rear_axle()
        poses = {"front": (s.x, s.y, s.theta), "rear": (rx, ry, rth + math.pi)}
        fs = plc.sets[plc.field_set]
        if k % 2 == 0:
            scan = scanners.scan(poses, fs, WALLS, person)
        out = plc.step(SafetyInputs(t=t, mode=Mode.AUTO, speed_a=s.v, speed_b=s.v, articulation=s.gamma,
                                    front_prot=scan.front_prot, front_warn=False, rear_prot=scan.rear_prot,
                                    rear_warn=False, side_prot=scan.side_prot))
        if out.state >= State.PROTECTIVE_STOP and t_stop is None:
            t_stop = t
        cmd = LoaderCommand(speed=v, brake=False) if out.brake_release else LoaderCommand(brake=True)
        loader.step(cmd)
        fp = loader.footprint()
        lead = max(p[0] for p in fp) if v > 0 else min(p[0] for p in fp)
        gap = (person[0, 0] - 0.25 - lead) if v > 0 else (lead - person[0, 0] - 0.25)
        min_gap = min(min_gap, gap)
        if t_stop is not None and abs(loader.state.v) < 1e-3:
            break
    return t_stop, min_gap


@pytest.mark.req("SR-02")
@pytest.mark.parametrize("v", [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, -0.5, -1.5, -2.5])
def test_stops_before_person_dry(v):
    b = StopBudget()
    v = math.copysign(min(abs(v), SafetyPLC(budget=b).field_cap), v)
    t_stop, gap = run_case(v, b, decel=b.decel_m_s2)
    assert t_stop is not None, "machine never stopped"
    assert gap > 0.0, f"contact at {v} m/s (gap {gap:.2f} m)"


@pytest.mark.req("SR-04")
@pytest.mark.parametrize("v", [1.0, 2.0, 2.6])
def test_stops_before_person_winter(v):
    b = StopBudget().winter()
    t_stop, gap = run_case(v, b, decel=b.decel_m_s2)
    assert t_stop is not None and gap > 0.0


def test_winter_speed_without_winter_cap_is_unsafe():
    """Documents why the winter cap exists: dry-weather speed on ice runs out of field."""
    b = StopBudget()
    _, gap = run_case(3.0, b, decel=1.2)
    assert gap < 0.5
