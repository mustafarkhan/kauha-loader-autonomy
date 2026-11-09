"""Check the CODESYS Structured Text safety program against the proved reference.

The safety logic exists twice: ``kauha/safety/core.py`` (the reference that the
z3 proofs, the SIL and the adversarial searches run on) and
``plc/codesys/src/FB_SafetyCore.st`` (what the PLC runs). This module compiles
the ST with the matiec IEC 61131-3 compiler, drives both with the same input
sequences and compares every output and every memory value after every cycle.
Doubles cross the process boundary as hex floats, so "equal" means bit-equal.

The input generator is biased toward the places where a port goes wrong: exact
band and threshold values, mode changes, link loss, resets pressed during and
after stops, field bits flickering, zones entered and left.

    python -m kauha.verify.st_equivalence --cycles 200000
"""
from __future__ import annotations

import argparse
import math
import os
import random
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path

from kauha.can.e2e import E2EReceiver, crc8_j1850
from kauha.config import REPO_ROOT
from kauha.safety import core
from kauha.safety.plc import Cause, SafetyInputs
from kauha.scale.profile import Profile, full_profile, scale_profile

ST_DIR = REPO_ROOT / "plc" / "codesys" / "src"
ST_SOURCES = ("DUT_Types.st", "FUN_Helpers.st", "FUN_Can.st", "FB_SafetyCore.st", "FB_SafetyNode.st")
HARNESS = REPO_ROOT / "plc" / "test" / "harness.c"

BOOL_INPUTS = ("front_prot", "front_warn", "rear_prot", "rear_warn", "side_prot", "side_contact", "scanners_ok",
               "remote_estop", "reset_req", "machine_estop", "dig_req", "dock_req", "in_operating_area",
               "in_dig_zone", "in_dock_zone", "truck_door_open", "boom_raised", "front_side", "under_near",
               "under_far", "under_mid", "position_valid")
OUT_FIELDS = ("state", "cause", "speed_limit", "field_set", "brake_release", "implement_enable", "reset_pending",
              "dock_blocked", "fs_direction", "fs_v_protected", "recovery", "ev_stop", "ev_recovery", "ev_reset")
MEM_FIELDS = ("state", "latched", "field_set", "recovery", "prev_reset", "over_on", "over_t0", "disc_on", "disc_t0",
              "posinv_on", "posinv_t0", "mute_on", "mute_t0", "clear_on", "clear_t0", "warn_until", "allowed",
              "prev_limit", "env_t0", "env_v0")
FLOAT_MEM = {"over_t0", "disc_t0", "posinv_t0", "mute_t0", "clear_t0", "warn_until", "allowed", "prev_limit",
             "env_t0", "env_v0"}


# ------------------------------------------------------------------ build
def find_iec2c() -> Path | None:
    for c in (os.environ.get("KAUHA_IEC2C"), shutil.which("iec2c"),
              str(Path.home() / "matiec" / "iec2c")):
        if c and Path(c).is_file():
            return Path(c)
    return None


def build(workdir: Path, iec2c: Path | None = None, mutation: tuple[str, str] | None = None) -> Path:
    """Compile the ST sources and the harness; returns the executable.

    ``mutation`` (old, new) edits FB_SafetyCore.st before compiling: the tests use
    it to show that the comparison catches a one-token fault."""
    iec2c = iec2c or find_iec2c()
    if iec2c is None:
        raise FileNotFoundError("matiec iec2c not found (set KAUHA_IEC2C)")
    lib = iec2c.parent / "lib"
    workdir.mkdir(parents=True, exist_ok=True)
    src = workdir / "kauha_safety.st"
    text = "\n".join((ST_DIR / s).read_text() for s in ST_SOURCES)
    if mutation:
        if text.count(mutation[0]) != 1:
            raise ValueError(f"mutation target not unique: {mutation[0]!r}")
        text = text.replace(*mutation)
    src.write_text(text)
    r = subprocess.run([str(iec2c), "-I", str(lib), "-T", str(workdir), str(src)], capture_output=True, text=True)
    if r.returncode != 0 or not (workdir / "POUS.c").exists():
        raise RuntimeError(f"iec2c failed:\n{r.stdout}\n{r.stderr}")
    exe = workdir / "harness"
    r = subprocess.run(["gcc", "-O1", "-ffp-contract=off", "-w", f"-I{lib / 'C'}", f"-I{workdir}", str(HARNESS),
                        "-o", str(exe), "-lm"], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"gcc failed:\n{r.stderr[-4000:]}")
    return exe


# ------------------------------------------------------------------ configuration
def config_values(p: Profile) -> list:
    t, prm = p.table, p.params
    ar = prm.auto_restart_clear_s
    return [prm.remote_timeout_s, prm.watchdog_timeout_s, prm.overspeed_tol, prm.decel_ramp, prm.ramp_reaction_s,
            prm.discrepancy_tol, prm.discrepancy_time_s, prm.position_invalid_s, prm.max_mute_s,
            ar if ar is not None else 0.0, prm.warning_hold_s, prm.auto_speed, prm.teleop_speed,
            prm.warning_speed, prm.standstill, prm.recovery_speed, prm.side_recovery_speed, t.standstill,
            t.dig_speed, t.dock_speed, t.speed_cap, int(ar is not None), *t.bands, *t.sectors, *t.v_protected,
            t.n_sectors, len(t.bands)]


def _fmt(x) -> str:
    if isinstance(x, bool):
        return "1" if x else "0"
    if isinstance(x, int):
        return str(x)
    return float(x).hex()


# ------------------------------------------------------------------ inputs
@dataclass
class _Sticky:
    p_on: float
    p_off: float
    value: bool = False

    def step(self, rnd: random.Random) -> bool:
        if self.value:
            if rnd.random() < self.p_off:
                self.value = False
        elif rnd.random() < self.p_on:
            self.value = True
        return self.value


def generate(p: Profile, n: int, seed: int) -> list[SafetyInputs]:
    """Random but structured input sequences (see the module docstring)."""
    rnd = random.Random(seed)
    t_tab = p.table
    prm = p.params
    edges = sorted({0.0, *t_tab.bands, *t_tab.v_protected, t_tab.standstill, prm.standstill, t_tab.dig_speed,
                    t_tab.dock_speed, t_tab.speed_cap, prm.warning_speed, prm.recovery_speed,
                    prm.side_recovery_speed, t_tab.dig_speed + prm.overspeed_tol,
                    t_tab.dock_speed + prm.overspeed_tol})
    top = t_tab.bands[-1] * 1.25
    sticky = {
        "front_prot": _Sticky(0.004, 0.02), "front_warn": _Sticky(0.01, 0.03), "rear_prot": _Sticky(0.003, 0.02),
        "rear_warn": _Sticky(0.008, 0.03), "side_prot": _Sticky(0.002, 0.03), "side_contact": _Sticky(0.003, 0.03),
        "scanners_ok": _Sticky(0.05, 0.0008, True), "remote_estop": _Sticky(0.0006, 0.02),
        "machine_estop": _Sticky(0.0005, 0.02), "dig_req": _Sticky(0.004, 0.006), "dock_req": _Sticky(0.004, 0.006),
        "in_operating_area": _Sticky(0.05, 0.0015, True), "in_dig_zone": _Sticky(0.004, 0.004),
        "in_dock_zone": _Sticky(0.004, 0.004), "truck_door_open": _Sticky(0.002, 0.02),
        "boom_raised": _Sticky(0.01, 0.01), "front_side": _Sticky(0.003, 0.03), "under_near": _Sticky(0.002, 0.04),
        "under_far": _Sticky(0.004, 0.03), "under_mid": _Sticky(0.002, 0.04),
        "position_valid": _Sticky(0.004, 0.0005, True),
    }
    mode = 1
    v = 0.0
    gamma = 0.0
    t = 0.0
    hb_age = cmd_age = 0.0
    link_down = cmd_down = False
    disc_off = 0.0
    out: list[SafetyInputs] = []
    # Phases: "calm" (hazards clear, so stops reset and the machine runs again),
    # "field" (one field held occupied, teleop and resets: the SR-12 recovery
    # paths) and "chaos" (everything flickers).
    phase, phase_left, held = "calm", 200, "front_prot"
    hazards = [k for k in sticky if k not in ("scanners_ok", "in_operating_area", "position_valid", "dig_req",
                                               "dock_req", "in_dig_zone", "in_dock_zone", "boom_raised")]
    for _ in range(n):
        phase_left -= 1
        if phase_left <= 0:
            phase = rnd.choice(("calm", "calm", "field", "field", "chaos", "dig", "dock"))
            phase_left = rnd.randint(80, 600) if phase != "dig" else int(prm.max_mute_s / 0.025 * rnd.uniform(0.5, 1.6))
            held = rnd.choice(("front_prot", "rear_prot", "side_prot", "under_near", "under_mid", "under_far",
                               "front_side", "side_contact"))
            if phase == "field":
                mode = rnd.choice((2, 2, 1))
            if phase != "chaos":
                link_down = cmd_down = False
                disc_off = 0.0
        t += 0.025 if rnd.random() > 0.01 else 0.05          # occasional cycle overrun
        if rnd.random() < 0.004:
            mode = rnd.choice((0, 1, 1, 2, 2))
        r = rnd.random()
        if phase == "field" and rnd.random() < 0.1:
            # creep edges of SR-12 recovery: exactly at and just above standstill
            e = rnd.choice((prm.standstill, t_tab.standstill, prm.recovery_speed, prm.side_recovery_speed))
            v = rnd.choice((e, math.nextafter(e, math.inf), math.nextafter(e, 0.0))) * rnd.choice((1, -1))
        elif phase != "chaos" and rnd.random() < 0.6:
            v = 0.0 if abs(v) < 0.02 * top else v * 0.9       # come to rest, so resets can happen
        elif r < 0.04:
            v = rnd.choice(edges) * rnd.choice((1, -1))       # exact thresholds, both directions
        elif r < 0.06:
            v = math.nextafter(rnd.choice(edges), math.inf) * rnd.choice((1, -1))
        elif r < 0.08:
            v = rnd.uniform(-top, top)
        else:
            v = max(-top, min(top, v + rnd.gauss(0, 0.02 * top)))
        if rnd.random() < 0.01:
            gamma = rnd.choice((*t_tab.sectors, rnd.uniform(-0.75, 0.75)))
        else:
            gamma = max(-0.75, min(0.75, gamma + rnd.gauss(0, 0.01)))
        if rnd.random() < 0.002:
            link_down = not link_down
        hb_age = hb_age + 0.025 if link_down else rnd.uniform(0, 0.12)
        if rnd.random() < 0.002:
            cmd_down = not cmd_down
        cmd_age = cmd_age + 0.025 if cmd_down else rnd.uniform(0, 0.06)
        if rnd.random() < 0.003:
            disc_off = rnd.choice((0.0, 0.0, prm.discrepancy_tol * rnd.uniform(0.5, 3.0)))
        b = {k: s.step(rnd) for k, s in sticky.items()}
        if phase != "chaos":
            for k in hazards:
                b[k] = False
            b.update(scanners_ok=True, in_operating_area=True)
            if phase != "calm":
                b["position_valid"] = True
            if phase == "field":
                b[held] = True
                if held in ("under_far",):
                    b["boom_raised"] = True
            if phase in ("dig", "dock"):
                door = phase == "dock" and sticky["truck_door_open"].step(rnd)
                b.update(dig_req=phase == "dig", in_dig_zone=phase == "dig", dock_req=phase == "dock",
                         in_dock_zone=phase == "dock", truck_door_open=door)
                mode = 1
                if rnd.random() < 0.5:
                    v = rnd.uniform(0, t_tab.dig_speed if phase == "dig" else t_tab.dock_speed)
        b["reset_req"] = rnd.random() < (0.03 if phase == "chaos" else 0.08)
        out.append(SafetyInputs(t=t, mode=core.IDLE if mode == 0 else mode, speed_a=v + disc_off, speed_b=v,
                                articulation=gamma, remote_hb_age=hb_age, autonomy_cmd_age=cmd_age, **b))
    return out


def scripted(p: Profile) -> list[SafetyInputs]:
    """Hand-written sequences for the edges random inputs rarely reach exactly:
    SR-12 recovery at and around the creep and standstill speeds, for every
    field cause and both directions, and SR-10 muting running past its limit."""
    prm, tab = p.params, p.table
    seq: list[SafetyInputs] = []
    t = 0.0
    calm = dict(mode=core.AUTO, speed_a=0.0, speed_b=0.0, articulation=0.0, remote_hb_age=0.01,
                autonomy_cmd_age=0.01)

    def run(cycles: int, **kw) -> None:
        nonlocal t
        for _ in range(cycles):
            t += 0.025
            d = {**calm, **kw}
            if "speed_b" in kw and "speed_a" not in kw:
                d["speed_a"] = kw["speed_b"]
            seq.append(SafetyInputs(t=t, **d))

    speeds = []
    for e in (prm.standstill, tab.standstill, prm.recovery_speed, prm.side_recovery_speed):
        speeds += [e, math.nextafter(e, math.inf), math.nextafter(e, 0.0)]
    held_fields = ({"front_prot": True}, {"rear_prot": True}, {"side_prot": True}, {"side_contact": True},
                   {"under_near": True}, {"under_mid": True}, {"boom_raised": True, "under_far": True},
                   {"front_side": True})
    for held in held_fields:
        for sign in (1, -1):
            for v in speeds:
                run(4, mode=core.TELEOP)                                   # calm, running
                run(3, mode=core.TELEOP, speed_b=sign * 0.3, **held)       # field trips while moving
                run(3, mode=core.TELEOP, **held)                           # stop to rest, field still occupied
                run(1, mode=core.TELEOP, reset_req=True, **held)           # supervisor reset -> recovery
                run(4, mode=core.TELEOP, speed_b=sign * v, **held)         # creep edge
                run(4, mode=core.TELEOP, speed_b=sign * v)                 # field clears while creeping
                run(3, mode=core.TELEOP)
                run(1, mode=core.TELEOP, reset_req=True)
                run(2, mode=core.TELEOP)
    # SR-10 muting past its time limit, then a reset attempt while the request continues.
    mute = dict(dig_req=True, in_dig_zone=True, speed_b=tab.dig_speed * 0.5)
    run(int(prm.max_mute_s / 0.025) + 20, **mute)
    run(1, reset_req=True, **mute)
    run(10)
    run(1, reset_req=True)
    run(5)
    return seq


# ------------------------------------------------------------------ run both
def reference(p: Profile, seq: list[SafetyInputs]) -> list[tuple]:
    mem = core.Memory()
    rows = []
    for i in seq:
        mem, o = core.step(core.CONCRETE, p.params, p.table, mem, i)
        ev = {"stop": False, "recovery": False, "reset": False}
        for cond, kind, _ in o.events:
            ev[kind] = ev[kind] or bool(cond)
        outs = (int(o.state), int(o.cause), float(o.speed_limit), int(o.field_set), bool(o.brake_release),
                bool(o.implement_enable), bool(o.reset_pending), bool(o.dock_blocked), int(o.fs_direction),
                float(o.fs_v_protected), int(mem.recovery), ev["stop"], ev["recovery"], ev["reset"])
        mems = tuple(float(getattr(mem, f)) if f in FLOAT_MEM else (bool(getattr(mem, f)) if isinstance(
            getattr(mem, f), bool) else int(getattr(mem, f))) for f in MEM_FIELDS)
        rows.append(outs + mems)
    return rows


def run_st(exe: Path, p: Profile, seq: list[SafetyInputs]) -> list[tuple]:
    lines = [" ".join(_fmt(x) for x in config_values(p))]
    for i in seq:
        vals = [i.t, int(i.mode), i.speed_a, i.speed_b, i.articulation, *(bool(getattr(i, b)) for b in BOOL_INPUTS),
                i.remote_hb_age, i.autonomy_cmd_age]
        lines.append(" ".join(_fmt(x) for x in vals))
    r = subprocess.run([str(exe), "core"], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
    rows = []
    kinds = ("i", "i", "f", "i", "b", "b", "b", "b", "i", "f", "i", "b", "b", "b",
             "i", "i", "i", "i", "b", "b", "f", "b", "f", "b", "f", "b", "f", "b", "f", "f", "f", "f", "f", "f")
    for line in r.stdout.splitlines():
        parts = line.split()
        rows.append(tuple(float.fromhex(x) if k == "f" else (x != "0" if k == "b" else int(x))
                          for x, k in zip(parts, kinds)))
    return rows


@dataclass
class Report:
    profile: str
    cycles: int
    mismatches: int
    first: str
    causes_seen: int     # bit mask of every cause latched during the run
    states_seen: list
    events: dict


NAMES = OUT_FIELDS + tuple(f"mem.{m}" for m in MEM_FIELDS)


def compare(exe: Path, p: Profile, n: int, seed: int) -> Report:
    seq = scripted(p)
    t0 = seq[-1].t
    seq += [replace(i, t=i.t + t0) for i in generate(p, n, seed)]
    ref = reference(p, seq)
    st = run_st(exe, p, seq)
    mism = 0
    first = ""
    for k, (a, b) in enumerate(zip(ref, st)):
        if a != b:
            mism += 1
            if not first:
                diff = [f"{NAMES[j]}: ref {x!r} st {y!r}" for j, (x, y) in enumerate(zip(a, b)) if x != y]
                first = f"cycle {k} t={seq[k].t:.3f}: " + "; ".join(diff)
    if len(st) != len(ref):
        mism += abs(len(st) - len(ref))
        first = first or f"length {len(st)} vs {len(ref)}"
    causes = 0
    for row in ref:
        causes |= row[1]
    return Report(p.name, n, mism, first, causes, sorted({row[0] for row in ref}),
                  {"stop": sum(r[11] for r in ref), "recovery": sum(r[12] for r in ref),
                   "reset": sum(r[13] for r in ref)})


def profiles() -> list[Profile]:
    out = []
    for p in (full_profile(), scale_profile("nanoscan3"), scale_profile("evaluator")):
        out.append(p)
        out.append(replace(p, name=p.name + "/supervisor-only",
                           params=replace(p.params, auto_restart_clear_s=None)))
    return out


# ------------------------------------------------------------------ bus functions
def check_crc(exe: Path, n: int, seed: int) -> int:
    rnd = random.Random(seed)
    cases = [(rnd.randrange(256), [rnd.randrange(256) for _ in range(8)]) for _ in range(n)]
    r = subprocess.run([str(exe), "crc"], input="\n".join(f"{i} " + " ".join(map(str, d)) for i, d in cases) + "\n",
                       capture_output=True, text=True, check=True)
    got = [int(x) for x in r.stdout.split()]
    return sum(g != crc8_j1850(bytes([i]) + bytes(d[:7])) for g, (i, d) in zip(got, cases)) + abs(len(got) - n)


def check_e2e(exe: Path, n: int, seed: int) -> int:
    from kauha.can.e2e import protect
    rnd = random.Random(seed)
    data_id, timeout = 0x42, 0.15
    rx = E2EReceiver(data_id, timeout)
    lines = [f"{data_id} {timeout.hex()} 2"]
    expect = []
    t, ctr = 0.0, 0
    for _ in range(n):
        t += rnd.choice((0.01, 0.025, 0.05, 0.2))
        new = rnd.random() < 0.8
        r = rnd.random()
        ctr = (ctr + (0 if r < 0.05 else 3 if r < 0.1 else 2 if r < 0.15 else 1)) % 16
        payload = bytearray(rnd.randrange(256) for _ in range(8))
        frame = bytearray(protect(payload, data_id, ctr))
        if rnd.random() < 0.05:
            frame[rnd.randrange(8)] ^= 1 << rnd.randrange(8)
        status = None
        if new:
            status = rx.check(bytes(frame), t)
        code = {"ok": 0, "ok_some_lost": 1, "repeated": 2, "wrong_sequence": 3, "crc_error": 4}
        expect.append((code[status.value] if status else None, rx.fresh(t)))
        lines.append(f"{t.hex()} {int(new)} " + " ".join(map(str, frame)))
    r = subprocess.run([str(exe), "e2e"], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
    bad = 0
    for (st, fresh), line in zip(expect, r.stdout.splitlines()):
        s, _acc, f, _age = line.split()
        if (st is not None and int(s) != st) or bool(int(f)) != fresh:
            bad += 1
    return bad


def check_status_frame(exe: Path, n: int, seed: int) -> int:
    from kauha.can.codec import Codec
    from kauha.can.e2e import protect
    codec = Codec()
    msg = codec.db.get_message_by_name("SafetyStatus")
    data_id = codec.data_id("SafetyStatus")
    rnd = random.Random(seed)
    cases = []
    for _ in range(n):
        o = dict(State=rnd.randrange(4), Cause=rnd.randrange(1 << 15), SpeedLimit=round(rnd.uniform(0, 3.2), 3),
                 ActiveFieldSet=rnd.choice((0, 1, 17, 40, 100, 101)), ResetPending=rnd.randrange(2),
                 DockBlocked=rnd.randrange(2))
        cases.append((o, rnd.randrange(16)))
    lines = [f"{o['State']} {o['Cause']} {o['SpeedLimit'].hex()} {o['ActiveFieldSet']} {o['ResetPending']} "
             f"{o['DockBlocked']} {c} {data_id}" for o, c in cases]
    r = subprocess.run([str(exe), "status"], input="\n".join(lines) + "\n", capture_output=True, text=True,
                       check=True)
    bad = 0
    for (o, c), line in zip(cases, r.stdout.splitlines()):
        full = {s.name: 0 for s in msg.signals}
        full.update(o)
        want = protect(bytearray(msg.encode(full, strict=True)), data_id, c)
        if bytes(int(x) for x in line.split()) != want:
            bad += 1
    return bad


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cycles", type=int, default=100_000)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as d:
        exe = build(Path(d))
        total = 0
        seen = 0
        for k, p in enumerate(profiles()):
            rep = compare(exe, p, a.cycles, a.seed + k)
            total += rep.mismatches
            seen |= rep.causes_seen
            print(f"{rep.profile:36s} cycles {rep.cycles:7d}  mismatches {rep.mismatches}  causes seen "
                  f"{bin(rep.causes_seen).count('1')}/15  states {rep.states_seen}  events {rep.events}"
                  + (f"\n    first: {rep.first}" if rep.first else ""))
        missing = [c.name for c in Cause if c and not (seen & c)]
        print(f"causes exercised over all profiles: {bin(seen).count('1')}/15" + (f", missing {missing}" if missing
                                                                                 else ""))
        for name, fn in (("crc", check_crc), ("e2e", check_e2e), ("SafetyStatus frame", check_status_frame)):
            bad = fn(exe, 20_000, a.seed)
            total += bad
            print(f"{name:36s} cases 20000  mismatches {bad}")
    raise SystemExit(1 if total else 0)


if __name__ == "__main__":
    main()

