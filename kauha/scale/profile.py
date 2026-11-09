"""The 1:8 scale testbed: one place where every scaled number comes from.

The testbed is Froude-scaled from the full-size reference machine: lengths by
1/k, speeds and times by 1/sqrt(k), masses and forces by 1/k^3, torques by
1/k^4; accelerations and angles are unchanged. Under this scaling gravity,
inertia and braking keep their ratios, so stopping distances, field lengths and
tipping behaviour scale like the geometry (Brennan & Alleyne 2001 used the same
argument for their 1:13 vehicle-dynamics testbed).

What does not scale, and is therefore taken from the real testbed hardware
instead of the formula:

- reaction times of real parts (scanner response, PLC cycle, relays);
- communication timeouts (heartbeat, watchdog), kept at their full-size values;
- soil: cohesion and grain size do not scale, so bucket-filling forces on the
  testbed are not similar to the full machine. The testbed is for navigation,
  the V-cycle, safety functions and the operator workflow, not for soil forces.

``python -m kauha.scale.profile`` writes the scaled profile, the field sets and
the CODESYS configuration list used by the PLC program.
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path

from kauha.config import REPO_ROOT, MachineConfig, load_machine
from kauha.safety.fields import FULL, FieldDesign, Geometry, build_field_sets, field_table
from kauha.safety.plc import SafetyParams
from kauha.safety.stopping import StopBudget

K = 8.0
OUT = REPO_ROOT / "hardware" / "scale" / "generated"


# ------------------------------------------------------------------ scaling laws
@dataclass(frozen=True)
class Froude:
    """Froude similarity for a length ratio k (full / model)."""
    k: float = K

    def length(self, x: float) -> float:
        return x / self.k

    def speed(self, v: float) -> float:
        return v / math.sqrt(self.k)

    def time(self, t: float) -> float:
        return t / math.sqrt(self.k)

    def rate(self, w: float) -> float:          # angular rate (rad/s)
        return w * math.sqrt(self.k)

    def mass(self, m: float) -> float:
        return m / self.k ** 3

    def force(self, f: float) -> float:
        return f / self.k ** 3

    def torque(self, t: float) -> float:
        return t / self.k ** 4

    def power(self, p: float) -> float:
        return p / self.k ** 3.5

    def volume(self, v: float) -> float:
        return v / self.k ** 3


F = Froude()


def scale_machine(m: MachineConfig, f: Froude = F) -> MachineConfig:
    L, V, T, R = f.length, f.speed, f.time, f.rate
    return replace(
        m, operating_mass_kg=f.mass(m.operating_mass_kg), lf=L(m.lf), lr=L(m.lr), track_width=L(m.track_width),
        body_length=L(m.body_length), body_width=L(m.body_width), articulation_rate=R(m.articulation_rate),
        max_speed=V(m.max_speed), actuator_latency=T(m.actuator_latency), bucket_volume=f.volume(m.bucket_volume),
        bucket_width=L(m.bucket_width), boom_pivot_forward=L(m.boom_pivot_forward),
        boom_pivot_height=L(m.boom_pivot_height), boom_length=L(m.boom_length), bucket_length=L(m.bucket_length),
        boom_rate=R(m.boom_rate), bucket_rate=R(m.bucket_rate), breakout_force=f.force(m.breakout_force),
        auto_max_speed=V(m.auto_max_speed), warning_speed=V(m.warning_speed),
        teleop_max_speed=V(m.teleop_max_speed))


# ------------------------------------------------------------------ testbed stop budget
# Reaction times of the real testbed parts. These are planning values: the
# commissioning tests (docs/scale/test_plan.md, T3) measure them, and the
# measured values replace these before the field sets are loaded.
NANOSCAN3_RESPONSE_S = 0.070       # SICK nanoScan3 I/O data sheet, basic response time
EVALUATOR_RESPONSE_S = 0.200       # 10 Hz hobby lidar + software field evaluator (2 scans)
RELAY_DROPOUT_S = 0.020            # automotive changeover relay release
TESTBED_DECEL_M_S2 = 2.0           # dynamic braking through the 50.9:1 gearboxes (to be measured)


def testbed_budget(scanner: str = "nanoscan3", f: Froude = F) -> StopBudget:
    response = {"nanoscan3": NANOSCAN3_RESPONSE_S, "evaluator": EVALUATOR_RESPONSE_S}[scanner]
    full = StopBudget()
    return StopBudget(scanner_response_s=response, plc_cycle_s=0.025, plc_cycles=2,
                      actuator_latency_s=RELAY_DROPOUT_S, brake_buildup_s=0.0,
                      decel_m_s2=TESTBED_DECEL_M_S2, margin_m=f.length(full.margin_m))


def testbed_geometry(f: Froude = F) -> Geometry:
    g = Geometry()
    return Geometry(**{x.name: f.length(getattr(g, x.name)) for x in fields(Geometry)})


def testbed_design(f: Froude = F) -> FieldDesign:
    d = FULL
    L, V, T = f.length, f.speed, f.time
    return replace(
        d, protective_range=L(d.protective_range), warning_range=L(d.warning_range), side_margin=L(d.side_margin),
        side_width=L(d.side_width), min_protective=L(d.min_protective), warning_extra=L(d.warning_extra),
        warning_time=T(d.warning_time), contact_band=L(d.contact_band), bands=tuple(V(v) for v in d.bands),
        dig_speed=V(d.dig_speed), dock_speed=V(d.dock_speed), standstill=V(d.standstill),
        speed_cap_margin=V(d.speed_cap_margin), lf=L(d.lf), lr=L(d.lr))


def testbed_params(f: Froude = F) -> SafetyParams:
    p = SafetyParams()
    V, T = f.speed, f.time
    # Communication timeouts are not scaled: the testbed uses the same links.
    return replace(
        p, overspeed_tol=V(p.overspeed_tol), discrepancy_tol=V(p.discrepancy_tol), max_mute_s=T(p.max_mute_s),
        auto_restart_clear_s=T(p.auto_restart_clear_s) if p.auto_restart_clear_s is not None else None,
        warning_hold_s=T(p.warning_hold_s), auto_speed=V(p.auto_speed), teleop_speed=V(p.teleop_speed),
        warning_speed=V(p.warning_speed), standstill=V(p.standstill), recovery_speed=V(p.recovery_speed),
        side_recovery_speed=V(p.side_recovery_speed), ramp_reaction_s=0.10)


@dataclass(frozen=True)
class Profile:
    name: str
    params: SafetyParams
    budget: StopBudget
    design: FieldDesign
    geometry: Geometry

    @property
    def table(self):
        return field_table(self.budget, self.design)

    def field_sets(self):
        return build_field_sets(self.budget, self.geometry, self.design)


def full_profile() -> Profile:
    return Profile("full", SafetyParams(), StopBudget(), FULL, Geometry())


def scale_profile(scanner: str = "nanoscan3", f: Froude = F) -> Profile:
    return Profile(f"scale{int(f.k)}_{scanner}", testbed_params(f), testbed_budget(scanner, f), testbed_design(f),
                   testbed_geometry(f))


def check(p: Profile) -> list[str]:
    """Consistency rules a profile must keep before it is loaded into the PLC."""
    t, prm = p.table, p.params
    out = []
    if prm.side_recovery_speed > prm.recovery_speed:
        out.append("side recovery creep is faster than the recovery creep")
    if t.speed_cap <= prm.warning_speed:
        out.append("the field speed cap is below the warning-field speed")
    if t.dig_speed > t.v_protected[0] + 1e-9 and t.dig_speed > t.speed_cap:
        out.append("dig speed above what the fields protect")
    if prm.overspeed_tol + 1e-12 < p.design.speed_cap_margin - 1e-9:
        out.append("speed cap margin and overspeed tolerance differ")
    for i in range(len(t.bands) - 1):
        if not t.bands[i] < t.bands[i + 1]:
            out.append("speed bands are not increasing")
    return out


# ------------------------------------------------------------------ exports
def _st_real(x: float) -> str:
    return f"{x:.9g}" if "e" not in f"{x:.9g}" else f"{x:.9e}"


def st_config(p: Profile, name: str) -> str:
    """A CODESYS global variable list holding one profile's configuration."""
    t, prm = p.table, p.params
    auto_restart = prm.auto_restart_clear_s
    vals = {
        "remote_timeout_s": prm.remote_timeout_s, "watchdog_timeout_s": prm.watchdog_timeout_s,
        "overspeed_tol": prm.overspeed_tol, "decel_ramp": prm.decel_ramp, "ramp_reaction_s": prm.ramp_reaction_s,
        "discrepancy_tol": prm.discrepancy_tol, "discrepancy_time_s": prm.discrepancy_time_s,
        "position_invalid_s": prm.position_invalid_s, "max_mute_s": prm.max_mute_s,
        "auto_restart_clear_s": auto_restart if auto_restart is not None else 0.0,
        "warning_hold_s": prm.warning_hold_s, "auto_speed": prm.auto_speed, "teleop_speed": prm.teleop_speed,
        "warning_speed": prm.warning_speed, "standstill": prm.standstill, "recovery_speed": prm.recovery_speed,
        "side_recovery_speed": prm.side_recovery_speed,
        "t_standstill": t.standstill, "dig_speed": t.dig_speed, "dock_speed": t.dock_speed,
        "speed_cap": t.speed_cap,
    }
    lines = [f"(* Generated by kauha.scale.profile for profile '{p.name}'. Do not edit by hand. *)",
             "VAR_GLOBAL CONSTANT", f"    {name} : ST_SafetyConfig := ("]
    items = [f"{k} := {_st_real(v)}" for k, v in vals.items()]
    items.append(f"auto_restart := {'TRUE' if auto_restart is not None else 'FALSE'}")
    items += [f"band{i} := {_st_real(v)}" for i, v in enumerate(t.bands)]
    items += [f"sector{i} := {_st_real(v)}" for i, v in enumerate(t.sectors)]
    items += [f"vprot{i} := {_st_real(v)}" for i, v in enumerate(t.v_protected)]
    items.append(f"n_sectors := {t.n_sectors}")
    items.append(f"n_bands := {len(t.bands)}")
    lines.append(",\n".join("        " + x for x in items))
    lines += ["    );", "END_VAR", ""]
    return "\n".join(lines)


def field_export(p: Profile) -> dict:
    sets = p.field_sets()
    out = {}
    for fid, fs in sets.items():
        d = asdict(fs)
        d["rear_protective"] = fs.rear_protective
        d["rear_warning"] = fs.rear_warning
        out[str(fid)] = {k: ([[round(a, 4), round(b, 4)] for a, b in v] if isinstance(v, list) else v)
                         for k, v in d.items()}
    return out


def summary(p: Profile) -> dict:
    t = p.table
    return {"profile": p.name, "params": asdict(p.params), "stop_budget": {**asdict(p.budget),
                                                                          "reaction_s": p.budget.reaction_s},
            "field_design": asdict(p.design), "geometry": asdict(p.geometry),
            "field_table": {"bands": t.bands, "sectors": t.sectors, "n_sectors": t.n_sectors,
                            "standstill": t.standstill, "dig_speed": t.dig_speed, "dock_speed": t.dock_speed,
                            "v_protected": t.v_protected, "field_cap": t.field_cap, "speed_cap": t.speed_cap},
            "n_field_sets": len(p.field_sets()), "checks": check(p) or "pass"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    machine = scale_machine(load_machine())
    profiles = [full_profile(), scale_profile("nanoscan3"), scale_profile("evaluator")]
    bad = {p.name: check(p) for p in profiles if check(p)}
    if bad:
        raise SystemExit(f"profile checks failed: {bad}")
    report = {"scale": K, "laws": "Froude: L/k, v and t /sqrt(k), m and F /k^3, torque /k^4",
              "machine": asdict(machine), "profiles": {p.name: summary(p) for p in profiles}}
    (a.out / "scale_profile.json").write_text(json.dumps(report, indent=2, default=str))
    for p in profiles:
        (a.out / f"fields_{p.name}.json").write_text(json.dumps(field_export(p), separators=(",", ":")))
    st = ["(* KAUHA safety configuration lists, one per profile. Select one in PRG_Safety. *)", ""]
    st += [st_config(p, f"CFG_{p.name.upper()}") for p in profiles]
    plc_dir = REPO_ROOT / "plc" / "codesys" / "src"
    plc_dir.mkdir(parents=True, exist_ok=True)
    (plc_dir / "GVL_Config.st").write_text("\n".join(st))
    for p in profiles:
        s = summary(p)["field_table"]
        print(f"{p.name:18s} reaction {p.budget.reaction_s:.3f} s  field cap {s['field_cap']:.3f} m/s  "
              f"speed cap {s['speed_cap']:.3f} m/s  auto {p.params.auto_speed:.3f} m/s  sets {len(p.field_sets())}")


if __name__ == "__main__":
    main()
