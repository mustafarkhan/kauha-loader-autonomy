"""Build the remote operations console prototype from three recorded twin shifts.

    python -m kauha.demo.console        # -> docs/results/kauha_console.html

The console plays back three simulated shifts as a fleet of three machines. It is a
prototype of the supervisor's station (Machinery Regulation Annex III 3.2.4: the
supervisor sees the machine's state and surroundings and can stop it). Stop, e-stop,
reset and teleop commands act on the playback; they do not run the twin.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from kauha.autonomy.planner import Phase

REPO_ROOT = Path(__file__).resolve().parents[2]
TEMPLATE = Path(__file__).with_name("console_template.html")
FLEET = [("KAUHA-01", "learned_s1", "Pit A", "Truck loading"),
         ("KAUHA-02", "carry_s1", "Pit B", "Load-and-carry to hopper"),
         ("KAUHA-03", "winter_s1", "Pit C", "Truck loading, winter braking")]
PHASES = [p.value for p in Phase]
STEP = 5          # keep every 5th 5 Hz frame: 1 Hz, interpolated in the page


def _r(v, n=2):
    return round(float(v), n)


def compact(frames: list[dict]) -> list[list]:
    out = []
    for f in frames[::STEP]:
        flags = (1 if f["dust"] else 0) | (2 if f["gnss"] else 0) | (4 if f["link"] else 0)
        out.append([_r(f["t"], 1), _r(f["x"]), _r(f["y"]), _r(f["th"], 3), _r(f["g"], 3), _r(f["v"]), _r(f["boom"]),
                    _r(f["bkt"]), _r(f["load"]), f["st"], f["fs"], PHASES.index(f["ph"]), f["mode"], flags,
                    _r(f["lim"]), f["tr"][0], _r(f["tr"][1], 1), [[_r(p[1], 1), _r(p[2], 1)] for p in f["ppl"]],
                    [[_r(o[0], 1), _r(o[1], 1), _r(o[2], 2)] for o in f.get("obs", [])]])
    return out


def fields(table: dict) -> dict:
    keep = ("prot", "warn", "rprot", "rwarn", "side", "rside")
    return {k: {"name": v["name"], "dir": v["dir"], **{f: [[_r(a), _r(b)] for a, b in v.get(f) or []] for f in keep}}
            for k, v in table.items()}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "docs" / "results" / "kauha_console.html")
    a = ap.parse_args()
    fleet, table = [], None
    for name, run, site, task in FLEET:
        d = REPO_ROOT / "results" / "runs" / run
        rep = json.loads((d / "replay.json").read_text())
        kp = json.loads((d / "kpis.json").read_text())
        log = json.loads((d / "safety_log.json").read_text())
        table = table or fields(rep["fields"])
        fleet.append({"name": name, "site_name": site, "task": task, "site": rep["site"],
                      "frames": compact(rep["frames"]),
                      "stops": rep["stops"], "interventions": rep["interventions"],
                      "log": [{k: e[k] for k in ("seq", "t", "kind", "data", "hash")} for e in log],
                      "kpi": {k: kp[k] for k in ("tonnes_per_hour", "cycle_time_median_s", "interventions_per_hour",
                                                  "protective_stops", "teleop_latency_p50_ms", "teleop_latency_p95_ms",
                                                  "teleop_budget_ms", "loc_rms_m", "log_chain_ok")}})
    data = {"fleet": fleet, "fields": table, "phases": PHASES, "machine": {"lf": 1.6, "lr": 1.6, "width": 2.55}}
    html = TEMPLATE.read_text().replace("/*__DATA__*/null", json.dumps(data, separators=(",", ":")))
    a.out.write_text(html)
    print(f"wrote {a.out} ({len(html) / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
