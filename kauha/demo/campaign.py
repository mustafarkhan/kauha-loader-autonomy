"""Run the shift scenario for several controllers and seeds, and aggregate KPIs.

A single simulated shift is one sample: small timing differences change which
truck a person walks past or where a stop happens. Comparisons are therefore
made on several seeds and reported as mean and range.

    python -m kauha.demo.campaign --seeds 1 2 3 --workers 2
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

from kauha.config import REPO_ROOT

CONFIGS = {"learned": ["--controller", "learned"], "scripted": ["--controller", "scripted"],
           "winter": ["--controller", "learned", "--winter"]}
NUMERIC = ("tonnes", "tonnes_per_hour", "cycles", "trucks", "cycle_time_median_s", "clean_cycle_time_median_s",
           "dig_time_mean_s", "fill_factor_mean", "truck_wait_s", "interventions_per_hour", "protective_stops",
           "false_stops", "min_person_distance_moving_m", "person_field_entries_while_moving",
           "person_under_raised_bucket_moving_s", "loc_rms_m",
           "loc_rms_gnss_outage_m", "teleop_latency_p95_ms", "fuel_l_per_t", "e2e_errors")


def run(cfg: str, seed: int, minutes: float, out: Path) -> Path:
    d = out / f"{cfg}_s{seed}"
    cmd = [sys.executable, "-m", "kauha.demo.shift", *CONFIGS[cfg], "--seed", str(seed),
           "--minutes", str(minutes), "--out", str(d)]
    subprocess.run(cmd, check=True, cwd=REPO_ROOT, stdout=subprocess.DEVNULL)
    return d


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--minutes", type=float, default=60.0)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "results" / "runs")
    ap.add_argument("--aggregate-only", action="store_true")
    args = ap.parse_args()
    if not args.aggregate_only:
        jobs = [(c, s) for c in CONFIGS for s in args.seeds]
        with ThreadPoolExecutor(args.workers) as ex:
            list(ex.map(lambda j: run(j[0], j[1], args.minutes, args.out), jobs))
    agg = {}
    for c in CONFIGS:
        ks = [json.loads((args.out / f"{c}_s{s}" / "kpis.json").read_text()) for s in args.seeds]
        agg[c] = {"seeds": args.seeds, "runs": ks, "summary": {}}
        for key in NUMERIC:
            v = np.array([k[key] for k in ks], dtype=float)
            agg[c]["summary"][key] = {"mean": float(v.mean()), "min": float(v.min()), "max": float(v.max())}
        reasons: dict[str, int] = {}
        causes: dict[str, int] = {}
        for k in ks:
            for r, n in k["interventions_by_reason"].items():
                reasons[r] = reasons.get(r, 0) + n
            for r, n in k["stops_by_cause"].items():
                causes[r] = causes.get(r, 0) + n
        agg[c]["interventions_by_reason_total"] = reasons
        agg[c]["stops_by_cause_total"] = causes
        agg[c]["person_stop_response_s"] = [x for k in ks for x in k["person_stop_response_s"]]
        agg[c]["fill_by_pile"] = {p: float(np.nanmean([k["fill_factor_by_pile"].get(p, np.nan) for k in ks]))
                                  for p in ("A", "B")}
        agg[c]["log_chain_ok"] = all(k["log_chain_ok"] for k in ks)
    (args.out.parent / "campaign.json").write_text(json.dumps(agg, indent=1))
    for c, a in agg.items():
        s = a["summary"]
        print(c, {k: round(s[k]["mean"], 3) for k in ("tonnes", "clean_cycle_time_median_s", "fill_factor_mean",
                                                       "interventions_per_hour", "protective_stops",
                                                       "min_person_distance_moving_m")})


if __name__ == "__main__":
    main()
