"""Run a SIL scenario with real processes and optional fault injection.

    python -m kauha.sil.run --scenario person
    python -m kauha.sil.run --scenario kill_autonomy
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import random
import tempfile
import time
from pathlib import Path

from kauha.sil import nodes
from kauha.sil.common import Wiring

SCENARIOS = {
    # name: (duration s, speed m/s, person, process to kill, kill time s)
    "nominal": (8.0, 2.0, None, None, None),
    "person": (12.0, 2.5, {"t": 4.0, "ahead": 8.0}, None, None),
    "kill_autonomy": (9.0, 2.0, None, "autonomy", 5.0),
    "kill_remote": (9.0, 2.0, None, "remote", 5.0),
    "kill_plc": (9.0, 2.0, None, "plc", 5.0),
}


def run(name: str, workdir: Path | None = None) -> dict:
    duration, speed, person, victim, t_kill = SCENARIOS[name]
    wd = Path(workdir or tempfile.mkdtemp(prefix=f"kauha_sil_{name}_"))
    port = random.randint(20000, 40000)
    w = Wiring(f"239.74.{random.randint(1, 254)}.{random.randint(1, 254)}", port, port + 1)
    ctx = mp.get_context("spawn")
    epoch = time.monotonic() + 1.0          # give the spawned processes time to import
    procs = {
        "plc": ctx.Process(target=nodes.plc, args=(w, epoch, duration, str(wd / "plc.json"))),
        "machine": ctx.Process(target=nodes.machine, args=(w, epoch, duration, str(wd / "machine.jsonl"), person)),
        "remote": ctx.Process(target=nodes.remote, args=(w, epoch, duration)),
        "autonomy": ctx.Process(target=nodes.autonomy, args=(w, epoch, duration, speed)),
    }
    for p in procs.values():
        p.start()
    if victim:
        while time.monotonic() - epoch < t_kill:
            time.sleep(0.005)
        procs[victim].kill()
        t_killed = time.monotonic() - epoch
    for p in procs.values():
        p.join(timeout=duration + 20)
    rows = [json.loads(line) for line in (wd / "machine.jsonl").read_text().splitlines()]
    tail = rows[-1]
    rows = rows[:-1]
    plc_log = json.loads((wd / "plc.json").read_text()) if (wd / "plc.json").exists() else []
    res = {"scenario": name, "workdir": str(wd), "overruns_machine": tail.get("overruns"),
           "v_max": max(r["v"] for r in rows), "v_end": rows[-1]["v"],
           "stops": [(r["t"], r["kind"], r["data"].get("cause")) for r in plc_log if r["kind"] == "stop"]}
    gaps = [r["gap"] for r in rows if r["gap"] is not None]
    if gaps:
        res["min_gap_m"] = min(gaps)
    if victim:
        # time from the kill to the moment the machine's brake was commanded
        after = [r for r in rows if r["t"] >= t_killed - 0.05]
        braked = next((r["t"] for r in after if r["gw_brake"] or not r["brake_release"]), None)
        stopped = next((r["t"] for r in after if abs(r["v"]) < 0.02), None)
        res.update(t_kill=round(t_killed, 3), brake_after_s=None if braked is None else round(braked - t_killed, 3),
                   standstill_after_s=None if stopped is None else round(stopped - t_killed, 3))
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="nominal")
    args = ap.parse_args()
    print(json.dumps(run(args.scenario), indent=1))


if __name__ == "__main__":
    main()
