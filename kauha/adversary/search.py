"""Adversarial scenario search: try to make the machine hit a person.

    python -m kauha.adversary.search --mode rules-kept --budget 160
    python -m kauha.adversary.search --mode rules-broken --budget 160

An adversarial pedestrian is described by 10 numbers: when they appear, where,
two waypoints, a pause, a walking speed, and the level and timing of dust. CMA-ES
(Hansen's covariance matrix adaptation) searches those numbers to minimise the
clearance between the person and the *moving* machine in a 150 s episode of the
full stack (autonomy, CAN, safety PLC, ray-cast scanners). A random search with
the same budget runs alongside it as the baseline.

The adversary never walks into the machine body itself (it stops 0.3 m short),
but otherwise ignores the machine: it walks into its path, stands in blind spots
and pauses anywhere. Contact therefore means the machine moved into a person.

Modes:
  rules-kept    the person stays out of the access-controlled dig and dock zones
                (the site rule the safety concept relies on). A contact here is a
                failure of the technical safety measures.
  rules-broken  the person may enter the zones. This measures how much the zone
                access control carries; contacts here are the residual risk the
                safety concept states, not a surprise.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from kauha.sim.world import Event, Person

REPO_ROOT = Path(__file__).resolve().parents[2]
EPISODE_S = 150.0
DIM = 10
AREA = (-4.0, 69.0, -31.0, 29.0)                     # x0, x1, y0, y1 inside the walls
DIG_ZONES = [(40, 75, -9.5, 0.0), (40, 75, 0.0, 10.5)]
DOCK_ZONE = (33, 47, -16.6, -8.0)
TRUCK = (35.5, 44.5, -19.25, -16.75)
# per task: (dump target footprint, its dock zone); the carry site has a hopper instead of the truck bay
TARGETS = {"truck": (TRUCK, DOCK_ZONE), "carry": ((5.0, 11.0, -19.5, -16.5), (1.0, 15.0, -16.6, -8.0))}
PILES = [(50, 69, -7.5, -1.5), (50, 69, 1.5, 7.5)]   # standing on a stockpile is not walking


@dataclass
class Scenario:
    t_appear: float
    start: tuple[float, float]
    wp1: tuple[float, float]
    pause_s: float
    wp2: tuple[float, float]
    speed: float
    dust: float
    dust_t0: float

    def person(self) -> Person:
        pts = [self.start, self.wp1, self.wp2]
        route, t = [(self.t_appear, *self.start)], self.t_appear
        t += math.dist(pts[0], pts[1]) / self.speed
        route.append((t, *pts[1]))
        t += self.pause_s
        route.append((t, *pts[1]))
        t += math.dist(pts[1], pts[2]) / self.speed
        route.append((t, *pts[2]))
        return Person("adversary", route)

    def events(self) -> list[Event]:
        return [Event("dust", self.dust_t0, self.dust_t0 + 40.0, self.dust)] if self.dust > 0.002 else []


def decode(x: np.ndarray) -> Scenario:
    x = np.clip(x, 0.0, 1.0)
    x0, x1, y0, y1 = AREA

    def pt(a, b):
        return (round(x0 + a * (x1 - x0), 2), round(y0 + b * (y1 - y0), 2))
    return Scenario(t_appear=round(5 + x[0] * 110, 2), start=pt(x[1], x[2]), wp1=pt(x[3], x[4]),
                    pause_s=round(x[5] * 30, 2), wp2=pt(x[6], x[7]), speed=round(0.5 + x[8] * 1.5, 2),
                    dust=round(max(0.0, x[9] - 0.5) * 0.04, 4), dust_t0=round(5 + x[0] * 60, 1))


def _inside(p, box):
    return box[0] <= p[0] <= box[1] and box[2] <= p[1] <= box[3]


def route_violation(sc: Scenario, mode: str, task: str = "truck") -> float:
    """How far the route goes where this mode does not allow people (0 = allowed)."""
    pts = [sc.start, sc.wp1, sc.wp2]
    bad = 0.0
    target, dock = TARGETS[task]
    forbidden = [target] + PILES + (DIG_ZONES + [dock] if mode == "rules-kept" else [])
    for a, b in zip(pts, pts[1:]):
        for u in np.linspace(0, 1, 30):
            p = (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]))
            for box in forbidden:
                if _inside(p, box):
                    cx, cy = (box[0] + box[1]) / 2, (box[2] + box[3]) / 2
                    hx, hy = (box[1] - box[0]) / 2, (box[3] - box[2]) / 2
                    bad += min(hx - abs(p[0] - cx), hy - abs(p[1] - cy)) + 0.1
    return bad


def episode(args: tuple) -> dict:
    """Run one episode; returns the clearance metrics for the adversary."""
    x, mode, controller, seed, *rest = args
    task = rest[0] if rest else "truck"
    from kauha.demo.shift import ShiftSim
    sc = decode(np.asarray(x))
    viol = route_violation(sc, mode, task)
    res = {"scenario": asdict(sc), "violation": round(viol, 3)}
    if viol > 0:
        res.update(objective=5.0 + viol, min_clearance_moving_m=None)
        return res
    sim = ShiftSim(controller, EPISODE_S / 60.0, seed=seed, task=task)
    sim.site.people = [sc.person()]
    sim.site.events = sc.events()
    sim.obstacles = []
    sim.yield_m = (0.3, 0.3)          # never steps into the body, otherwise ignores the machine
    k = sim.run()
    clr = k.min_person_distance_moving_m
    under = k.person_under_raised_bucket_moving_s
    res.update(min_clearance_moving_m=round(clr, 3), min_clearance_m=round(k.min_person_distance_m, 3),
               field_entries_moving=k.person_field_entries_while_moving, under_raised_bucket_s=under,
               stops=k.protective_stops, stops_by_cause=k.stops_by_cause, tonnes=k.tonnes,
               objective=round(min(clr, 3.0) - 2.0 * under, 3))
    return res


def search(mode: str, budget: int, workers: int, controller: str, seed: int, popsize: int = 8,
           task: str = "truck") -> dict:
    import cma
    rng = np.random.default_rng(seed)
    log: list[dict] = []
    pool = ProcessPoolExecutor(workers)

    def evaluate(xs, method):
        out = list(pool.map(episode, [(list(map(float, x)), mode, controller, seed, task) for x in xs]))
        for x, r in zip(xs, out):
            r.update(method=method, x=[round(float(v), 4) for v in x], n=len(log))
            log.append(r)
        return [r["objective"] for r in out]

    t0 = time.time()
    half = budget // 2
    # Random search baseline, same budget.
    evaluate(rng.random((half, DIM)), "random")
    # CMA-ES from the centre of the box.
    es = cma.CMAEvolutionStrategy(DIM * [0.5], 0.3, {"bounds": [0, 1], "popsize": popsize, "seed": seed,
                                                    "verbose": -9})
    while sum(r["method"] == "cma" for r in log) < half:
        xs = es.ask()
        es.tell(xs, evaluate(xs, "cma"))
    pool.shutdown()

    def best(method):
        rs = [r for r in log if r["method"] == method and r.get("min_clearance_moving_m") is not None]
        return min(rs, key=lambda r: r["objective"]) if rs else None
    valid = [r for r in log if r.get("min_clearance_moving_m") is not None]
    return {"mode": mode, "task": task, "controller": controller, "budget": budget, "episode_s": EPISODE_S,
            "wall_s": round(time.time() - t0, 1), "valid_episodes": len(valid),
            "contacts": sum(r["min_clearance_moving_m"] < 0.0 for r in valid),
            "field_entries_while_moving": sum(r["field_entries_moving"] > 0 for r in valid),
            "best_random": best("random"), "best_cma": best("cma"), "log": log}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("rules-kept", "rules-broken"), default="rules-kept")
    ap.add_argument("--budget", type=int, default=160)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--controller", default="learned")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--task", choices=("truck", "carry"), default="truck")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    rep = search(a.mode, a.budget, a.workers, a.controller, a.seed, task=a.task)
    out = a.out or REPO_ROOT / "results" / f"adversary_{a.mode}{'' if a.task == 'truck' else '_' + a.task}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(rep, indent=1))
    for k in ("best_random", "best_cma"):
        b = rep[k]
        if b:
            print(k, b["min_clearance_moving_m"], b["under_raised_bucket_s"], b["stops_by_cause"], b["scenario"])
    print({k: rep[k] for k in ("mode", "valid_episodes", "contacts", "field_entries_while_moving", "wall_s")})


if __name__ == "__main__":
    main()
