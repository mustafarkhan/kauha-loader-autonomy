"""Grid-search the rule-based bucket-filling baseline on the training objective.

    python -m kauha.train.tune_baseline --out models/baseline_params.json
"""
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

from kauha.train.bucket_env import ScriptedOperator, evaluate

GRID = {
    "throttle": (0.7, 1.0),
    "pen_lift": (0.05, 0.15, 0.25, 0.4),
    "boom1": (0.4, 0.7, 1.0),
    "f_thr": (0.4, 0.6),
    "load_end": (0.9, 1.0, 1.1),
    "t_end": (0.4, 0.5, 0.6),
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=24)
    ap.add_argument("--out", type=Path, default=Path("models/baseline_params.json"))
    args = ap.parse_args()
    best = None
    for combo in itertools.product(*GRID.values()):
        r = evaluate(ScriptedOperator(combo), n=args.episodes, seed=500)
        key = r["score_mean"]
        if best is None or key > best[0]:
            best = (key, combo, r)
    score, combo, r = best
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"params": dict(zip(GRID, combo)), "tuning_eval": r}, indent=1))
    print(json.dumps({"params": dict(zip(GRID, combo)), "score": score}))


if __name__ == "__main__":
    main()
