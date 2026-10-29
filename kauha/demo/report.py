"""Build the shift replay + results page from the simulation outputs.

    python -m kauha.demo.report --out docs/results/kauha_shift.html
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

from kauha.config import REPO_ROOT

TEMPLATE = Path(__file__).with_name("report_template.html")


def phase_breakdown(frames: list[dict]) -> dict:
    """Mean seconds per phase per completed cycle, from 5 Hz frames."""
    if len(frames) < 2:
        return {}
    dt = frames[1]["t"] - frames[0]["t"]
    c = Counter(f["ph"] for f in frames if f["st"] < 2 and f["mode"] == 1)
    return {k: v * dt for k, v in c.items()}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", type=Path, default=REPO_ROOT / "results")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "docs" / "results" / "kauha_shift.html")
    ap.add_argument("--every", type=int, default=2, help="keep every n-th 5 Hz frame")
    args = ap.parse_args()

    r = args.results
    replay = json.loads((r / "runs" / "learned_s1" / "replay.json").read_text())
    kpis_one = json.loads((r / "runs" / "learned_s1" / "kpis.json").read_text())
    campaign = json.loads((r / "campaign.json").read_text())
    for c in campaign.values():
        c.pop("runs", None)
    frames = replay["frames"]
    breakdown = phase_breakdown(frames)
    replay["frames"] = frames[:: args.every]
    bucket = json.loads((REPO_ROOT / "models" / "bucket_training.json").read_text())
    ppe_file = REPO_ROOT / "models" / "ppe_detector_metrics.json"
    ppe = json.loads(ppe_file.read_text()) if ppe_file.exists() else None
    data = {
        "replay": replay, "campaign": campaign, "one": kpis_one, "breakdown": breakdown,
        "bucket": {"test": bucket["test"], "selected_step": bucket["selected_step"],
                   "log": [{k: x[k] for k in ("step", "score_mean", "fill_factor_mean", "time_s_mean")}
                           for x in bucket["log"]]},
        "ppe": ppe,
    }
    html = TEMPLATE.read_text().replace("/*__DATA__*/null", json.dumps(data, separators=(",", ":")))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html)
    print(f"wrote {args.out} ({len(html) / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
