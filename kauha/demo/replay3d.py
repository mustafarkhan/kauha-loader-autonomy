"""Build the 3D shift replay page (three.js) from a shift run.

    python -m kauha.demo.replay3d [--run results/runs/learned_s1]

Writes docs/results/kauha_3d.html: one self-contained page with the shift data
embedded. three.js loads from jsDelivr (pinned).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
TEMPLATE = Path(__file__).with_name("replay3d_template.html")
OUT = REPO_ROOT / "docs" / "results" / "kauha_3d.html"
KEEP = ("t", "x", "y", "th", "g", "v", "boom", "bkt", "load", "st", "fs", "ph", "mode", "ppl", "tr", "obs",
        "dust", "gnss", "link", "lim")


def build(run_dir: Path, every: int = 2) -> dict:
    rep = json.loads((run_dir / "replay.json").read_text())
    kpis = json.loads((run_dir / "kpis.json").read_text())
    site_cfg = yaml.safe_load((REPO_ROOT / "hardware" / "config" / "site_demo.yaml").read_text())
    mach = yaml.safe_load((REPO_ROOT / "hardware" / "config" / "loader_medium.yaml").read_text())
    frames = []
    for i, f in enumerate(rep["frames"]):
        if i % every == 0 or "toes" in f:
            g = {k: f[k] for k in KEEP if k in f}
            if "toes" in f:
                g["toes"] = f["toes"]
            frames.append(g)
    used = {f["fs"] for f in frames}
    fields = {str(k): v for k, v in rep["fields"].items() if int(k) in used}
    heights = {p["name"]: p["height"] for p in site_cfg["piles"]}
    b = mach["bucket"]
    return {
        "frames": frames, "fields": fields, "stops": rep["stops"], "site": rep["site"], "pile_heights": heights,
        "machine": {"lf": rep["machine"]["lf"], "lr": rep["machine"]["lr"], "width": rep["machine"]["width"],
                    "pivot_fwd": b["boom_pivot_forward_m"], "pivot_h": b["boom_pivot_height_m"],
                    "boom_len": b["boom_length_m"], "bucket_len": b["bucket_length_m"], "bucket_w": b["width_m"],
                    "track": mach["machine"]["track_width_m"]},
        "kpis": {k: kpis[k] for k in ("tonnes", "trucks", "cycles", "protective_stops", "interventions",
                                      "min_person_distance_moving_m")},
        "run": run_dir.name,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, default=REPO_ROOT / "results" / "runs" / "learned_s1")
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    data = build(a.run)
    page = TEMPLATE.read_text().replace("/*__DATA__*/null", json.dumps(data, separators=(",", ":")))
    if data.get("site", {}).get("target_kind") == "hopper":
        page = page.replace("<title>KAUHA 3D Shift Replay</title>", "<title>KAUHA 3D Load-and-Carry Replay</title>", 1)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(page)
    print(f"wrote {a.out} ({a.out.stat().st_size / 1e6:.2f} MB, {len(data['frames'])} frames)")


if __name__ == "__main__":
    main()
