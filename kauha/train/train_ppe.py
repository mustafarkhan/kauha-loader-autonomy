"""Fine-tune a person/PPE detector for the loader's cameras.

Dataset: Ultralytics Construction-PPE (1,132 train / 143 val / 141 test
images of construction sites; classes include Person, helmet, vest and their
absence). Model: YOLO11n, pretrained on COCO. Both are AGPL-3.0; a product
would either comply with AGPL or license commercially / retrain on another
architecture. See docs/results/README.md.

The camera detector is a *non-safety* function: it lets the autonomy slow
down for people beyond the safety scanners' 4 m protective range. Its
measured person recall and false-positive rate feed the digital twin's
camera model, so the shift simulation uses the detector's real test-set
performance rather than an assumed one.

    python -m kauha.train.train_ppe --data-root ../kauha_data/ppe --epochs 15
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

import numpy as np

from kauha.config import REPO_ROOT

PERSON = "Person"


def iou(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """IoU between boxes a (N,4) and b (M,4) in xyxy."""
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    aa = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    bb = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / (aa[:, None] + bb[None, :] - inter + 1e-9)


def person_rates(model, img_dir: Path, lbl_dir: Path, cls_id: int, imgsz: int, conf: float) -> dict:
    """Recall and false positives per image for the Person class at an operating threshold."""
    tp = fn = fp = 0
    times = []
    images = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    for img in images:
        t0 = time.perf_counter()
        r = model.predict(str(img), imgsz=imgsz, conf=conf, device="cpu", verbose=False)[0]
        times.append((time.perf_counter() - t0) * 1000)
        h, w = r.orig_shape
        pred = r.boxes.xyxy.cpu().numpy()[r.boxes.cls.cpu().numpy() == cls_id]
        gt = []
        lf = lbl_dir / (img.stem + ".txt")
        if lf.exists():
            for line in lf.read_text().split("\n"):
                parts = line.split()
                if len(parts) == 5 and int(parts[0]) == cls_id:
                    cx, cy, bw, bh = map(float, parts[1:])
                    gt.append([(cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h])
        gt = np.array(gt).reshape(-1, 4)
        matched = set()
        if len(pred) and len(gt):
            m = iou(pred, gt)
            for i in np.argsort(-m.max(axis=1)):
                j = int(np.argmax(m[i]))
                if m[i, j] >= 0.5 and j not in matched:
                    matched.add(j)
                    tp += 1
                else:
                    fp += 1
        else:
            fp += len(pred)
        fn += len(gt) - len(matched)
    return {"person_recall": tp / max(1, tp + fn), "person_precision": tp / max(1, tp + fp),
            "person_fp_per_image": fp / max(1, len(images)), "person_instances": tp + fn,
            "test_images": len(images), "cpu_ms_mean": float(np.mean(times[3:])),
            "cpu_ms_p95": float(np.percentile(times[3:], 95)), "conf_threshold": conf}


def main() -> None:
    from ultralytics import YOLO

    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", type=Path, default=REPO_ROOT.parent / "kauha_data" / "ppe")
    ap.add_argument("--weights", type=Path, default=REPO_ROOT.parent / "kauha_data" / "weights" / "yolo11n.pt")
    ap.add_argument("--epochs", type=int, default=15)
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--runs", type=Path, default=REPO_ROOT / "runs")
    args = ap.parse_args()

    root = args.data_root.resolve()
    src = (root / "data.yaml").read_text()
    names_block = src[src.index("names:"):src.index("# Download")].rstrip()
    yaml_path = args.runs / "construction_ppe.yaml"
    args.runs.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(f"path: {root}\ntrain: images/train\nval: images/val\ntest: images/test\n{names_block}\n")

    model = YOLO(str(args.weights))
    model.train(data=str(yaml_path), epochs=args.epochs, imgsz=args.imgsz, batch=16, workers=2, device="cpu",
                project=str(args.runs), name="ppe", exist_ok=True, plots=False, seed=0, deterministic=True,
                verbose=False)
    best = args.runs / "ppe" / "weights" / "best.pt"
    model = YOLO(str(best))
    val = model.val(data=str(yaml_path), split="test", imgsz=args.imgsz, device="cpu", plots=False, verbose=False)
    names = model.names
    per_class = {}
    for i, c in enumerate(val.box.ap_class_index):
        per_class[names[int(c)]] = {"mAP50": float(val.box.ap50[i]), "mAP50_95": float(val.box.ap[i])}
    cls_id = next(k for k, v in names.items() if v == PERSON)
    rates = person_rates(model, root / "images" / "test", root / "labels" / "test", cls_id, args.imgsz, args.conf)
    metrics = {
        "model": "YOLO11n (COCO-pretrained) fine-tuned on Construction-PPE",
        "licence": "AGPL-3.0 (dataset and model)",
        "epochs": args.epochs, "imgsz": args.imgsz,
        "test_mAP50_all": float(val.box.map50), "test_mAP50_95_all": float(val.box.map),
        "per_class": per_class, **rates,
    }
    out = REPO_ROOT / "models"
    out.mkdir(exist_ok=True)
    shutil.copy(best, out / "ppe_yolo11n.pt")
    (out / "ppe_detector_metrics.json").write_text(json.dumps(metrics, indent=1))
    print(json.dumps(metrics, indent=1))


if __name__ == "__main__":
    main()
