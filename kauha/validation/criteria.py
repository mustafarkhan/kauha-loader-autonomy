"""Validation criteria: external references KAUHA is checked against.

Every criterion names its source, the metric and the target. This file is
committed before the checks that evaluate it (kauha/validation/checks.py), so a
target cannot be adjusted after a result is seen; changing one later shows in
the history. `python -m kauha.validation.report` evaluates them and writes
results/validation.json and docs/results/validation.md.

Kinds:
  standard   a fixed requirement from a standard
  published  a measured result from a paper or an open recording
  dataset    recorded data replayed through KAUHA (needs the data on disk)
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Criterion:
    id: str
    part: str            # the KAUHA part under test
    kind: str
    source: str
    url: str
    metric: str
    target: str
    data_env: str = ""   # environment variable naming the dataset directory, when data is needed


CRITERIA = [
    # ---------------------------------------------------------------- standards
    Criterion("V-STD-01", "service brake", "standard", "ISO 3450, clause 7.5 (holding)",
              "https://cdn.standards.iteh.ai/samples/20731/1f8d9fdd767b4ec6afcd5e753ced32ba/ISO-3450-1996.pdf",
              "deceleration the assumed service brake can hold against gravity",
              "holds on a 25 % slope: brake deceleration >= g * sin(atan 0.25) = 2.38 m/s2"),
    Criterion("V-STD-02", "service brake", "standard", "ISO 3450, clause 7.6 (stopping distance)",
              "https://cdn.standards.iteh.ai/samples/20731/1f8d9fdd767b4ec6afcd5e753ced32ba/ISO-3450-1996.pdf",
              "stopping distance from the standard's test speed (top speed when below 32 km/h)",
              "below the table limit; the limit formula must be read from the purchased standard"),
    Criterion("V-STD-03", "walker model and adversarial search", "standard",
              "ISO 13855 (approach speed of a walking person)", "https://www.iso.org/standard/42845.html",
              "highest walking speed the adversarial search and the shifts use",
              ">= 1.6 m/s in the adversarial search"),
    Criterion("V-STD-04", "safety scanner fields (twin)", "standard",
              "ISO 3691-4 (personnel detection with test pieces)",
              "https://webstore.ansi.org/preview-pages/ISO/preview_ISO+3691-4-2023.pdf",
              "a standing test piece (200 mm diameter) just inside the far edge of each protective field",
              "detected in every field set, on the axis and at both far corners"),
    Criterion("V-STD-05", "safety scanner mounting", "standard", "ISO 3691-4 (lying test piece, 70 mm diameter)",
              "https://webstore.ansi.org/preview-pages/ISO/preview_ISO+3691-4-2023.pdf",
              "scan plane height against the lying test piece",
              "scan plane at or below 70 mm, or another means of detecting a person lying down"),
    Criterion("V-STD-06", "1:8 testbed stability", "standard", "ISO 14397-1 (rated operating load)",
              "https://www.iso.org/standard/39806.html",
              "rated operating load against the full-turn static tipping load and a bucket of sand",
              "rated load = 50 % of the tipping load and >= the struck bucket of sand"),
    Criterion("V-STD-07", "stop chain", "standard", "ISO 13849-1 (performance level)",
              "https://www.iso.org/standard/69883.html", "PL of the stop function from component data",
              "PL d calculated from the selected components' MTTFd, DC and CCF"),
    # ---------------------------------------------------------------- published measurements
    Criterion("V-PUB-01", "learned bucket filling", "published",
              "Dadhich et al. 2019, Volvo L180H field test (expert fills in 6-8 s; automatic 26 % longer)",
              "https://www.sciencedirect.com/science/article/pii/S0926580518305119",
              "mean fill time and fill factor of the learned policy on the held-out test episodes",
              "fill time <= 1.26 x 7 s = 8.8 s and fill factor >= 0.90"),
    Criterion("V-PUB-02", "bucket-pile model uncertainty", "published",
              "Aoshima & Servin 2024, WA320-7: simulation differs from the machine by about 11 % in force",
              "https://arxiv.org/pdf/2310.05765",
              "learned policy on gravel with digging resistance 11 % lower and 11 % higher",
              "fill factor >= 0.85 and every episode completes in both cases"),
    Criterion("V-PUB-03", "cycle time baseline", "published",
              "Frank et al. 2012, 73 operators on a Volvo loader, short loading cycle",
              "https://www2.iea.lth.se/publications/Papers/Frank_2012.pdf",
              "KAUHA cycle time against the measured novice-to-professional spread",
              "reported against the spread; the paper gives it only in plots"),
    Criterion("V-PUB-04", "loader motion model", "published",
              "Tampere University, 6 t wheel loader short Y-cycle recording (CC BY 4.0)",
              "https://zenodo.org/records/10639199",
              "twin driven with the recorded operator inputs: cylinder trajectories and phase durations",
              "within 15 % of the recording", data_env="KAUHA_DATA_TUNI_LOADER"),
    # ---------------------------------------------------------------- recorded data
    Criterion("V-DAT-01", "machine bus (E2E, gateway, PLC)", "dataset",
              "attack classes of the ROAD CAN dataset: fabrication, fuzzing, replay, masquerade, suspension",
              "https://ar5iv.labs.arxiv.org/html/2012.14600",
              "attack frames accepted by the gateway or the PLC, and unsafe commands that follow",
              "0 forged frames accepted; replay and suspension end in a stop within the frame timeout"),
    Criterion("V-DAT-02", "machine bus (E2E, gateway)", "dataset", "ROAD CAN dataset, recorded logs",
              "https://ar5iv.labs.arxiv.org/html/2012.14600",
              "real vehicle traffic replayed onto the KAUHA bus, as recorded and remapped onto KAUHA identifiers",
              "0 frames accepted", data_env="KAUHA_DATA_ROAD"),
    Criterion("V-DAT-03", "person detector", "dataset", "SODA construction-site images (CC BY 4.0)",
              "https://arxiv.org/pdf/2202.09554", "person recall and precision at confidence 0.25",
              "recall >= 0.80, precision >= 0.75 (the values measured on Construction-PPE)",
              data_env="KAUHA_DATA_SODA"),
    Criterion("V-DAT-04", "field evaluation in dust", "dataset", "LiDARDustX (research use)",
              "https://github.com/vincentweikey/LiDARDustX",
              "frames with a protective field occupied although no labelled object is inside it",
              "false-occupied rate reported per sensor; target below 1 % of frames after the dust filter",
              data_env="KAUHA_DATA_LIDARDUSTX"),
    Criterion("V-DAT-05", "perception in snow", "dataset", "Boreas multi-season dataset",
              "https://arxiv.org/pdf/2203.10168", "false-occupied rate and missed people in snow against dry runs",
              "not built yet", data_env="KAUHA_DATA_BOREAS"),
    Criterion("V-DAT-06", "radar in dust and snow", "dataset", "RADIATE; 4D radar in an open-pit mine",
              "https://arxiv.org/pdf/2010.09076", "radar detections of people when lidar is degraded",
              "not built yet", data_env="KAUHA_DATA_RADIATE"),
    Criterion("V-DAT-07", "localisation", "dataset", "ISPRS RTK-SLAM benchmark",
              "https://www.isprs.org/resources/datasets/benchmarks/RTK-SLAM/default.aspx",
              "position error and time to declare the position invalid", "not built yet",
              data_env="KAUHA_DATA_RTKSLAM"),
]

BY_ID = {c.id: c for c in CRITERIA}
