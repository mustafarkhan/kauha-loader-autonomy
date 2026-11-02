"""Start one SIL process from the command line (used by the ROS 2 launch files).

    python -m kauha.sil.proc machine --group 239.74.0.1 --port 31000 --epoch <monotonic s> --duration 20

All processes of one run get the same --epoch: a CLOCK_MONOTONIC time, shared by every
process on the machine, at which they start their loops together.
"""
from __future__ import annotations

import argparse
import json

from kauha.sil import nodes
from kauha.sil.common import Wiring


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("role", choices=("machine", "plc", "remote", "autonomy"))
    ap.add_argument("--group", required=True)
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--epoch", type=float, required=True)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--person", default="", help='JSON, e.g. {"t": 6, "ahead": 8}')
    ap.add_argument("--log", default="/tmp/kauha_sil")
    a = ap.parse_args()
    w = Wiring(a.group, a.port, a.port + 1)
    if a.role == "machine":
        nodes.machine(w, a.epoch, a.duration, a.log + "_machine.jsonl", json.loads(a.person) if a.person else None)
    elif a.role == "plc":
        nodes.plc(w, a.epoch, a.duration, a.log + "_plc.json")
    elif a.role == "remote":
        nodes.remote(w, a.epoch, a.duration)
    else:
        nodes.autonomy(w, a.epoch, a.duration, a.speed)


if __name__ == "__main__":
    main()
