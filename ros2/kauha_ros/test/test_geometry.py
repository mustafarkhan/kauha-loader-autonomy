"""The URDF must put each safety scanner exactly where the twin's ray-caster puts it.

For random poses and articulation angles, the scanner origin and the direction of every
beam are computed twice: through the URDF's joint chain (as robot_state_publisher and TF
would) and through kauha.sim.sensors (as the twin ray-casts). They must agree, otherwise
LaserScan data would be drawn in the wrong place in RViz or Foxglove.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from kauha_ros.convert import SCANNER_LINKS

from kauha.config import load_machine
from kauha.sim.loader import Loader, LoaderState
from kauha.sim.sensors import FRONT_MOUNTS, REAR_MOUNTS

URDF = Path(__file__).resolve().parents[1] / "urdf" / "kauha_loader.urdf"


def _joints():
    root = ET.parse(URDF).getroot()
    out = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1]
        out[j.find("child").get("link")] = (j.get("name"), j.find("parent").get("link"), j.get("type"), xyz, rpy, axis)
    return out


def _planar_pose(link: str, joints: dict, q: dict) -> tuple[float, float, float]:
    """(x, y, yaw) of a link in base_link, for the planar part of the chain."""
    chain = []
    while link != "base_link":
        chain.append(joints[link])
        link = joints[link][1]
    x = y = yaw = 0.0
    for name, _, typ, xyz, rpy, axis in reversed(chain):
        x, y = x + math.cos(yaw) * xyz[0] - math.sin(yaw) * xyz[1], y + math.sin(yaw) * xyz[0] + math.cos(yaw) * xyz[1]
        yaw += rpy[2]
        if typ == "revolute" and axis[2] != 0:
            yaw += axis[2] * q.get(name, 0.0)
    return x, y, yaw


@pytest.mark.parametrize("seed", range(5))
def test_scanner_frames_match_the_twin(seed):
    rng = np.random.default_rng(seed)
    cfg = load_machine()
    joints = _joints()
    for _ in range(20):
        st = LoaderState(x=rng.uniform(0, 60), y=rng.uniform(-20, 20), theta=rng.uniform(-math.pi, math.pi),
                         gamma=rng.uniform(-0.69, 0.69))
        loader = Loader(cfg, st, dt=0.025)
        rx, ry, rth = loader.rear_axle()
        bodies = {"front": (st.x, st.y, st.theta), "rear": (rx, ry, rth + math.pi)}
        for m in FRONT_MOUNTS + REAR_MOUNTS:
            # twin: origin and boresight of the scanner in the world
            bx, by, bth = bodies[m.body]
            ox = bx + math.cos(bth) * m.x - math.sin(bth) * m.y
            oy = by + math.sin(bth) * m.x + math.cos(bth) * m.y
            heading = bth + m.heading
            # URDF: base_link at the front axle with yaw theta
            lx, ly, lyaw = _planar_pose(SCANNER_LINKS[m.name], joints, {"articulation": st.gamma})
            ux = st.x + math.cos(st.theta) * lx - math.sin(st.theta) * ly
            uy = st.y + math.sin(st.theta) * lx + math.cos(st.theta) * ly
            uyaw = st.theta + lyaw
            assert math.hypot(ux - ox, uy - oy) < 1e-6, m.name
            d = (uyaw - heading + math.pi) % (2 * math.pi) - math.pi
            assert abs(d) < 1e-6, m.name


def test_joint_names_match_the_bridge():
    names = {v[0] for v in _joints().values()}
    assert {"articulation", "boom", "bucket"} <= names
