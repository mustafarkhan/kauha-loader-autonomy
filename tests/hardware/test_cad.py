"""The concept CAD must agree with the twin and stay free of interference over the joint ranges."""
from __future__ import annotations

import pytest

cq = pytest.importorskip("cadquery")

from hardware.cad import kauha_cad as K  # noqa: E402


@pytest.fixture(scope="module")
def parts():
    return K.build_parts()


def test_kit_places_scanners_where_the_twin_ray_casts_and_has_no_interference(parts):
    assert K.check(parts) == []


@pytest.mark.parametrize("finding", K.LAYOUT_FINDINGS, ids=lambda f: f["part"])
def test_layout_findings_still_hold(parts, finding):
    """The documented radar position really clashes, so the move stays justified."""
    moved = [p for p in parts if p.name != finding["part"]]
    target = next(p for p in parts if p.name == finding["part"])
    x, y, z = finding["was"]
    bb = target.shape.val().BoundingBox()
    old = K.Part(target.name, target.group, target.channel, K.box(bb.xlen, bb.ylen, bb.zlen, x, y, z), kit=True)
    assert any(finding["part"] in pr for pr in K.check(moved + [old]))
