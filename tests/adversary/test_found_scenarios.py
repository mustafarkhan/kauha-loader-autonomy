"""Scenarios the adversarial search found, kept as regression tests (each ~10 s)."""
from __future__ import annotations

import pytest

from kauha.adversary.search import episode

# CMA-ES, rules-kept, v3 search before SR-16: a person walks in front of the loader as it
# reverses away from the truck with the bucket raised, and stops under it; the boom is lowered
# onto them. Clearance was -0.25 m with no protective stop.
UNDER_RAISED_BUCKET = [0.906, 0.4855, 0.5373, 0.5065, 0.7656, 0.1036, 0.4861, 0.4773, 0.9433, 0.784]


@pytest.mark.req("SR-16")
def test_person_under_the_raised_bucket_is_not_struck():
    r = episode((UNDER_RAISED_BUCKET, "rules-kept", "learned", 1))
    assert r["violation"] == 0
    assert r["min_clearance_moving_m"] > 0.0, r
