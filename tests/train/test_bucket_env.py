from __future__ import annotations

import numpy as np
import pytest

from kauha.config import REPO_ROOT
from kauha.train.bucket_env import BucketFillEnv, cycle_score, evaluate, scripted_operator


def test_env_api_and_determinism():
    a, b = BucketFillEnv(), BucketFillEnv()
    oa, _ = a.reset(seed=3)
    ob, _ = b.reset(seed=3)
    assert np.allclose(oa, ob) and a.observation_space.contains(oa.clip(-10, 10))
    for _ in range(50):
        act = a.action_space.sample()
        ra, rb = a.step(act), b.step(act)
        assert np.allclose(ra[0], rb[0]) and ra[1] == pytest.approx(rb[1])


def test_baseline_completes_buckets():
    r = evaluate(scripted_operator, n=20, seed=100)
    assert r["success_rate"] == 1.0 and 0.8 < r["fill_factor_mean"] < 1.15


def test_cycle_score_prefers_full_bucket_over_quick_partial():
    assert cycle_score(1.0, 9.0) > cycle_score(0.75, 4.0)
    assert cycle_score(1.0, 9.0) > cycle_score(1.05, 14.0)


@pytest.mark.skipif(not (REPO_ROOT / "models" / "bucket_policy.npz").exists(), reason="no trained policy")
def test_learned_policy_is_no_worse_than_baseline():
    from kauha.autonomy.bucket_policy import BucketPolicy
    pol = BucketPolicy.load(REPO_ROOT / "models" / "bucket_policy.npz")
    rp = evaluate(pol.act, n=40, seed=7000)
    rb = evaluate(scripted_operator, n=40, seed=7000)
    assert rp["success_rate"] >= 0.95
    assert rp["score_mean"] >= rb["score_mean"] - 0.01
