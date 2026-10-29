"""Collect requirement IDs from @pytest.mark.req and write a traceability matrix.

    KAUHA_TRACE=results/traceability.json pytest
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import pytest


def pytest_configure(config):
    config.addinivalue_line("markers", "req(id): requirement this test verifies")


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    rep = outcome.get_result()
    if rep.when == "call" or (rep.when == "setup" and rep.outcome != "passed"):
        item._kauha_outcome = rep.outcome


def pytest_sessionfinish(session, exitstatus):
    out = os.environ.get("KAUHA_TRACE")
    if not out:
        return
    matrix: dict[str, dict] = {}
    for item in session.items:
        for m in item.iter_markers("req"):
            r = m.args[0]
            res = getattr(item, "_kauha_outcome", "not run")
            entry = matrix.setdefault(r, {"tests": [], "all_passed": True})
            entry["tests"].append({"test": item.nodeid, "outcome": res})
            entry["all_passed"] &= res == "passed"
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text(json.dumps(dict(sorted(matrix.items())), indent=1))
