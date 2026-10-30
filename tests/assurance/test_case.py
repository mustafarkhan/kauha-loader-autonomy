"""The generated safety case: structure, and that failing evidence propagates to the top claim."""
from __future__ import annotations

import yaml

from kauha.assurance.case import REPO_ROOT, Case, _req_texts, build


def test_every_hazard_requirement_exists_and_every_safety_requirement_is_argued():
    hz = yaml.safe_load((REPO_ROOT / "docs" / "safety" / "hazards.yaml").read_text())
    reqs = _req_texts()
    used = {r for h in hz["hazards"] for r in h["requirements"]}
    assert used <= set(reqs), f"hazards.yaml names unknown requirements: {used - set(reqs)}"
    safety = {r for r in reqs if r.startswith("SR-")}
    assert safety <= used, f"safety requirements not allocated to any hazard: {safety - used}"
    assumptions = {a["id"] for a in hz["assumptions"]}
    assert {a for h in hz["hazards"] for a in h.get("assumptions", [])} <= assumptions


def test_case_builds_with_a_node_per_allocated_requirement():
    case = build()
    nodes = case["nodes"]
    hz = yaml.safe_load((REPO_ROOT / "docs" / "safety" / "hazards.yaml").read_text())
    for h in hz["hazards"]:
        for r in h["requirements"]:
            assert f"{h['id']}.S.{r}" in nodes
    assert case["summary"]["open"] == [u["id"] for u in hz["open"]]


def test_failing_evidence_propagates_to_the_top_claim():
    c = Case()
    c.add("G0", "goal", "top", None)
    c.add("S0", "strategy", "s", "G0")
    c.add("G1", "goal", "a", "S0")
    c.add("E1", "solution", "ok", "G1", "passed")
    c.add("G2", "goal", "b", "S0")
    c.add("E2", "solution", "bad", "G2", "failed")
    assert c.resolve("G0") == "failed"
    c.nodes["E2"]["status"] = "missing"
    assert c.resolve("G0") == "missing"
    c.nodes["E2"]["status"] = "passed"
    c.add("U1", "undeveloped", "later", "S0", "open")
    assert c.resolve("G0") == "passed-open"
