"""Generate the safety case from the evidence, and check it.

    python -m kauha.assurance.case            # write results/safety_case.json and docs/results/safety_case.html
    python -m kauha.assurance.case --check    # exit 1 if any claim lacks passing evidence

The argument is in Goal Structuring Notation (GSN):

  G0  the system is acceptably safe for people on foot, in scope, under A1-A3
   S0  argued over the hazards in docs/safety/hazards.yaml
    H*  each hazard is controlled
     S   argued over the safety requirements allocated to it, then validated
      SR-*  each requirement is correctly implemented
       Sn   formal proofs (results/verification.json)
       Sn   requirement-traced tests (results/traceability.json)
      V*  integrated validation: 15 simulated shifts, adversarial search, SIL
  U*  undeveloped goals: work that needs hardware or certification

Nothing in the case is written by hand except the hazard list: every status
comes from an evidence file produced by CI or by the campaign scripts.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS = REPO_ROOT / "results"


def _load(path: Path):
    return json.loads(path.read_text()) if path.exists() else None


def _req_texts() -> dict[str, str]:
    out = {}
    for line in (REPO_ROOT / "docs" / "safety" / "requirements.md").read_text().splitlines():
        m = re.match(r"\| (SR-\d\d|AR-\d\d) \| (.+?) \|", line)
        if m:
            out[m.group(1)] = m.group(2)
    return out


class Case:
    def __init__(self) -> None:
        self.nodes: dict[str, dict] = {}

    def add(self, nid: str, kind: str, text: str, parent: str | None, status: str | None = None,
            evidence: list | None = None, **extra) -> str:
        self.nodes[nid] = {"id": nid, "kind": kind, "text": text, "parent": parent, "status": status,
                           "evidence": evidence or [], "children": [], **extra}
        if parent:
            self.nodes[parent]["children"].append(nid)
        return nid

    def resolve(self, nid: str) -> str:
        """Goal status from its children: failed > missing > passed. Undeveloped stays open."""
        n = self.nodes[nid]
        if n["kind"] in ("solution", "undeveloped"):
            return n["status"]
        sts = [self.resolve(c) for c in n["children"] if self.nodes[c]["kind"] not in ("context", "assumption")]
        if not sts:
            n["status"] = "missing"
        elif "failed" in sts:
            n["status"] = "failed"
        elif "missing" in sts:
            n["status"] = "missing"
        elif "open" in sts:
            n["status"] = "passed-open"
        else:
            n["status"] = "passed"
        if n["status"] == "passed" and any(self.nodes[c].get("status") == "passed-open" for c in n["children"]):
            n["status"] = "passed-open"
        return n["status"]


def build() -> dict:
    hz = yaml.safe_load((REPO_ROOT / "docs" / "safety" / "hazards.yaml").read_text())
    reqs = _req_texts()
    ver = _load(RESULTS / "verification.json")
    trace = _load(RESULTS / "traceability.json") or {}
    camp = _load(RESULTS / "campaign.json")
    adv_k = _load(RESULTS / "adversary_rules-kept.json")
    adv_b = _load(RESULTS / "adversary_rules-broken.json")
    adv_c = _load(RESULTS / "adversary_rules-kept_carry.json")

    c = Case()
    c.add("G0", "goal", "People on foot are not harmed by the KAUHA-equipped loader operating autonomously "
          "within its scope, given assumptions A1–A3.", None)
    c.add("C0", "context", hz["scope"].strip(), "G0")
    c.add("C1", "context", "Safety requirements SR-01…SR-16 (docs/safety/requirements.md).", "G0")
    for a in hz["assumptions"]:
        c.add(a["id"], "assumption", a["text"].strip(), "G0")
    c.add("S0", "strategy", "Argue over each identified hazard (docs/safety/hazards.yaml), then over the work "
          "that only hardware and certification can close.", "G0")

    # ---------------------------------------------------- requirement goals
    def proof_evidence(r: str) -> list:
        if not ver:
            return []
        props = {}
        for p in ver["properties"]:
            if p["req"] == r:
                props.setdefault(p["id"], {"id": p["id"], "text": p["text"], "proven": 0, "configs": 0})
                props[p["id"]]["configs"] += 1
                props[p["id"]]["proven"] += p["status"] == "proven"
        return list(props.values())

    def test_evidence(r: str) -> list:
        ent = trace.get(r, {"tests": []})
        return [t for t in ent["tests"] if not t["test"].startswith("tests/verify/")]

    req_goal: dict[str, str] = {}

    def req_node(r: str, parent: str) -> None:
        nid = f"{parent}.{r}"
        req_goal[nid] = r
        c.add(nid, "goal", f"{r} is correctly implemented: {reqs.get(r, '?')}", parent, req=r)
        pe = proof_evidence(r)
        if pe:
            ok = all(p["proven"] == p["configs"] for p in pe)
            n = sum(p["configs"] for p in pe)
            c.add(f"{nid}.proof", "solution", f"Formal proof: {len(pe)} propert{'y' if len(pe) == 1 else 'ies'} "
                  f"× 4 configurations, {sum(p['proven'] for p in pe)}/{n} proven", nid,
                  "passed" if ok else "failed", pe, source="results/verification.json")
        te = test_evidence(r)
        if te:
            npass = sum(t["outcome"] == "passed" for t in te)
            c.add(f"{nid}.tests", "solution", f"Requirement-traced tests: {npass}/{len(te)} passed", nid,
                  "passed" if npass == len(te) else "failed", te, source="results/traceability.json")
        if not pe and not te:
            c.add(f"{nid}.none", "solution", "No evidence found", nid, "missing")

    # ---------------------------------------------------- validation evidence
    def shift_evidence(parent: str) -> None:
        if not camp:
            c.add(f"{parent}.shift", "solution", "Simulated shifts: results/campaign.json missing", parent, "missing")
            return
        runs = [(cfg, k) for cfg, v in camp.items() for k in v["runs"]]
        contacts = sum(k["min_person_distance_moving_m"] < 0 for _, k in runs)
        entries = sum(k["person_field_entries_while_moving"] for _, k in runs)
        under = sum(k["person_under_raised_bucket_moving_s"] for _, k in runs)
        worst = min(k["min_person_distance_moving_m"] for _, k in runs)
        ok = contacts == 0 and under == 0
        c.add(f"{parent}.shift", "solution",
              f"{len(runs)} simulated 60-min shifts ({len(camp)} configurations): {contacts} contacts with the moving "
              f"machine, {entries} protective-field entries while moving, {under:.1f} s under a raised bucket; "
              f"closest approach {worst:.2f} m", parent, "passed" if ok else "failed",
              [{"config": cfg, "seed": i % 3 + 1, "closest_m": k["min_person_distance_moving_m"],
                "field_entries": k["person_field_entries_while_moving"],
                "under_bucket_s": k["person_under_raised_bucket_moving_s"]} for i, (cfg, k) in enumerate(runs)],
              source="results/campaign.json")

    def adversary_evidence(parent: str, rep: dict | None, residual: bool) -> None:
        tag = "adv-zones" if residual else ("adv-carry" if rep and rep.get("task") == "carry" else "adv")
        if not rep:
            c.add(f"{parent}.{tag}", "solution", "Adversarial search: results missing", parent, "missing")
            return
        bc, br = rep["best_cma"], rep["best_random"]
        where = "load-and-carry site, " if rep.get("task") == "carry" else ""
        text = (f"Adversarial search ({where}{rep['mode']}): {rep['valid_episodes']} episodes, CMA-ES vs random; "
                f"{rep['contacts']} contacts; worst clearance found {bc['min_clearance_moving_m']:.2f} m "
                f"(random search: {br['min_clearance_moving_m']:.2f} m)")
        if residual:
            status = "info"
            text += ". Measures what zone access control (A1) carries; not a pass/fail criterion."
        else:
            status = "passed" if rep["contacts"] == 0 else "failed"
        c.add(f"{parent}.{tag}", "solution", text, parent, status,
              [{"method": r["method"], "clearance_m": r["min_clearance_moving_m"], "scenario": r["scenario"]}
               for r in sorted([r for r in rep["log"] if r.get("min_clearance_moving_m") is not None],
                               key=lambda r: r["objective"])[:5]],
              source=f"results/adversary_{rep['mode']}.json")

    def sil_evidence(parent: str) -> None:
        te = [t for r in trace.values() for t in r["tests"] if t["test"].startswith("tests/sil/")]
        te = list({t["test"]: t for t in te}.values())
        if not te:
            c.add(f"{parent}.sil", "solution", "Multi-process SIL: not run in this evidence set", parent, "missing")
            return
        npass = sum(t["outcome"] == "passed" for t in te)
        c.add(f"{parent}.sil", "solution", f"Multi-process SIL fault injection (process kills): {npass}/{len(te)} "
              "passed", parent, "passed" if npass == len(te) else "failed", te, source="results/traceability.json")

    for h in hz["hazards"]:
        hid = h["id"]
        c.add(hid, "goal", f"{hid}: {h['title']} is controlled.", "S0")
        for a in h.get("assumptions", []):
            c.add(f"{hid}.{a}", "assumption", f"Relies on {a}.", hid, ref=a)
        s = c.add(f"{hid}.S", "strategy", "Argue that each allocated safety requirement is correctly implemented, "
                  "then validate the integrated system against the hazard.", hid)
        for r in h["requirements"]:
            req_node(r, s)
        v = c.add(f"{hid}.V", "goal", f"Integrated validation shows {h['title'].lower()} does not occur.", s)
        for kind in h.get("validation", []):
            if kind == "shift":
                shift_evidence(v)
            elif kind == "adversary":
                adversary_evidence(v, adv_k, residual=False)
            elif kind == "adversary-carry":
                if adv_c:
                    adversary_evidence(v, adv_c, residual=False)
                else:
                    c.add(f"{v}.adv-carry", "solution", "Adversarial search (load-and-carry): results missing", v,
                          "missing")
            elif kind == "adversary-zones":
                adversary_evidence(v, adv_b, residual=True)
            elif kind == "sil":
                sil_evidence(v)
    for u in hz["open"]:
        c.add(u["id"], "undeveloped", u["text"], "S0", "open")

    c.resolve("G0")
    nodes = c.nodes
    summary = {
        "root": nodes["G0"]["status"],
        "goals": sum(n["kind"] == "goal" for n in nodes.values()),
        "solutions": sum(n["kind"] == "solution" for n in nodes.values()),
        "failed": [n["id"] for n in nodes.values() if n["kind"] == "solution" and n["status"] == "failed"],
        "missing": [n["id"] for n in nodes.values() if n["kind"] == "solution" and n["status"] == "missing"],
        "open": [n["id"] for n in nodes.values() if n["kind"] == "undeveloped"],
        "proofs": (sum(p["status"] == "proven" for p in ver["properties"]), len(ver["properties"])) if ver else None,
    }
    return {"summary": summary, "nodes": nodes}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="exit 1 unless every claim has passing evidence")
    ap.add_argument("--allow-missing", nargs="*", default=[], help="solution kinds allowed to be missing (CI subsets)")
    args = ap.parse_args()
    case = build()
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "safety_case.json").write_text(json.dumps(case, indent=1))
    from kauha.assurance.render import render
    out = render(case)
    s = case["summary"]
    print(f"safety case: root {s['root']}, {s['goals']} goals, {s['solutions']} evidence items, "
          f"failed {s['failed']}, missing {s['missing']}, open {s['open']} -> {out}")
    if args.check:
        missing = [m for m in s["missing"] if not any(m.endswith("." + k) for k in args.allow_missing)]
        if s["failed"] or missing:
            sys.exit("safety case does not hold: failed=" + str(s["failed"]) + " missing=" + str(missing))


if __name__ == "__main__":
    main()
