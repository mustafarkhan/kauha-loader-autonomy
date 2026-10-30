"""Render the generated safety case as one self-contained HTML page (GSN tree)."""
from __future__ import annotations

import datetime as dt
import html
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
OUT = REPO_ROOT / "docs" / "results" / "safety_case.html"

STATUS = {
    "passed": ("Supported", "ok"),
    "passed-open": ("Supported, open items", "okopen"),
    "failed": ("Evidence failing", "bad"),
    "missing": ("No evidence", "warn"),
    "open": ("Undeveloped", "open"),
    "info": ("Residual risk measured", "info"),
}

GLYPH = {
    "goal": '<svg viewBox="0 0 28 18" aria-hidden="true"><rect x="1" y="1" width="26" height="16"/></svg>',
    "strategy": '<svg viewBox="0 0 28 18" aria-hidden="true"><path d="M6 1H27L22 17H1Z"/></svg>',
    "solution": '<svg viewBox="0 0 28 18" aria-hidden="true"><circle cx="14" cy="9" r="8"/></svg>',
    "context": '<svg viewBox="0 0 28 18" aria-hidden="true"><rect x="1" y="1" width="26" height="16" rx="8"/></svg>',
    "assumption": '<svg viewBox="0 0 28 18" aria-hidden="true"><ellipse cx="14" cy="9" rx="13" ry="8"/>'
                  '<text x="22" y="17" font-size="7">A</text></svg>',
    "undeveloped": '<svg viewBox="0 0 28 18" aria-hidden="true"><rect x="1" y="1" width="26" height="11"/>'
                   '<path d="M14 12l3 3-3 3-3-3z"/></svg>',
}
KIND_LABEL = {"goal": "Goal", "strategy": "Strategy", "solution": "Evidence", "context": "Context",
              "assumption": "Assumption", "undeveloped": "Undeveloped goal"}

CSS = """
/* Layout: one reading column; the argument is an indented GSN tree with glyph, id, claim and status per row */
:root {
  --bg: #eef0ea; --panel: #f8f9f5; --ink: #1f2420; --muted: #5d665d; --line: #c9cec3;
  --machine: #e0a100; --ok: #2f8a4c; --bad: #c8322b; --warn: #b76e12; --info: #2c6fae; --open: #6b7680;
  --ok-bg: rgba(47,138,76,.12); --bad-bg: rgba(200,50,43,.12); --warn-bg: rgba(183,110,18,.13);
  --info-bg: rgba(44,111,174,.12); --open-bg: rgba(107,118,128,.12);
  --display: "Barlow Condensed", "Arial Narrow", sans-serif;
  --body: "IBM Plex Sans", "Segoe UI", system-ui, sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, Menlo, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #161a17; --panel: #1e231f; --ink: #e6e9e2; --muted: #9aa398; --line: #343b35;
  --machine: #f2b616; --ok: #4fb66e; --bad: #ef5a50; --warn: #e39a3c; --info: #6aa7e0; --open: #9aa5ae;
  --ok-bg: rgba(79,182,110,.14); --bad-bg: rgba(239,90,80,.14); --warn-bg: rgba(227,154,60,.14);
  --info-bg: rgba(106,167,224,.14); --open-bg: rgba(154,165,174,.14); color-scheme: dark } }
:root[data-theme="dark"] {
  --bg: #161a17; --panel: #1e231f; --ink: #e6e9e2; --muted: #9aa398; --line: #343b35;
  --machine: #f2b616; --ok: #4fb66e; --bad: #ef5a50; --warn: #e39a3c; --info: #6aa7e0; --open: #9aa5ae;
  --ok-bg: rgba(79,182,110,.14); --bad-bg: rgba(239,90,80,.14); --warn-bg: rgba(227,154,60,.14);
  --info-bg: rgba(106,167,224,.14); --open-bg: rgba(154,165,174,.14); color-scheme: dark }
* { box-sizing: border-box }
body { background: var(--bg); color: var(--ink); font: 15px/1.55 var(--body); margin: 0 }
.wrap { max-width: 1080px; margin: 0 auto; padding-block: 28px 64px;
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)) }
h1, h2 { font-family: var(--display); font-weight: 600; letter-spacing: .01em; text-wrap: balance; margin: 0 }
h1 { font-size: clamp(32px, 5vw, 46px); line-height: 1.02 }
h2 { font-size: 26px; margin-top: 36px }
.eyebrow { font: 500 12px/1.3 var(--mono); letter-spacing: .09em; text-transform: uppercase; color: var(--muted) }
.lede { max-width: 68ch; color: var(--muted); margin: 12px 0 0 }
.claim { margin-top: 22px; display: grid; grid-template-columns: auto minmax(0, 1fr); gap: 4px 14px; align-items: start;
  background: var(--panel); border: 1px solid var(--line); border-left: 6px solid var(--machine); padding: 16px 18px }
.claim p { margin: 0; font-size: 17px; max-width: 70ch }
.stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 0; margin-top: 18px;
  border-top: 2px solid var(--ink) }
.stats div { padding: 10px 14px 4px 0 }
.stats b { display: block; font: 600 34px/1 var(--display); font-variant-numeric: tabular-nums }
.stats span { font-size: 13px; color: var(--muted) }
.legend { display: flex; flex-wrap: wrap; gap: 6px 18px; font-size: 13px; color: var(--muted); margin-top: 14px }
.legend span { display: inline-flex; align-items: center; gap: 6px }
.g { width: 28px; height: 18px; flex: none; display: inline-block }
.g svg { width: 28px; height: 18px; fill: var(--panel); stroke: var(--ink); stroke-width: 1.3; overflow: visible }
.g svg text { fill: var(--ink); stroke: none; font-family: var(--mono) }
.tree { margin-top: 14px; border-top: 1px solid var(--line) }
.node { display: grid; grid-template-columns: 28px 4.6rem minmax(0, 1fr) auto; gap: 10px; align-items: start;
  padding: 7px 0; border-bottom: 1px solid var(--line) }
.node .id { font: 500 12px/1.9 var(--mono); color: var(--muted); overflow-wrap: anywhere }
.node .text { min-width: 0; overflow-wrap: anywhere }
.node.goal > .text { font-weight: 500 }
.node.strategy > .text, .node.context > .text, .node.assumption > .text { color: var(--muted); font-size: 14px }
.chip { font: 500 11px/1 var(--mono); letter-spacing: .04em; text-transform: uppercase; padding: 5px 7px;
  border-radius: 3px; white-space: nowrap; margin-top: 2px }
.chip.ok { color: var(--ok); background: var(--ok-bg) } .chip.bad { color: var(--bad); background: var(--bad-bg) }
.chip.warn { color: var(--warn); background: var(--warn-bg) } .chip.info { color: var(--info); background: var(--info-bg) }
.chip.open { color: var(--open); background: var(--open-bg) }
.chip.okopen { color: var(--ok); background: transparent; box-shadow: inset 0 0 0 1px var(--ok) }
.kids { margin-left: 22px; border-left: 1px dashed var(--line); padding-left: 12px }
details.hz > summary { list-style: none; cursor: pointer }
details.hz > summary::-webkit-details-marker { display: none }
details.hz > summary .node { border-bottom: 1px solid var(--line) }
details.hz > summary:focus-visible { outline: 2px solid var(--info); outline-offset: 2px }
details.hz > summary .text::after { content: " ▸"; color: var(--muted) }
details.hz[open] > summary .text::after { content: " ▾" }
details.ev { grid-column: 3 / 5; font-size: 13px }
details.ev summary { cursor: pointer; color: var(--info); width: max-content }
details.ev summary:focus-visible { outline: 2px solid var(--info) }
.evbox { overflow-x: auto; margin-top: 6px }
table { border-collapse: collapse; font-size: 12.5px; font-variant-numeric: tabular-nums }
th, td { text-align: left; padding: 4px 10px 4px 0; border-bottom: 1px solid var(--line); vertical-align: top }
th { font: 500 11px/1.4 var(--mono); text-transform: uppercase; letter-spacing: .05em; color: var(--muted) }
td.pass { color: var(--ok) } td.fail { color: var(--bad) }
.src { font: 12px var(--mono); color: var(--muted) }
.foot { margin-top: 32px; font-size: 13px; color: var(--muted); max-width: 75ch }
code { font-family: var(--mono); font-size: .92em }
@media (max-width: 560px) { .node { grid-template-columns: 28px minmax(0, 1fr); } .node .id { grid-column: 2 }
  .node .text { grid-column: 2 } .node .chip { grid-column: 2; justify-self: start } details.ev { grid-column: 2 }
  .kids { margin-left: 8px; padding-left: 8px } }
@media (prefers-reduced-motion: reduce) { * { transition: none !important } }
"""


def _e(x) -> str:
    return html.escape(str(x))


def _evidence_table(n: dict) -> str:
    ev = n["evidence"]
    if not ev:
        return ""
    if "test" in ev[0]:
        rows = "".join(f'<tr><td><code>{_e(t["test"])}</code></td><td class="{"pass" if t["outcome"] == "passed" else "fail"}">'
                       f'{_e(t["outcome"])}</td></tr>' for t in ev)
        body = f"<table><tr><th>Test</th><th>Outcome</th></tr>{rows}</table>"
    elif "proven" in ev[0]:
        rows = "".join(f'<tr><td>{_e(p["id"])}</td><td>{_e(p["text"])}</td><td class="{"pass" if p["proven"] == p["configs"] else "fail"}">'
                       f'{p["proven"]}/{p["configs"]}</td></tr>' for p in ev)
        body = f"<table><tr><th>Property</th><th>Statement</th><th>Proven</th></tr>{rows}</table>"
    elif "closest_m" in ev[0]:
        rows = "".join(f'<tr><td>{_e(r["config"])}</td><td>{r["seed"]}</td><td>{r["closest_m"]:.2f}</td>'
                       f'<td>{r["field_entries"]}</td><td>{r["under_bucket_s"]:.1f}</td></tr>' for r in ev)
        body = ("<table><tr><th>Configuration</th><th>Seed</th><th>Closest to moving machine (m)</th>"
                f"<th>Field entries while moving</th><th>Under raised bucket (s)</th></tr>{rows}</table>")
    elif "clearance_m" in ev[0]:
        rows = ""
        for r in ev:
            s = r["scenario"]
            dust = "" if s["dust"] < 0.002 else ", dust {:.3f}".format(s["dust"])
            rows += (f'<tr><td>{_e(r["method"])}</td><td>{r["clearance_m"]:.2f}</td>'
                     f'<td>appears {s["t_appear"]:.0f} s at ({s["start"][0]:.0f}, {s["start"][1]:.0f}), '
                     f'via ({s["wp1"][0]:.0f}, {s["wp1"][1]:.0f}) pausing {s["pause_s"]:.0f} s, '
                     f'to ({s["wp2"][0]:.0f}, {s["wp2"][1]:.0f}) at {s["speed"]:.1f} m/s{dust}</td></tr>')
        body = ("<table><tr><th>Search</th><th>Clearance (m)</th>"
                f"<th>Worst scenarios found (site coordinates, m)</th></tr>{rows}</table>")
    else:
        return ""
    label = {"test": "tests", "proven": "properties", "closest_m": "runs", "clearance_m": "worst cases"}
    key = next(k for k in label if k in ev[0])
    return (f'<details class="ev"><summary>Show {len(ev)} {label[key]}</summary><div class="evbox">{body}'
            f'<p class="src">Source: {_e(n.get("source", ""))}</p></div></details>')


def _row(n: dict) -> str:
    chip = ""
    if n["status"] and n["kind"] not in ("context", "assumption", "strategy"):
        label, cls = STATUS[n["status"]]
        chip = f'<span class="chip {cls}">{label}</span>'
    return (f'<div class="node {n["kind"]}"><span class="g" title="{KIND_LABEL[n["kind"]]}">{GLYPH[n["kind"]]}</span>'
            f'<span class="id">{_e(n["id"].split(".")[-1] if n["kind"] == "solution" else n["id"].split(".S.")[-1])}</span>'
            f'<span class="text">{_e(n["text"])}</span>{chip}{_evidence_table(n)}</div>')


def _subtree(nodes: dict, nid: str) -> str:
    n = nodes[nid]
    kids = "".join(_subtree(nodes, c) for c in n["children"])
    if n["kind"] == "goal" and n["parent"] == "S0":           # hazard goals collapse
        return f'<details class="hz" open><summary>{_row(n)}</summary><div class="kids">{kids}</div></details>'
    return _row(n) + (f'<div class="kids">{kids}</div>' if kids else "")


def render(case: dict, out: Path = OUT) -> Path:
    nodes, s = case["nodes"], case["summary"]
    try:
        sha = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT, capture_output=True,
                             text=True).stdout.strip()
    except OSError:
        sha = ""
    root = nodes["G0"]
    label, cls = STATUS[root["status"]]
    proofs = f"{s['proofs'][0]}/{s['proofs'][1]}" if s["proofs"] else "—"
    hazards = sum(1 for n in nodes.values() if n["parent"] == "S0" and n["kind"] == "goal")
    reqs = len({n["req"] for n in nodes.values() if n.get("req")})
    legend = "".join(f'<span><span class="g">{GLYPH[k]}</span>{v}</span>' for k, v in KIND_LABEL.items())
    tree = "".join(_subtree(nodes, c) for c in root["children"])
    page = f"""<title>KAUHA Safety Case</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@600&family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500&display=swap">
<style>{CSS}</style>
<div class="wrap">
  <div class="eyebrow">KAUHA autonomous wheel-loader kit · safety case · generated from evidence</div>
  <h1>Is the loader safe for people on foot?</h1>
  <p class="lede">This argument is generated by <code>python -m kauha.assurance.case</code> from the hazard list and the
  evidence CI produces: formal proofs, requirement-traced tests, 15 simulated shifts and an adversarial search. No status
  on this page is typed by hand. CI fails if any claim loses its evidence.</p>
  <div class="claim"><span class="g">{GLYPH["goal"]}</span><p>{_e(root["text"])}<br>
  <span class="chip {cls}" style="display:inline-block;margin-top:8px">{label}</span></p></div>
  <div class="stats">
    <div><b>{proofs}</b><span>proofs of safety properties</span></div>
    <div><b>{hazards}</b><span>hazards argued</span></div>
    <div><b>{reqs}</b><span>safety requirements with evidence</span></div>
    <div><b>{s["solutions"]}</b><span>evidence items</span></div>
    <div><b>{len(s["failed"]) + len(s["missing"])}</b><span>failing or missing</span></div>
    <div><b>{len(s["open"])}</b><span>goals that need hardware or certification</span></div>
  </div>
  <h2>The argument</h2>
  <div class="legend">{legend}</div>
  <div class="tree">{tree}</div>
  <p class="foot">Goal Structuring Notation (GSN). Evidence comes from the digital twin, a multi-process software-in-the-loop
  run and z3 proofs of the safety PLC's step function over real arithmetic. It is not field data and not a certification
  artefact. The undeveloped goals list what only a hardware bench, the target machine and a certified PLC toolchain can close.
  {f"Generated at commit <code>{_e(sha)}</code>, " if sha else ""}{dt.date.today().isoformat()}.</p>
</div>
"""
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(page)
    return out
