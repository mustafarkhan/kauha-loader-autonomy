"""Build the interactive CAD viewer page from the exported GLB and part list.

    python -m hardware.cad.kauha_cad && python -m hardware.cad.viewer
writes docs/results/kauha_cad.html (self-contained: the model is embedded).
"""
from __future__ import annotations

import base64
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "docs" / "results" / "kauha_cad.html"


def main() -> None:
    meta = json.loads((HERE / "export" / "parts.json").read_text())
    glb = base64.b64encode((HERE / "export" / "kauha_kit.glb").read_bytes()).decode()
    page = (HERE / "viewer_template.html").read_text()
    page = page.replace("/*__META__*/null", json.dumps(meta, ensure_ascii=False)).replace("/*__GLB__*/", glb)
    OUT.write_text(page)
    print(f"wrote {OUT} ({OUT.stat().st_size / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
