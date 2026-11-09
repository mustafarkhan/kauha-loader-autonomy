# -*- coding: utf-8 -*-
"""Import the KAUHA safety PLC sources into an open CODESYS project.

Run inside CODESYS (V3.5 SP17 or later): open or create a "Standard project" for
your controller (for the testbed: CODESYS Control for Raspberry Pi SL), then
Tools > Scripting > Execute Script File... and pick this file.

It creates one DUT per TYPE, one POU per FUNCTION / FUNCTION_BLOCK / PROGRAM and
one GVL per VAR_GLOBAL list found in src/*.st and src/codesys_only/*.st, under
the first Application of the primary project. Existing objects of the same name
are replaced, so the script can be re-run after `python -m kauha.scale.profile`
regenerates GVL_Config.st.

Still done by hand afterwards (docs/scale/plc.md has the steps):
  * Library Manager: add "CAA CAN Low Level" (and "Standard", normally present).
  * Task configuration: cyclic task, 25 ms, calling PRG_Main.
  * Device: enable the CAN interface (can0) and map GVL_IO to the GPIO channels.

The CODESYS script engine is IronPython 2.7: no f-strings, no pathlib.
"""
from __future__ import print_function

import io
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE_DIRS = [os.path.join(HERE, "src"), os.path.join(HERE, "src", "codesys_only")]

UNIT_RE = re.compile(r"^(TYPE|FUNCTION_BLOCK|FUNCTION|PROGRAM|VAR_GLOBAL)\b", re.M)
END = {"TYPE": "END_TYPE", "FUNCTION_BLOCK": "END_FUNCTION_BLOCK", "FUNCTION": "END_FUNCTION",
       "PROGRAM": "END_PROGRAM", "VAR_GLOBAL": "END_VAR"}


def strip_comments_for_scan(text):
    """Blank out comments (keeping offsets) so keywords inside them are not matched."""
    out = list(text)
    for m in re.finditer(r"\(\*.*?\*\)", text, re.S):
        for i in range(m.start(), m.end()):
            if out[i] != "\n":
                out[i] = " "
    return "".join(out)


def units(text):
    """Yield (kind, name, declaration, implementation) for each unit in one file."""
    scan = strip_comments_for_scan(text)
    pos = 0
    gvl_parts = []
    while True:
        m = UNIT_RE.search(scan, pos)
        if not m:
            break
        kind = m.group(1)
        if kind == "VAR_GLOBAL":
            end = scan.index("END_VAR", m.end()) + len("END_VAR")
            gvl_parts.append(text[m.start():end])
            pos = end
            continue
        end_kw = END[kind]
        end = re.compile(r"\b%s\b" % end_kw).search(scan, m.end())
        body = text[m.start():end.start()]
        body_scan = scan[m.start():end.start()]
        name = re.match(r"\w+\s+(\w+)", body).group(1)
        pos = end.end()
        if kind == "TYPE":
            decl = body.rstrip() + "\nEND_TYPE\n"
            decl = decl.replace("END_STRUCT;", "END_STRUCT")
            yield "DUT", name, decl, None
            continue
        # declaration = header + VAR blocks; implementation = the rest
        last = 0
        for v in re.finditer(r"\bEND_VAR\b", body_scan):
            last = v.end()
        if last == 0:
            last = body_scan.index("\n") + 1
        yield kind, name, body[:last].rstrip() + "\n", body[last:].strip() + "\n"
    if gvl_parts:
        yield "GVL", None, "\n\n".join(gvl_parts) + "\n", None


def collect():
    found = []
    for d in SOURCE_DIRS:
        for fn in sorted(os.listdir(d)):
            if not fn.endswith(".st"):
                continue
            with io.open(os.path.join(d, fn), encoding="utf-8") as f:
                text = f.read()
            for kind, name, decl, impl in units(text):
                if kind == "GVL":
                    name = os.path.splitext(fn)[0]
                found.append((kind, name, decl, impl))
    return found


def run_in_codesys():
    from scriptengine import PouType, projects  # provided by CODESYS
    proj = projects.primary
    if proj is None:
        raise RuntimeError("open a CODESYS project first")
    app = proj.find("Application", True)[0]

    def replace_existing(name):
        for obj in app.find(name, False):
            obj.remove()

    for kind, name, decl, impl in collect():
        replace_existing(name)
        if kind == "DUT":
            obj = app.create_dut(name)
        elif kind == "GVL":
            obj = app.create_gvl(name)
        elif kind == "FUNCTION":
            ret = re.match(r"FUNCTION\s+\w+\s*:\s*(\w+)", decl).group(1)
            obj = app.create_pou(name, PouType.Function, return_type=ret)
        elif kind == "FUNCTION_BLOCK":
            obj = app.create_pou(name, PouType.FunctionBlock)
        else:
            obj = app.create_pou(name, PouType.Program)
        obj.textual_declaration.replace(decl)
        if impl is not None:
            obj.textual_implementation.replace(impl)
        print("imported %-16s %s" % (kind, name))
    print("done: now add the CAA CAN Low Level library, a 25 ms task calling PRG_Main, and the I/O mapping")


if __name__ == "__main__":
    try:
        import scriptengine  # noqa: F401
    except ImportError:
        # outside CODESYS: list what would be imported (used by the tests)
        for kind, name, _decl, _impl in collect():
            print("%-16s %s" % (kind, name))
    else:
        run_in_codesys()
