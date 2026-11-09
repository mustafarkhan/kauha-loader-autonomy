"""The CODESYS sources are complete and in step with the Python profiles."""
from __future__ import annotations

import re
import sys

from kauha.config import REPO_ROOT
from kauha.scale.profile import full_profile, scale_profile, st_config
from kauha.verify.st_equivalence import config_values

sys.path.insert(0, str(REPO_ROOT / "plc" / "codesys"))
import import_into_codesys as imp  # noqa: E402

GVL = REPO_ROOT / "plc" / "codesys" / "src" / "GVL_Config.st"


def test_generated_configuration_is_current():
    """GVL_Config.st is generated; regenerate with `python -m kauha.scale.profile` after a change."""
    text = GVL.read_text()
    for p in (full_profile(), scale_profile("nanoscan3"), scale_profile("evaluator")):
        assert st_config(p, f"CFG_{p.name.upper()}") in text, f"{p.name} out of date"


def test_configuration_fields_match_the_struct_and_the_harness_order():
    dut = (REPO_ROOT / "plc" / "codesys" / "src" / "DUT_Types.st").read_text()
    struct = re.search(r"TYPE ST_SafetyConfig :\s*STRUCT(.*?)END_STRUCT", dut, re.S).group(1)
    fields = re.findall(r"^\s*(\w+)\s*:", re.sub(r"\(\*.*?\*\)", "", struct), re.M)
    gvl_fields = re.findall(r"^\s+(\w+) := ", st_config(full_profile(), "X"), re.M)
    assert fields == gvl_fields
    assert len(config_values(full_profile())) == len(fields)


def test_import_script_finds_every_unit():
    names = {(k, n) for k, n, _, _ in imp.collect()}
    for expected in [("FUNCTION_BLOCK", "FB_SafetyCore"), ("FUNCTION_BLOCK", "FB_SafetyNode"),
                     ("FUNCTION_BLOCK", "FB_E2ERx"), ("FUNCTION_BLOCK", "FB_CanPort"), ("PROGRAM", "PRG_Main"),
                     ("DUT", "ST_SafetyConfig"), ("DUT", "ST_RxBatch"), ("GVL", "GVL_Config"), ("GVL", "GVL_IO"),
                     ("FUNCTION", "CRC8_J1850")]:
        assert expected in names
    for kind, name, decl, impl in imp.collect():
        if kind in ("FUNCTION", "FUNCTION_BLOCK", "PROGRAM"):
            assert decl.lstrip().startswith(kind) and "END_" + kind not in (impl or ""), name
