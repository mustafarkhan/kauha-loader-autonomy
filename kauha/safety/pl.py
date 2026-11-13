"""Performance level of the protective stop, as far as the parts are selected (ISO 13849-1).

Simplified method: every subsystem contributes a probability of dangerous failure per
hour (PFHd) and the contributions add. PL d needs the sum below 1e-6 1/h.

  * Devices sold with a PL (scanner, safety relay) bring their data-sheet PFHd.
  * The output stage is built from two monitored channels (category 3). Its PFHd
    comes from the channel MTTFd and the diagnostic coverage through table K.1.
    MTTFd of a switching part: B10d / (0.1 * operations per year), capped at 100 years.

Not selected yet, so no number: the safety radar system and the safety controller
of the full-size machine. The result is therefore a budget: what those two may add.

Table K.1 values below were read from a manufacturer's reproduction of the table
(Omron, "PFHD and PL"); the column assignment was matched on the well-known values
at 100 years. Confirm them in the standard before using this in a real assessment.
This is a design calculation, not a validation to ISO 13849-2.
"""
from __future__ import annotations

from dataclasses import dataclass

PL_D_LIMIT = 1e-6          # PL d: 1e-7 <= PFHd < 1e-6 per hour
PL_E_LIMIT = 1e-7

# MTTFd per channel (years) -> PFHd (1/h), category 3
K1_CAT3 = {
    "low": {30: 6.94e-7, 36: 5.16e-7, 43: 3.87e-7, 51: 2.93e-7, 62: 2.13e-7, 75: 1.57e-7, 91: 1.14e-7, 100: 1.01e-7},
    "medium": {30: 2.65e-7, 36: 2.01e-7, 43: 1.54e-7, 51: 1.19e-7, 62: 8.84e-8, 75: 6.62e-8, 91: 4.94e-8,
               100: 4.29e-8},
}

# Common-cause failure measures claimed by the design (ISO 13849-1 annex F scores; 65 needed).
CCF = (("separation of the two channels' wiring", 15), ("diversity: scanner and radar, valve and relay", 20),
       ("overvoltage and overcurrent protection", 15), ("failure mode and effects analysis done", 5),
       ("training of designers", 5), ("EMC and supply disturbance immunity", 25),
       ("temperature, shock, vibration, humidity", 10))


def mttfd_years(b10d: float, operations_per_year: float) -> float:
    return min(100.0, b10d / (0.1 * operations_per_year))


def pfhd_cat3(mttfd: float, dc: str) -> float:
    """Table value for the largest tabulated MTTFd not above the channel's (conservative)."""
    rows = K1_CAT3[dc]
    usable = [m for m in rows if m <= mttfd]
    if not usable:
        raise ValueError("MTTFd below 30 years: category 3 cannot reach PL d here")
    return rows[max(usable)]


@dataclass(frozen=True)
class Part:
    name: str
    pfhd: float | None      # None: not selected yet
    basis: str


def stop_chain(stops_per_hour: float = 7.0, hours_per_year: float = 16 * 250) -> dict:
    """The protective stop of the full-size machine: sensors -> logic -> brake and drive cut-off."""
    n_op = stops_per_hour * hours_per_year
    m = mttfd_years(1.3e6, n_op)                     # contactor at nominal load, SISTEMA typical value
    parts = [
        Part("4 safety laser scanners", 4 * 8.0e-8, "SICK outdoorScan3 data sheet: PL d, cat. 3, PFHd 8.0e-8 each; "
             "all four counted although one direction uses two"),
        Part("output stage: brake valve and drive cut-off, two monitored channels", pfhd_cat3(m, "medium"),
             f"category 3, contactor B10d 1.3e6, {n_op:.0f} operations a year -> MTTFd {m:.0f} years (capped), "
             "DC medium by read-back contacts"),
        Part("safety radar system (6 sensors, 1 control unit)", None, "PL d, cat. 3 per product data; PFHd to be "
             "obtained from the manufacturer"),
        Part("safety controller", None, "not selected; rated controllers are typically 1e-9 to 1e-8"),
    ]
    known = sum(p.pfhd for p in parts if p.pfhd is not None)
    return {"parts": parts, "known_pfhd": known, "budget_for_unselected": PL_D_LIMIT - known,
            "ccf_score": sum(s for _, s in CCF), "mttfd_output_years": m, "operations_per_year": n_op}


def testbed_relay_stage() -> dict:
    """The one rated part of the 1:8 testbed's stop chain. With the Raspberry Pi as logic the testbed
    has no performance level; this is the relay's own figure only."""
    return {"part": "Pilz PNOZ s4", "pl": "e", "category": 4, "pfhd": 2.31e-9, "mission_years": 20}
