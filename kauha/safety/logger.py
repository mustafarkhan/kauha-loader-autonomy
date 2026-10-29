"""Tamper-evident safety event log (hash chain).

Each record stores the SHA-256 of the previous record, so deleting or editing
any record breaks verification of every later one. This supports incident
reconstruction and the event-logging duties for autonomous machinery; it is
not a substitute for a certified data recorder.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field

GENESIS = "0" * 64


def _digest(prev: str, body: dict) -> str:
    blob = json.dumps(body, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(prev.encode() + blob).hexdigest()


@dataclass
class EventLog:
    records: list[dict] = field(default_factory=list)

    @property
    def head(self) -> str:
        return self.records[-1]["hash"] if self.records else GENESIS

    def append(self, t: float, kind: str, **data) -> dict:
        body = {"seq": len(self.records), "t": round(t, 3), "kind": kind, "data": data}
        rec = {**body, "prev": self.head, "hash": _digest(self.head, body)}
        self.records.append(rec)
        return rec

    def verify(self) -> int | None:
        """Return the index of the first bad record, or None if the chain is intact."""
        prev = GENESIS
        for i, r in enumerate(self.records):
            body = {k: r[k] for k in ("seq", "t", "kind", "data")}
            if r["prev"] != prev or r["hash"] != _digest(prev, body) or r["seq"] != i:
                return i
            prev = r["hash"]
        return None
