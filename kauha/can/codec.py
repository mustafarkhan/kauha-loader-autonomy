"""Encode/decode KAUHA machine-bus frames from the DBC, with E2E protection."""
from __future__ import annotations

from functools import lru_cache

import cantools

from kauha.can.e2e import E2EReceiver, protect
from kauha.config import REPO_ROOT

DBC_FILE = REPO_ROOT / "hardware" / "can" / "kauha_machine.dbc"

# J1939: priority(3) | reserved(1) | DP(1) | PF(8) | PS(8) | SA(8)
SA = {"AUTONOMY": 0x80, "SAFETY_PLC": 0x81, "REMOTE_GW": 0x82, "MACHINE_GW": 0x83}


def j1939_id(priority: int, pgn: int, source: int) -> int:
    return (priority << 26) | (pgn << 8) | source


def pgn_of(frame_id: int) -> int:
    return (frame_id >> 8) & 0x3FFFF


@lru_cache(maxsize=1)
def database() -> cantools.database.Database:
    return cantools.database.load_file(str(DBC_FILE))


class Codec:
    """Stateful encoder (alive counters per message) and E2E-checking decoder."""

    def __init__(self) -> None:
        self.db = database()
        self._ctr: dict[str, int] = {}

    def data_id(self, name: str) -> int:
        return pgn_of(self.db.get_message_by_name(name).frame_id) & 0xFF

    def encode(self, name: str, signals: dict) -> tuple[int, bytes]:
        msg = self.db.get_message_by_name(name)
        full = {s.name: 0 for s in msg.signals}
        full.update(signals)
        raw = bytearray(msg.encode(full, strict=True))
        ctr = (self._ctr.get(name, -1) + 1) % 16
        self._ctr[name] = ctr
        return msg.frame_id, protect(raw, self.data_id(name), ctr)

    def decode(self, frame_id: int, data: bytes) -> tuple[str, dict]:
        msg = self.db.get_message_by_frame_id(frame_id)
        return msg.name, msg.decode(data, decode_choices=False)

    def receiver(self, name: str, timeout_s: float, max_delta: int = 2) -> E2EReceiver:
        return E2EReceiver(self.data_id(name), timeout_s, max_delta)
