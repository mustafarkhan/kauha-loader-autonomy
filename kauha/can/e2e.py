"""End-to-end protection for safety-relevant CAN frames.

Each protected frame carries a 4-bit alive counter (byte 6, low nibble) and a
CRC-8 (byte 7). The CRC uses the SAE J1850 polynomial and is computed over a
per-message data ID followed by bytes 0..6, so a frame that lands on the wrong
identifier fails the check (masquerade). This follows the pattern of AUTOSAR
E2E profile 1/2; it is not a certified implementation.

The receiver detects: corruption (CRC), repetition (counter unchanged),
loss (counter jump larger than the allowed delta) and timeout.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

POLY = 0x1D
INIT = 0xFF
XOR_OUT = 0xFF


def _table() -> list[int]:
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ POLY) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return t


_T = _table()


def crc8_j1850(data: bytes, init: int = INIT) -> int:
    c = init
    for b in data:
        c = _T[c ^ b]
    return c ^ XOR_OUT


def protect(payload: bytearray, data_id: int, counter: int) -> bytes:
    """Write counter and CRC into an 8-byte payload."""
    if len(payload) != 8:
        raise ValueError("protected frames are 8 bytes")
    payload[6] = (payload[6] & 0xF0) | (counter & 0x0F)
    payload[7] = crc8_j1850(bytes([data_id & 0xFF]) + bytes(payload[:7]))
    return bytes(payload)


class E2EStatus(Enum):
    OK = "ok"
    OK_SOME_LOST = "ok_some_lost"
    REPEATED = "repeated"
    WRONG_SEQUENCE = "wrong_sequence"
    CRC_ERROR = "crc_error"
    TIMEOUT = "timeout"
    NO_DATA = "no_data"


@dataclass
class E2EReceiver:
    data_id: int
    timeout_s: float
    max_delta: int = 2
    last_counter: int | None = None
    last_ok_time: float | None = None

    def check(self, payload: bytes, t: float) -> E2EStatus:
        if crc8_j1850(bytes([self.data_id & 0xFF]) + payload[:7]) != payload[7]:
            return E2EStatus.CRC_ERROR
        ctr = payload[6] & 0x0F
        if self.last_counter is None:
            status = E2EStatus.OK
        else:
            delta = (ctr - self.last_counter) % 16
            if delta == 0:
                return E2EStatus.REPEATED
            if delta > self.max_delta:
                status = E2EStatus.WRONG_SEQUENCE
            else:
                status = E2EStatus.OK if delta == 1 else E2EStatus.OK_SOME_LOST
        self.last_counter = ctr
        if status in (E2EStatus.OK, E2EStatus.OK_SOME_LOST):
            self.last_ok_time = t
        return status

    def fresh(self, t: float) -> bool:
        return self.last_ok_time is not None and t - self.last_ok_time <= self.timeout_s
