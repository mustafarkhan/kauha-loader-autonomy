"""Message authentication for frames that cross from the open bus segment to the safety segment.

The CRC and alive counter of ``kauha.can.e2e`` detect faults. They use no secret, so
anyone who can put a frame on the bus can compute them. Authentication adds a secret:

  * AES-128 CMAC (RFC 4493, the algorithm AUTOSAR SecOC uses), truncated to 32 bits;
  * a 24-bit freshness counter that must increase, so a recorded frame is refused;
  * a session key derived from the sender's key and a random challenge the guard
    issues at power-up, so frames recorded in an earlier session are refused too.

Classic CAN frames have no room for this, so each authenticated frame is followed by
a companion frame ``FrameAuth`` from the same sender:

    byte 0      data ID of the frame it authenticates (as in the E2E CRC)
    bytes 1..3  freshness counter, little endian
    bytes 4..7  first 4 bytes of CMAC(session key, data ID | frame id (4, big endian) |
                payload (8) | freshness (3, little endian))        -- one AES block

This is a reference implementation in plain Python so that the firmware
(firmware/common/kauha_cmac.c) can be checked against it bit for bit. It is not a
certified cryptographic module and is not constant-time; the guard's microcontroller
runs the C version. Keys are provisioned at commissioning and never stored in this
repository: ``TEST_KEYS`` are the public RFC 4493 example key and variations of it,
for tests only.
"""
from __future__ import annotations

import json
import struct
from pathlib import Path

_SBOX = []


def _init_sbox() -> None:
    p = q = 1
    box = [0] * 256
    while True:
        p = p ^ ((p << 1) & 0xFF) ^ (0x1B if p & 0x80 else 0)          # multiply by 3
        q ^= q << 1
        q ^= q << 2
        q ^= q << 4
        q &= 0xFF
        if q & 0x80:
            q ^= 0x09                                                   # divide by 3
        x = q ^ ((q << 1 | q >> 7) & 0xFF) ^ ((q << 2 | q >> 6) & 0xFF) ^ ((q << 3 | q >> 5) & 0xFF) \
            ^ ((q << 4 | q >> 4) & 0xFF)
        box[p] = x ^ 0x63
        if p == 1:
            break
    box[0] = 0x63
    _SBOX.extend(box)


_init_sbox()
_RCON = (0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36)


def _xt(a: int) -> int:
    return ((a << 1) ^ 0x1B) & 0xFF if a & 0x80 else a << 1


def expand_key(key: bytes) -> list[bytes]:
    if len(key) != 16:
        raise ValueError("AES-128 key is 16 bytes")
    w = [list(key[i:i + 4]) for i in range(0, 16, 4)]
    for i in range(4, 44):
        t = list(w[i - 1])
        if i % 4 == 0:
            t = [_SBOX[t[1]] ^ _RCON[i // 4 - 1], _SBOX[t[2]], _SBOX[t[3]], _SBOX[t[0]]]
        w.append([a ^ b for a, b in zip(w[i - 4], t)])
    return [bytes(sum(w[r * 4:r * 4 + 4], [])) for r in range(11)]


def aes128_encrypt(rk: list[bytes], block: bytes) -> bytes:
    s = [b ^ k for b, k in zip(block, rk[0])]
    for rnd in range(1, 11):
        s = [_SBOX[b] for b in s]
        s = [s[(i + 4 * (i % 4)) % 16] for i in range(16)]             # shift rows (column-major state)
        if rnd < 10:
            m = []
            for c in range(0, 16, 4):
                a0, a1, a2, a3 = s[c:c + 4]
                t = a0 ^ a1 ^ a2 ^ a3
                m += [a0 ^ t ^ _xt(a0 ^ a1), a1 ^ t ^ _xt(a1 ^ a2), a2 ^ t ^ _xt(a2 ^ a3), a3 ^ t ^ _xt(a3 ^ a0)]
            s = m
        s = [b ^ k for b, k in zip(s, rk[rnd])]
    return bytes(s)


def _dbl(b: bytes) -> bytes:
    n = int.from_bytes(b, "big") << 1
    if n >> 128:
        n = (n & ((1 << 128) - 1)) ^ 0x87
    return n.to_bytes(16, "big")


class Cmac:
    """AES-128 CMAC with the key schedule and subkeys computed once."""

    def __init__(self, key: bytes):
        self.rk = expand_key(key)
        self.k1 = _dbl(aes128_encrypt(self.rk, bytes(16)))
        self.k2 = _dbl(self.k1)

    def tag(self, msg: bytes) -> bytes:
        n = max(1, -(-len(msg) // 16))
        last = msg[(n - 1) * 16:]
        if len(last) == 16:
            last = bytes(a ^ b for a, b in zip(last, self.k1))
        else:
            last = bytes(a ^ b for a, b in zip(last + b"\x80" + bytes(15 - len(last)), self.k2))
        x = bytes(16)
        for i in range(n - 1):
            x = aes128_encrypt(self.rk, bytes(a ^ b for a, b in zip(x, msg[i * 16:i * 16 + 16])))
        return aes128_encrypt(self.rk, bytes(a ^ b for a, b in zip(x, last)))


def cmac(key: bytes, msg: bytes) -> bytes:
    """AES-128 CMAC, full 16-byte tag."""
    return Cmac(key).tag(msg)


RFC4493_KEY = bytes.fromhex("2b7e151628aed2a6abf7158809cf4f3c")
# Test keys only (public values). Real keys are generated and loaded at commissioning.
TEST_KEYS = {0x80: RFC4493_KEY, 0x82: bytes(b ^ 0x82 for b in RFC4493_KEY), 0x86: bytes(b ^ 0x86 for b in RFC4493_KEY)}

SESSION_LABEL = b"KAUHA-S1"
MAC_BYTES = 4
FRESHNESS_MAX = (1 << 24) - 1


def session_key(sender_key: bytes, challenge: bytes) -> bytes:
    """Key for one power cycle of the guard: CMAC(sender key, label | 8-byte challenge)."""
    if len(challenge) != 8:
        raise ValueError("challenge is 8 bytes")
    return cmac(sender_key, SESSION_LABEL + challenge)


def mac_input(data_id: int, frame_id: int, payload: bytes, freshness: int) -> bytes:
    return bytes([data_id & 0xFF]) + struct.pack(">I", frame_id & 0x1FFFFFFF) + payload[:8] \
        + struct.pack("<I", freshness)[:3]


def auth_payload(skey: "bytes | Cmac", data_id: int, frame_id: int, payload: bytes, freshness: int) -> bytes:
    """The 8 data bytes of the FrameAuth companion frame."""
    c = skey if isinstance(skey, Cmac) else Cmac(skey)
    tag = c.tag(mac_input(data_id, frame_id, payload, freshness))[:MAC_BYTES]
    return bytes([data_id & 0xFF]) + struct.pack("<I", freshness)[:3] + tag


class AuthSender:
    """Sender side: wraps each protected frame with its companion frame."""

    def __init__(self, source: int, key: bytes):
        self.source, self.key = source, key
        self.challenge: bytes | None = None
        self.skey: Cmac | None = None
        self.freshness = 0

    def on_challenge(self, challenge: bytes) -> None:
        if challenge != self.challenge:                 # the guard restarted: new session, counter from 1
            self.challenge, self.skey, self.freshness = challenge, Cmac(session_key(self.key, challenge)), 0

    def sign(self, frame_id: int, payload: bytes) -> tuple[int, bytes] | None:
        """(FrameAuth id, data) to send right after the frame, or None before the first challenge
        or when the counter is used up (the guard then issues a new challenge)."""
        if self.skey is None or self.freshness >= FRESHNESS_MAX:
            return None
        self.freshness += 1
        data_id = (frame_id >> 8) & 0xFF
        return auth_frame_id(self.source), auth_payload(self.skey, data_id, frame_id, payload, self.freshness)


AUTH_PGN = 0xFF60          # FrameAuth, one identifier per sender (source address in the low byte)
CHALLENGE_PGN = 0xFF61     # AuthChallenge, sent by the guard on the open segment


def auth_frame_id(source: int) -> int:
    return (6 << 26) | (AUTH_PGN << 8) | (source & 0xFF)


def challenge_frame_id(source: int = 0x83) -> int:
    return (6 << 26) | (CHALLENGE_PGN << 8) | source


def load_keys(path: "str | Path") -> dict[int, bytes]:
    """Keys from the file written at commissioning: {"0x80": "<32 hex digits>", ...}. A node needs
    only its own entry; the guard needs all of them. The file stays outside the repository."""
    raw = json.loads(Path(path).read_text())
    keys = {int(k, 0): bytes.fromhex(v) for k, v in raw.items()}
    if any(len(v) != 16 for v in keys.values()):
        raise ValueError("every key is 16 bytes")
    return keys
