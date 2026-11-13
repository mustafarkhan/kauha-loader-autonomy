"""Segment guard: the only connection between the open bus segment and the safety segment.

The machine bus is split in two:

  safety segment  safety PLC, sensor node, field evaluation, the gateway's machine
                  side. Inside the sealed control cabinet and the machine harness;
                  no diagnostic connector, no radio, no Ethernet.
  open segment    autonomy computer, remote-control gateway, zone source. These
                  have Ethernet, radio and USB, so this segment is treated as
                  reachable by an attacker.

The guard runs on the gateway controller, which has a CAN port on each segment
(firmware/common/guard_core.c is the same logic in C). Rules:

  safety -> open   every frame is copied (status for the autonomy computer and console).
  open -> safety   only the identifiers in ``ALLOWED`` and only when the frame is
                   followed within ``PAIR_WINDOW_S`` by a valid FrameAuth from the
                   same sender (kauha.can.auth): right key, increasing freshness.
                   Everything else is dropped and counted. An identifier that
                   belongs to a safety-segment node (SafetyStatus, SafeSensors,
                   ScannerStatus, MachineState) seen on the open segment is counted
                   as an intrusion.

What this does and does not give:
  * A device attached to the open segment, or a compromised remote link, cannot
    forge the PLC's status, the sensor frames, the remote heartbeat or the zone
    status, and cannot replay recorded commands.
  * A compromised autonomy computer holds its own key and can still send any
    DriveCmd. That is the case the safety design already assumes: the PLC limits
    speed, watches the fields and stops the machine whatever the commands say.
  * Someone with a tool inside the cabinet is on the safety segment and is not
    stopped by this; neither is someone who can cut the brake line. Physical
    access to the cabinet is controlled by the site, not by the bus.
  * The stop does not depend on any of it: relays, not frames, cut the power.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from kauha.can.auth import AUTH_PGN, FRESHNESS_MAX, MAC_BYTES, Cmac, auth_payload, challenge_frame_id, session_key

PAIR_WINDOW_S = 0.02
CHALLENGE_PERIOD_S = 1.0
GUARD_SA = 0x83


def _pgn(frame_id: int) -> int:
    return (frame_id >> 8) & 0x3FFFF


def _sa(frame_id: int) -> int:
    return frame_id & 0xFF


# Frames allowed from the open segment: (PGN, source address) -> name. From kauha_machine.dbc.
ALLOWED = {(0xFF10, 0x80): "DriveCmd", (0xFF11, 0x80): "ImplementCmd", (0xFF40, 0x82): "RemoteHeartbeat",
           (0xFF52, 0x86): "ZoneStatus"}
# Source addresses that exist only on the safety segment.
SAFETY_SOURCES = (0x81, 0x83, 0x84, 0x85)


@dataclass
class GuardStats:
    forwarded: int = 0
    dropped_not_allowed: int = 0
    dropped_no_auth: int = 0          # data frame never followed by its FrameAuth
    dropped_bad_mac: int = 0
    dropped_stale: int = 0            # freshness not above the last accepted one
    intrusions: int = 0               # a safety-segment identifier seen on the open segment


@dataclass
class _Pending:
    frame_id: int
    data: bytes
    t: float


@dataclass
class SegmentGuard:
    keys: dict[int, bytes]                        # sender source address -> 128-bit key
    challenge: bytes                              # 8 random bytes drawn at power-up
    stats: GuardStats = field(default_factory=GuardStats)

    def __post_init__(self) -> None:
        if len(self.challenge) != 8:
            raise ValueError("challenge is 8 bytes")
        self._skey = {sa: Cmac(session_key(k, self.challenge)) for sa, k in self.keys.items()}
        self._pending: dict[tuple[int, int], _Pending] = {}     # (sender, data ID) -> frame waiting for its MAC
        self._last: dict[tuple[int, int], int] = {}             # (sender, data ID) -> last accepted freshness
        self._next_challenge = 0.0

    def challenge_frame(self, t: float) -> tuple[int, bytes] | None:
        """The AuthChallenge frame to put on the open segment, once per period."""
        if t < self._next_challenge:
            return None
        self._next_challenge = t + CHALLENGE_PERIOD_S
        return challenge_frame_id(GUARD_SA), self.challenge

    def from_safety(self, frame_id: int, data: bytes) -> tuple[int, bytes]:
        """Safety segment -> open segment: copied unchanged."""
        return frame_id, data

    def from_open(self, t: float, frame_id: int, data: bytes) -> tuple[int, bytes] | None:
        """One frame received on the open segment. Returns the frame to forward to the safety
        segment, or None."""
        pgn, sa = _pgn(frame_id), _sa(frame_id)
        if pgn == AUTH_PGN:
            return self._on_auth(t, sa, data)
        if sa in SAFETY_SOURCES:
            self.stats.intrusions += 1
            return None
        if (pgn, sa) not in ALLOWED or len(data) != 8:
            self.stats.dropped_not_allowed += 1
            return None
        key = (sa, pgn & 0xFF)
        if key in self._pending:
            self.stats.dropped_no_auth += 1
        self._pending[key] = _Pending(frame_id, bytes(data), t)
        return None

    def _on_auth(self, t: float, sa: int, data: bytes) -> tuple[int, bytes] | None:
        if sa not in self._skey or len(data) != 8:
            self.stats.dropped_not_allowed += 1
            return None
        key = (sa, data[0])
        p = self._pending.pop(key, None)
        if p is None or t - p.t > PAIR_WINDOW_S:
            self.stats.dropped_no_auth += 1
            return None
        fresh = int.from_bytes(data[1:4], "little")
        if fresh <= self._last.get(key, 0) or fresh > FRESHNESS_MAX:
            self.stats.dropped_stale += 1
            return None
        want = auth_payload(self._skey[sa], data[0], p.frame_id, p.data, fresh)
        diff = 0
        for a, b in zip(want[4:4 + MAC_BYTES], data[4:4 + MAC_BYTES]):
            diff |= a ^ b
        if diff:
            self.stats.dropped_bad_mac += 1
            return None
        self._last[key] = fresh
        self.stats.forwarded += 1
        return p.frame_id, p.data
