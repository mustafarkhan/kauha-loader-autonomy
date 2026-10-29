"""Remote supervision station and a simple remote-operator model.

One supervisor watches the machine from a control room. The station sends a
heartbeat (with the remote e-stop) over the site radio. When the machine
needs help (a latched protective stop that requires a reset, or an assist
request from the autonomy computer) the operator responds after a human
reaction delay, checks the cameras, and either resets the stop or takes
over by teleoperation.

Teleoperation latency is modelled end to end (video capture/encode, radio
uplink and downlink, decode/display, command transmission) and every sample
is kept so the glass-to-glass p95 can be compared with the 150 ms budget.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from kauha.can.codec import Codec


@dataclass
class LatencyModel:
    rng: np.random.Generator
    capture_encode_ms: float = 28.0
    decode_display_ms: float = 22.0
    radio_median_ms: float = 18.0      # private LTE/5G one-way, lognormal
    radio_sigma: float = 0.45

    def video_ms(self) -> float:
        radio = self.radio_median_ms * math.exp(self.rng.normal(0, self.radio_sigma))
        return self.capture_encode_ms + radio + self.decode_display_ms + self.rng.uniform(0, 16.7)

    def command_ms(self) -> float:
        return self.radio_median_ms * math.exp(self.rng.normal(0, self.radio_sigma)) + 5.0


@dataclass
class Intervention:
    t_request: float
    t_response: float
    t_done: float
    kind: str
    reason: str


@dataclass
class RemoteStation:
    rng: np.random.Generator
    codec: Codec = field(default_factory=Codec)
    reaction_median_s: float = 12.0
    estop: bool = False
    mode_req: int = 1
    interventions: list[Intervention] = field(default_factory=list)
    latency_samples_ms: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.lat = LatencyModel(self.rng)
        self._pending: tuple[float, str, str, float] | None = None   # (t_req, kind, reason, t_act)
        self._reset_pulse_until = -1.0
        self._last_reset_t = -1e9
        self.teleop_active = False
        self.teleop_reason = ""
        self._tele_t0 = 0.0
        self._cmd_queue: deque = deque()

    def heartbeat(self, t: float, link_ok: bool) -> list[tuple[int, bytes]]:
        if not link_ok:
            return []
        return [self.codec.encode("RemoteHeartbeat", {
            "EStop": int(self.estop), "ResetReq": int(t < self._reset_pulse_until), "ModeReq": self.mode_req})]

    def request(self, t: float, kind: str, reason: str) -> None:
        if t - self._last_reset_t < 2.0:       # the last reset is still propagating
            return
        if self._pending is None and not self.teleop_active:
            delay = self.reaction_median_s * math.exp(self.rng.normal(0, 0.4))
            self._pending = (t, kind, reason, t + delay)

    @property
    def busy(self) -> bool:
        return self._pending is not None or self.teleop_active

    def update(self, t: float, link_ok: bool) -> str | None:
        """Returns an action for the twin: 'reset' | 'teleop_start' | None."""
        if self._pending is None or t < self._pending[3] or not link_ok:
            return None
        t_req, kind, reason, _ = self._pending
        self._pending = None
        if kind == "reset":
            self._reset_pulse_until = t + 0.2
            self._last_reset_t = t
            self.interventions.append(Intervention(t_req, t, t + 0.2, "reset", reason))
            return "reset"
        self.teleop_active, self.teleop_reason, self._tele_t0 = True, reason, t
        self.mode_req = 2
        self._tele_req_t = t_req
        return "teleop_start"

    def end_teleop(self, t: float) -> None:
        self.interventions.append(Intervention(self._tele_req_t, self._tele_t0, t, "teleop", self.teleop_reason))
        self.teleop_active = False
        self.mode_req = 1

    def delay_command(self, t: float, cmd) -> object | None:
        """Pass an operator command through the latency model; return the one due now."""
        g2g = self.lat.video_ms() + self.lat.command_ms()
        self.latency_samples_ms.append(g2g)
        self._cmd_queue.append((t + g2g / 1000.0, cmd))
        due = None
        while self._cmd_queue and self._cmd_queue[0][0] <= t:
            due = self._cmd_queue.popleft()[1]
        return due
