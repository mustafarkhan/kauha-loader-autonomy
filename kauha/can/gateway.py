"""Drive-by-wire gateway: the only path from the autonomy computer to the machine.

It decodes DriveCmd/ImplementCmd frames, checks end-to-end protection and
freshness, applies the speed limit published by the safety PLC and rate
limits, and produces the actuator command for the OEM machine bus. It is a
functional (non-safety) arbiter: the safety PLC's hardwired brake and pilot
cut-off act on the machine independently of anything decided here.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from kauha.can.codec import Codec
from kauha.can.e2e import E2EStatus
from kauha.sim.loader import LoaderCommand

CMD_TIMEOUT_S = 0.15
SAFETY_TIMEOUT_S = 0.1


@dataclass
class VirtualBus:
    """In-process CAN bus for the twin (python-can is used in the HIL tests)."""
    frames: list[tuple[float, int, bytes]] = field(default_factory=list)
    count: int = 0

    def send(self, t: float, frame_id: int, data: bytes) -> None:
        self.frames.append((t, frame_id, data))
        self.count += 1

    def drain(self) -> list[tuple[float, int, bytes]]:
        out, self.frames = self.frames, []
        return out


@dataclass
class GatewayStats:
    e2e_errors: int = 0
    timeouts: int = 0
    clamped: int = 0


class MachineGateway:
    def __init__(self, codec: Codec | None = None):
        self.codec = codec or Codec()
        self.rx_drive = self.codec.receiver("DriveCmd", CMD_TIMEOUT_S)
        self.rx_impl = self.codec.receiver("ImplementCmd", CMD_TIMEOUT_S)
        self.rx_safety = self.codec.receiver("SafetyStatus", SAFETY_TIMEOUT_S)
        self.drive: dict = {}
        self.impl: dict = {}
        self.safety: dict = {}
        self.stats = GatewayStats()
        self._timed_out = False

    def receive(self, t: float, frame_id: int, data: bytes) -> None:
        try:
            name, sig = self.codec.decode(frame_id, data)
        except KeyError:
            return
        rx = {"DriveCmd": self.rx_drive, "ImplementCmd": self.rx_impl, "SafetyStatus": self.rx_safety}.get(name)
        if rx is None:
            return
        st = rx.check(data, t)
        if st not in (E2EStatus.OK, E2EStatus.OK_SOME_LOST):
            self.stats.e2e_errors += 1
            return
        {"DriveCmd": self.drive, "ImplementCmd": self.impl, "SafetyStatus": self.safety}[name].update(sig)

    def command(self, t: float) -> LoaderCommand:
        fresh = self.rx_drive.fresh(t) and self.rx_impl.fresh(t) and self.rx_safety.fresh(t)
        if not fresh:
            if not self._timed_out:
                self.stats.timeouts += 1
            self._timed_out = True
            return LoaderCommand(brake=True)
        self._timed_out = False
        d, im, s = self.drive, self.impl, self.safety
        if d.get("Mode", 0) == 0 or s.get("State", 3) >= 2:
            return LoaderCommand(brake=True, articulation=d.get("ArticulationReq", 0.0))
        hold = im.get("Hold", 1)
        boom = 0.0 if hold else im.get("BoomRate", 0.0)
        bucket = 0.0 if hold else im.get("BucketRate", 0.0)
        if d.get("BrakeReq", 1):
            # Parked on the service brake; the implement may still work (dumping).
            return LoaderCommand(brake=True, articulation=d.get("ArticulationReq", 0.0), boom=boom, bucket=bucket)
        limit = s.get("SpeedLimit", 0.0)
        v = d.get("SpeedReq", 0.0)
        if abs(v) > limit:
            self.stats.clamped += 1
            v = max(-limit, min(limit, v))
        return LoaderCommand(speed=v, articulation=d.get("ArticulationReq", 0.0),
                             boom=boom, bucket=bucket, brake=False)
