"""Shared plumbing for the SIL processes: CAN bus, hardwired-I/O link, pacing."""
from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass

import can

CYCLE_S = 0.025


@dataclass(frozen=True)
class Wiring:
    can_group: str          # multicast group = one CAN "cable"
    io_to_plc: int          # UDP port: machine -> PLC (scanner OSSDs, speed, articulation, e-stop)
    io_to_machine: int      # UDP port: PLC -> machine (brake/neutral relays, pilot cut-off)

    def bus(self) -> can.BusABC:
        return can.Bus(interface="udp_multicast", channel=self.can_group, receive_own_messages=False)


class IOLink:
    """Point-to-point datagrams standing in for hardwired 24 V signals."""

    def __init__(self, listen_port: int, send_port: int):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", listen_port))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_port = send_port
        self.last: dict | None = None
        self.last_t: float | None = None

    def send(self, msg: dict) -> None:
        self.tx.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.send_port))

    def poll(self) -> dict | None:
        while True:
            try:
                data, _ = self.rx.recvfrom(65536)
            except BlockingIOError:
                break
            self.last = json.loads(data)
            self.last_t = time.monotonic()
        return self.last

    def age(self) -> float:
        return 1e9 if self.last_t is None else time.monotonic() - self.last_t


class Pacer:
    """Fixed-rate loop on the wall clock; reports overruns."""

    def __init__(self, period: float):
        self.period = period
        self.t0 = time.monotonic()
        self.k = 0
        self.overruns = 0

    def wait(self) -> None:
        self.k += 1
        target = self.t0 + self.k * self.period
        now = time.monotonic()
        if now < target:
            time.sleep(target - now)
        elif now - target > self.period:
            self.overruns += 1

    @property
    def t(self) -> float:
        return time.monotonic() - self.t0


def drain(bus: can.BusABC) -> list[can.Message]:
    out = []
    while (m := bus.recv(timeout=0.0)) is not None:
        out.append(m)
    return out


def wait_for(epoch: float) -> None:
    """Block until the shared start time, so every process begins on the same tick."""
    while time.monotonic() < epoch:
        time.sleep(0.002)
