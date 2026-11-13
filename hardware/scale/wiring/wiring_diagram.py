"""Wiring diagram of the 1:8 testbed (PDF): power and stop chain, CAN bus and signals.

    python -m hardware.scale.wiring.wiring_diagram

docs/scale/wiring.md is the authoritative wire list; this drawing shows the same
connections at a glance. Reference designators match the BOM and the doc.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

OUT = Path(__file__).resolve().parent / "kauha_scale8_wiring.pdf"
RED, BLUE, GREY, BLACK, AMBER = "#c53a2a", "#1f5fbf", "#5d636b", "#111111", "#c98f0a"


def block(ax, x, y, w, h, title, lines=(), color=BLACK):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.004,rounding_size=0.008", fc="white",
                                ec=color, lw=1.4))
    ax.text(x + 0.008, y + h - 0.012, title, fontsize=8.5, weight="bold", va="top", color=color)
    for i, ln in enumerate(lines):
        ax.text(x + 0.008, y + h - 0.034 - i * 0.019, ln, fontsize=6.6, va="top", family="monospace")


def wire(ax, pts, color=BLACK, label=None, lw=1.3, at=0.5, dy=0.008):
    xs, ys = zip(*pts, strict=True)
    ax.plot(xs, ys, color=color, lw=lw, solid_capstyle="round")
    if label:
        i = max(0, min(len(pts) - 2, int(at * (len(pts) - 1))))
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + dy, label, fontsize=6.2, color=color, ha="center", va="bottom")


def page_power(pdf):
    fig = plt.figure(figsize=(16.54, 11.69))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(0.02, 0.975, "KAUHA 1:8 testbed - sheet 1: power, stop chain and hardwired outputs", fontsize=14,
            weight="bold", va="top")
    ax.text(0.02, 0.95, "Red: safety chain (24 V). Amber: 36 V traction. Black: logic. Every stop path de-energises: "
            "a broken wire, a dropped relay or a dead PLC all stop the machine.", fontsize=8.5, va="top")
    block(ax, 0.02, 0.70, 0.13, 0.17, "BAT  36 V 10S", ["Li-ion 10 Ah, BMS", "F1 30 A blade fuse", "S1 main switch",
                                                       "XT60"], AMBER)
    block(ax, 0.20, 0.74, 0.14, 0.13, "PSU  DC-DC", ["U1 36->24 V 10 A", "U2 36->12 V 10 A", "U3 24->5 V 5 A",
                                                    "F2..F4 polyfuses"])
    wire(ax, [(0.15, 0.80), (0.20, 0.80)], AMBER, "36 V")
    block(ax, 0.40, 0.70, 0.20, 0.20, "K1  PNOZ s4 safety relay", ["A1/A2 24 V", "S11-S12 / S21-S22: e-stop chain",
                                                                   "S33-S34: reset (monitored)",
                                                                   "Y1-Y2 feedback: K2..K6 NC mirrors",
                                                                   "13-14: K-DRIVE path", "23-24: K-IMPL path",
                                                                   "33-34: 24 V to PLC IN1 = chain OK"], RED)
    block(ax, 0.40, 0.42, 0.20, 0.20, "E-stop chain (2 channels)",
          ["SB1 e-stop cab left   2 NC", "SB2 e-stop cab right  2 NC", "SB3 e-stop at the bench 2 NC",
           "KR radio e-stop relay  2 NC", "  (fail-safe: opens on", "   link loss or battery)", "SB4 reset (NO)"], RED)
    wire(ax, [(0.50, 0.62), (0.50, 0.70)], RED, "ch1 + ch2")
    block(ax, 0.66, 0.70, 0.14, 0.20, "PLC relay board", ["KP1 DRIVE_ENABLE (GPIO23)", "KP2 IMPL_ENABLE  (GPIO24)",
                                                          "5 V coils, driven by", "PRG_Main outputs", "",
                                                          "contacts in series", "after K1 13-14 / 23-24"])
    wire(ax, [(0.60, 0.83), (0.66, 0.83)], RED, "13-14")
    wire(ax, [(0.60, 0.76), (0.66, 0.76)], RED, "23-24")
    block(ax, 0.85, 0.66, 0.13, 0.26, "K-DRIVE  K2..K5", ["Finder 60.13, 3 CO,", "24 V coil, one per wheel",
                                                          "NO: ODrive phase -> motor", "NC: motor phases joined",
                                                          "    via 3 x 0.47 ohm 10 W", "    = dynamic brake",
                                                          "coil off = braking", "NC mirror -> K1 Y1-Y2",
                                                          "and PLC IN2"], RED)
    wire(ax, [(0.80, 0.83), (0.85, 0.83)], RED, "KP1")
    block(ax, 0.85, 0.44, 0.13, 0.17, "K-IMPL  K6",
          ["Finder 60.12, 2 CO,", "24 V coil", "switches 12 V to the", "three MDD10A boards", "coil off = actuators",
           "unpowered (self-locking)"], RED)
    wire(ax, [(0.80, 0.76), (0.82, 0.76), (0.82, 0.55), (0.85, 0.55)], RED, "KP2", at=0.9)
    block(ax, 0.62, 0.20, 0.16, 0.16, "ODrive x2 (56 V)", ["DC in 36 V via F5 40 A", "M0/M1 phases -> K2..K5 NO",
                                                         "Halls -> J4/J5", "CAN2 to the gateway",
                                                         "brake resistor fitted"], AMBER)
    block(ax, 0.82, 0.20, 0.16, 0.16, "Hub motors x4", ["6.5 in, 36 V, 350 W", "3 phases via K2..K5",
                                                       "Halls: ODrive (FL also", "to the sensor node via",
                                                       "a 74HC14 buffer)"], AMBER)
    wire(ax, [(0.78, 0.28), (0.82, 0.28)], AMBER, "phases via K-DRIVE")
    wire(ax, [(0.91, 0.66), (0.91, 0.36)], RED, "", lw=0.8)
    block(ax, 0.20, 0.42, 0.16, 0.20, "Supplies", ["24 V: K1, relays K2..K6, scanners,", "     PLC inputs (opto)",
                                                   "12 V: actuators via K6", "5 V: Pi PLC, Teensys (own LDOs)",
                                                   "Sensor node on its own", "  5 V regulator from 24 V"])
    wire(ax, [(0.27, 0.74), (0.27, 0.62)], BLACK, "24 / 12 / 5 V")
    wire(ax, [(0.36, 0.52), (0.40, 0.52)], RED, "24 V")
    block(ax, 0.02, 0.20, 0.28, 0.16, "Rules this sheet follows", [
        "1. Stop = de-energise. No stop needs a working CAN bus,",
        "   a working Pi or a working gateway.",
        "2. Two channels from every e-stop to K1 (cat. 3 structure).",
        "3. K1 monitors K2..K6 through their NC mirror contacts:",
        "   a welded contact blocks the next start.",
        "4. The Pi PLC is NOT a safety controller: on the testbed",
        "   it repeats the stop decision (KP1/KP2 in series).",
    ], GREY)
    fig.text(0.02, 0.02, "KAUHA 1:8 testbed | wiring sheet 1 | see docs/scale/wiring.md for the wire list", fontsize=8)
    pdf.savefig(fig)
    plt.close(fig)


def page_can(pdf):
    fig = plt.figure(figsize=(16.54, 11.69))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(0.02, 0.975, "KAUHA 1:8 testbed - sheet 2: CAN bus and signals", fontsize=14, weight="bold", va="top")
    ax.text(0.02, 0.95, "Machine bus: 500 kbit/s, 29-bit J1939 identifiers, hardware/can/kauha_machine.dbc. Two "
            "separate twisted pairs, each with 120 ohm at both ends. The gateway's segment guard is the only "
            "connection between them. Motor bus (CAN2) is separate.", fontsize=8.5, va="top")
    y = 0.62
    open_col = "#b45309"
    for x0, x1, col, label in ((0.03, 0.585, BLUE, "CAN_H / CAN_L  safety segment (inside the electronics tray)"),
                               (0.615, 0.97, open_col, "CAN_H / CAN_L  open segment")):
        wire(ax, [(x0, y), (x1, y)], col, None, lw=3)
        wire(ax, [(x0, y - 0.012), (x1, y - 0.012)], col, None, lw=3)
        ax.text(x0, y + 0.012, label, fontsize=8, color=col)
        for x in (x0, x1):
            ax.add_patch(plt.Rectangle((x - 0.006, y - 0.026), 0.012, 0.04, fc="white", ec=col, zorder=3))
            ax.text(x, y - 0.042, "120R", fontsize=7, ha="center", color=col)
    nodes = [
        (0.05, "Safety PLC 0x81", ["Pi 4 + CAN HAT (can0)", "CODESYS PRG_Main 25 ms", "tx SafetyStatus 40 Hz",
                                   "rx all frames below", "GPIO: IN1..IN8, OUT1..OUT9"], RED, BLUE, 0.05),
        (0.21, "Sensor node 0x84", ["Teensy 4.0", "AS5600 hinge (Wire)", "AS5600 boom (Wire1)",
                                    "FL Halls via 74HC14", "tx SafeSensors 40 Hz"], RED, BLUE, 0.05),
        (0.37, "Field evaluator 0x85", ["evaluator profile only:", "Pi + 2 x LD19 lidar",
                                        "kauha/scale/field_eval.py", "tx ScannerStatus 40 Hz",
                                        "(rated scanners: wired to", " PLC IN3..IN8 instead)"], GREY, BLUE, 0.05),
        (0.53, "Gateway 0x83 + segment guard", ["Teensy 4.1: CAN1 safety, CAN3 open", "copies safety -> open",
                                                "open -> safety: 4 command frames,", "  only with a valid FrameAuth",
                                                "tx MachineState1/2, AuthChallenge", "CAN2 -> ODrives, PWM -> MDD10A"],
         BLACK, BLUE, 0.02),
        (0.69, "Autonomy 0x80", ["Pi 5 / Jetson, ROS 2", "kauha_ros can_bridge (key_file)",
                                 "tx DriveCmd, ImplementCmd", "  each followed by FrameAuth",
                                 "rx SafetyStatus, MachineState"], BLUE, open_col, 0.05),
        (0.845, "Remote 0x82 / zones 0x86", ["supervisor laptop:", "tx RemoteHeartbeat 20 Hz", "tx ZoneStatus 10 Hz",
                                            "  each followed by FrameAuth", "(console + zone switches)"],
         BLUE, open_col, 0.05),
    ]
    for x, title, lines, col, bus_col, tap in nodes:
        block(ax, x - 0.02, 0.30, 0.15, 0.24, title, lines, col)
        wire(ax, [(x + tap, 0.54), (x + tap, y - 0.012)], bus_col, None)
    wire(ax, [(0.635, 0.54), (0.635, y - 0.012)], open_col, None)       # the gateway's second port
    block(ax, 0.02, 0.70, 0.30, 0.18, "Frame timing and timeouts (PLC side)", [
        "RemoteHeartbeat 20 Hz   timeout 0.5 s  -> REMOTE_LINK",
        "DriveCmd        20 Hz   timeout 0.2 s  -> WATCHDOG",
        "MachineState1   40 Hz   stale 0.1 s    -> SPEED_DISCREPANCY",
        "SafeSensors     40 Hz   stale 0.1 s    -> SCANNER_FAULT",
        "ScannerStatus   40 Hz   stale 0.1 s    -> SCANNER_FAULT",
        "ZoneStatus      10 Hz   stale 0.5 s    -> POSITION_INVALID (2 s)",
    ], BLACK)
    block(ax, 0.36, 0.70, 0.30, 0.18, "Every safety-relevant frame", [
        "byte 6 low nibble: alive counter (+1 per frame)",
        "byte 7: CRC-8 SAE J1850 over data ID + bytes 0..6",
        "data ID = PGN low byte (unique per message)",
        "checked in FB_E2ERx (PLC) and kauha_e2e.c (firmware)",
        "both tested bit for bit against kauha/can/e2e.py",
        "open segment: + FrameAuth (AES-128 CMAC, 32 bit,",
        "  24-bit freshness), checked by the segment guard",
    ], BLACK)
    block(ax, 0.70, 0.70, 0.28, 0.18, "Connectors", [
        "CAN: 4-pin M8 or JST-XH: CAN_H, CAN_L, GND, shield",
        "Stub length < 0.3 m at 500 kbit/s",
        "E-stop and relay wiring: 0.5 mm2, ferrules,",
        "  two colours for the two channels",
        "Traction: 2.5 mm2 silicone, XT60/XT90",
    ], BLACK)
    fig.text(0.02, 0.02, "KAUHA 1:8 testbed | wiring sheet 2", fontsize=8)
    pdf.savefig(fig)
    plt.close(fig)


def main() -> None:
    with PdfPages(OUT) as pdf:
        page_power(pdf)
        page_can(pdf)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
