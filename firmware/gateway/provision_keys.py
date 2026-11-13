"""Create the bus authentication keys for one machine and the sketch that stores them in the gateway.

    python firmware/gateway/provision_keys.py /path/outside/the/repository

Writes two files into that folder:

  kauha_bus_keys.json   the three sender keys (autonomy 0x80, remote gateway 0x82, zone source 0x86).
                        Copy it to each sender (ros2 can_bridge parameter ``key_file``); a sender
                        needs only its own entry.
  provision/provision.ino
                        flash it once to the gateway Teensy: it writes the keys to EEPROM and
                        blinks. Then flash the gateway firmware and delete this folder.

The keys never go into the repository: the script refuses a folder inside it. One set of keys per
machine. Generate new ones if a sender is replaced or a key file may have been copied.
"""
from __future__ import annotations

import json
import secrets
import sys
from pathlib import Path

SENDERS = (0x80, 0x82, 0x86)
REPO = Path(__file__).resolve().parents[2]


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    out = Path(sys.argv[1]).resolve()
    if out == REPO or REPO in out.parents:
        sys.exit("refusing to write keys inside the repository")
    keys = {sa: secrets.token_bytes(16) for sa in SENDERS}
    out.mkdir(parents=True, exist_ok=True)
    kf = out / "kauha_bus_keys.json"
    kf.write_text(json.dumps({f"0x{sa:02x}": k.hex() for sa, k in keys.items()}, indent=1) + "\n")
    kf.chmod(0o600)
    flat = ", ".join(f"0x{b:02x}" for sa in SENDERS for b in keys[sa])
    (out / "provision").mkdir(exist_ok=True)
    (out / "provision" / "provision.ino").write_text(
        "// One-time key provisioning for the KAUHA gateway. Flash, wait for the blinking LED, then\n"
        "// flash the gateway firmware. Delete this file afterwards.\n"
        "#include <EEPROM.h>\n"
        f"static const uint8_t KEYS[{len(SENDERS) * 16}] = {{{flat}}};\n"
        "void setup() {\n"
        "  for (unsigned i = 0; i < sizeof KEYS; i++) EEPROM.write(i, KEYS[i]);\n"
        "  EEPROM.write(sizeof KEYS, 0xA5);\n"
        "  pinMode(13, OUTPUT);\n"
        "}\n"
        "void loop() { digitalWrite(13, (millis() >> 8) & 1); }\n")
    print(f"wrote {kf} and {out / 'provision' / 'provision.ino'}")


if __name__ == "__main__":
    main()
