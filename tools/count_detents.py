#!/usr/bin/env python3
"""Hand-rotation detent counter.

Streams mechanical angle (debug param 100 = angle_acc, radians) from
the device and lets you press Enter each time you feel a detent.
At the end (Ctrl-C or one full revolution), prints the count and the
spacing between detents.

Usage:
    1. Put motor in idle so it spins freely:
           ./tools/motor_cli.py move idle 0
    2. Run this script:
           ./tools/count_detents.py
    3. Slowly rotate by hand through one full revolution.
       Press Enter (or any key + Enter) every time you feel a detent.
       Press 'q' + Enter to stop.
"""
from __future__ import annotations

import math
import select
import socket
import struct
import sys
import time

CAN_DEVICE_BASE = 0x008
CONTROLLER_BIT  = 0x80
CMD_GET_MOTOR_PARAM = 0x20
DEV_ID = 5
DEBUG_ANGLE_ACC = 100

IFACE = sys.argv[1] if len(sys.argv) > 1 else "can0"


def open_can(iface: str) -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(0.05)
    return s


def send(s: socket.socket, can_id: int, data: bytes) -> None:
    s.send(struct.pack("=IB3x8s", can_id, len(data),
                       data + b"\x00" * (8 - len(data))))


def poll_angle(s: socket.socket) -> float | None:
    payload = bytes([DEBUG_ANGLE_ACC]) + struct.pack("<f", 0.0)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_MOTOR_PARAM | CONTROLLER_BIT]) + payload)
    deadline = time.monotonic() + 0.1
    while time.monotonic() < deadline:
        try:
            raw = s.recv(16)
        except socket.timeout:
            return None
        cid, dlc, _, _, _, payload = struct.unpack("=IBBBB8s", raw)
        cid &= 0x7FF
        if cid == CAN_DEVICE_BASE + DEV_ID and dlc >= 6:
            data = payload[:dlc]
            if (data[0] & 0x7F) == CMD_GET_MOTOR_PARAM and data[1] == DEBUG_ANGLE_ACC:
                return struct.unpack("<f", data[2:6])[0]
    return None


def stdin_ready() -> bool:
    return select.select([sys.stdin], [], [], 0)[0] != []


def main() -> None:
    s = open_can(IFACE)
    print(f"[+] connected on {IFACE}, dev id {DEV_ID}")
    a0 = poll_angle(s)
    if a0 is None:
        print("no angle reply -- is device idle and on the bus?")
        sys.exit(1)
    print(f"[+] start angle = {a0:+.4f} rad")
    print("    rotate slowly; tap Enter at each detent.")
    print("    'q'+Enter to quit; auto-stops after >2π travel.\n")

    detents: list[float] = []
    last_print = 0.0
    last_angle = a0

    try:
        while True:
            a = poll_angle(s)
            if a is not None:
                last_angle = a
            now = time.monotonic()
            if now - last_print > 0.1:
                travel = last_angle - a0
                turns = travel / (2 * math.pi)
                sys.stdout.write(
                    f"\rangle={last_angle:+8.4f} rad   "
                    f"Δ={travel:+7.4f} rad ({turns:+.3f} rev)   "
                    f"detents={len(detents)}   "
                )
                sys.stdout.flush()
                last_print = now

            if stdin_ready():
                line = sys.stdin.readline().strip().lower()
                if line == "q":
                    break
                detents.append(last_angle)
                sys.stdout.write(f"\n  detent #{len(detents)} at {last_angle:+.4f} rad\n")

            if abs(last_angle - a0) >= 2 * math.pi + 0.05:
                sys.stdout.write("\n[+] reached one full revolution.\n")
                break
    except KeyboardInterrupt:
        sys.stdout.write("\n[!] interrupted\n")

    print("\n=== summary ===")
    travel = last_angle - a0
    print(f"total travel: {travel:+.4f} rad  ({travel / (2*math.pi):+.3f} rev)")
    print(f"detents marked: {len(detents)}")
    if len(detents) >= 2:
        deltas = [detents[i+1] - detents[i] for i in range(len(detents) - 1)]
        mean = sum(deltas) / len(deltas)
        print(f"detent spacing: mean={mean:+.4f} rad "
              f"({math.degrees(mean):+.2f} deg)")
        print(f"  min={min(deltas):+.4f}  max={max(deltas):+.4f}")
        if abs(travel) > 1e-3:
            est = abs(2 * math.pi / mean) if abs(mean) > 1e-6 else float('nan')
            print(f"implied detents per revolution: {est:.2f}")
    print()
    print("Pole-pair interpretation:")
    print("  • detents/rev == pole_pairs   →  cogging once per electrical cycle")
    print("  • detents/rev == 2*pole_pairs →  cogging twice per electrical cycle")
    print("  • detents/rev == LCM(slots, magnets) for slotted designs")


if __name__ == "__main__":
    main()
