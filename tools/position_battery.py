#!/usr/bin/env python3
"""Position-move accuracy battery.

Runs a sequence of position moves and reports landing error for each.
Uses abs-shortest mode (auto-wraps target into shortest path) for the
short tests, and raw position mode for the multi-turn tests.

Usage:
    ./tools/position_battery.py
"""
from __future__ import annotations

import math
import socket
import struct
import sys
import time

CAN_DEVICE_BASE = 0x008
CONTROLLER_BIT = 0x80
CMD_GET_MOTOR_PARAM = 0x20
CMD_SET_MOTOR_COMMAND = 0x23
DEV_ID = 5
DEBUG_ANGLE_ACC = 100
DEBUG_TARGET = 102
DEBUG_TRAJ_SETPOINT = 109

MODE_IDLE = 0
MODE_POSITION = 3
MODE_ABS_SHORTEST = 5


def open_can(iface: str = "can0") -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(0.1)
    return s


def send(s: socket.socket, cmd: int, payload: bytes) -> None:
    data = bytes([cmd | CONTROLLER_BIT]) + payload
    s.send(struct.pack("=IB3x8s", CAN_DEVICE_BASE + DEV_ID,
                       len(data), data + b"\x00" * (8 - len(data))))


def recv_reply(s: socket.socket, cmd: int, timeout: float = 0.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            raw = s.recv(16)
        except socket.timeout:
            continue
        cid, dlc, _, _, _, payload = struct.unpack("=IBBBB8s", raw)
        cid &= 0x7FF
        if cid == CAN_DEVICE_BASE + DEV_ID and dlc >= 1:
            d = payload[:dlc]
            if (d[0] & 0x7F) == cmd:
                return d
    return None


def get_param(s, idx: int) -> float:
    send(s, CMD_GET_MOTOR_PARAM, bytes([idx]) + struct.pack("<f", 0.0))
    r = recv_reply(s, CMD_GET_MOTOR_PARAM)
    if r is None:
        return float('nan')
    return struct.unpack("<f", r[2:6])[0]


def set_mode(s, mode: int, target: float) -> None:
    send(s, CMD_SET_MOTOR_COMMAND, bytes([mode]) + struct.pack("<f", target))
    recv_reply(s, CMD_SET_MOTOR_COMMAND)


def wait_settle(s, timeout: float = 3.0, settle_band: float = 0.005,
                settle_dur: float = 0.3) -> float:
    """Poll angle_acc until it stays within `settle_band` for `settle_dur`."""
    deadline = time.monotonic() + timeout
    last_a = get_param(s, DEBUG_ANGLE_ACC)
    settled_since = None
    while time.monotonic() < deadline:
        a = get_param(s, DEBUG_ANGLE_ACC)
        if math.isnan(a):
            continue
        if abs(a - last_a) < settle_band:
            if settled_since is None:
                settled_since = time.monotonic()
            elif time.monotonic() - settled_since >= settle_dur:
                return a
        else:
            settled_since = None
        last_a = a
        time.sleep(0.02)
    return last_a


def wrap_pi(x: float) -> float:
    while x > math.pi:  x -= 2 * math.pi
    while x <= -math.pi: x += 2 * math.pi
    return x


def main() -> None:
    s = open_can()

    # Park briefly in idle to read starting angle.
    set_mode(s, MODE_IDLE, 0.0)
    time.sleep(0.3)
    a0 = get_param(s, DEBUG_ANGLE_ACC)
    print(f"[+] starting angle = {a0:+.4f} rad")

    # Snap to a clean reference: command position = current angle, hold.
    set_mode(s, MODE_POSITION, a0)
    time.sleep(0.3)
    a_ref = get_param(s, DEBUG_ANGLE_ACC)
    print(f"[+] holding at {a_ref:+.4f} rad as reference\n")

    # Tests: (name, mode, target) where target is interpreted by the mode.
    # For abs-shortest, target is in (-π, π]; we'll express it as relative
    # to a_ref by wrapping for clarity.
    tests = [
        ("shortest +π/4", MODE_ABS_SHORTEST, wrap_pi(a_ref + math.pi/4)),
        ("shortest -π/4", MODE_ABS_SHORTEST, wrap_pi(a_ref - math.pi/4)),
        ("shortest +π/2", MODE_ABS_SHORTEST, wrap_pi(a_ref + math.pi/2)),
        ("shortest -π/2", MODE_ABS_SHORTEST, wrap_pi(a_ref - math.pi/2)),
        ("shortest +1.0", MODE_ABS_SHORTEST, wrap_pi(a_ref + 1.0)),
        ("shortest -1.0", MODE_ABS_SHORTEST, wrap_pi(a_ref - 1.0)),
        # Stay short to avoid the known pole-slip issue on long moves.
        ("position +0.5", MODE_POSITION, a_ref + 0.5),
        ("position -0.5", MODE_POSITION, a_ref - 0.5),
        ("position back to ref", MODE_POSITION, a_ref),
    ]

    results = []
    for name, mode, target in tests:
        before = get_param(s, DEBUG_ANGLE_ACC)
        set_mode(s, mode, target)
        landed = wait_settle(s)
        # Expected absolute landing:
        if mode == MODE_ABS_SHORTEST:
            # Landing should be at (some integer turn k) such that
            # landed mod 2π == target (with wrap_pi). Compute error
            # by wrapping the difference.
            err = wrap_pi(landed - target)
        else:
            err = landed - target
        print(f"  {name:<22}  before={before:+7.4f}  landed={landed:+7.4f}  "
              f"target={target:+7.4f}  err={err*1000:+7.2f} mrad")
        results.append((name, err))
        time.sleep(0.2)

    set_mode(s, MODE_IDLE, 0.0)

    print("\n=== summary ===")
    n_ok = sum(1 for _, e in results if abs(e) < 0.005)
    n_warn = sum(1 for _, e in results if 0.005 <= abs(e) < 0.020)
    n_bad = sum(1 for _, e in results if abs(e) >= 0.020)
    print(f"  <  5 mrad: {n_ok}/{len(results)}")
    print(f"  5–20 mrad: {n_warn}")
    print(f"  ≥ 20 mrad: {n_bad}")
    worst = max(results, key=lambda r: abs(r[1]))
    print(f"  worst:    {worst[0]:<22}  err={worst[1]*1000:+7.2f} mrad")


if __name__ == "__main__":
    main()
