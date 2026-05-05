#!/usr/bin/env python3
"""Measure pole-pair ratio via open-loop electrical sweep.

In OpenLoop mode the firmware applies (Vd=vlim, Vq=0) at a rotating
electrical angle theta_cmd advancing at `target` rad/s. The rotor
locks onto and follows that field, so:

    mech_travel  =  (target * T) / pole_pairs

i.e. pp = elec_travel / mech_travel. We sweep at a slow electrical
rate, sample angle_acc (debug 100) before/after, and report the ratio.

Usage:
    ./tools/measure_pp.py            # uses defaults: 6 rad/s elec, 4 s
    ./tools/measure_pp.py 4 6        # 4 rad/s elec, 6 s
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
CMD_SET_MOTOR_PARAM = 0x21
CMD_SET_MOTOR_COMMAND = 0x23
DEV_ID = 5
DEBUG_ANGLE_ACC = 100
PARAM_VOLTAGE_LIMIT = 4

MODE_IDLE = 0
MODE_OPENLOOP = 4

ELEC_RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0   # rad/s electrical
DURATION  = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0   # seconds
VOLTAGE   = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0   # Vd during sweep


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
        raise RuntimeError("no reply to GetMotorParam")
    return struct.unpack("<f", r[2:6])[0]


def set_param(s, idx: int, val: float) -> None:
    send(s, CMD_SET_MOTOR_PARAM, bytes([idx]) + struct.pack("<f", val))
    recv_reply(s, CMD_SET_MOTOR_PARAM)


def set_mode(s, mode: int, target: float) -> None:
    send(s, CMD_SET_MOTOR_COMMAND, bytes([mode]) + struct.pack("<f", target))
    recv_reply(s, CMD_SET_MOTOR_COMMAND)


def main() -> None:
    s = open_can()
    print(f"[+] elec rate = {ELEC_RATE} rad/s, duration = {DURATION} s, "
          f"Vd = {VOLTAGE} V")

    # Ensure voltage_limit is high enough.
    vlim = get_param(s, PARAM_VOLTAGE_LIMIT)
    if vlim < VOLTAGE:
        print(f"[+] bumping voltage_limit {vlim} -> {VOLTAGE}")
        set_param(s, PARAM_VOLTAGE_LIMIT, VOLTAGE)

    # Park briefly so the rotor settles before reading start angle.
    print("[+] parking at theta_cmd=0 for 0.5 s ...")
    set_mode(s, MODE_OPENLOOP, 0.0)
    time.sleep(0.5)

    a0 = get_param(s, DEBUG_ANGLE_ACC)
    print(f"[+] start angle = {a0:+.4f} rad")

    print(f"[+] sweeping ...")
    set_mode(s, MODE_OPENLOOP, ELEC_RATE)
    t_start = time.monotonic()
    time.sleep(DURATION)
    set_mode(s, MODE_OPENLOOP, 0.0)
    t_elapsed = time.monotonic() - t_start
    time.sleep(0.3)  # let rotor settle in final park

    a1 = get_param(s, DEBUG_ANGLE_ACC)
    set_mode(s, MODE_IDLE, 0.0)

    elec_travel = ELEC_RATE * t_elapsed
    mech_travel = a1 - a0
    print(f"[+] end angle   = {a1:+.4f} rad")
    print(f"    elapsed     = {t_elapsed:.3f} s")
    print(f"    elec_travel = {elec_travel:+.3f} rad ({elec_travel/(2*math.pi):+.3f} elec rev)")
    print(f"    mech_travel = {mech_travel:+.4f} rad ({mech_travel/(2*math.pi):+.4f} mech rev)")
    if abs(mech_travel) < 1e-3:
        print("    ! rotor did not move -- increase voltage or rate")
        return
    pp = elec_travel / mech_travel
    print(f"\n  ==> pole_pairs = {pp:+.3f}")
    print(f"      |pp|         = {abs(pp):.3f}")
    print(f"      sign        = {'+1 (encoder agrees with field rotation)' if pp > 0 else '-1 (encoder dir reversed)'}")


if __name__ == "__main__":
    main()
