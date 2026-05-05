#!/usr/bin/env python3
"""Sweep traj_a_max and re-test the +π/2 move that slipped.

If lower a_max eliminates the slip, the failure is jerk-related
(infinite-jerk corners of the trapezoidal velocity profile cause
the inner velocity loop to saturate Vq momentarily and pole-slip).
"""
import math, socket, struct, sys, time

CAN_DEVICE_BASE = 0x008
CONTROLLER_BIT = 0x80
CMD_GET_MOTOR_PARAM = 0x20
CMD_SET_MOTOR_PARAM = 0x21
CMD_SET_MOTOR_COMMAND = 0x23
DEV_ID = 5
DEBUG_ANGLE_ACC = 100

PARAM_TRAJ_A_MAX = 17

MODE_IDLE = 0
MODE_POSITION = 3


def open_can():
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind(("can0",)); s.settimeout(0.1); return s


def send(s, cmd, p):
    d = bytes([cmd | CONTROLLER_BIT]) + p
    s.send(struct.pack("=IB3x8s", CAN_DEVICE_BASE + DEV_ID, len(d),
                       d + b"\x00" * (8 - len(d))))


def recv_reply(s, cmd, t=0.5):
    deadline = time.monotonic() + t
    while time.monotonic() < deadline:
        try:
            raw = s.recv(16)
        except socket.timeout:
            continue
        cid, dl, _, _, _, p = struct.unpack("=IBBBB8s", raw)
        cid &= 0x7FF
        if cid == CAN_DEVICE_BASE + DEV_ID and dl >= 1 and (p[0] & 0x7F) == cmd:
            return p[:dl]
    return None


def get_param(s, idx):
    send(s, CMD_GET_MOTOR_PARAM, bytes([idx]) + struct.pack("<f", 0.0))
    r = recv_reply(s, CMD_GET_MOTOR_PARAM)
    return struct.unpack("<f", r[2:6])[0] if r else float('nan')


def set_param(s, idx, val):
    send(s, CMD_SET_MOTOR_PARAM, bytes([idx]) + struct.pack("<f", val))
    recv_reply(s, CMD_SET_MOTOR_PARAM)


def set_mode(s, m, t):
    send(s, CMD_SET_MOTOR_COMMAND, bytes([m]) + struct.pack("<f", t))
    recv_reply(s, CMD_SET_MOTOR_COMMAND)


def settle(s, t=2.5, band=0.005, dur=0.3):
    deadline = time.monotonic() + t
    last = get_param(s, DEBUG_ANGLE_ACC); since = None
    while time.monotonic() < deadline:
        a = get_param(s, DEBUG_ANGLE_ACC)
        if math.isnan(a): continue
        if abs(a - last) < band:
            if since is None: since = time.monotonic()
            elif time.monotonic() - since >= dur: return a
        else: since = None
        last = a; time.sleep(0.02)
    return last


def main():
    s = open_can()
    a_orig = get_param(s, PARAM_TRAJ_A_MAX)
    print(f"[+] original traj_a_max = {a_orig}")
    print(f"{'a_max':>8}  {'+pi/2 err':>12}  {'-pi/2 err':>12}  {'+1.0 err':>12}  {'-1.0 err':>12}")
    print("-" * 64)

    sweeps = [50.0, 25.0, 10.0, 5.0, 2.0]
    for a_max in sweeps:
        set_param(s, PARAM_TRAJ_A_MAX, a_max)
        time.sleep(0.05)

        # Reset to a known reference
        set_mode(s, MODE_IDLE, 0.0); time.sleep(0.2)
        ref = get_param(s, DEBUG_ANGLE_ACC)
        set_mode(s, MODE_POSITION, ref); time.sleep(0.3)

        errs = []
        for delta in (+math.pi/2, -math.pi/2, +1.0, -1.0):
            target = ref + delta
            set_mode(s, MODE_POSITION, target)
            landed = settle(s)
            errs.append(landed - target)
            # come back to reference between tests
            set_mode(s, MODE_POSITION, ref)
            settle(s)

        e = [f"{x*1000:+8.2f} mr" for x in errs]
        print(f"{a_max:>8.1f}  {e[0]:>12}  {e[1]:>12}  {e[2]:>12}  {e[3]:>12}")

    # Restore.
    set_param(s, PARAM_TRAJ_A_MAX, a_orig)
    set_mode(s, MODE_IDLE, 0.0)


if __name__ == "__main__":
    main()
