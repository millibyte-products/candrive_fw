#!/usr/bin/env python3
"""candrive motor parameter / calibration CLI.

Quick-and-dirty host-side helper for poking the new MotorParam +
RunCalibration commands. Assumes a previously discovered device at
ASSIGN_ID = 5 on can0 (matches fw_update.py).

Examples
--------
    ./tools/motor_cli.py get-param 0           # read pole_pairs
    ./tools/motor_cli.py set-param 0 11.0      # write pole_pairs
    ./tools/motor_cli.py calibrate 10 4000 25  # 1.0 Hz, 4 s, 25% Vbus
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

CAN_DEVICE_BASE = 0x008
CONTROLLER_BIT  = 0x80

CMD_GET_MOTOR_PARAM = 0x20
CMD_SET_MOTOR_PARAM = 0x21
CMD_RUN_CALIBRATION = 0x22
CMD_SET_MOTOR_COMMAND = 0x23
CMD_SAVE_MOTOR_PARAMS = 0x24

CMD_GET_LED = 0x09
CMD_SET_LED = 0x0A

CMD_GET_POSITION = 0x03

# `update_flag` bits in the SetLed payload.
LED_UPDATE_STAT = 0x01
LED_UPDATE_SYS  = 0x02

DEV_ID = 5


def open_can(iface: str) -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(0.1)
    return s


def send(s: socket.socket, can_id: int, data: bytes) -> None:
    assert len(data) <= 8
    s.send(struct.pack("=IB3x8s", can_id, len(data), data + b"\x00" * (8 - len(data))))


def recv(s: socket.socket, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            raw = s.recv(16)
        except socket.timeout:
            continue
        can_id, dlc, _, _, _, payload = struct.unpack("=IBBBB8s", raw)
        return can_id & 0x7FF, payload[:dlc]
    return None


def expect_reply(s: socket.socket, cmd: int, timeout: float = 2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        f = recv(s, deadline - time.monotonic())
        if f is None:
            continue
        cid, data = f
        if cid == CAN_DEVICE_BASE + DEV_ID and len(data) >= 1:
            if (data[0] & 0x7F) == cmd:
                return data
    return None


def cmd_get_param(s: socket.socket, idx: int) -> None:
    payload = bytes([idx]) + struct.pack("<f", 0.0)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_MOTOR_PARAM | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_GET_MOTOR_PARAM)
    if r is None:
        print("timeout"); sys.exit(1)
    idx_r = r[1]
    val = struct.unpack("<f", r[2:6])[0]
    print(f"param[{idx_r}] = {val}")


def cmd_set_param(s: socket.socket, idx: int, value: float) -> None:
    payload = bytes([idx]) + struct.pack("<f", value)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_SET_MOTOR_PARAM | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_SET_MOTOR_PARAM)
    if r is None:
        print("timeout"); sys.exit(1)
    idx_r = r[1]
    val = struct.unpack("<f", r[2:6])[0]
    print(f"stored param[{idx_r}] = {val}")


def cmd_calibrate(s: socket.socket, freq_dhz: int, dur_ms: int, valign_pct: int) -> None:
    payload = struct.pack("<HHB", freq_dhz, dur_ms, valign_pct)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_RUN_CALIBRATION | CONTROLLER_BIT]) + payload)
    # Calibration blocks the device for ~(warmup + 2*sweep) ≈
    # max(dur/2,300) + 2*max(dur,800) ms ≈ 2.5 * dur_ms in the worst case.
    timeout = 3.0 + 3.0 * (dur_ms / 1000.0)
    r = expect_reply(s, CMD_RUN_CALIBRATION, timeout=timeout)
    if r is None:
        print("timeout"); sys.exit(1)
    pp = struct.unpack("<f", r[1:5])[0]
    zero = struct.unpack("<H", r[5:7])[0]
    flags = r[7]
    direction = -1 if (flags & 0x02) else +1
    fault = bool(flags & 0x01)
    print(f"pole_pairs={pp:.3f} zero_offset={zero} direction={direction:+d} "
          f"fault={fault} flags=0x{flags:02x}")


MODE_NAMES = {
    "idle": 0,
    "voltage": 1,
    "velocity": 2,
    "position": 3,
    "openloop": 4,
    "abs-shortest": 5,
    "abs-forward":  6,
    "abs-backward": 7,
    "relative":     8,
}


def cmd_save_params(s: socket.socket) -> None:
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_SAVE_MOTOR_PARAMS | CONTROLLER_BIT]))
    r = expect_reply(s, CMD_SAVE_MOTOR_PARAMS)
    if r is None:
        print("timeout"); sys.exit(1)
    print("saved")


def cmd_move(s: socket.socket, mode: str, target: float) -> None:
    if mode not in MODE_NAMES:
        print(f"unknown mode {mode!r}; want one of {list(MODE_NAMES)}"); sys.exit(2)
    code = MODE_NAMES[mode]
    payload = bytes([code]) + struct.pack("<f", target)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_SET_MOTOR_COMMAND | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_SET_MOTOR_COMMAND)
    if r is None:
        print("timeout"); sys.exit(1)
    mode_r = r[1]
    target_r = struct.unpack("<f", r[2:6])[0]
    print(f"mode={mode_r} target={target_r}")


def cmd_get_led(s: socket.socket) -> None:
    # GetLed / SetLed share the 3-byte Led payload format on the wire;
    # firmware decoder requires the full 3 bytes even on a query.
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_LED | CONTROLLER_BIT, 0, 0, 0]))
    r = expect_reply(s, CMD_GET_LED)
    if r is None:
        print("timeout"); sys.exit(1)
    sys_duty, stat_duty, flag = r[1], r[2], r[3]
    print(f"sys={sys_duty}% stat={stat_duty}% update_flag=0x{flag:02x}")


def cmd_set_led(s: socket.socket, sys_duty: int, stat_duty: int, mask: int) -> None:
    payload = bytes([sys_duty & 0xFF, stat_duty & 0xFF, mask & 0xFF])
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_SET_LED | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_SET_LED)
    if r is None:
        print("timeout"); sys.exit(1)
    sys_d, stat_d = r[1], r[2]
    print(f"sys={sys_d}% stat={stat_d}%")


def cmd_get_position(s: socket.socket) -> None:
    # Payload is required to be 2 bytes by the firmware decoder.
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_POSITION | CONTROLLER_BIT, 0, 0]))
    r = expect_reply(s, CMD_GET_POSITION)
    if r is None:
        print("timeout"); sys.exit(1)
    # Wire format is Q-format radians: value = angle_rad * 65536 / (2*pi).
    q = r[1] | (r[2] << 8)
    rad = (q / 65536.0) * 2.0 * 3.141592653589793
    print(f"angle_q={q} angle_rad={rad:.6f}")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--iface", default="can0")
    sub = p.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("get-param")
    g.add_argument("index", type=int)

    st = sub.add_parser("set-param")
    st.add_argument("index", type=int)
    st.add_argument("value", type=float)

    c = sub.add_parser("calibrate")
    c.add_argument("freq_dhz", type=int, help="electrical freq in tenths of Hz")
    c.add_argument("dur_ms", type=int)
    c.add_argument("valign_pct", type=int)

    mv = sub.add_parser("move", help="set closed-loop control mode + target")
    mv.add_argument("mode", choices=list(MODE_NAMES))
    mv.add_argument("target", type=float, help="voltage (V) for voltage mode, radians for position")

    sub.add_parser("save-params", help="commit current motor params to flash")

    sub.add_parser("get-led", help="read SYS+STAT LED duty cycles")
    sl = sub.add_parser("set-led", help="set SYS+STAT LED duty cycles (0..100)")
    sl.add_argument("--sys",  type=int, default=0, help="SYS duty 0..100")
    sl.add_argument("--stat", type=int, default=0, help="STAT duty 0..100")
    sl.add_argument("--mask", type=lambda v: int(v, 0), default=None,
                    help="update_flag override (default: bits set for whichever "
                         "of --sys / --stat were given)")

    sub.add_parser("get-position", help="read single-turn encoder angle (counts + rad)")

    args = p.parse_args()
    s = open_can(args.iface)
    if args.cmd == "get-param":
        cmd_get_param(s, args.index)
    elif args.cmd == "set-param":
        cmd_set_param(s, args.index, args.value)
    elif args.cmd == "calibrate":
        cmd_calibrate(s, args.freq_dhz, args.dur_ms, args.valign_pct)
    elif args.cmd == "move":
        cmd_move(s, args.mode, args.target)
    elif args.cmd == "save-params":
        cmd_save_params(s)
    elif args.cmd == "get-led":
        cmd_get_led(s)
    elif args.cmd == "set-led":
        if args.mask is None:
            mask = 0
            # Build the mask from whichever flags were supplied on the CLI.
            argv = sys.argv
            if any(a.startswith("--sys")  for a in argv): mask |= LED_UPDATE_SYS
            if any(a.startswith("--stat") for a in argv): mask |= LED_UPDATE_STAT
            if mask == 0:
                mask = LED_UPDATE_SYS | LED_UPDATE_STAT
        else:
            mask = args.mask
        cmd_set_led(s, args.sys, args.stat, mask)
    elif args.cmd == "get-position":
        cmd_get_position(s)


if __name__ == "__main__":
    main()
