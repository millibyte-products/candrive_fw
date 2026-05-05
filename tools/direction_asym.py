#!/usr/bin/env python3
"""Test +ve and -ve moves of fixed magnitude from many starting angles.

If the slip is calibration-related (electrical zero offset), it
should depend on absolute rotor angle, not on direction. We'll
walk the rotor around the circle in -π/4 steps (which work) and
attempt a +π/4 move from each landing point.
"""
import math, socket, struct, time
CAN_DEVICE_BASE = 0x008; CONTROLLER_BIT = 0x80
CMD_GET_MOTOR_PARAM = 0x20; CMD_SET_MOTOR_COMMAND = 0x23
DEV_ID = 5; DEBUG_ANGLE_ACC = 100
MODE_IDLE = 0; MODE_POSITION = 3

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
        try: raw = s.recv(16)
        except socket.timeout: continue
        cid, dl, _, _, _, p = struct.unpack("=IBBBB8s", raw)
        cid &= 0x7FF
        if cid == CAN_DEVICE_BASE + DEV_ID and dl >= 1 and (p[0] & 0x7F) == cmd:
            return p[:dl]
    return None

def get_param(s, idx):
    send(s, CMD_GET_MOTOR_PARAM, bytes([idx]) + struct.pack("<f", 0.0))
    r = recv_reply(s, CMD_GET_MOTOR_PARAM)
    return struct.unpack("<f", r[2:6])[0] if r else float('nan')

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
    set_mode(s, MODE_IDLE, 0.0); time.sleep(0.2)
    ref = get_param(s, DEBUG_ANGLE_ACC)
    set_mode(s, MODE_POSITION, ref); time.sleep(0.3)
    print(f"[+] starting reference = {ref:+.4f} rad\n")
    print(f"{'iter':>4}  {'start':>8}  {'+0.5 land':>10}  {'+0.5 err':>10}  {'-0.5 land':>10}  {'-0.5 err':>10}")
    print("-" * 60)

    cur = ref
    for i in range(8):
        # Try a +0.5 move.
        target = cur + 0.5
        set_mode(s, MODE_POSITION, target)
        landed_p = settle(s)
        err_p = landed_p - target

        # Walk back to cur (this should always work — backward direction).
        set_mode(s, MODE_POSITION, cur)
        settle(s)

        # Try a -0.5 move.
        target = cur - 0.5
        set_mode(s, MODE_POSITION, target)
        landed_n = settle(s)
        err_n = landed_n - target

        print(f"{i:>4}  {cur:+8.4f}  {landed_p:+10.4f}  {err_p*1000:+8.2f} mr  "
              f"{landed_n:+10.4f}  {err_n*1000:+8.2f} mr")

        # Now intentionally walk forward by 0.785 (always going neg fails-safe,
        # then jumping +0.785 once accumulates).
        # Actually, do small backward steps to advance the absolute angle in
        # the direction that works:
        cur -= 0.4
        set_mode(s, MODE_POSITION, cur)
        settle(s)

    set_mode(s, MODE_IDLE, 0.0)


if __name__ == "__main__":
    main()
