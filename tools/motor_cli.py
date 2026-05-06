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
CMD_SET_POSITION = 0x04
CMD_GET_STATUS   = 0x05
CMD_GET_ANALOG   = 0x06
CMD_GET_SERVO    = 0x07
CMD_SET_SERVO    = 0x08
CMD_GET_MOTOR    = 0x0B
CMD_SET_MOTOR    = 0x0C
CMD_GET_FOC      = 0x0D
CMD_SET_FOC      = 0x0E

CMD_NETWORK_RESET    = 0x10
CMD_ERASE_USER_STORE = 0x11
CMD_REVOKE_CONFIG    = 0x13

# Servo update_flag bits (matches the firmware servo module).
SRV_UPDATE_S0 = 0x01
SRV_UPDATE_S1 = 0x02

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


def _read_param(s: socket.socket, idx: int, timeout: float = 0.5):
    payload = bytes([idx]) + struct.pack("<f", 0.0)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_MOTOR_PARAM | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_GET_MOTOR_PARAM, timeout=timeout)
    if r is None:
        return None
    return struct.unpack("<f", r[2:6])[0]


# Curated snapshot of the most useful indices for FOC / position-loop
# debugging. Pulled from fw/app/src/motor.rs (config) and
# fw/app/src/control.rs::debug_get (live state).
_DUMP_FIELDS = [
    # (index, label, format_str)
    (  0, "pole_pairs",            "{:6.2f}"),
    ( 11, "electrical_zero_offset","{:6.0f} counts"),
    ( 13, "direction",             "{:+.0f}"),
    (  4, "voltage_limit",         "{:5.2f} V"),
    (  5, "voltage_supply",        "{:5.2f} V"),
    (  6, "velocity_limit",        "{:6.2f} rad/s"),
    ( 16, "traj_v_max",            "{:6.2f} rad/s"),
    ( 17, "traj_a_max",            "{:6.1f} rad/s^2"),
    ( 10, "pid_pos_p",             "{:5.2f}"),
    ( 14, "pid_pos_i",             "{:5.2f}"),
    (  7, "pid_vel_p",             "{:5.2f}"),
    (  8, "pid_vel_i",             "{:5.2f}"),
    (106, "mode",                  "{:.0f}  (0=Idle 1=V 2=Vel 3=Pos 4=OL)"),
    (102, "target",                "{:+8.4f} rad"),
    (109, "traj_setpoint",         "{:+8.4f} rad"),
    (110, "traj_v",                "{:+7.3f} rad/s"),
    (100, "angle_acc",             "{:+8.4f} rad"),
    (101, "vel",                   "{:+7.3f} rad/s"),
    (104, "last_mech_counts",      "{:6.0f}"),
    (105, "last_theta_e",          "{:+7.4f} rad"),
    (103, "last_vq",               "{:+6.3f} V"),
    (111, "pos_i",                 "{:+6.3f}"),
    (108, "step_counter",          "{:.0f}"),
    (132, "mech_total",            "{:.0f} counts"),
    (130, "dcount_glitches",       "{:.0f}"),
    (131, "max_|dcounts|",         "{:.0f}"),
]


def cmd_dump(s: socket.socket) -> None:
    """Snapshot all params useful for FOC / position-loop debugging."""
    print("=== motor state snapshot ===")
    for idx, label, fmt in _DUMP_FIELDS:
        v = _read_param(s, idx)
        if v is None:
            print(f"  [{idx:3d}] {label:24s} = <timeout>")
            continue
        try:
            value_str = fmt.format(v)
        except (ValueError, TypeError):
            value_str = f"{v}"
        print(f"  [{idx:3d}] {label:24s} = {value_str}")


def cmd_watch(s: socket.socket, indices: list[int], hz: float,
              duration: float | None) -> None:
    """Poll the given debug indices repeatedly and print one row per sample.

    Header row labels each column. Useful to watch theta_e / angle_acc /
    last_vq evolve during a move:
        ./tools/motor_cli.py watch 100 105 103 102 --hz 20
    """
    if not indices:
        indices = [102, 100, 105, 103, 101]   # target, angle_acc, theta_e, vq, vel
    period = 1.0 / hz if hz > 0 else 0.0
    print("t_s," + ",".join(f"p{i}" for i in indices))
    t0 = time.monotonic()
    next_t = t0
    deadline = (t0 + duration) if duration is not None else None
    try:
        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break
            row = [f"{now - t0:.3f}"]
            for idx in indices:
                v = _read_param(s, idx, timeout=0.1)
                row.append("nan" if v is None else f"{v:+.5g}")
            print(",".join(row))
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # Falling behind; reset cadence so we don't spin tight.
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass


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


def cmd_get_status(s: socket.socket) -> None:
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_STATUS | CONTROLLER_BIT, 0]))
    r = expect_reply(s, CMD_GET_STATUS)
    if r is None:
        print("timeout"); sys.exit(1)
    b = r[1]
    endstop0 = bool(b & 0x01)
    endstop1 = bool(b & 0x02)
    misc     = bool(b & 0x04)
    fault    = bool(b & 0x08)
    mag      = (b >> 4) & 0x0F
    # MT6701 status nibble: bit3 push, bit2 no-mag, bit1 weak-mag, bit0 strong-mag.
    mag_tags = []
    if mag & 0x01: mag_tags.append("strong")
    if mag & 0x02: mag_tags.append("weak")
    if mag & 0x04: mag_tags.append("no_mag")
    if mag & 0x08: mag_tags.append("push")
    print(f"raw=0x{b:02x} fault={fault} mag=0x{mag:x} [{','.join(mag_tags) or 'ok'}] "
          f"endstop0={endstop0} endstop1={endstop1} misc={misc}")


def cmd_get_analog(s: socket.socket) -> None:
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_ANALOG | CONTROLLER_BIT, 0, 0, 0, 0]))
    r = expect_reply(s, CMD_GET_ANALOG)
    if r is None:
        print("timeout"); sys.exit(1)
    a0 = r[1] | (r[2] << 8)
    a1 = r[3] | (r[4] << 8)
    print(f"a0={a0} ({a0 * 3.3 / 4095.0:.3f} V)  a1={a1} ({a1 * 3.3 / 4095.0:.3f} V)")


def cmd_get_servo(s: socket.socket) -> None:
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_SERVO | CONTROLLER_BIT, 0, 0, 0, 0, 0]))
    r = expect_reply(s, CMD_GET_SERVO)
    if r is None:
        print("timeout"); sys.exit(1)
    s0 = r[1] | (r[2] << 8)
    s1 = r[3] | (r[4] << 8)
    print(f"srv0={s0} us  srv1={s1} us")


def cmd_set_servo(s: socket.socket, s0: int, s1: int, mask: int) -> None:
    payload = struct.pack("<HHB", s0 & 0xFFFF, s1 & 0xFFFF, mask & 0xFF)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_SET_SERVO | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_SET_SERVO)
    if r is None:
        print("timeout"); sys.exit(1)
    s0_r = r[1] | (r[2] << 8)
    s1_r = r[3] | (r[4] << 8)
    print(f"srv0={s0_r} us  srv1={s1_r} us")


def cmd_get_motor(s: socket.socket) -> None:
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_MOTOR | CONTROLLER_BIT, 0, 0, 0]))
    r = expect_reply(s, CMD_GET_MOTOR)
    if r is None:
        print("timeout"); sys.exit(1)
    value = r[1] | (r[2] << 8)
    flags = r[3]
    rst = bool(flags & 0x01)
    sleep = bool(flags & 0x02)
    print(f"value={value} rst={rst} sleep={sleep} (bridge_enabled={rst and sleep})")


def cmd_set_motor(s: socket.socket, enable: bool) -> None:
    flags = 0x03 if enable else 0x00
    payload = bytes([0, 0, flags])
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_SET_MOTOR | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_SET_MOTOR)
    if r is None:
        print("timeout"); sys.exit(1)
    flags_r = r[3]
    rst, slp = bool(flags_r & 0x01), bool(flags_r & 0x02)
    print(f"bridge_enabled={rst and slp} (rst={rst} sleep={slp})")


def cmd_get_foc(s: socket.socket) -> None:
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_FOC | CONTROLLER_BIT, 0, 0, 0, 0]))
    r = expect_reply(s, CMD_GET_FOC)
    if r is None:
        print("timeout"); sys.exit(1)
    f1, f2, f3, en = r[1], r[2], r[3], r[4]
    pct = lambda v: 100.0 * v / 255.0
    print(f"phase_a={f1}/255 ({pct(f1):.1f}%) phase_c={f2}/255 ({pct(f2):.1f}%) "
          f"phase_b={f3}/255 ({pct(f3):.1f}%) en={en}")


def cmd_set_position(s: socket.socket, target_rad: float) -> None:
    # Wrap to [0, 2pi) then encode as Q-format radians.
    two_pi = 2.0 * 3.141592653589793
    wrapped = target_rad % two_pi
    if wrapped < 0:
        wrapped += two_pi
    q = int(round(wrapped * 65536.0 / two_pi)) & 0xFFFF
    payload = struct.pack("<H", q)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_SET_POSITION | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_SET_POSITION)
    if r is None:
        print("timeout"); sys.exit(1)
    q_r = r[1] | (r[2] << 8)
    rad_r = (q_r / 65536.0) * two_pi
    print(f"target_q={q_r} target_rad={rad_r:.6f}")


def cmd_quick_test(s: socket.socket, dwell_s: float, mode: str,
                   loops: int) -> None:
    """One CW revolution then one CCW revolution, dwelling at each
    90\u00b0 cardinal position.

    Sequence (per loop), starting from the rotor's current angle θ₀:
        θ₀, θ₀+90\u00b0, θ₀+180\u00b0, θ₀+270\u00b0, θ₀+360\u00b0   (CW, 4 moves)
        θ₀+270\u00b0, θ₀+180\u00b0, θ₀+90\u00b0, θ₀                (CCW, 4 moves)

    Targets are computed as **absolute multi-turn angles** off the
    starting position and dispatched as raw Position mode (mode 3)
    so each step has a fixed goal. Re-issuing the command during a
    move converges to the same absolute target — the rotor never
    chases a moving setpoint, which is what causes the "free-running"
    behaviour seen with the relative / abs-* modes if dwell is too
    short to fully settle.
    """
    if mode not in ("auto", "relative", "abs-shortest", "abs-forward", "abs-backward"):
        print(f"unknown quick-test mode {mode!r}")
        sys.exit(2)
    pi = 3.141592653589793
    two_pi = 2.0 * pi

    # Make sure the bridge is on so the position controller can act.
    cmd_set_motor(s, True)
    time.sleep(0.05)

    # Capture starting angle. Position mode targets the multi-turn
    # accumulator (which starts at 0 the moment Position mode is first
    # entered), so we anchor to "0 rad" relative to the test's first
    # mode entry. We still print the wrapped single-turn angle for
    # operator reference.
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_POSITION | CONTROLLER_BIT, 0, 0]))
    pr = expect_reply(s, CMD_GET_POSITION, timeout=0.5)
    if pr is not None:
        q = pr[1] | (pr[2] << 8)
        rad = (q / 65536.0) * two_pi
        print(f"start angle (wrapped) = {rad * 180.0 / pi:6.1f}\u00b0")

    cw_targets  = [0.0, 0.5 * pi, pi, 1.5 * pi, two_pi]
    ccw_targets = [1.5 * pi, pi, 0.5 * pi, 0.0]

    if mode == "auto":
        # Mode 3 = raw Position (absolute multi-turn from mode-entry
        # zero). This is the only mode that doesn't re-resolve its
        # target against the current rotor angle on each new command.
        code = 3
    else:
        code = MODE_NAMES[mode]

    try:
        for loop in range(loops):
            print(f"\n=== loop {loop+1}/{loops}: CW revolution ===")
            for i, target in enumerate(cw_targets):
                _quick_test_step(s, code, target, dwell_s,
                                 f"CW step {i}/{len(cw_targets)-1}", pi, two_pi)
            print(f"\n=== loop {loop+1}/{loops}: CCW revolution ===")
            for i, target in enumerate(ccw_targets):
                _quick_test_step(s, code, target, dwell_s,
                                 f"CCW step {i+1}/{len(ccw_targets)}", pi, two_pi)
    finally:
        # Always leave the controller in Idle so a stuck loop / Ctrl-C
        # can't leave the rotor under torque.
        payload = bytes([MODE_NAMES["idle"]]) + struct.pack("<f", 0.0)
        send(s, CAN_DEVICE_BASE + DEV_ID,
             bytes([CMD_SET_MOTOR_COMMAND | CONTROLLER_BIT]) + payload)
        expect_reply(s, CMD_SET_MOTOR_COMMAND, timeout=0.5)


def _quick_test_step(s: socket.socket, mode_code: int, target: float,
                     dwell_s: float, label: str, pi: float, two_pi: float) -> None:
    deg = target * 180.0 / pi
    print(f"  {label}: target = {deg:7.1f}\u00b0")
    payload = bytes([mode_code]) + struct.pack("<f", target)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_SET_MOTOR_COMMAND | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, CMD_SET_MOTOR_COMMAND, timeout=1.0)
    if r is None:
        print("    timeout sending move"); sys.exit(1)
    time.sleep(dwell_s)
    # Read back encoder for visibility (single-turn wrapped angle).
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_GET_POSITION | CONTROLLER_BIT, 0, 0]))
    pr = expect_reply(s, CMD_GET_POSITION, timeout=0.5)
    if pr is not None:
        q = pr[1] | (pr[2] << 8)
        rad = (q / 65536.0) * two_pi
        print(f"    measured (wrapped) = {rad * 180.0 / pi:6.1f}\u00b0")


def cmd_network_reset(s: socket.socket) -> None:
    # Broadcast on the controller channel (CAN ID 0).
    send(s, 0x000, bytes([CMD_NETWORK_RESET | CONTROLLER_BIT]))
    print("network_reset broadcast sent (no reply expected)")


def cmd_erase_user_store(s: socket.socket) -> None:
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_ERASE_USER_STORE | CONTROLLER_BIT]))
    r = expect_reply(s, 0x7F, timeout=2.0)  # Ack
    if r is None:
        print("timeout"); sys.exit(1)
    print(f"acked: 0x{r[0]:02x}")


def cmd_revoke_config(s: socket.socket, serial_no: int) -> None:
    payload = struct.pack("<I", serial_no & 0xFFFFFFFF)
    send(s, CAN_DEVICE_BASE + DEV_ID,
         bytes([CMD_REVOKE_CONFIG | CONTROLLER_BIT]) + payload)
    r = expect_reply(s, 0x7F, timeout=2.0)  # Ack
    if r is None:
        print("timeout"); sys.exit(1)
    print(f"acked: 0x{r[0]:02x}")


# --- Quick post-flash bring-up verifier --------------------------------------

CMD_GET_INFO     = 0x01

CAN_ID_DISCOVERY_ASSIGN = 0x001
CAN_ID_DISCOVERY_REQ    = 0x002


def _expect_reply_on(s: socket.socket, dev_id: int, cmd: int,
                     timeout: float):
    """Like expect_reply() but parameterised by device id (not the
    module-level DEV_ID)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        f = recv(s, deadline - time.monotonic())
        if f is None:
            continue
        cid, data = f
        if cid == CAN_DEVICE_BASE + dev_id and len(data) >= 1:
            if (data[0] & 0x7F) == cmd:
                return data
    return None


def cmd_verify(s: socket.socket, *, dev_id: int, listen_for: float,
               expect_serial: int | None) -> int:
    """Quick post-flash verification: assign + GetInfo + GetStatus.

    If `listen_for > 0`, first watch the bus for a DiscoveryReq from a
    freshly booted device to learn its serial. Otherwise assume the
    device is already discovered at `dev_id`.

    Distinguishes app vs bootloader by probing GetStatus after GetInfo:
    the bootloader doesn't implement GetStatus, so a timeout there
    means we're talking to the bootloader.
    """
    serial: int | None = None
    if listen_for > 0:
        print(f"[*] listening {listen_for:.1f}s for DiscoveryReq on can …")
        deadline = time.monotonic() + listen_for
        while time.monotonic() < deadline:
            f = recv(s, deadline - time.monotonic())
            if f is None:
                continue
            cid, data = f
            if cid == CAN_ID_DISCOVERY_REQ and len(data) == 5:
                serial = struct.unpack("<I", data[:4])[0]
                prev_id = data[4]
                print(f"    DiscoveryReq: serial=0x{serial:08X} "
                      f"prev_id={prev_id}")
                break
        if serial is None and expect_serial is None:
            print("[!] no DiscoveryReq seen; assuming device is already "
                  f"assigned at id {dev_id}")

    # Assign if we have a serial. (Idempotent: re-assigning the same
    # serial+id is a no-op for the device.)
    if serial is not None or expect_serial is not None:
        the_serial = serial if serial is not None else expect_serial
        send(s, CAN_ID_DISCOVERY_ASSIGN,
             struct.pack("<IB", the_serial & 0xFFFFFFFF, dev_id))
        time.sleep(0.05)

    # GetInfo. Wire payload: firmware's decoder requires a 7-byte
    # payload here (matches the reply layout: serial u32 + version u8x3).
    # Pad with zeros so we don't trip `need(7)` and get silently dropped.
    send(s, CAN_DEVICE_BASE + dev_id,
         bytes([CMD_GET_INFO | CONTROLLER_BIT]) + b"\x00" * 7)
    r = _expect_reply_on(s, dev_id, CMD_GET_INFO, timeout=1.5)
    if r is None:
        print(f"[x] no GetInfo reply from device id {dev_id} — not on bus, "
              "wrong id, or unassigned")
        return 2
    if len(r) < 8:
        print(f"[x] short GetInfo reply: {r.hex()}")
        return 2
    rep_serial = struct.unpack("<I", r[1:5])[0]
    fw_major, fw_minor, fw_patch = r[5], r[6], r[7]

    # Probe for app-only command to distinguish app vs bootloader.
    send(s, CAN_DEVICE_BASE + dev_id,
         bytes([CMD_GET_STATUS | CONTROLLER_BIT, 0]))
    status_reply = _expect_reply_on(s, dev_id, CMD_GET_STATUS, timeout=0.5)
    image = "app" if status_reply is not None else "bootloader"

    print(f"[+] device id={dev_id}  serial=0x{rep_serial:08X}  "
          f"fw=v{fw_major}.{fw_minor}.{fw_patch}  image={image}")

    rc = 0
    if expect_serial is not None and rep_serial != expect_serial:
        print(f"[!] serial mismatch: expected 0x{expect_serial:08X}, "
              f"got 0x{rep_serial:08X}")
        rc = 3
    return rc


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

    sub.add_parser("dump",
        help="snapshot all FOC / position-loop debug params (params + live state)")
    w = sub.add_parser("watch",
        help="continuously poll a list of param indices and print CSV rows")
    w.add_argument("indices", nargs="*", type=int,
                   help="param indices to poll (default: target, angle_acc, theta_e, vq, vel)")
    w.add_argument("--hz", type=float, default=10.0,
                   help="poll rate in Hz (default 10)")
    w.add_argument("--duration", type=float, default=None,
                   help="stop after this many seconds (default: run until Ctrl-C)")

    sub.add_parser("get-led", help="read SYS+STAT LED duty cycles")
    sl = sub.add_parser("set-led", help="set SYS+STAT LED duty cycles (0..100)")
    sl.add_argument("--sys",  type=int, default=0, help="SYS duty 0..100")
    sl.add_argument("--stat", type=int, default=0, help="STAT duty 0..100")
    sl.add_argument("--mask", type=lambda v: int(v, 0), default=None,
                    help="update_flag override (default: bits set for whichever "
                         "of --sys / --stat were given)")

    sub.add_parser("get-position", help="read single-turn encoder angle (counts + rad)")
    sub.add_parser("get-status", help="read status byte (endstops, fault, mag)")
    sub.add_parser("get-analog", help="read A0 / A1 ADC inputs (raw + volts)")
    sub.add_parser("get-servo", help="read SRV0 / SRV1 servo pulse widths (us)")

    ss = sub.add_parser("set-servo", help="set SRV0/SRV1 pulse width in us (0=off)")
    ss.add_argument("--s0", type=int, default=0)
    ss.add_argument("--s1", type=int, default=0)
    ss.add_argument("--mask", type=lambda v: int(v, 0), default=None,
                    help="override update mask (default: bits set for whichever "
                         "of --s0 / --s1 were given)")

    sub.add_parser("get-motor", help="read bridge enable state (NRST/NSLEEP)")
    sm = sub.add_parser("set-motor", help="force bridge enable / disable")
    sm.add_argument("state", choices=["on", "off"])

    sub.add_parser("get-foc", help="read current per-phase PWM duties")

    sp = sub.add_parser("set-position", help="absolute-shortest move to target (rad)")
    sp.add_argument("target", type=float, help="target angle in radians")

    sub.add_parser("network-reset", help="broadcast: every device drops its assigned id")
    sub.add_parser("erase-user-store", help="factory reset: erase id + saved params")
    rc = sub.add_parser("revoke-config", help="reset id only; --serial must match this device")
    rc.add_argument("--serial", type=lambda v: int(v, 0), required=True,
                    help="device serial_no (factory default 0xCAFEBABE)")

    qt = sub.add_parser("quick-test",
        help="one CW revolution then one CCW revolution, dwelling at each 90\u00b0")
    qt.add_argument("--dwell", type=float, default=1.0,
                    help="seconds to hold each cardinal position (default 1.0)")
    qt.add_argument("--mode", default="auto",
                    choices=["auto", "relative", "abs-shortest", "abs-forward", "abs-backward"],
                    help="`auto` (default): raw Position mode (3) with absolute "
                         "multi-turn targets — does not chase a moving setpoint, "
                         "needs no calibration. `relative` / `abs-*` use the "
                         "matching firmware mode (may chase the setpoint if "
                         "dwell is shorter than a move).")
    qt.add_argument("--loops", type=int, default=1,
                    help="how many CW+CCW cycles to run (default 1)")

    vfy = sub.add_parser("verify",
        help="post-flash sanity: discover (or assume), GetInfo, probe app vs bootloader")
    vfy.add_argument("--device-id", type=int, default=DEV_ID,
                     help=f"CAN device id to assign / query (default {DEV_ID})")
    vfy.add_argument("--listen", type=float, default=3.0,
                     help="seconds to wait for a DiscoveryReq before falling "
                     "back to assuming an already-assigned device (default 3)")
    vfy.add_argument("--expect-serial", type=lambda v: int(v, 0),
                     help="if given, fail with rc=3 unless GetInfo reports "
                     "this serial (matches what production_flash.py wrote)")

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
    elif args.cmd == "dump":
        cmd_dump(s)
    elif args.cmd == "watch":
        cmd_watch(s, args.indices, args.hz, args.duration)
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
    elif args.cmd == "get-status":
        cmd_get_status(s)
    elif args.cmd == "get-analog":
        cmd_get_analog(s)
    elif args.cmd == "get-servo":
        cmd_get_servo(s)
    elif args.cmd == "set-servo":
        if args.mask is None:
            argv = sys.argv
            mask = 0
            if any(a.startswith("--s0") for a in argv): mask |= SRV_UPDATE_S0
            if any(a.startswith("--s1") for a in argv): mask |= SRV_UPDATE_S1
            if mask == 0:
                mask = SRV_UPDATE_S0 | SRV_UPDATE_S1
        else:
            mask = args.mask
        cmd_set_servo(s, args.s0, args.s1, mask)
    elif args.cmd == "get-motor":
        cmd_get_motor(s)
    elif args.cmd == "set-motor":
        cmd_set_motor(s, args.state == "on")
    elif args.cmd == "get-foc":
        cmd_get_foc(s)
    elif args.cmd == "set-position":
        cmd_set_position(s, args.target)
    elif args.cmd == "network-reset":
        cmd_network_reset(s)
    elif args.cmd == "erase-user-store":
        cmd_erase_user_store(s)
    elif args.cmd == "revoke-config":
        cmd_revoke_config(s, args.serial)
    elif args.cmd == "quick-test":
        cmd_quick_test(s, args.dwell, args.mode, args.loops)
    elif args.cmd == "verify":
        rc = cmd_verify(s,
                        dev_id=args.device_id,
                        listen_for=args.listen,
                        expect_serial=args.expect_serial)
        sys.exit(rc)


if __name__ == "__main__":
    main()
