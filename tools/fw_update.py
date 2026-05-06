#!/usr/bin/env python3
"""Phase-3 firmware-update verifier.

Drives the bootloader's CAN update mode end-to-end:
  1. Listen for the bootloader's DiscoveryReq (CAN id 0x002).
  2. Send DiscoveryAssign (CAN id 0x001).
  3. Send StreamStart with the binary's length.
  4. Stream the binary in 7-byte StreamWrite fragments.
  5. Verify each Ack and watch for the post-reset DiscoveryReq from the app.
"""
from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time

CAN_ID_DISCOVERY_ASSIGN = 0x001
CAN_ID_DISCOVERY_REQ    = 0x002
CAN_DEVICE_BASE         = 0x008
CONTROLLER_BIT          = 0x80

CMD_GET_INFO     = 0x01
CMD_ACK          = 0x12
CMD_ERROR        = 0x18
CMD_STREAM_START = 0x0F
CMD_STREAM_WRITE = 0x11
CMD_STREAM_COMMIT = 0x19

SERIAL = 0xCD000001
ASSIGN_ID = 5

# CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, no reflection, no XOR-out).
def crc32_mpeg2(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
    return crc

def open_can(iface: str) -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(0.05)
    return s

def send_frame(s: socket.socket, can_id: int, data: bytes) -> None:
    assert len(data) <= 8
    fmt = "=IB3x8s"
    s.send(struct.pack(fmt, can_id, len(data), data + b"\x00" * (8 - len(data))))

def recv_frame(s: socket.socket, deadline: float):
    while time.monotonic() < deadline:
        try:
            raw = s.recv(16)
        except socket.timeout:
            continue
        can_id, dlc, _, _, _, payload = struct.unpack("=IBBBB8s", raw)
        return can_id & 0x7FF, payload[:dlc]
    return None

def expect_ack(s: socket.socket, dev_id: int, timeout: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout
    while True:
        f = recv_frame(s, deadline)
        if f is None:
            return False
        cid, data = f
        if cid == CAN_DEVICE_BASE + dev_id and len(data) >= 1:
            cmd = data[0] & 0x7F
            if cmd == CMD_ACK:
                return True
            if cmd == CMD_ERROR:
                print(f"  [device error frame: {data.hex()}]")
                return False


def expect_ack_or_unknown_cmd(s: socket.socket, dev_id: int,
                              timeout: float = 1.0) -> str:
    """Like expect_ack(), but distinguishes the bootloader's "unknown
    command" Error{code:4} (returned when we send FirmwareUpdate while
    already in the BL) from a real fault. Returns one of:
        "ack"          - device Ack'd
        "unknown_cmd"  - device replied Error{code:4} (treat as already-in-BL)
        "error"        - device replied a different Error{...}
        "timeout"      - no reply within `timeout`
    """
    deadline = time.monotonic() + timeout
    while True:
        f = recv_frame(s, deadline)
        if f is None:
            return "timeout"
        cid, data = f
        if cid == CAN_DEVICE_BASE + dev_id and len(data) >= 1:
            cmd = data[0] & 0x7F
            if cmd == CMD_ACK:
                return "ack"
            if cmd == CMD_ERROR:
                # Error payload: code(u8) + message(u32_le)
                err_code = data[1] if len(data) >= 2 else 0xFF
                print(f"  [device error frame: {data.hex()} "
                      f"(code={err_code})]")
                if err_code == 4:
                    return "unknown_cmd"
                return "error"

def wait_for_discovery(s: socket.socket, timeout: float = 5.0):
    """Return the device serial captured from a DiscoveryReq, or None."""
    deadline = time.monotonic() + timeout
    while True:
        f = recv_frame(s, deadline)
        if f is None:
            return None
        cid, data = f
        if cid == CAN_ID_DISCOVERY_REQ and len(data) == 5:
            return struct.unpack("<I", data[:4])[0]

def wait_for_device_frame(s: socket.socket, timeout: float = 5.0) -> bool:
    """Wait for any frame originating from our assigned device id.
    After Phase 4, the app loads its identity from user_store and skips the
    initial DiscoveryReq, so we instead listen for the post-reboot status
    chatter on `CAN_DEVICE_BASE + ASSIGN_ID`."""
    deadline = time.monotonic() + timeout
    while True:
        f = recv_frame(s, deadline)
        if f is None:
            return False
        cid, _ = f
        if cid == CAN_DEVICE_BASE + ASSIGN_ID:
            return True

def probe_get_info(s: socket.socket, dev_id: int, timeout: float = 1.0):
    """Send GetInfo to dev_id; return (serial, ver_tuple) or None."""
    # Drain any pending frames so we only see the response.
    while recv_frame(s, time.monotonic() + 0.02) is not None:
        pass
    send_frame(s, CAN_DEVICE_BASE + dev_id,
               bytes([CMD_GET_INFO | CONTROLLER_BIT]) + b"\x00" * 7)
    deadline = time.monotonic() + timeout
    while True:
        f = recv_frame(s, deadline)
        if f is None:
            return None
        cid, data = f
        if cid == CAN_DEVICE_BASE + dev_id and len(data) >= 8 \
                and (data[0] & 0x7F) == CMD_GET_INFO:
            serial = struct.unpack("<I", data[1:5])[0]
            return serial, (data[5], data[6], data[7])

def main() -> int:
    global ASSIGN_ID
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--bin", required=True)
    ap.add_argument("--device-id", type=lambda v: int(v, 0), default=ASSIGN_ID,
                    help=f"CAN device id of the running app (default {ASSIGN_ID})")
    ap.add_argument("--trigger", action="store_true",
                    help="First send FirmwareUpdate to currently-running app to drop into BL")
    args = ap.parse_args()

    # Late-bind: the rest of the script still references ASSIGN_ID directly.
    ASSIGN_ID = args.device_id

    payload = open(args.bin, "rb").read()
    print(f"[+] firmware: {args.bin} ({len(payload)} bytes)")

    s = open_can(args.iface)

    already_in_bl = False
    if args.trigger:
        print(f"[*] triggering reboot via app FirmwareUpdate (dev_id={ASSIGN_ID})")

        # First: probe with GetInfo to find out whether the app already has
        # an assigned id and is responsive at the expected one. This makes
        # failures much easier to diagnose than just timing out on the Ack.
        info = probe_get_info(s, ASSIGN_ID, timeout=1.0)
        if info is None:
            # App may be unassigned (just-flashed via SWD with no user_store)
            # or assigned to a different id. Listen briefly for DiscoveryReq
            # so we can pick up its serial and assign it.
            print(f"    no GetInfo reply at id {ASSIGN_ID}; "
                  "listening for DiscoveryReq …")
            serial = SERIAL
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                f = recv_frame(s, deadline)
                if f is None:
                    break
                cid, data = f
                if cid == CAN_ID_DISCOVERY_REQ and len(data) == 5:
                    serial = struct.unpack("<I", data[:4])[0]
                    print(f"    saw DiscoveryReq serial=0x{serial:08X}")
                    break
            send_frame(s, CAN_ID_DISCOVERY_ASSIGN,
                       struct.pack("<IB", serial, ASSIGN_ID))
            time.sleep(0.2)
            # Re-probe; if still nothing, give up with a useful message.
            info = probe_get_info(s, ASSIGN_ID, timeout=1.0)
            if info is None:
                print(f"    !! still no response at id {ASSIGN_ID} — is the "
                      "app running? Try --device-id or flash via SWD.")
                return 2
        rep_serial, ver = info
        print(f"    device responding: serial=0x{rep_serial:08X} "
              f"fw=v{ver[0]}.{ver[1]}.{ver[2]}")

        send_frame(s, CAN_DEVICE_BASE + ASSIGN_ID,
                   bytes([0x13 | CONTROLLER_BIT]))
        result = expect_ack_or_unknown_cmd(s, ASSIGN_ID, 2.0)
        if result == "ack":
            print("    app Ack'd FirmwareUpdate, waiting for BL")
        elif result == "unknown_cmd":
            # Bootloader doesn't implement FirmwareUpdate (it's already
            # in update mode). Skip the wait_for_discovery() below and
            # go straight to StreamStart on the same device id.
            print("    Error{code:4} → device is already in bootloader, "
                  "skipping reboot")
            already_in_bl = True
        else:
            print(f"    !! no Ack to FirmwareUpdate ({result})")
            return 2

    if not already_in_bl:
        print("[*] waiting for bootloader DiscoveryReq")
        bl_serial = wait_for_discovery(s, 5.0)
        if bl_serial is None:
            print("    !! no DiscoveryReq from bootloader")
            return 3
        # Prefer the freshly-captured BL serial; fall back to the probe's
        # reported serial if for some reason the DiscoveryReq frame was
        # short. Hardcoded SERIAL is only used if both are unavailable.
        assign_serial = bl_serial or rep_serial or SERIAL
        print(f"[+] BL discovered (serial=0x{assign_serial:08X}), "
              f"assigning id={ASSIGN_ID}")
        send_frame(s, CAN_ID_DISCOVERY_ASSIGN,
                   struct.pack("<IB", assign_serial, ASSIGN_ID))
        time.sleep(0.2)

    # Drain any leftover frames so subsequent expect_ack only sees post-StreamStart traffic.
    while recv_frame(s, time.monotonic() + 0.05) is not None:
        pass

    print(f"[*] StreamStart length={len(payload)}")
    send_frame(s, CAN_DEVICE_BASE + ASSIGN_ID,
               bytes([CMD_STREAM_START | CONTROLLER_BIT]) +
               struct.pack("<I", len(payload)))
    # Bootloader erases every app page before Ack-ing. STM32F103 page
    # erase runs ~20-40 ms each; a full ~51 KiB app is 51 pages, plus
    # IWDG pets between pages, so the total can easily reach 3-4 s.
    # 15 s gives plenty of headroom against worst-case flash timing
    # without making real failures hang forever.
    if not expect_ack(s, ASSIGN_ID, 15.0):
        print("    !! StreamStart Ack timeout (erase failed?)")
        return 4
    print("    erase + StreamStart Ack'd")

    fragments = (len(payload) + 6) // 7
    print(f"[*] streaming {fragments} fragments")
    for i in range(fragments):
        chunk = payload[i*7:(i+1)*7]
        if len(chunk) < 7:
            chunk = chunk + b"\xFF" * (7 - len(chunk))
        send_frame(s, CAN_DEVICE_BASE + ASSIGN_ID,
                   bytes([CMD_STREAM_WRITE | CONTROLLER_BIT]) + chunk)
        if not expect_ack(s, ASSIGN_ID, 1.0):
            print(f"    !! Ack timeout at fragment {i}")
            return 5
        if i % 50 == 0 or i == fragments - 1:
            print(f"    {i+1}/{fragments} acked")

    crc = crc32_mpeg2(payload)
    print(f"[*] StreamCommit crc32=0x{crc:08x}")
    send_frame(s, CAN_DEVICE_BASE + ASSIGN_ID,
               bytes([CMD_STREAM_COMMIT | CONTROLLER_BIT]) +
               struct.pack("<I", crc))
    if not expect_ack(s, ASSIGN_ID, 2.0):
        print("    !! StreamCommit Ack timeout (CRC mismatch?)")
        return 7
    print("    commit Ack'd")

    print("[*] waiting for app reboot (post-update device chatter)")
    if not wait_for_device_frame(s, 5.0):
        print("    !! no post-update device frame")
        return 6
    print("[+] PHASE 4 OK: device rebooted into new app with persisted identity")
    return 0

if __name__ == "__main__":
    sys.exit(main())
