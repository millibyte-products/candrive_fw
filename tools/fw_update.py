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

def wait_for_discovery(s: socket.socket, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while True:
        f = recv_frame(s, deadline)
        if f is None:
            return False
        cid, data = f
        if cid == CAN_ID_DISCOVERY_REQ and len(data) == 5:
            return True

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

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--bin", required=True)
    ap.add_argument("--trigger", action="store_true",
                    help="First send FirmwareUpdate to currently-running app to drop into BL")
    args = ap.parse_args()

    payload = open(args.bin, "rb").read()
    print(f"[+] firmware: {args.bin} ({len(payload)} bytes)")

    s = open_can(args.iface)

    if args.trigger:
        # Assume app is running and unassigned; do a quick discovery + FW update.
        print("[*] triggering reboot via app FirmwareUpdate")
        if not wait_for_discovery(s, 5.0):
            print("    no DiscoveryReq from app; assuming app already up, continuing")
        send_frame(s, CAN_ID_DISCOVERY_ASSIGN,
                   struct.pack("<IB", SERIAL, ASSIGN_ID))
        time.sleep(0.2)
        # Cmd | controller_bit
        send_frame(s, CAN_DEVICE_BASE + ASSIGN_ID,
                   bytes([0x13 | CONTROLLER_BIT]))
        if not expect_ack(s, ASSIGN_ID, 1.0):
            print("    !! no Ack to FirmwareUpdate")
            return 2
        print("    app Ack'd FirmwareUpdate, waiting for BL")

    print("[*] waiting for bootloader DiscoveryReq")
    if not wait_for_discovery(s, 5.0):
        print("    !! no DiscoveryReq from bootloader")
        return 3
    print("[+] BL discovered, assigning id=5")
    send_frame(s, CAN_ID_DISCOVERY_ASSIGN, struct.pack("<IB", SERIAL, ASSIGN_ID))
    time.sleep(0.2)

    # Drain any leftover frames so subsequent expect_ack only sees post-StreamStart traffic.
    while recv_frame(s, time.monotonic() + 0.05) is not None:
        pass

    print(f"[*] StreamStart length={len(payload)}")
    send_frame(s, CAN_DEVICE_BASE + ASSIGN_ID,
               bytes([CMD_STREAM_START | CONTROLLER_BIT]) +
               struct.pack("<I", len(payload)))
    if not expect_ack(s, ASSIGN_ID, 5.0):
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
