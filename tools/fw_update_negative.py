#!/usr/bin/env python3
"""Phase-4 negative test: corrupt one byte of the streamed image but send the
*correct* CRC so the device's commit verification fails. Expect:
  - StreamCommit Ack timeout (BL replies Error code=3, message=local_crc).
  - Device stays in BL update mode (no post-reboot app chatter).
"""
import socket, struct, sys, time
sys.path.insert(0, "tools")
import fw_update as fu

def main():
    payload = bytearray(open("build/app.bin", "rb").read())
    real_crc = fu.crc32_mpeg2(bytes(payload))
    # Corrupt one byte deep inside the image (after the vector table).
    payload[300] ^= 0x55
    bad_crc_we_will_send = real_crc  # mismatch on purpose
    print(f"[+] orig crc = 0x{real_crc:08x}")
    print(f"[+] corrupted_payload crc would be different; sending orig anyway")

    s = fu.open_can("can0")

    # Trigger reboot via app
    print("[*] triggering reboot via app FirmwareUpdate")
    fu.send_frame(s, fu.CAN_ID_DISCOVERY_ASSIGN,
                  struct.pack("<IB", fu.SERIAL, fu.ASSIGN_ID))
    time.sleep(0.2)
    fu.send_frame(s, fu.CAN_DEVICE_BASE + fu.ASSIGN_ID,
                  bytes([0x13 | fu.CONTROLLER_BIT]))
    if not fu.expect_ack(s, fu.ASSIGN_ID, 1.0):
        print("    !! no Ack to FirmwareUpdate")
        return 2

    if not fu.wait_for_discovery(s, 5.0):
        print("    !! BL never DiscoveryReq'd")
        return 3
    fu.send_frame(s, fu.CAN_ID_DISCOVERY_ASSIGN,
                  struct.pack("<IB", fu.SERIAL, fu.ASSIGN_ID))
    time.sleep(0.2)
    while fu.recv_frame(s, time.monotonic() + 0.05) is not None:
        pass

    fu.send_frame(s, fu.CAN_DEVICE_BASE + fu.ASSIGN_ID,
                  bytes([fu.CMD_STREAM_START | fu.CONTROLLER_BIT]) +
                  struct.pack("<I", len(payload)))
    if not fu.expect_ack(s, fu.ASSIGN_ID, 5.0):
        print("    !! StreamStart not Ack'd")
        return 4

    n = (len(payload) + 6) // 7
    for i in range(n):
        chunk = bytes(payload[i*7:(i+1)*7])
        if len(chunk) < 7:
            chunk = chunk + b"\xFF" * (7 - len(chunk))
        fu.send_frame(s, fu.CAN_DEVICE_BASE + fu.ASSIGN_ID,
                      bytes([fu.CMD_STREAM_WRITE | fu.CONTROLLER_BIT]) + chunk)
        if not fu.expect_ack(s, fu.ASSIGN_ID, 1.0):
            print(f"    !! Ack timeout @ frag {i}")
            return 5
    print(f"[*] all {n} fragments Ack'd; sending bad-CRC commit (0x{bad_crc_we_will_send:08x})")
    fu.send_frame(s, fu.CAN_DEVICE_BASE + fu.ASSIGN_ID,
                  bytes([fu.CMD_STREAM_COMMIT | fu.CONTROLLER_BIT]) +
                  struct.pack("<I", bad_crc_we_will_send))

    # Now expect an Error frame, NOT an Ack.
    deadline = time.monotonic() + 2.0
    while True:
        f = fu.recv_frame(s, deadline)
        if f is None:
            print("    !! no commit response (timeout)"); return 6
        cid, data = f
        if cid != fu.CAN_DEVICE_BASE + fu.ASSIGN_ID or len(data) < 1:
            continue
        cmd = data[0] & 0x7F
        if cmd == fu.CMD_ACK:
            print("    !! BL Ack'd a corrupted image -- BUG"); return 7
        if cmd == fu.CMD_ERROR:
            print(f"[+] BL Error frame: {data.hex()}")
            print("[+] PHASE 4 NEGATIVE OK: bad CRC rejected, device stays in BL")
            # Sanity: no post-reboot device chatter for ~1.5s
            quiet_deadline = time.monotonic() + 1.5
            seen_app = False
            while time.monotonic() < quiet_deadline:
                ff = fu.recv_frame(s, quiet_deadline)
                if ff is None: break
                cidd, dd = ff
                if cidd == fu.CAN_DEVICE_BASE + fu.ASSIGN_ID and (dd[0] & 0x80) == 0:
                    # status frame from app would have controller_bit clear
                    seen_app = True; break
            print("[+] no app reboot detected" if not seen_app
                  else "    ?? unexpected app traffic after rejection")
            return 0

if __name__ == "__main__":
    sys.exit(main())
