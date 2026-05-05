#!/usr/bin/env python3
"""Walk the rotor slowly through one full revolution while polling
encoder health. If the magnet is misaligned/eccentric, weak-mag
or no-mag samples will spike at certain absolute angles, and CRC
errors / hitches will correlate.
"""
import math, socket, struct, time
DEV = 0x008 + 5; CB = 0x80
def open_can():
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind(("can0",)); s.settimeout(0.1); return s
def send(s, c, p):
    d = bytes([c|CB])+p
    s.send(struct.pack("=IB3x8s", DEV, len(d), d+b"\x00"*(8-len(d))))
def rr(s, c, t=0.4):
    end = time.monotonic()+t
    while time.monotonic()<end:
        try: r=s.recv(16)
        except: continue
        cid,dl,_,_,_,p = struct.unpack("=IBBBB8s", r)
        if (cid&0x7FF)==DEV and dl>=1 and (p[0]&0x7F)==c: return p[:dl]
def gp(s, idx):
    send(s, 0x20, bytes([idx])+struct.pack("<f",0))
    r = rr(s, 0x20); return struct.unpack("<f", r[2:6])[0] if r else float("nan")
def cmd(s, m, t):
    send(s, 0x23, bytes([m])+struct.pack("<f",t)); rr(s, 0x23)

def main():
    s = open_can()
    # baseline
    cmd(s, 0, 0); time.sleep(0.2)
    a0 = gp(s, 100)
    samples0 = int(gp(s, 120)); crc0 = int(gp(s, 121))
    nomag0 = int(gp(s, 122)); weak0 = int(gp(s, 123)); bus0 = int(gp(s, 124))
    print(f"[+] start angle={a0:+.4f}  samples={samples0}  crc={crc0}  "
          f"no_mag={nomag0}  weak_mag={weak0}  bus_faults={bus0}")
    print()
    print(f"  {'angle':>7}  {'tgt':>7}  {'land':>7}  {'err':>8}  "
          f"{'Δsamp':>6}  {'Δcrc':>5}  {'Δnoma':>5}  {'Δweak':>5}  status")
    print("-"*72)

    # Walk in 10 deg (~0.175 rad) steps backward (proven safe).
    cur = a0
    cmd(s, 3, cur); time.sleep(0.3)
    for i in range(40):
        # snapshot
        prev_h = (int(gp(s,120)), int(gp(s,121)), int(gp(s,122)),
                  int(gp(s,123)))
        target = cur - math.radians(10)
        cmd(s, 3, target)
        # wait settle
        last = gp(s,100); since=None; t0=time.monotonic()
        while time.monotonic()-t0 < 1.5:
            a = gp(s,100)
            if math.isnan(a): continue
            if abs(a-last)<0.005:
                if since is None: since=time.monotonic()
                elif time.monotonic()-since>=0.2: break
            else: since=None
            last=a; time.sleep(0.02)
        landed = last
        cur_h = (int(gp(s,120)), int(gp(s,121)), int(gp(s,122)),
                 int(gp(s,123)))
        d = [cur_h[i]-prev_h[i] for i in range(4)]
        st = int(gp(s,125)) if not math.isnan(gp(s,125)) else 0
        err = landed - target
        flag = " <<< SLIP" if abs(err) > 0.05 else ""
        print(f"  {cur:+7.3f}  {target:+7.3f}  {landed:+7.3f}  {err*1000:+6.1f}mr  "
              f"{d[0]:>6}  {d[1]:>5}  {d[2]:>5}  {d[3]:>5}  0x{st:01x}{flag}")
        cur = landed
    cmd(s,0,0)

if __name__=="__main__":
    main()
