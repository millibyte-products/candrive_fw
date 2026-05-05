#!/usr/bin/env python3
"""Long-move position battery: ±π, ±2π, ±3π, ±5 from current position.

For each test:
  1. Read current angle as ref.
  2. Command position = ref + delta (multi-turn target).
  3. Settle, read landed angle, compute err.
  4. Return to ref between tests.

Reports per-test glitch count (debug 130) and final mech_total (132).
"""
import math
import subprocess
import sys
import time
from pathlib import Path

CLI = Path(__file__).parent / "motor_cli.py"


def cli(*args, timeout=4.0):
    r = subprocess.run(
        [sys.executable, str(CLI), *args],
        capture_output=True, text=True, timeout=timeout,
    )
    return r.stdout.strip()


def get_param(idx):
    out = cli("get-param", str(idx))
    # "param[N] = V"
    return float(out.split("=", 1)[1].strip())


def move(mode, target=0.0):
    cli("move", mode, str(target))


def settle(t=6.0, eps=0.005, hold=0.3):
    end = time.monotonic() + t
    last = get_param(100)
    stable_since = None
    while time.monotonic() < end:
        a = get_param(100)
        if abs(a - last) < eps:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= hold:
                return a
        else:
            stable_since = None
        last = a
        time.sleep(0.02)
    return last


def main():
    print("Idle, waiting...")
    move("idle")
    time.sleep(0.3)
    ref = get_param(100)
    print(f"ref angle = {ref:+.4f} rad")

    move("position", ref)
    time.sleep(0.4)
    ref = settle(t=2.0)
    print(f"settled at = {ref:+.4f}\n")

    g0 = int(get_param(130))

    tests = [
        ("+pi",  +math.pi),
        ("-pi",  -math.pi),
        ("+2pi", +2 * math.pi),
        ("-2pi", -2 * math.pi),
        ("+3pi", +3 * math.pi),
        ("-3pi", -3 * math.pi),
        ("+5",   +5.0),
        ("-5",   -5.0),
    ]

    fails = 0
    for name, delta in tests:
        target = ref + delta
        move("position", target)
        landed = settle(t=8.0)
        err = landed - target
        glitches = int(get_param(130)) - g0
        ok = abs(err) < 0.05
        flag = "OK " if ok else "FAIL"
        print(f"  {flag}  {name:>4}  delta={delta:+7.4f}  land={landed:+8.4f}  "
              f"err={err*1000:+9.2f} mr  glitch+{glitches}")
        if not ok:
            fails += 1

        # Return to ref before next test.
        move("position", ref)
        settle(t=8.0)
        g0 = int(get_param(130))

    move("idle")
    print(f"\nfinal mech_total = {int(get_param(132))} counts")
    print(f"total glitches   = {int(get_param(130))}")
    print(f"max |dcounts|    = {int(get_param(131))}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
