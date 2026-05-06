#!/usr/bin/env python3
"""
candrive development bring-up / flash / monitor helper.

Hardware expected on the host:
  * USB-CAN adapter (SocketCAN, e.g. can0 — Peak/InnoMaker/Kvaser/etc.)
  * STLinkV3 on the candrive board's SWD header
  * USB-UART adapter on the candrive board's debug serial (Serial3, 115200 8N1)

Subcommands:
  doctor   : verify all three pieces of hardware are present
  up       : bring the CAN interface up at 1 Mbit/s
  flash    : program firmware over SWD (default) or CAN
  monitor  : tail the UART (and optionally STM32 SWO/ITM via OpenOCD)
  all      : doctor + up + flash + monitor (the usual dev loop)

Examples:
  ./dev_setup.py doctor
  sudo ./dev_setup.py up
  ./dev_setup.py flash --env release            # builds with PlatformIO + flashes via SWD
  ./dev_setup.py flash --image .pio/build/release/firmware.bin --addr 0x08000000
  ./dev_setup.py flash --transport can --device-id 1 --image path/to/app.bin
  ./dev_setup.py monitor --swo
  ./dev_setup.py all --env release
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent

# --------- defaults (override on the command line) ---------------------------
DEFAULT_CAN_IFACE = "can0"
DEFAULT_CAN_BITRATE = 1_000_000          # matches src/common/bxcan.c
DEFAULT_UART_BAUD = 115200               # matches src/common/usart_dbg.c
DEFAULT_FLASH_ADDR = 0x08000000
STM32_CPU_HZ = 64_000_000                # HSI/2 * 16 from clocks_init_64mhz
SWO_HZ = 2_000_000                       # safe SWO trace clock for STM32F103

# Image -> (ELF path, flash address). The make build produces these.
IMAGES = {
    "bootloader": ("build/bootloader.elf", 0x08000000),
    "common":     ("build/common.elf",     0x08002000),
    "app":        ("build/app.elf",        0x08003400),
}

ANSI = {
    "red":    "\033[31m",
    "green":  "\033[32m",
    "yellow": "\033[33m",
    "blue":   "\033[34m",
    "cyan":   "\033[36m",
    "dim":    "\033[2m",
    "reset":  "\033[0m",
}


def color(name: str, s: str) -> str:
    if not sys.stdout.isatty():
        return s
    return f"{ANSI.get(name, '')}{s}{ANSI['reset']}"


def info(msg: str) -> None:
    print(color("cyan", "[*] ") + msg)


def ok(msg: str) -> None:
    print(color("green", "[+] ") + msg)


def warn(msg: str) -> None:
    print(color("yellow", "[!] ") + msg)


def fail(msg: str, code: int = 1):
    print(color("red", "[x] ") + msg, file=sys.stderr)
    sys.exit(code)


# -----------------------------------------------------------------------------
# Hardware detection
# -----------------------------------------------------------------------------
@dataclass
class Hardware:
    can_iface: Optional[str]
    can_state: Optional[str]            # "UP" / "DOWN" / None
    stlink_serial: Optional[str]
    uart_device: Optional[str]


def _run(cmd: list[str], check: bool = False, capture: bool = True,
         **kw) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        check=check,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
        **kw,
    )


def detect_can(iface: str) -> tuple[Optional[str], Optional[str]]:
    """Return (iface, state) where state is 'UP' or 'DOWN'. None if absent."""
    if shutil.which("ip") is None:
        warn("`ip` not found; cannot inspect CAN interface")
        return None, None
    res = _run(["ip", "-details", "link", "show", iface])
    if res.returncode != 0:
        return None, None
    state = "UP" if re.search(r"state UP", res.stdout) else "DOWN"
    # Some CAN adapters report state UNKNOWN even when up; fall back to flags.
    if state == "DOWN" and "<NOARP,UP" in res.stdout:
        state = "UP"
    return iface, state


def detect_stlink() -> Optional[str]:
    """Return STLink serial string if visible, else None."""
    if shutil.which("st-info"):
        res = _run(["st-info", "--probe"])
        if res.returncode == 0 and "serial:" in res.stdout.lower():
            m = re.search(r"serial:\s*([0-9A-Fa-f]+)", res.stdout)
            return m.group(1) if m else "present"
    if shutil.which("lsusb"):
        res = _run(["lsusb"])
        # STLinkV3 = 0483:374e / 374f / 3753 / 3754 (variants)
        for line in res.stdout.splitlines():
            if "0483:374" in line.lower() or "st-link" in line.lower():
                return line.strip()
    return None


def detect_uart(preferred: Optional[str] = None) -> Optional[str]:
    if preferred and Path(preferred).exists():
        return preferred
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        # Prefer obvious USB-UART bridges, exclude STLink VCPs.
        candidates = []
        for entry in sorted(by_id.iterdir()):
            name = entry.name.lower()
            if "stlink" in name or "st-link" in name:
                continue
            if any(x in name for x in ("ftdi", "cp210", "ch340", "ch341",
                                       "pl2303", "usb-uart", "usb_serial")):
                candidates.append(entry.resolve())
        if candidates:
            return str(candidates[0])
        # Fall back to any non-STLink entry.
        for entry in sorted(by_id.iterdir()):
            if "stlink" in entry.name.lower():
                continue
            return str(entry.resolve())
    # Last resort: /dev/ttyUSB0
    if Path("/dev/ttyUSB0").exists():
        return "/dev/ttyUSB0"
    return None


def probe_hardware(args) -> Hardware:
    can_iface, can_state = detect_can(args.can_iface)
    stlink = detect_stlink()
    uart = detect_uart(getattr(args, "uart", None))
    return Hardware(can_iface, can_state, stlink, uart)


def cmd_doctor(args) -> int:
    hw = probe_hardware(args)
    rc = 0
    print(color("blue", "== candrive dev hardware =="))
    if hw.can_iface:
        ok(f"CAN: {hw.can_iface} present (state={hw.can_state})")
    else:
        warn(f"CAN: interface {args.can_iface} not found "
             "(plug in USB-CAN adapter, or load slcand/gs_usb)")
        rc = 1
    if hw.stlink_serial:
        ok(f"STLink: {hw.stlink_serial}")
    else:
        warn("STLink: no STLinkV3 detected via st-info/lsusb")
        rc = 1
    if hw.uart_device:
        ok(f"UART:  {hw.uart_device}")
    else:
        warn("UART: no USB-UART adapter found under /dev/serial/by-id")
        rc = 1
    return rc


# -----------------------------------------------------------------------------
# CAN bring-up
# -----------------------------------------------------------------------------
def cmd_up(args) -> int:
    iface = args.can_iface
    bitrate = args.bitrate
    iface_present, state = detect_can(iface)
    if iface_present is None:
        fail(f"CAN interface {iface} is not present. "
             "Plug in the USB-CAN adapter or check `dmesg`.")
    if state == "UP" and not args.force:
        ok(f"{iface} already up; skipping (use --force to reconfigure)")
        return 0
    sudo = [] if os.geteuid() == 0 else ["sudo"]
    info(f"Configuring {iface} @ {bitrate} bps")
    cmds = [
        sudo + ["ip", "link", "set", iface, "down"],
        sudo + ["ip", "link", "set", iface, "type", "can",
                "bitrate", str(bitrate)],
        sudo + ["ip", "link", "set", iface, "up"],
    ]
    for c in cmds:
        r = _run(c, capture=False)
        if r.returncode != 0:
            fail(f"command failed: {' '.join(c)}")
    ok(f"{iface} is up at {bitrate} bps")
    return 0


# -----------------------------------------------------------------------------
# Flashing
# -----------------------------------------------------------------------------
def _make_build(targets: Optional[list[str]] = None, debug: bool = False) -> None:
    """Run `make` for the requested image targets (default: all three)."""
    cmd = ["make"]
    if debug:
        cmd.append("DEBUG=1")
    if targets:
        cmd.extend(targets)
    info("Running: " + " ".join(cmd))
    r = subprocess.run(cmd, cwd=ROOT)
    if r.returncode != 0:
        fail("make build failed")


def flash_swd(image: Path, addr: Optional[int] = None) -> int:
    """Program the device with OpenOCD + stlink.
    For .elf inputs the addr is embedded; for raw .bin/.hex it must be given.
    """
    if shutil.which("openocd") is None:
        fail("openocd not installed")
    is_elf = image.suffix.lower() == ".elf"
    program = (f"program {image} verify" if is_elf
               else f"program {image} 0x{(addr or 0):08X} verify")
    info(f"Flashing {image}" + ("" if is_elf else f" @ 0x{(addr or 0):08X}") + " via STLink")
    cmd = [
        "openocd",
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32f1x.cfg",
        "-c", "init",
        "-c", "reset halt",
        "-c", program,
        "-c", "reset run",
        "-c", "exit",
    ]
    r = subprocess.run(cmd, cwd=ROOT)
    if r.returncode != 0:
        fail("openocd flash failed")
    ok("Flash complete (SWD)")
    return 0


def flash_swd_all(images: list[str]) -> int:
    """Single OpenOCD session that programs multiple ELFs in order."""
    if shutil.which("openocd") is None:
        fail("openocd not installed")
    info("Flashing " + ", ".join(images) + " via STLink")
    args = ["openocd", "-f", "interface/stlink.cfg",
            "-f", "target/stm32f1x.cfg",
            "-c", "init", "-c", "reset halt"]
    for name in images:
        elf, _ = IMAGES[name]
        path = ROOT / elf
        if not path.exists():
            fail(f"{name}: {path} not built; run `make {name}` first")
        args += ["-c", f"program {path} verify"]
    args += ["-c", "reset run", "-c", "exit"]
    r = subprocess.run(args, cwd=ROOT)
    if r.returncode != 0:
        fail("openocd flash failed")
    ok("Flash complete")
    return 0


def flash_can(image: Path, device_id: int) -> int:
    """Push firmware to a running device over CAN via fw_update.py.

    Drives the bootloader's stream-update path: triggers a reboot into
    BL on the running app, assigns a device id, streams the binary,
    and waits for the post-reset chatter from the new app.
    """
    script = ROOT / "tools" / "fw_update.py"
    if not script.exists():
        fail(f"fw_update.py missing at {script}")
    info(f"Sending firmware {image} to device 0x{device_id:X} over CAN")
    r = subprocess.run(
        [str(script), "--bin", str(image), "--trigger"],
        cwd=ROOT,
    )
    if r.returncode != 0:
        fail(f"fw_update.py exited with code {r.returncode}")
    ok("Flash command dispatched (CAN)")
    return 0


def cmd_flash(args) -> int:
    # SWD multi-image path: by default, program bootloader + common + app
    # in a single OpenOCD session.
    if args.transport == "swd":
        if args.image:
            image = Path(args.image).resolve()
            if not image.exists():
                fail(f"image not found: {image}")
            return flash_swd(image, args.addr)
        # Build whatever's requested, then flash.
        targets = args.images or list(IMAGES.keys())
        for t in targets:
            if t not in IMAGES:
                fail(f"unknown image '{t}'; choose from {list(IMAGES)}")
        _make_build(targets, debug=args.debug)
        return flash_swd_all(targets)

    # CAN-based firmware update (app image only — bootloader/common are
    # programmed via SWD, then app is updated in the field over CAN).
    if args.transport == "can":
        device_id = args.device_id if args.device_id is not None else 5
        if not args.image:
            _make_build(["app"], debug=args.debug)
            image = ROOT / IMAGES["app"][0].replace(".elf", ".bin")
        else:
            image = Path(args.image).resolve()
            if not image.exists():
                fail(f"image not found: {image}")
        _, state = detect_can(args.can_iface)
        if state != "UP":
            warn(f"{args.can_iface} is not up; running `up` first")
            cmd_up(args)
        return flash_can(image, device_id)
    fail(f"unknown transport: {args.transport}")
    return 1


# -----------------------------------------------------------------------------
# Monitoring
# -----------------------------------------------------------------------------
class _PrefixedTail(threading.Thread):
    """Read a stream line-by-line and echo with a colored prefix."""
    def __init__(self, stream, prefix: str, color_name: str, stop_evt):
        super().__init__(daemon=True)
        self.stream = stream
        self.prefix = color(color_name, prefix)
        self.stop_evt = stop_evt

    def run(self):
        try:
            for line in iter(self.stream.readline, b""):
                if self.stop_evt.is_set():
                    return
                try:
                    text = line.decode(errors="replace").rstrip()
                except Exception:
                    text = repr(line)
                print(f"{self.prefix} {text}")
        except Exception as e:  # noqa: BLE001
            warn(f"tail thread {self.prefix} stopped: {e}")


def _monitor_uart(device: str, baud: int, stop_evt) -> None:
    try:
        import serial  # type: ignore
    except ImportError:
        fail("pyserial not installed. `pip install pyserial`")
    info(f"Opening UART {device} @ {baud}")
    try:
        port = serial.Serial(device, baudrate=baud, timeout=0.2)
    except Exception as e:  # noqa: BLE001
        warn(f"could not open UART: {e}")
        return
    prefix = color("green", "[uart]")
    while not stop_evt.is_set():
        try:
            line = port.readline()
        except Exception as e:  # noqa: BLE001
            warn(f"UART read error: {e}")
            break
        if not line:
            continue
        try:
            txt = line.decode(errors="replace").rstrip()
        except Exception:
            txt = repr(line)
        print(f"{prefix} {txt}")
    port.close()


def _monitor_swo(stop_evt) -> Optional[subprocess.Popen]:
    """Spawn OpenOCD configured to dump ITM/SWO port 0 to stdout."""
    if shutil.which("openocd") is None:
        warn("openocd not found, skipping SWO monitor")
        return None
    info(f"Starting OpenOCD SWO trace ({SWO_HZ} Hz, CPU {STM32_CPU_HZ} Hz)")
    # tpiu/tcl_trace -> openocd prints ITM port 0 chars on stdout
    cmd = [
        "openocd",
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32f1x.cfg",
        "-c", "init",
        "-c", f"tpiu config internal - uart off {STM32_CPU_HZ}",
        "-c", f"itm port 0 on",
        "-c", "reset run",
    ]
    proc = subprocess.Popen(
        cmd, cwd=ROOT,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        bufsize=0,
    )
    _PrefixedTail(proc.stdout, "[swo]", "yellow", stop_evt).start()
    return proc


def cmd_monitor(args) -> int:
    hw = probe_hardware(args)
    uart = args.uart or hw.uart_device
    if uart is None:
        fail("no UART device available (pass --uart /dev/ttyUSBx)")
    stop_evt = threading.Event()
    swo_proc = _monitor_swo(stop_evt) if args.swo else None

    def _shutdown(signum, frame):  # noqa: ARG001
        stop_evt.set()
        if swo_proc and swo_proc.poll() is None:
            swo_proc.terminate()
        print()
        info("monitor stopping")

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        _monitor_uart(uart, args.baud, stop_evt)
    finally:
        stop_evt.set()
        if swo_proc and swo_proc.poll() is None:
            swo_proc.terminate()
            try:
                swo_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                swo_proc.kill()
    return 0


# -----------------------------------------------------------------------------
# `all`: doctor -> up -> flash -> monitor
# -----------------------------------------------------------------------------
def cmd_all(args) -> int:
    rc = cmd_doctor(args)
    if rc != 0 and not args.ignore_missing:
        fail("hardware check failed; pass --ignore-missing to continue anyway")
    cmd_up(args)
    cmd_flash(args)
    return cmd_monitor(args)


# -----------------------------------------------------------------------------
# argparse glue
# -----------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="candrive dev bring-up / flash / monitor helper")
    p.add_argument("--can-iface", default=DEFAULT_CAN_IFACE,
                   help=f"SocketCAN interface (default: {DEFAULT_CAN_IFACE})")
    p.add_argument("--bitrate", type=int, default=DEFAULT_CAN_BITRATE,
                   help="CAN bitrate (default: 1 Mbit/s)")
    p.add_argument("--uart", default=None,
                   help="UART device (default: auto-detect /dev/serial/by-id)")
    p.add_argument("--baud", type=int, default=DEFAULT_UART_BAUD,
                   help=f"UART baud (default: {DEFAULT_UART_BAUD})")

    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("doctor", help="check that all hardware is present")

    p_up = sub.add_parser("up", help="bring the CAN interface up")
    p_up.add_argument("--force", action="store_true",
                      help="reconfigure even if already up")

    p_fl = sub.add_parser("flash", help="program firmware (SWD or CAN)")
    p_fl.add_argument("--transport", choices=["swd", "can"], default="swd")
    p_fl.add_argument("--images", nargs="+", default=None,
                      choices=list(IMAGES.keys()),
                      help="which images to (re)build and flash via SWD; "
                           "default: all three")
    p_fl.add_argument("--debug", action="store_true",
                      help="build with DEBUG=1 (-Og -g3)")
    p_fl.add_argument("--image", default=None,
                      help="prebuilt .elf/.bin/.hex to flash (skips make)")
    p_fl.add_argument("--addr", type=lambda x: int(x, 0),
                      default=DEFAULT_FLASH_ADDR,
                      help="raw .bin flash address (default: 0x08000000)")
    p_fl.add_argument("--device-id", type=lambda x: int(x, 0), default=None,
                      help="CAN device id for --transport can")

    p_mon = sub.add_parser("monitor", help="tail UART (and optionally SWO)")
    p_mon.add_argument("--swo", action="store_true",
                       help="also run OpenOCD ITM/SWO trace")

    p_all = sub.add_parser("all", help="doctor + up + flash + monitor")
    p_all.add_argument("--transport", choices=["swd", "can"], default="swd")
    p_all.add_argument("--images", nargs="+", default=None,
                       choices=list(IMAGES.keys()))
    p_all.add_argument("--debug", action="store_true")
    p_all.add_argument("--image", default=None)
    p_all.add_argument("--addr", type=lambda x: int(x, 0),
                      default=DEFAULT_FLASH_ADDR)
    p_all.add_argument("--device-id", type=lambda x: int(x, 0), default=None)
    p_all.add_argument("--swo", action="store_true")
    p_all.add_argument("--force", action="store_true")
    p_all.add_argument("--ignore-missing", action="store_true",
                       help="continue even if doctor finds missing hardware")

    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "doctor":
        return cmd_doctor(args)
    if args.command == "up":
        return cmd_up(args)
    if args.command == "flash":
        return cmd_flash(args)
    if args.command == "monitor":
        return cmd_monitor(args)
    if args.command == "all":
        return cmd_all(args)
    return 1


if __name__ == "__main__":
    sys.exit(main())
