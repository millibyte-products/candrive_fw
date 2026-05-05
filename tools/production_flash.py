#!/usr/bin/env python3
"""
candrive production flashing helper.

Programs `bootloader.bin`, `common.bin`, an auto-generated `user_store`
page (containing a freshly allocated serial number), and `app.bin` over
SWD using OpenOCD + an ST-Link adapter, then records the unit in a
SQLite database for traceability.

Subcommands:

  flash    : program one device that is already attached. The next free
             serial number is allocated atomically from the database and
             written into the USER_STORE flash page.
  monitor  : watch for ST-Link adapters being plugged in and run `flash`
             once for each fresh insertion. This is the press-and-flash
             production loop.
  list     : dump recent serials from the DB.
  show     : show one serial's record.
  preview  : compute the user_store bytes for a given serial and dump
             them as hex (no flashing, no DB write). Useful for
             validating CRC byte-for-byte against the firmware.

Examples:

  ./tools/production_flash.py flash
  ./tools/production_flash.py monitor
  ./tools/production_flash.py list --limit 20
  ./tools/production_flash.py preview 0xCD000007

Design notes:

  * The user_store record format mirrors `fw/shared/src/user_store.rs`
    exactly: 28 bytes
        magic u32 (0x43525355 "USRC")
        version u16 (1)
        flags u16 (0)
        seq u32 (0 — first record)
        serial_no u32
        assigned_id u8 (0xFF — UNASSIGNED, host will set via discovery)
        _pad [u8; 7] (zeros)
        crc32 u32 (CRC-32/MPEG-2 over leading 24 bytes)
    padded with 0xFF up to one full 1 KiB flash page.

  * Atomic serial allocation: every flash attempt opens a transaction,
    picks `MAX(serial_no, FACTORY_SERIAL) + 1`, and INSERTs a row with
    status='in_progress'. The UNIQUE constraint on serial_no makes
    collisions a hard error rather than silent overwrite. On failure
    the row is updated to status='failed' so the serial is not reused.

  * OpenOCD selects a specific adapter when multiple ST-Links are
    attached via `adapter serial <hexserial>`. The hex-serial of each
    plugged adapter is read from sysfs.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import os
import re
import shutil
import signal
import sqlite3
import struct
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

# --- Repo / firmware layout (mirrors fw/shared/src/{flash_layout,user_store}.rs) ---

ROOT = Path(__file__).resolve().parent.parent
BUILD_DIR = ROOT / "build"

BOOTLOADER_BIN = BUILD_DIR / "bootloader.bin"
COMMON_BIN     = BUILD_DIR / "common.bin"
APP_BIN        = BUILD_DIR / "app.bin"

ADDR_BOOTLOADER = 0x0800_0000
ADDR_COMMON     = 0x0800_2000
ADDR_USER_STORE = 0x0800_3000
ADDR_APP        = 0x0800_3400
USER_STORE_SIZE = 1024  # one F1 erase page

USER_STORE_MAGIC   = 0x4352_5355  # "USRC" LE
USER_STORE_VERSION = 1
SLOT_RAW_LEN       = 28
FACTORY_SERIAL     = 0xCD00_0001  # reserved; production starts at +1
UNASSIGNED_ID      = 0xFF

DEFAULT_DB_PATH = Path("/mnt/bulk/backup/documents/candrive_fw_serials_prod.db")

# ST-Link USB IDs (V2 / V2-1 / V3 family).
STLINK_VENDOR = "0483"
STLINK_PRODUCTS = {
    "3744",  # ST-Link/V1
    "3748",  # ST-Link/V2
    "374a",  # ST-Link/V2-1 (mass storage)
    "374b",  # ST-Link/V2-1
    "374d",  # ST-Link/V2 (DFU)
    "374e",  # ST-Link/V3 (no MSD)
    "374f",  # ST-Link/V3 (MSD)
    "3752",  # ST-Link/V3
    "3753",  # ST-Link/V3
    "3754",  # ST-Link/V3
    "3755",
}

# --- ANSI helpers ------------------------------------------------------------

ANSI = {"red":"\033[31m","green":"\033[32m","yellow":"\033[33m",
        "cyan":"\033[36m","dim":"\033[2m","bold":"\033[1m","reset":"\033[0m"}

def _c(name: str, s: str) -> str:
    return f"{ANSI.get(name,'')}{s}{ANSI['reset']}" if sys.stdout.isatty() else s

def info(m: str)  -> None: print(_c("cyan",  "[*] ") + m)
def ok(m: str)    -> None: print(_c("green", "[+] ") + m)
def warn(m: str)  -> None: print(_c("yellow","[!] ") + m)
def err(m: str)   -> None: print(_c("red",   "[x] ") + m, file=sys.stderr)

# --- CRC-32/MPEG-2 (matches fw/shared/src/crc32_mpeg2.rs) --------------------

def crc32_mpeg2(data: bytes) -> int:
    crc = 0xFFFF_FFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C1_1DB7) & 0xFFFF_FFFF if crc & 0x8000_0000 \
                  else (crc << 1) & 0xFFFF_FFFF
    return crc

# --- user_store image generator ---------------------------------------------

def build_user_store_record(serial_no: int,
                             assigned_id: int = UNASSIGNED_ID,
                             seq: int = 0,
                             flags: int = 0) -> bytes:
    """Return the 28-byte slot record with a valid CRC."""
    if not (0 < serial_no <= 0xFFFF_FFFF):
        raise ValueError(f"serial_no out of range: {serial_no:#x}")
    body = struct.pack("<IHHIIB7s",
                       USER_STORE_MAGIC,
                       USER_STORE_VERSION,
                       flags,
                       seq,
                       serial_no,
                       assigned_id & 0xFF,
                       b"\x00" * 7)
    assert len(body) == SLOT_RAW_LEN - 4, len(body)
    crc = crc32_mpeg2(body)
    return body + struct.pack("<I", crc)

def build_user_store_page(serial_no: int) -> bytes:
    """28-byte record padded to a full 1 KiB erase page with 0xFF."""
    record = build_user_store_record(serial_no)
    return record + b"\xFF" * (USER_STORE_SIZE - len(record))

# --- ST-Link discovery via sysfs --------------------------------------------

@dataclass(frozen=True)
class StLink:
    sysfs: str            # e.g. /sys/bus/usb/devices/1-2
    product: str          # 4-hex-char USB product id
    serial: str           # adapter serial (typed into `adapter serial ...`)

def _read(path: str) -> Optional[str]:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None

def list_stlinks() -> list[StLink]:
    out: list[StLink] = []
    base = "/sys/bus/usb/devices"
    if not os.path.isdir(base):
        return out
    for name in os.listdir(base):
        node = os.path.join(base, name)
        v = _read(os.path.join(node, "idVendor"))
        if v != STLINK_VENDOR:
            continue
        p = _read(os.path.join(node, "idProduct"))
        if not p or p.lower() not in STLINK_PRODUCTS:
            continue
        s = _read(os.path.join(node, "serial")) or ""
        out.append(StLink(sysfs=node, product=p.lower(), serial=s))
    out.sort(key=lambda x: x.serial)
    return out

# --- DB ----------------------------------------------------------------------

SCHEMA = """
CREATE TABLE IF NOT EXISTS devices (
    serial_no       INTEGER PRIMARY KEY,
    status          TEXT NOT NULL CHECK (status IN ('in_progress','ok','failed')),
    assigned_id     INTEGER,
    started_at      TEXT NOT NULL,
    finished_at     TEXT,
    duration_ms     INTEGER,
    operator        TEXT,
    host            TEXT,
    stlink_serial   TEXT,
    git_sha         TEXT,
    git_dirty       INTEGER,
    bootloader_sha  TEXT,
    common_sha      TEXT,
    app_sha         TEXT,
    user_store_sha  TEXT,
    error           TEXT,
    notes           TEXT
);
CREATE INDEX IF NOT EXISTS idx_devices_status ON devices(status);
CREATE INDEX IF NOT EXISTS idx_devices_started ON devices(started_at);
"""

def open_db(path: Path) -> sqlite3.Connection:
    path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(path), isolation_level=None, timeout=30.0)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA synchronous=FULL")
    conn.executescript(SCHEMA)
    return conn

def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()) + "Z"

def allocate_serial(conn: sqlite3.Connection, *,
                    stlink_serial: str,
                    operator: str,
                    host: str,
                    git_sha: str,
                    git_dirty: bool) -> int:
    """Pick the next free serial under a write transaction and INSERT a
    row in status='in_progress'. Returns the serial."""
    cur = conn.cursor()
    cur.execute("BEGIN IMMEDIATE")
    try:
        row = cur.execute("SELECT MAX(serial_no) FROM devices").fetchone()
        cur_max = row[0] if row and row[0] is not None else 0
        next_serial = max(cur_max, FACTORY_SERIAL) + 1
        cur.execute(
            "INSERT INTO devices (serial_no,status,started_at,operator,host,"
            "stlink_serial,git_sha,git_dirty) "
            "VALUES (?,?,?,?,?,?,?,?)",
            (next_serial, "in_progress", now_iso(), operator, host,
             stlink_serial, git_sha, 1 if git_dirty else 0),
        )
        cur.execute("COMMIT")
        return next_serial
    except Exception:
        with contextlib.suppress(Exception):
            cur.execute("ROLLBACK")
        raise

def finalize_ok(conn: sqlite3.Connection, serial_no: int, *,
                duration_ms: int,
                bootloader_sha: str, common_sha: str,
                app_sha: str, user_store_sha: str) -> None:
    conn.execute(
        "UPDATE devices SET status='ok', finished_at=?, duration_ms=?, "
        "bootloader_sha=?, common_sha=?, app_sha=?, user_store_sha=? "
        "WHERE serial_no=?",
        (now_iso(), duration_ms, bootloader_sha, common_sha, app_sha,
         user_store_sha, serial_no),
    )

def finalize_failed(conn: sqlite3.Connection, serial_no: int, *,
                    duration_ms: int, error: str) -> None:
    conn.execute(
        "UPDATE devices SET status='failed', finished_at=?, duration_ms=?, "
        "error=? WHERE serial_no=?",
        (now_iso(), duration_ms, error, serial_no),
    )

# --- Helpers -----------------------------------------------------------------

def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()

def sha256_bytes(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()

def git_info() -> tuple[str, bool]:
    try:
        sha = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
        ).strip()
        dirty = subprocess.run(
            ["git", "diff", "--quiet"], cwd=ROOT,
        ).returncode != 0
        return sha, dirty
    except Exception:
        return "", False

def ensure_images_present() -> None:
    missing = [str(p) for p in (BOOTLOADER_BIN, COMMON_BIN, APP_BIN) if not p.is_file()]
    if missing:
        raise FileNotFoundError(
            "missing firmware images: " + ", ".join(missing) +
            " — run `make` first")

# --- OpenOCD flashing --------------------------------------------------------

OPENOCD = os.environ.get("OPENOCD", "openocd")
INTERFACE_CFG = "interface/stlink.cfg"
TARGET_CFG    = "target/stm32f1x.cfg"

def run_openocd_program(stlink_serial: str,
                        user_store_bin: Path,
                        timeout_s: float = 60.0) -> tuple[bool, str]:
    """One OpenOCD session that erases and programs all four regions.

    OpenOCD's `program` sub-command takes care of `init` + `reset halt`
    + erase + write + verify + reset internally."""
    pre = []
    if stlink_serial:
        # Must come before the target.cfg that calls `init`.
        pre += [f"adapter serial {stlink_serial}"]
    flash = [
        f"program {BOOTLOADER_BIN} 0x{ADDR_BOOTLOADER:08x} verify",
        f"program {COMMON_BIN} 0x{ADDR_COMMON:08x} verify",
        f"program {user_store_bin} 0x{ADDR_USER_STORE:08x} verify",
        f"program {APP_BIN} 0x{ADDR_APP:08x} verify",
    ]
    cmds = flash + ["reset run", "exit"]

    argv = [OPENOCD, "-f", INTERFACE_CFG]
    for c in pre:
        argv += ["-c", c]
    argv += ["-f", TARGET_CFG]
    for c in cmds:
        argv += ["-c", c]

    try:
        proc = subprocess.run(argv, text=True, capture_output=True,
                              timeout=timeout_s)
    except subprocess.TimeoutExpired as e:
        return False, f"openocd timed out after {timeout_s:.0f}s\n{e}"
    log = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode == 0, log

# --- Flash one device --------------------------------------------------------

def select_stlink(arg: Optional[str]) -> StLink:
    links = list_stlinks()
    if not links:
        raise RuntimeError("no ST-Link adapter found on USB")
    if arg:
        for l in links:
            if l.serial == arg:
                return l
        raise RuntimeError(f"ST-Link with serial {arg!r} not found "
                           f"(see: {[l.serial for l in links]})")
    if len(links) > 1:
        raise RuntimeError(
            "multiple ST-Links attached — pass --stlink-serial to "
            f"disambiguate: {[l.serial for l in links]}")
    return links[0]

def cmd_flash(args: argparse.Namespace) -> int:
    ensure_images_present()
    db = open_db(args.db)
    link = select_stlink(args.stlink_serial)
    info(f"using ST-Link serial {link.serial!r} (USB pid {link.product})")

    sha, dirty = git_info()
    serial = allocate_serial(db,
                             stlink_serial=link.serial,
                             operator=args.operator or os.environ.get("USER",""),
                             host=os.uname().nodename,
                             git_sha=sha, git_dirty=dirty)
    info(f"allocated serial {serial:#010x} (decimal {serial})")

    user_store_page = build_user_store_page(serial)
    user_store_sha  = sha256_bytes(user_store_page)

    t0 = time.monotonic()
    with tempfile.NamedTemporaryFile(prefix=f"user_store_{serial:08x}_",
                                     suffix=".bin", delete=False) as tf:
        tf.write(user_store_page)
        tmp_path = Path(tf.name)
    try:
        ok_, log = run_openocd_program(link.serial, tmp_path,
                                       timeout_s=args.openocd_timeout)
    finally:
        with contextlib.suppress(OSError):
            tmp_path.unlink()
    dt_ms = int((time.monotonic() - t0) * 1000)

    if not ok_:
        # Capture the tail of the log as the error message.
        tail = "\n".join(log.splitlines()[-30:]) if log else "<no openocd output>"
        finalize_failed(db, serial, duration_ms=dt_ms,
                        error=tail[-3500:])
        err(f"OpenOCD failed for serial {serial:#010x} after {dt_ms} ms")
        if args.verbose:
            print(log, file=sys.stderr)
        else:
            print(_c("dim", tail), file=sys.stderr)
        return 2

    finalize_ok(db, serial, duration_ms=dt_ms,
                bootloader_sha=sha256_file(BOOTLOADER_BIN),
                common_sha=sha256_file(COMMON_BIN),
                app_sha=sha256_file(APP_BIN),
                user_store_sha=user_store_sha)
    ok(f"flashed serial {_c('bold', f'{serial:#010x}')} "
       f"(decimal {serial}) in {dt_ms} ms")
    return 0

# --- Monitor mode ------------------------------------------------------------

def cmd_monitor(args: argparse.Namespace) -> int:
    ensure_images_present()
    info(f"watching for ST-Link insertions; DB={args.db}")
    info("Ctrl-C to exit")

    seen: set[str] = {l.serial for l in list_stlinks() if l.serial}
    if seen and not args.flash_existing:
        info(f"already attached at startup (skipped): {sorted(seen)}")
    elif seen and args.flash_existing:
        # Treat them as freshly inserted.
        seen.clear()

    stop = False
    def _stop(*_a):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    while not stop:
        time.sleep(args.poll_interval)
        cur = list_stlinks()
        cur_serials = {l.serial for l in cur if l.serial}
        new = cur_serials - seen
        gone = seen - cur_serials
        for g in gone:
            info(f"ST-Link removed: {g}")
        for s in sorted(new):
            link = next((l for l in cur if l.serial == s), None)
            if link is None:
                continue
            info(f"ST-Link inserted: {s} (pid {link.product}) — flashing…")
            # Settle a moment for the kernel to finish enumeration.
            time.sleep(args.settle_delay)
            ns = argparse.Namespace(**vars(args))
            ns.stlink_serial = s
            try:
                rc = cmd_flash(ns)
            except Exception as exc:
                err(f"flash exception: {exc}")
                rc = 99
            if rc == 0:
                ok("ready for next unit — unplug + plug the next ST-Link")
            else:
                err(f"flash returned {rc}; unit logged as failed")
        seen = cur_serials
    info("monitor exiting")
    return 0

# --- list / show / preview ---------------------------------------------------

def cmd_list(args: argparse.Namespace) -> int:
    db = open_db(args.db)
    rows = db.execute(
        "SELECT serial_no,status,started_at,finished_at,duration_ms,"
        "stlink_serial,operator,git_sha "
        "FROM devices ORDER BY serial_no DESC LIMIT ?",
        (args.limit,),
    ).fetchall()
    if not rows:
        info("no records yet")
        return 0
    print(f"{'serial':>10}  {'status':<12} {'started_at':<21} "
          f"{'dur_ms':>7}  {'op':<10} {'stlink':<14} git")
    for sn, status, started, _finished, dur, stlink, op, sha in rows:
        sn_s = f"0x{sn:08X}"
        print(f"{sn_s:>10}  {status:<12} {started or '':<21} "
              f"{(dur or 0):>7}  {(op or ''):<10} "
              f"{(stlink or ''):<14} {(sha or '')[:10]}")
    return 0

def cmd_show(args: argparse.Namespace) -> int:
    db = open_db(args.db)
    sn = parse_int(args.serial)
    row = db.execute(
        "SELECT * FROM devices WHERE serial_no=?", (sn,),
    ).fetchone()
    if not row:
        err(f"no record for serial {sn:#010x}")
        return 1
    cols = [c[0] for c in db.execute("SELECT * FROM devices LIMIT 0").description]
    width = max(len(c) for c in cols)
    for c, v in zip(cols, row):
        if c == "serial_no" and isinstance(v, int):
            v = f"0x{v:08X} ({v})"
        print(f"{c:<{width}}  {v}")
    return 0

def cmd_preview(args: argparse.Namespace) -> int:
    sn = parse_int(args.serial)
    rec = build_user_store_record(sn)
    page = build_user_store_page(sn)
    print(f"# serial 0x{sn:08X}")
    print(f"# record (28 bytes):")
    print("  " + " ".join(f"{b:02x}" for b in rec))
    print(f"# crc32: 0x{int.from_bytes(rec[-4:], 'little'):08x}")
    print(f"# full page sha256: {sha256_bytes(page)}")
    return 0

def parse_int(s: str) -> int:
    s = s.strip()
    return int(s, 0)

# --- argparse ---------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--db", type=Path, default=DEFAULT_DB_PATH,
                   help=f"sqlite DB path (default {DEFAULT_DB_PATH})")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="dump full openocd output on success/failure")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("flash", help="program one attached unit")
    sp.add_argument("--stlink-serial", help="select a specific ST-Link "
                    "(required when more than one is plugged in)")
    sp.add_argument("--operator", help="record an operator name in the DB "
                    "(default: $USER)")
    sp.add_argument("--openocd-timeout", type=float, default=60.0)
    sp.set_defaults(func=cmd_flash)

    sp = sub.add_parser("monitor", help="press-and-flash production loop")
    sp.add_argument("--operator")
    sp.add_argument("--openocd-timeout", type=float, default=60.0)
    sp.add_argument("--poll-interval", type=float, default=0.5,
                    help="USB polling interval in seconds (default 0.5)")
    sp.add_argument("--settle-delay", type=float, default=0.7,
                    help="seconds to wait after a new ST-Link appears "
                    "before invoking openocd (default 0.7)")
    sp.add_argument("--flash-existing", action="store_true",
                    help="also flash ST-Links already attached at startup")
    sp.set_defaults(func=cmd_monitor)

    sp = sub.add_parser("list", help="list recent serials")
    sp.add_argument("--limit", type=int, default=20)
    sp.set_defaults(func=cmd_list)

    sp = sub.add_parser("show", help="show one serial record")
    sp.add_argument("serial", help="serial (decimal or 0x-hex)")
    sp.set_defaults(func=cmd_show)

    sp = sub.add_parser("preview", help="dump user_store bytes (no flash, no DB)")
    sp.add_argument("serial", help="serial (decimal or 0x-hex)")
    sp.set_defaults(func=cmd_preview)

    return p

def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except KeyboardInterrupt:
        warn("interrupted")
        return 130
    except Exception as exc:
        err(str(exc))
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())
