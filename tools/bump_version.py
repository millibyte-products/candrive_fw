#!/usr/bin/env python3
"""
Bump the candrive firmware version.

Reads `[workspace.package].version` from `fw/Cargo.toml`, bumps it
according to the chosen rule, writes it back, and prints the new
version on stdout.

Usage:
  tools/bump_version.py patch          # 0.9.0 -> 0.9.1   (default)
  tools/bump_version.py minor          # 0.9.5 -> 0.10.0
  tools/bump_version.py major          # 0.9.5 -> 1.0.0
  tools/bump_version.py set 1.2.3      # explicit
  tools/bump_version.py show           # just print the current version

Used by:
  * humans cutting an interim bump
  * .github/workflows/release.yml after a release tag is published

The script touches Cargo.toml only — it does not commit or push.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WORKSPACE_TOML = ROOT / "fw" / "Cargo.toml"

_SECTION_RE = re.compile(r"(\[workspace\.package\][^\[]*)", re.DOTALL)
_VERSION_RE = re.compile(r'(version\s*=\s*")([^"]+)(")')


def read_version(toml_path: Path = WORKSPACE_TOML) -> tuple[str, str, str]:
    """Return (full_text, section_text, current_version)."""
    text = toml_path.read_text()
    m = _SECTION_RE.search(text)
    if not m:
        raise RuntimeError(f"no [workspace.package] section in {toml_path}")
    section = m.group(1)
    vm = _VERSION_RE.search(section)
    if not vm:
        raise RuntimeError(f"no version=\"…\" in [workspace.package]")
    return text, section, vm.group(2)


def bump(version: str, kind: str) -> str:
    parts = version.split(".")
    if len(parts) != 3 or not all(p.isdigit() for p in parts):
        raise ValueError(f"non-semver version {version!r}")
    major, minor, patch = (int(p) for p in parts)
    if kind == "patch":
        patch += 1
    elif kind == "minor":
        minor += 1; patch = 0
    elif kind == "major":
        major += 1; minor = 0; patch = 0
    else:
        raise ValueError(f"unknown bump kind {kind!r}")
    return f"{major}.{minor}.{patch}"


def write_version(new_version: str, toml_path: Path = WORKSPACE_TOML) -> None:
    text, section, current = read_version(toml_path)
    new_section = _VERSION_RE.sub(rf'\g<1>{new_version}\g<3>', section, count=1)
    new_text = text.replace(section, new_section, 1)
    toml_path.write_text(new_text)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    for kind in ("patch", "minor", "major"):
        sp = sub.add_parser(kind, help=f"bump the {kind} component")
        sp.set_defaults(func=lambda a, k=kind: do_bump(k))
    sp = sub.add_parser("set", help="set an explicit version (X.Y.Z)")
    sp.add_argument("version")
    sp.set_defaults(func=lambda a: do_set(a.version))
    sp = sub.add_parser("show", help="print the current version")
    sp.set_defaults(func=lambda _a: do_show())
    args = p.parse_args(argv)
    return args.func(args)


def do_show() -> int:
    _, _, v = read_version()
    print(v)
    return 0


def do_bump(kind: str) -> int:
    _, _, current = read_version()
    new = bump(current, kind)
    write_version(new)
    print(new)
    return 0


def do_set(target: str) -> int:
    if not re.fullmatch(r"\d+\.\d+\.\d+", target):
        print(f"error: not a semver version: {target!r}", file=sys.stderr)
        return 2
    write_version(target)
    print(target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
