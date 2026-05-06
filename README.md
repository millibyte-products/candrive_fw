# candrive_fw

Firmware for the candrive motor-control board: an STM32F103-based
brushless servo driver with a CAN interface, an SPI absolute encoder
(MT6701), and a gate driver for a 3-phase BLDC. The repository also
contains the host-side CLI used to talk to the board over CAN.

![CI](https://github.com/millibyte-products/candrive_fw/actions/workflows/ci.yml/badge.svg)

## Layout

```
fw/             Rust embedded firmware (Cargo workspace, thumbv7m-none-eabi)
  bootloader/   4 KiB bootloader at 0x0800_0000
  common/       4 KiB shared API table at 0x0800_2000
  app/          ~51 KiB application at 0x0800_3400
  shared/       no_std crate shared between app/bootloader/host
tools/          Host CLI + Rust library (Linux, SocketCAN). `motor_cli.py`
                etc. are convenience wrappers over the CLI binary.
legacy/         Original C++/PlatformIO firmware (frozen, reference only)
legacy_c/       Intermediate bare-metal C port (frozen, reference only)
vendor/         CMSIS + STM32F1 HAL sources used by legacy_c
build/          Output ELFs/BINs/HEXes (created by `make`)
dev_setup.py    Bring-up helper (CAN up, SWD flash, UART monitor)
Makefile        Orchestrates `cargo build` + objcopy for the three images
```

## Memory map

| Image      | Address       | Size     |
|------------|---------------|----------|
| bootloader | `0x0800_0000` | 4 KiB    |
| common     | `0x0800_2000` | 4 KiB    |
| app        | `0x0800_3400` | ~51 KiB  |

`common` exposes a fixed API table at offset 0 of its slot so the app
and bootloader can share code (CRC, flash helpers, USART, etc.) without
relinking. Each crate has its own `memory.x` / linker script.

## Prerequisites

Host (Linux):

- Rust stable toolchain with the `thumbv7m-none-eabi` target:
  `rustup target add thumbv7m-none-eabi`
- `arm-none-eabi-binutils` (for `arm-none-eabi-objcopy` and `-size`)
- `openocd` (for SWD flashing via the bundled `make flash*` targets)
- A SocketCAN-capable USB-CAN adapter (CANable2, Peak, etc.) and
  `iproute2` (`ip` command)

For development convenience: an STLinkV3 on the SWD header and a
USB-UART on Serial3 (115200 8N1).

## Build

```sh
make                 # release build of bootloader + common + app
make PROFILE=debug   # debug build
make size            # text/data/bss for all three ELFs
make test            # run host-side tests for the shared crate
make clean
```

Per-image rebuild:

```sh
cd fw && cargo build --release -p candrive-app      # or -bootloader / -common
```

## Flashing

The simplest path is the dev helper, which brings up CAN, programs all
three images over SWD, and tails UART:

```sh
sudo ./dev_setup.py up                    # bring can0 up at 1 Mbit/s
./dev_setup.py flash                      # SWD flash all three images
./dev_setup.py monitor                    # UART tail (+ optional SWO)
./dev_setup.py all                        # doctor + up + flash + monitor
```

Make targets (one-off OpenOCD invocations):

```sh
make flash-bootloader
make flash-common
make flash-app
make flash               # all three in one OpenOCD session
```

### Rapid iteration over CAN

Once the bootloader + common are programmed via SWD, the app can be
re-flashed over CAN in one command — no probe, no cables to swap:

```sh
make flash-can                    # rebuild app, push over can0 to device id 5
make flash-can CAN_IFACE=can1     # use a different SocketCAN iface
```

This rebuilds [build/app.bin](build/app.bin) and runs
`tools/fw_update.py --trigger`, which:

1. Sends the running app a `FirmwareUpdate` request over CAN, causing
   it to reboot into the bootloader.
2. Re-discovers the device, streams the new binary, verifies the CRC,
   and waits for the rebooted app to come back up.

Equivalent path through `dev_setup.py`:

```sh
./dev_setup.py flash --transport can            # rebuild app + flash over CAN
```

## Host CLI

A Rust CLI lives in `tools/`. Build once with `cd tools && cargo build
--release`; the binary is `tools/target/release/candrive-cli`. The
Python wrappers (`motor_cli.py`, `fw_update.py`, `position_battery.py`,
…) drive it for common operations:

```sh
./tools/motor_cli.py get-param 100        # read an arbitrary debug param
./tools/motor_cli.py save-params          # persist motor params to flash
./tools/fw_update.py path/to/app.bin      # OTA update over CAN
```

CAN bus: 1 Mbit/s, default device ID 1.

### `motor_cli.py`

[tools/motor_cli.py](tools/motor_cli.py) is the day-to-day bring-up
and debugging tool. It speaks the binary CAN protocol directly via
SocketCAN (no separate Rust binary required) and assumes the target
is already discovered at device ID **5** on `can0`. Override the
interface with `--iface canX`; the device ID is currently hard-coded
in the script (`DEV_ID = 5`).

Bring the bus up first (once per boot):

```sh
sudo ip link set can0 up type can bitrate 1000000
```

Then run any of the subcommands below. Every subcommand prints a
single line summarizing the firmware reply and exits non-zero on
timeout.

#### Discovery / identity

| Command | Purpose |
|---|---|
| `verify [--device-id N] [--listen S] [--expect-serial 0xXXXX]` | Post-flash sanity check. Optionally listens for a `DiscoveryReq` from a freshly booted device, assigns it `--device-id`, then issues `GetInfo` and probes `GetStatus` to distinguish app vs. bootloader. Returns rc=2 on no-reply, rc=3 on serial mismatch. |
| `network-reset` | Broadcast on CAN ID 0: every device on the bus drops its assigned ID and re-enters discovery. |
| `erase-user-store` | Factory reset: erase saved device ID **and** saved motor params (USER_STORE 1 KiB page). |
| `revoke-config --serial 0xCAFEBABE` | Drop only the assigned device ID (params kept). `--serial` must match the target's `serial_no` (factory default `0xCAFEBABE`). |

#### Telemetry / inspection

| Command | Reads |
|---|---|
| `get-position` | Single-turn encoder angle, raw Q-format counts and radians. |
| `get-status` | Status byte: endstop0/1, misc, fault, MT6701 magnet status nibble (`strong`/`weak`/`no_mag`/`push`). |
| `get-analog` | A0 / A1 ADC inputs, raw 12-bit counts and volts. |
| `get-servo` | Current SRV0 / SRV1 pulse widths (µs). |
| `get-motor` | Bridge-enable state (`NRST` / `NSLEEP`); `bridge_enabled` is true only when both are high. |
| `get-foc` | Live per-phase PWM duties (`phase_a`, `phase_c`, `phase_b` out of 255) and FOC enable flag. |
| `get-led` | SYS / STAT LED duty cycles (0–100%). |

#### Direct actuation

| Command | Notes |
|---|---|
| `set-motor on \| off` | Force the gate-driver bridge enable / disable. Required before any closed-loop move. |
| `set-position TARGET_RAD` | Absolute-shortest move to `TARGET_RAD`. Wrapped to `[0, 2π)` and encoded as Q16 radians. |
| `set-servo [--s0 US] [--s1 US] [--mask 0xN]` | Set SRV0/SRV1 pulse widths in µs (0 = off). The update mask is auto-built from whichever flags you passed; override with `--mask`. |
| `set-led [--sys 0..100] [--stat 0..100] [--mask 0xN]` | Set LED duty cycles. Auto-mask follows the same rule as `set-servo`. |
| `move MODE TARGET` | Set closed-loop control mode + target in one shot. `MODE` ∈ `idle`, `voltage`, `velocity`, `position`, `openloop`, `abs-shortest`, `abs-forward`, `abs-backward`, `relative`. `TARGET` units depend on the mode (volts for `voltage`, radians for the position modes, etc.). |

#### Motor parameters & calibration

| Command | Purpose |
|---|---|
| `get-param INDEX` | Read motor param / debug param `INDEX` as f32. Indices < ~32 are tunable params; ≥ 100 are read-only debug telemetry (e.g. param 200 = top byte of `RCC_CSR`). |
| `set-param INDEX VALUE` | Write a tunable param (RAM only). Use `save-params` afterwards to persist. |
| `calibrate FREQ_DHZ DUR_MS VALIGN_PCT` | Open-loop spin-up → forward + reverse sweep → reports estimated pole-pairs, encoder zero offset, electrical direction (+1 / −1), and a fault flag. `FREQ_DHZ` is electrical frequency in tenths of Hz, `DUR_MS` per-sweep duration, `VALIGN_PCT` voltage as % of Vbus. Typical: `calibrate 10 4000 25`. The CLI scales its reply timeout with `DUR_MS`. |
| `save-params` | Commit the current in-RAM motor params (including calibration results) to the USER_STORE flash page. |

#### Quick test sequence

`quick-test` exercises closed-loop position control by stepping
through ±90° increments forward and then reversing the sequence,
holding each step for `--dwell` seconds and reading back the encoder
after each move. It auto-asserts the bridge with `set-motor on`
before stepping, and forces the controller back to `idle` on exit
(including Ctrl-C) so the rotor never stays under torque after the
test ends.

```sh
./tools/motor_cli.py quick-test                            # one fwd+reverse cycle
./tools/motor_cli.py quick-test --dwell 1.0 --loops 5      # five cycles, 1 s/step
./tools/motor_cli.py quick-test --mode abs-shortest        # absolute 0/90/180/270°
```

Options:

- `--dwell SECONDS` (default `0.5`) — settle time per step.
- `--mode` (default `relative`):
  - `relative` — issue ±π/2 deltas from the current rotor angle. Works
    without a calibrated encoder zero; this is the safe default for
    bring-up.
  - `abs-shortest` / `abs-forward` / `abs-backward` — target the
    cardinal angles 0°, 90°, 180°, 270° in absolute multi-turn space.
    These **require** a prior successful `calibrate` (and ideally
    `save-params`); without calibration the FOC has no
    encoder→electrical mapping and the rotor will freewheel.
- `--loops N` (default `1`) — repeat the full forward+reverse
  sequence `N` times.

## Production flashing

For programming new units on the bench (bootloader + common + app +
a freshly allocated serial number written into `USER_STORE`), use:

```sh
./tools/production_flash.py monitor
```

The ST-Link adapter stays connected; the script polls the SWD bus for
a target's 96-bit STM32 factory UID (`0x1FFFF7E0`) and runs one full
flash pass per fresh UID. Re-flashing a unit whose UID already has an
`ok` row in the DB is refused unless `--force` is passed, so the same
chip can't accidentally be re-provisioned with a new serial.

Each unit's serial, STM32 UID, image hashes, git SHA, operator, and
timestamps are recorded in the SQLite database at
`/mnt/bulk/backup/documents/candrive_fw_serials_prod.db` (override with
`--db`). Subcommands: `flash` (one-shot), `monitor` (loop), `list`,
`show <serial>`, `lookup <uid>`, `read-uid` (print the attached
target's UID without flashing), `preview <serial>` (dump the
user_store bytes without flashing or DB writes).

A systemd unit at [tools/systemd/candrive-prod-flash.service](tools/systemd/candrive-prod-flash.service)
runs `monitor` as a daemon for fully hands-off operation:

```sh
sudo cp tools/systemd/candrive-prod-flash.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now candrive-prod-flash.service
journalctl -u candrive-prod-flash -f
```

The user_store record format mirrors `fw/shared/src/user_store.rs`:
28-byte slot with magic `"USRC"`, serial number, and a CRC-32/MPEG-2
checksum, padded with `0xFF` to fill the 1 KiB flash page.

## Continuous integration

`.github/workflows/ci.yml` runs on every push to `main` and on every
pull request. It installs the ARM toolchain and stable Rust with the
`thumbv7m-none-eabi` target, then runs:

```
make            # builds bootloader + common + app
make size       # reports image sizes
make test       # shared-crate host tests
cd tools && cargo build --release
```

## Releases

The firmware version lives in a single place — `[workspace.package]`
in [fw/Cargo.toml](fw/Cargo.toml) — and is inherited by every crate.
The `app`/`bootloader` build scripts also bake the short git SHA in
via `env!("CANDRIVE_GIT_SHA")`, so the boot banner and the `GetInfo`
reply both report the version the firmware was actually built from.

Cutting a release:

```sh
tools/bump_version.py set 0.9.0          # or `tools/bump_version.py minor`
git commit -am "release: v0.9.0"
git tag v0.9.0
git push origin main v0.9.0
```

`.github/workflows/release.yml` triggers on `v*.*.*` tags, validates
the tag against the workspace version, builds the firmware, computes
SHA-256 sums, and creates a GitHub release with `bootloader`,
`common`, and `app` (`.elf`/`.bin`/`.hex`) attached. After a successful
release built from `main`, a follow-up commit auto-bumps the patch
component on `main` so development continues at the next version.

`production_flash.py` records `fw_version` and the git SHA into every
DB row, so each provisioned unit is traceable back to a specific
release artifact.

## Hardware notes

- Encoder: MT6701 over SPI mode 0. CRC reads as mismatched (cosmetic);
  glitches are filtered in `fw/app/src/control.rs` with an integer
  multi-turn accumulator.
- Watchdog: IWDG ~500 ms, petted in the main loop and in calibration
  hold loops.
- Reset cause: top byte of `RCC_CSR` is exposed via UART boot banner
  and CAN debug param 200.

## License

See [LICENSE](LICENSE).
