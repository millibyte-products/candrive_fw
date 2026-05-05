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
