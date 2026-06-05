# BSides Dublin 2026 — Harp Badge

Source code, hardware design files, and manufacturing data for the BSides Dublin harp badge.

## Contents

```
firmware/
  application/       USB CDC application — touch, LEDs, buzzer, CLI, CTF
  bootloader/         TinyUSB DFU bootloader
  Makefile            unified build (merges bootloader + application)

production_files/     gerbers, BOM, pick-and-place, JLCPCB firmware package

docs/
  BADGE.md            pinout, memory map, boot sequence, component list

tools/
  touch_log.py        serial touch sensor logger
```

## Hardware

| | |
|---|---|
| MCU | STM32L053R8Tx — Cortex-M0+, 64 KB flash, 8 KB RAM |
| Touch | 6 capacitive strings (TSC) |
| LEDs | 7x yellow, PWM-controlled |
| Sound | Piezo buzzer, two octaves (D5–D7) |
| Power | CR2032 or USB, auto-detected |
| USB | USB-C, crystal-less (HSI48 + CRS) |

## Memory Layout

| Region | Address | Size |
|---|---|---|
| Bootloader | `0x08000000` | 16 KB |
| Application | `0x08004000` | 48 KB |
| RAM | `0x20000000` | 8 KB |
| Data EEPROM | `0x08080000` | 2 KB |

## Building

Requires `arm-none-eabi-gcc` and `make`.

```bash
# application only
cd firmware/application && make

# unified factory image (bootloader + application)
cd firmware && make
```

## Flashing

### USB DFU (recommended)
The bootloader cannot erase itself, a failed application flash is always recoverable.

```bash
dfu-util -a 0 -D firmware/application/build/stm32l053-app.bin -R -w
```

### SWD (factory programming, requires st-link)

```bash
cd firmware && make flash
```

## Recovery

If the application is corrupted and the CLI is unreachable:

1. Unplug USB
2. Run the `dfu-util` command above (`-w` makes it wait)
3. Set the power switch to USB/OFF
4. Plug in — the bootloader handles the rest

To force bootloader mode manually: hold **strings 1 and 6** while plugging in USB.

## Serial CLI

```bash
screen /dev/ttyACM0 9600
```
Baud rate doesn't matter, any value works. 
Type `help` for the command list.

4 CTF flags are hidden in the badge.

## References

- Hardware details: [`docs/BADGE.md`](docs/BADGE.md)
- Manufacturing: [`production_files/README.md`](production_files/README.md)
- Touch diagnostics: `python3 tools/touch_log.py /dev/ttyACM0`
