# JVM410 Firmware

This project contains firmware for the ATmega8A-based Marshall JVM410 hardware, written in AVR assembly and built with PlatformIO.

## What is this project?

The firmware in `src/main.S` controls the device logic for the JVM410 hardware and uses:

- AVR assembly (`.S`) source code
- PlatformIO for building and flashing
- a post-build script (`generate_eeprom.py`) to generate EEPROM data output

The target configuration is defined in `platformio.ini` for the `ATmega8` board running at 4 MHz.

## Project structure

- `src/main.S` — main firmware source in AVR assembly
- `generate_eeprom.py` — post-build script that extracts EEPROM data into `.eep`
- `platformio.ini` — PlatformIO configuration for the `atmega8_asm` environment
- `include/` — additional headers or support files
- `lib/` — custom libraries
- `test/` — tests and project notes

## Requirements

To build this project, you need:

- Python 3
- PlatformIO Core (`pio`)
- AVR toolchain available through PlatformIO

## Build

From the project root, run:

```sh
pio run
```

This will build the firmware and generate artifacts in `.pio/build/atmega8_asm/`, including:

- `firmware.hex`
- `firmware.elf`
- `firmware.eep`

## EEPROM generation

The project uses the post-build hook in `generate_eeprom.py` to create an EEPROM image from the assembled firmware. This is useful when the firmware uses `.eeprom` sections that must be programmed separately.

## CI/CD

The repository includes a GitHub Actions workflow in `.github/workflows/compile.yml` that:

- installs PlatformIO
- runs `pio run`
- uploads firmware artifacts on pushes to `master`
- prepares release assets for tagged releases

## Notes

This repository is focused on firmware development for the ATmega8A / JVM410 platform. If you plan to flash hardware, ensure that the device configuration and programmer setup match your environment.
