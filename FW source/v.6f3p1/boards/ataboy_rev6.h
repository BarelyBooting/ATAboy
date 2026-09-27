/*
 * ATAboy Rev .6 (RP2354B: RP2350 in QFN-80 with 2 MB of flash stacked).
 * Palimpsest fork, holistic review 2026-09-26 (the owner's RP2354B guide,
 * L1). Until then the firmware was built as a pico2, whose board header
 * names GPIO 25 as its LED, GPIO 23 as the SMPS mode pin, GPIO 24 as VBUS
 * sense and GPIO 29 as VSYS sense. On this board those are CS1-, RESET-,
 * CS0- and IORDY. Nothing in this firmware used those names, but nothing
 * could now either: this header defines no LED, no UART, I2C or SPI default
 * pins, and no power pins, and ATAboy.c refuses to build if any is set.
 *
 * The rest is as pico2: RP2350 platform, the A-package pin count (every pin
 * this board uses is below 32, which is what the p4 to p9 builds ran with
 * on hardware), boot stage 2 for a W25Q080-type flash, the A2 flash
 * workaround kept. The flash is 2 MB (the p9 flashes' picotool info read
 * "flash size: 2048K"; config.c's sector is at 2 MB minus 4 KB).
 */
#ifndef _BOARDS_ATABOY_REV6_H
#define _BOARDS_ATABOY_REV6_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

#define ATABOY_REV6
// The pin count. GPIO30 and GPIO31 (JP1, JP2) and GPIO32 (the Err LED) exist
// in the B package only (RP2354B, QFN-80). Every other pin this board uses is
// below 30, and the builds without the jumpers keep the A-package count they
// were proven with, byte for byte (palimpsest api/SPEC-v1.md 1.3, 8.1). With
// ATABOY_BOARD_JUMPERS (CMakeLists.txt, which defines it for every source of
// the image, the SDK's included) the count is the true one, 48, and the PIO
// keeps the code it has always had: no GPIO base handling (every PIO pin is
// 0 to 29, and GPIOBASE is left at its reset value, 0, as in every build
// before; the SDK's default for 48 pins would change pio_sm_config and the
// SM set-up code for no pin this board has on a PIO).
#if ATABOY_BOARD_JUMPERS
#define PICO_RP2350A 0
#define PICO_PIO_USE_GPIO_BASE 0
#else
#define PICO_RP2350A 1
#endif

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (2 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
