# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Holistic review 2026-09-26 (the owner's RP2354B guide, L1): the board is
# the ATAboy Rev .6, not a pico2, whose header names GPIO 23, 24, 25 and 29
# (RESET-, CS0-, CS1-, IORDY here) as power pins and an LED. FORCE: a
# -DPICO_BOARD=pico2 on the command line (the CI workflow's, the README's)
# must not bring the pico2 header back.
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)
set(PICO_BOARD ataboy_rev6 CACHE STRING "Board type" FORCE)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ATAboy C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ATAboy
        ATAboy.c
        ide_pio.c
        ide.c
        menus.c
        usb.c
        usb_descriptors.c
        config.c
        fault.c)

pico_set_program_name(ATAboy "ATAboy")

# Generate PIO header
pico_generate_pio_header(ATAboy ${CMAKE_CURRENT_LIST_DIR}/ataboy.pio)

# No pico_stdio — all console I/O through TinyUSB CDC via queues
pico_enable_stdio_uart(ATAboy 0)
pico_enable_stdio_usb(ATAboy 0)

# Link libraries
target_link_libraries(ATAboy
        pico_stdlib
        pico_bootrom
        pico_multicore
        pico_unique_id
        hardware_flash
        hardware_sync
        hardware_pio
        hardware_dma
        hardware_clocks
        tinyusb_device
        tinyusb_board
        )

# Add the standard include files to the build
target_include_directories(ATAboy PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# palimpsest fork, holistic review 2026-09-26 (L-4): stacks. Core 0 gets all of
# SCRATCH_Y (4 KB; it peaked near 1.7 KB of the SDK's 2 KB), core 1 its own 8 KB
# (ATAboy.c), and both a stack limit (MSPLIM) at the bottom.
target_compile_definitions(ATAboy PRIVATE PICO_STACK_SIZE=0x1000 PICO_CORE1_STACK_SIZE=0 PICO_USE_STACK_GUARDS=1)

# palimpsest fork: never zero-fill reads beyond the configured medium (see usb.c)
target_compile_definitions(ATAboy PRIVATE ATABOY_STRICT_MEDIA_BOUNDS=1)

# palimpsest fork: read-only ATA PASS-THROUGH (SAT) for IDENTIFY, READ SECTORS,
# READ VERIFY, SMART reads and READ NATIVE MAX; allowlist in sat_policy.c.
# -DATABOY_SAT=0 leaves it out entirely.
option(ATABOY_SAT "Restricted read-only ATA PASS-THROUGH" ON)
if(ATABOY_SAT)
    target_sources(ATAboy PRIVATE sat.c sat_policy.c)
    target_compile_definitions(ATAboy PRIVATE ATABOY_SAT=1)
endif()
# SMART READ DATA and SMART RETURN STATUS save attribute values to the drive
# on ATA-3 drives (sat_policy.h), so they are refused unless this is ON.
# Never ON in a build used on archaeology specimens.
option(ATABOY_SAT_SMART_SAVES "Also allow SMART READ DATA and RETURN STATUS through SAT" OFF)
if(ATABOY_SAT AND ATABOY_SAT_SMART_SAVES)
    target_compile_definitions(ATAboy PRIVATE ATABOY_SAT_SMART_SAVES=1)
    # The opt-in build names itself, so picotool info tells it apart (review
    # L-2); the setup screen banner says so too (menus.c).
    pico_set_program_version(ATAboy "0.6f3p10-palimpsest-smartsaves")
else()
    pico_set_program_version(ATAboy "0.6f3p10-palimpsest")
endif()

pico_add_extra_outputs(ATAboy)

