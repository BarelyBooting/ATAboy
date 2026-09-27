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
        fault.c
        obs.c)

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
        hardware_sha256
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
# palimpsest fork: the ATAboy Control API (palimpsest api/SPEC-v1.md), a
# second CDC port (interfaces 3 and 4) speaking one JSON object per line.
# Stage 1: the port, the framing and parser, ids and the replay table, and
# the read-only ops. Stage 2: the ratchet (write protect on, Pipelined Reads
# off, forbid IDENTIFY, unmount and detach naming the mount). Stage 3, the
# jumpers, is its own option below (ATABOY_BOARD_JUMPERS). OFF by
# default, and an OFF build is byte for byte the image of the sources
# without it.
option(ATABOY_CTL "ATAboy Control API: a second CDC port, JSON lines" OFF)
if(ATABOY_CTL)
    target_sources(ATAboy PRIVATE ctl.c ctl_job.c ctl_json.c ctl_policy.c ctl_sha256.c)
    target_compile_definitions(ATAboy PRIVATE ATABOY_CTL=1)
    target_link_libraries(ATAboy pico_rand)     # dev.boot, a random id per boot (SPEC-v1 3.3)
endif()
if(ATABOY_SAT AND ATABOY_SAT_SMART_SAVES)
    target_compile_definitions(ATAboy PRIVATE ATABOY_SAT_SMART_SAVES=1)
endif()
# palimpsest fork: the Control API's stage 3 (api/SPEC-v1.md section 8), the
# physical-presence jumpers JP1 (LOCK, GPIO30) and JP2 (PERMIT, GPIO31). They
# are the API's, so this needs ATABOY_CTL. It also makes the image an RP2350B
# one (48 pins, boards/ataboy_rev6.h): the definition below reaches every
# source compiled into the image, the SDK's own included, and nothing else
# changes that count. OFF by default: the build without it is the stage 2
# image, and with the API off too, the 0.6f3p10 one, byte for byte.
option(ATABOY_BOARD_JUMPERS "JP1 LOCK and JP2 PERMIT on GPIO30/31 (needs ATABOY_CTL)" OFF)
if(ATABOY_BOARD_JUMPERS)
    if(NOT ATABOY_CTL)
        message(FATAL_ERROR "ATABOY_BOARD_JUMPERS needs ATABOY_CTL: the jumpers are the Control API's (SPEC-v1 8)")
    endif()
    target_sources(ATAboy PRIVATE board_jumpers.c)
    target_compile_definitions(ATAboy PRIVATE ATABOY_BOARD_JUMPERS=1)
endif()
if(ATABOY_CTL)
    # Owner decision Q5: the API build is 0.6f4, so neither picotool info nor
    # the setup screen can pass it off as the CP3044's 0.6f3p10.
    if(ATABOY_SAT AND ATABOY_SAT_SMART_SAVES)
        set(ATABOY_VERSION "0.6f4-palimpsest-smartsaves")
        set(ATABOY_BANNER "v0.6f4 (fork+api+smartsaves)")
    else()
        set(ATABOY_VERSION "0.6f4-palimpsest")
        set(ATABOY_BANNER "v0.6f4 (fork+api)")
    endif()
    pico_set_program_version(ATAboy "${ATABOY_VERSION}")
    # hello's build identity (SPEC-v1 4.2), one generated header that the
    # banner reads too. The commit comes from git archive, which writes it
    # into ctl_build_id.txt (export-subst, .gitattributes), or from git in a
    # checkout, with dirty set when the source folder differs from it.
    file(STRINGS ${CMAKE_CURRENT_LIST_DIR}/ctl_build_id.txt ATABOY_GIT LIMIT_COUNT 1)
    string(STRIP "${ATABOY_GIT}" ATABOY_GIT)             # a CRLF copy leaves a carriage return
    set(ATABOY_DIRTY 0)
    if(NOT ATABOY_GIT MATCHES "^[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f]$")
        set(ATABOY_GIT "0000000000000000000000000000000000000000")
        set(ATABOY_DIRTY 1)
        find_package(Git QUIET)
        if(GIT_FOUND)
            execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse HEAD WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                            OUTPUT_VARIABLE head OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE rc ERROR_QUIET)
            execute_process(COMMAND ${GIT_EXECUTABLE} status --porcelain -- . WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                            OUTPUT_VARIABLE changes RESULT_VARIABLE rc2 ERROR_QUIET)
            if(rc EQUAL 0 AND rc2 EQUAL 0 AND head MATCHES "^[0-9a-f]+$")
                set(ATABOY_GIT ${head})
                if(changes STREQUAL "")
                    set(ATABOY_DIRTY 0)
                endif()
            endif()
        endif()
    endif()
    file(SHA256 ${CMAKE_CURRENT_LIST_DIR}/sat_policy.c ATABOY_SAT_POLICY_SHA256)
    file(SHA256 ${CMAKE_CURRENT_LIST_DIR}/ctl_policy.c ATABOY_CTL_POLICY_SHA256)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                 ${CMAKE_CURRENT_LIST_DIR}/sat_policy.c ${CMAKE_CURRENT_LIST_DIR}/ctl_policy.c
                 ${CMAKE_CURRENT_LIST_DIR}/ctl_build_id.txt)
    configure_file(${CMAKE_CURRENT_LIST_DIR}/ctl_build.h.in ${CMAKE_CURRENT_BINARY_DIR}/generated/ctl_build.h @ONLY)
    target_include_directories(ATAboy PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
elseif(ATABOY_SAT AND ATABOY_SAT_SMART_SAVES)
    # The opt-in build names itself, so picotool info tells it apart (review
    # L-2); the setup screen banner says so too (menus.c).
    pico_set_program_version(ATAboy "0.6f3p10-palimpsest-smartsaves")
else()
    pico_set_program_version(ATAboy "0.6f3p10-palimpsest")
endif()

pico_add_extra_outputs(ATAboy)

