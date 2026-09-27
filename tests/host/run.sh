#!/bin/sh
# Build and run the host tests against the firmware sources: test_read (ide.c,
# usb.c, sat.c against the simulated drive), then test_fwupdate (menus.c).
#   tests/host/run.sh [firmware source dir]
# CXX picks the compiler (default g++). Any C++17 compiler should do.
set -e
here=$(cd "$(dirname "$0")" && pwd)
src=${1:-"$here/../../FW source/v.6f3p1"}
out=${OUT:-"$here/build"}
mkdir -p "$out"
# Use the same ATABOY_* compile definitions as the firmware build (CMakeLists.txt).
# Only target_compile_definitions lines count: a comment mentions -DATABOY_SAT=0.
# SAT=0 in the environment tests the build with the SAT option turned off.
defs=$(grep 'target_compile_definitions' "$src/CMakeLists.txt" | grep -o 'ATABOY_[A-Z_]*=[0-9]*' | sed 's/^/-D/' | tr '\n' ' ')
if [ "${SAT:-1}" = 0 ]; then defs=$(echo "$defs" | sed 's/-DATABOY_SAT=1//'); fi
# ATABOY_SAT_SMART_SAVES is an OFF-by-default option; the scan above would
# pick up its define whatever the option says. This script sets it itself.
defs=$(echo "$defs" | sed 's/-DATABOY_SAT_SMART_SAVES=1//')
# ATABOY_CTL (the ATAboy Control API) is OFF by default in CMake, but this
# branch is its build, so the suite runs with it ON unless CTL=0 is set.
if [ "${CTL:-1}" = 0 ]; then defs=$(echo "$defs" | sed 's/-DATABOY_CTL=1//'); fi
# ATABOY_BOARD_JUMPERS (stage 3: JP1 LOCK and JP2 PERMIT) is OFF by default in
# CMake too, and the API image of this branch has it, so the suite runs with
# it ON unless JUMPERS=0 is set; never without the API, which it needs.
if [ "${JUMPERS:-1}" = 0 ] || [ "${CTL:-1}" = 0 ]; then defs=$(echo "$defs" | sed 's/-DATABOY_BOARD_JUMPERS=1//'); fi
echo "firmware defines: $defs"
# test_read runs twice with SAT on. The full suite is built with SMART READ
# DATA and RETURN STATUS opted in (ATABOY_SAT_SMART_SAVES=1), because the
# shipping build refuses both and much of the SAT path is tested through
# them; the shipping build then runs the test that proves they are refused
# and never reach the drive. The firmware code is the same apart from that.
smart=""
if echo "$defs" | grep -q -- '-DATABOY_SAT=1'; then smart="-DATABOY_SAT_SMART_SAVES=1"; fi
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function $defs $smart \
    -I "$here/mock" -I "$here" -I "$src" \
    -o "$out/test_read" "$here/test_read.cpp"
ATABOY_TRANSCRIPT="$out/transcript-off.txt" "$out/test_read"
# F1 stage 2: the same tests with Pipelined Reads On, and what the host got
# and the drive saw in every READ(10) and WRITE(10) compared with Off.
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function $defs $smart -DATABOY_TEST_PIPELINED=1 \
    -I "$here/mock" -I "$here" -I "$src" \
    -o "$out/test_read_pipelined" "$here/test_read.cpp"
ATABOY_TRANSCRIPT="$out/transcript-on.txt" "$out/test_read_pipelined"
if cmp -s "$out/transcript-off.txt" "$out/transcript-on.txt"; then
    echo "transcripts Off and On: $(wc -l < "$out/transcript-off.txt") commands, 1 checks, 0 failed"
else
    diff "$out/transcript-off.txt" "$out/transcript-on.txt" | head -20
    echo "transcripts Off and On: 1 checks, 1 failed"
    exit 1
fi
if [ -n "$smart" ]; then
    ${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function $defs \
        -I "$here/mock" -I "$here" -I "$src" \
        -o "$out/test_read_shipping" "$here/test_read.cpp"
    "$out/test_read_shipping"
fi
# Firmware update mode: fwupdate.h and menus.c (see test_fwupdate.cpp).
# (menus.c's upstream debug code puts two ifs on a line; that warning is off.)
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function -Wno-misleading-indentation $defs \
    -I "$here/mock" -I "$here" -I "$src" \
    -o "$out/test_fwupdate" "$here/test_fwupdate.cpp"
"$out/test_fwupdate"
# ...and once more with the SMART opt-in, whose banner must say so (review L-2).
if [ -n "$smart" ]; then
    ${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function -Wno-misleading-indentation $defs $smart \
        -I "$here/mock" -I "$here" -I "$src" \
        -o "$out/test_fwupdate_smart" "$here/test_fwupdate.cpp"
    "$out/test_fwupdate_smart"
fi
# Manual CHS with no IDENTIFY (0.6f3p8): menus.c and ide.c together against
# the simulated drive (see test_manual_chs.cpp).
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function -Wno-misleading-indentation $defs \
    -I "$here/mock" -I "$here" -I "$src" \
    -o "$out/test_manual_chs" "$here/test_manual_chs.cpp"
"$out/test_manual_chs"
# Q7: core 1's main loop after a trip on core 0, as its own run (the loop
# never returns; see test_q7_core1_loop).
"$out/test_manual_chs" q7-core1
# The USB serial number is the board's own (0.6f3p9): usb_descriptors.c with
# a stand-in board id (see test_usb_desc.cpp).
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function $defs \
    -I "$here/mock" -I "$here" -I "$src" \
    -o "$out/test_usb_desc" "$here/test_usb_desc.cpp"
"$out/test_usb_desc"
# PIO bus timing against ATA mode 0, read from ataboy.pio, and the clock
# divider (holistic review 2026-09-26, M-2 and L-6; see test_pio_timing.cpp).
${CXX:-g++} -std=c++17 -O1 -g -Wall -I "$src" -o "$out/test_pio_timing" "$here/test_pio_timing.cpp"
"$out/test_pio_timing" "$src/ataboy.pio"
# The configuration descriptor's layout against TinyUSB's real headers
# (palimpsest api/SPEC-v1.md 11.1, finding F-3), with the Control API off
# (today's bytes) and on (interfaces 3 and 4). Without the headers this
# could not run, which is not a pass.
tinyusb=${TINYUSB:-${PICO_SDK_PATH:-D:/Development/pico/pico-sdk}/lib/tinyusb/src}
if [ ! -f "$tinyusb/device/usbd.h" ]; then echo "COULD NOT RUN: no TinyUSB headers at $tinyusb"; exit 98; fi
for ctl in 0 1; do
    ${CXX:-g++} -std=c++17 -O1 -g -Wall -DATABOY_CTL=$ctl -I "$here/layout" -I "$tinyusb" -I "$src" \
        -o "$out/test_usb_layout$ctl" "$here/test_usb_layout.cpp"
    "$out/test_usb_layout$ctl"
done
# The ATAboy Control API (palimpsest api/SPEC-v1.md, stages 1 and 2): its
# tests, core 1's real loop taking a job (and not in a confirm box), the
# same loop with keys (core1-tui: the Features toggles' epoch, F: Force after
# identify.forbid, an API unmount at the MOUNTED screen), config_save's
# race, the parser and policy fuzzer (fixed seed and count;
# UndefinedBehaviorSanitizer in trap mode and the stack protector, since
# this g++ has no AddressSanitizer), and the real host client end to end.
# Only in a build with the API.
if echo "$defs" | grep -q -- '-DATABOY_CTL=1'; then
    ${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function -Wno-misleading-indentation $defs \
        -I "$here/mock" -I "$here" -I "$src" -o "$out/test_ctl" "$here/test_ctl.cpp"
    "$out/test_ctl"
    "$out/test_ctl" core1
    "$out/test_ctl" core1-confirm
    "$out/test_ctl" core1-tui
    # config_save() with core 0 storing into config at each point before the
    # lockout (SPEC-v1 5.5, R-API-1 finding 15): the copy first, the CRC over it.
    ${CXX:-g++} -std=c++17 -O1 -g -Wall $defs -I "$here/config_mock" -I "$here/mock" -I "$src"         -o "$out/test_config_save" "$here/test_config_save.cpp"
    "$out/test_config_save"
    ${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function -Wno-misleading-indentation $defs \
        -fsanitize=undefined -fsanitize-undefined-trap-on-error -fstack-protector-all -D_GLIBCXX_ASSERTIONS \
        -I "$here/mock" -I "$here" -I "$src" -o "$out/ctl_fuzz" "$here/ctl_fuzz.cpp"
    "$out/ctl_fuzz" "$here/ctl_corpus" 100000 1
    ${PYTHON:-python} "$here/ctl_e2e.py" "$out/test_ctl"
    # Stage 4: the presence row takes no cell any screen uses (pyte).
    ${PYTHON:-python} "$here/presence_rows.py" "$src" $defs
fi
