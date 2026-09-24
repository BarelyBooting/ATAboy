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
"$out/test_read"
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
