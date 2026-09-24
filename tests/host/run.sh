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
echo "firmware defines: $defs"
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function $defs \
    -I "$here/mock" -I "$here" -I "$src" \
    -o "$out/test_read" "$here/test_read.cpp"
"$out/test_read"
# Firmware update mode: fwupdate.h and menus.c (see test_fwupdate.cpp).
# (menus.c's upstream debug code puts two ifs on a line; that warning is off.)
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function -Wno-misleading-indentation $defs \
    -I "$here/mock" -I "$here" -I "$src" \
    -o "$out/test_fwupdate" "$here/test_fwupdate.cpp"
"$out/test_fwupdate"
