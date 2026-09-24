#!/bin/sh
# Build and run the host tests against the firmware sources.
#   tests/host/run.sh [firmware source dir]
# CXX picks the compiler (default g++). Any C++17 compiler should do.
set -e
here=$(cd "$(dirname "$0")" && pwd)
src=${1:-"$here/../../FW source/v.6f3p1"}
out=${OUT:-"$here/build"}
mkdir -p "$out"
# Use the same ATABOY_* compile definitions as the firmware build (CMakeLists.txt).
defs=$(grep -o 'ATABOY_[A-Z_]*=[0-9]*' "$src/CMakeLists.txt" | sed 's/^/-D/' | tr '\n' ' ')
echo "firmware defines: $defs"
${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function $defs \
    -I "$here/mock" -I "$here" -I "$src" \
    -o "$out/test_read" "$here/test_read.cpp"
"$out/test_read"
