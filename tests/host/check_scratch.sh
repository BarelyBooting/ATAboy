#!/bin/sh
# F1 stage 2 (d): the IDE bus code runs from SCRATCH_X (ide_pio.c, ide.c).
# Reads the built ELF and checks each function lies inside SCRATCH_X, that
# SCRATCH_X ends inside its 4 KB bank, and that core 0's stack (SCRATCH_Y)
# and core 1's (main RAM, size 0 here) do not overlap it. Not part of run.sh:
# it needs the Arm build.
#   tests/host/check_scratch.sh <ATAboy.elf> [nm]
elf=$1
nm=${2:-arm-none-eabi-nm}
[ -f "$elf" ] || { echo "COULD NOT RUN: no ELF at $elf"; exit 98; }
syms=$("$nm" "$elf") || { echo "COULD NOT RUN: $nm failed"; exit 98; }
addr() { echo "$syms" | awk -v s="$1" '$3 == s { print $1; exit }'; }
start=$(addr __scratch_x_start__); end=$(addr __scratch_x_end__)
[ -n "$start" ] && [ -n "$end" ] || { echo "COULD NOT RUN: no SCRATCH_X symbols"; exit 98; }
fail=0; checks=0
for f in ide_pio_read ide_pio_write ide_read_reg ide_write_reg ide_read_alt_status ide_write_control; do
    checks=$((checks + 1))
    a=$(addr "$f")
    if [ -z "$a" ] || [ $((0x$a)) -lt $((0x$start)) ] || [ $((0x$a)) -ge $((0x$end)) ]; then
        echo "FAIL: $f at ${a:-nowhere}, not in SCRATCH_X ($start..$end)"; fail=$((fail + 1))
    fi
done
checks=$((checks + 1))
if [ $((0x$end)) -gt $((0x20081000)) ]; then echo "FAIL: SCRATCH_X ends at $end, past its bank"; fail=$((fail + 1)); fi
checks=$((checks + 1))
sb=$(addr __StackBottom); s1b=$(addr __StackOneBottom); s1t=$(addr __StackOneTop)
if [ $((0x$sb)) -lt $((0x$end)) ] || { [ "$s1b" != "$s1t" ] && [ $((0x$s1b)) -lt $((0x$end)) ]; }; then
    echo "FAIL: a stack overlaps SCRATCH_X (core 0 from $sb, core 1 $s1b..$s1t)"; fail=$((fail + 1))
fi
echo "SCRATCH_X $start..$end ($((0x$end - 0x$start)) bytes): $checks checks, $fail failed"
exit $fail
