#!/bin/sh
# Regression gate for "the API changes nothing where it is not compiled in, and
# nothing on the READ path where it is" (palimpsest review B M-1, 2026-09-28).
# FORK-README items 33 to 36 claim it; this is what asserts it, against a base
# revision (default 6bb7da4, the tree before the API).
#
#   tests/host/check_identity.sh [base rev] [--no-image]
#
# Five parts, each reported apart as PASS, FAIL or COULD NOT RUN, and only PASS
# counts; a part that could not run never contributes to a green:
#   1. host transcripts: the base revision's test_read against the base sources,
#      compared byte for byte with this tree's test_read with the API off, on
#      without the jumpers, and on with them (every READ(10) and WRITE(10) the
#      tests make, and the drive's commands).
#   2. tui_compare.py --session: the setup screen and a whole session, base tree
#      against this tree with the API off, and with the API and the jumpers.
#   3. the default image: base and this tree built with the Arm toolchain
#      (ATABOY_CTL off) with the same pinned build date; ATAboy.uf2 and
#      ATAboy.bin must be identical.
#   4. the READ and log path of the API images (CTL, CTL with the jumpers), by
#      check_hotpath.py, instruction for instruction against the base ELF.
#   5. the link check (check_ctl.py) on those two API images, passing, and
#      failing with --forbid ide_read_reg (a check that cannot fail is no check).
# Parts 3 to 5 need the Arm toolchain, cmake, ninja and the Pico SDK
# (PICO_SDK_PATH, PICO_TOOLCHAIN_PATH, PICOTOOL_DIR; the estate's defaults are
# under D:/Development/pico). Without them they are COULD NOT RUN.
#
# The base tree is a `git archive` of the base revision: nothing in the working
# tree is touched. Everything is built under $OUT (default a temp directory).
# Exit: 0 all parts passed; 1 a part failed; 98 nothing failed but a part could
# not run.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
base=${1:-6bb7da4}
image=1
for a in "$@"; do [ "$a" = "--no-image" ] && image=0; done
[ "$base" = "--no-image" ] && base=6bb7da4
out=${OUT:-$(mktemp -d)}
mkdir -p "$out"
rel="FW source/v.6f3p1"
pass=0; fail=0; nocan=0
say() { echo "$1: $2"; }
ok()  { pass=$((pass+1));   say PASS "$1"; }
bad() { fail=$((fail+1));   say FAIL "$1"; }
cant() { nocan=$((nocan+1)); say "COULD NOT RUN" "$1"; }

git -C "$repo" rev-parse --verify -q "$base^{commit}" >/dev/null || { echo "no such revision: $base"; exit 2; }
git -C "$repo" archive "$base" | { mkdir -p "$out/base" && tar -x -C "$out/base"; } || { echo "git archive failed"; exit 2; }
# This tree as committed (HEAD), not the working tree: the gate is about what is committed.
if [ -n "$HEAD_TREE" ]; then    # a tree to judge instead of HEAD (to show the gate can fail: a scratch copy with a planted change)
    mkdir -p "$out/head" && cp -r "$HEAD_TREE"/. "$out/head"/ || { echo "cannot copy $HEAD_TREE"; exit 2; }
else
    git -C "$repo" archive HEAD | { mkdir -p "$out/head" && tar -x -C "$out/head"; } || { echo "git archive failed"; exit 2; }
fi

# ---- 1. host transcripts -------------------------------------------------------
# The defines are the CMake ones, as run.sh takes them, plus the SMART opt-in
# run.sh builds test_read with.
defs_of() {
    d=$(grep 'target_compile_definitions' "$1/CMakeLists.txt" | grep -o 'ATABOY_[A-Z_]*=[0-9]*' | sed 's/^/-D/' | tr '\n' ' ')
    echo "$d" | sed 's/-DATABOY_SAT_SMART_SAVES=1//'
}
transcript() {   # <tree root> <name> <defs>
    t="$1"; tn="$2"; shift 2
    ${CXX:-g++} -std=c++17 -O1 -g -Wall -Wno-unused-function "$@" -DATABOY_SAT_SMART_SAVES=1 \
        -I "$t/tests/host/mock" -I "$t/tests/host" -I "$t/$rel" -o "$out/tr-$tn" "$t/tests/host/test_read.cpp" 2>"$out/tr-$tn.err" || return 1
    ATABOY_TRANSCRIPT="$out/tr-$tn.txt" "$out/tr-$tn" >"$out/tr-$tn.log" 2>&1
}
bdefs=$(defs_of "$out/base/$rel")
hdefs=$(defs_of "$out/head/$rel")
if transcript "$out/base" base $bdefs; then
    want=$(sha256sum "$out/tr-base.txt" | cut -d' ' -f1); n=$(wc -l < "$out/tr-base.txt")
    if [ "$n" -lt 1000 ]; then bad "transcripts: the base transcript has only $n lines (an empty population is a refusal)"; else
        for cfg in "off:$(echo "$hdefs" | sed 's/-DATABOY_CTL=1//; s/-DATABOY_BOARD_JUMPERS=1//')" \
                   "ctl:$(echo "$hdefs" | sed 's/-DATABOY_BOARD_JUMPERS=1//')" \
                   "ctl+jumpers:$hdefs"; do
            name=${cfg%%:*}; d=${cfg#*:}
            if transcript "$out/head" "$name" $d; then
                got=$(sha256sum "$out/tr-$name.txt" | cut -d' ' -f1)
                if [ "$got" = "$want" ]; then ok "transcripts, API $name: $n commands identical to $base ($want)"
                else bad "transcripts, API $name: differs from $base (line $(cmp "$out/tr-base.txt" "$out/tr-$name.txt" | sed 's/.*line //'))"; fi
            else bad "transcripts, API $name: test_read did not build or did not pass (see $out/tr-$name.err, .log)"; fi
        done
    fi
else bad "transcripts: the base revision's test_read did not build or pass (see $out/tr-base.err)"; fi

# ---- 2. the setup screen, a whole session -----------------------------------------
if python -c "import pyte" 2>/dev/null; then
    for cfg in "off:" "ctl+jumpers:-DATABOY_CTL=1 -DATABOY_BOARD_JUMPERS=1"; do
        name=${cfg%%:*}; d=${cfg#*:}
        if python "$here/tui_compare.py" "$out/base/$rel" "$out/head/$rel" --session --expect '*:1' ${d:+--new-defs "$d"} >"$out/tui-$name.log" 2>&1 \
           && grep -q "the rest IDENTICAL" "$out/tui-$name.log"; then
            ok "setup screen and session, API $name: identical to $base (the banner apart)"
        else bad "setup screen and session, API $name: see $out/tui-$name.log"; fi
    done
else cant "setup screen and session: no pyte"; fi

# ---- 3. the default image, and 4. the READ path of the API images ---------------------
build() {   # <tree root> <name> [cmake -D...]  -> $out/b-<name>/ATAboy.{elf,uf2,bin}
    t="$1"; tn="$2"; shift 2
    # The build date is pinned, as build-firmware.yml does: the same in every tree.
    grep -q PICO_PROGRAM_BUILD_DATE "$t/$rel/CMakeLists.txt" ||
        printf '\ntarget_compile_definitions(ATAboy PRIVATE PICO_PROGRAM_BUILD_DATE="Jan  1 2026")\n' >> "$t/$rel/CMakeLists.txt"
    cmake -G Ninja -S "$t/$rel" -B "$out/b-$tn" -DCMAKE_BUILD_TYPE=Release -DPICO_SDK_PATH="$sdk" \
        -Dpicotool_DIR="$picotool" -DPICO_TOOLCHAIN_PATH="$tc" "$@" >"$out/b-$tn.cfg.log" 2>&1 \
        && ninja -C "$out/b-$tn" >"$out/b-$tn.build.log" 2>&1
}
sdk=${PICO_SDK_PATH:-D:/Development/pico/pico-sdk}
tc=${PICO_TOOLCHAIN_PATH:-D:/Development/pico/toolchain}
picotool=${PICOTOOL_DIR:-D:/Development/pico/picotool/picotool}
if [ "$image" = 0 ]; then
    cant "default image and READ path: --no-image"
elif ! command -v cmake >/dev/null || ! command -v ninja >/dev/null || [ ! -d "$sdk" ] || [ ! -d "$tc/bin" ]; then
    cant "default image and READ path: no cmake, ninja, Pico SDK ($sdk) or Arm toolchain ($tc)"
else
    export PATH="$tc/bin:$PATH"
    if build "$out/base" base && build "$out/head" off; then
        for f in ATAboy.uf2 ATAboy.bin; do
            a=$(sha256sum "$out/b-base/$f" | cut -d' ' -f1); b=$(sha256sum "$out/b-off/$f" | cut -d' ' -f1)
            if [ "$a" = "$b" ]; then ok "default image, $f identical to $base ($a)"; else bad "default image, $f differs from $base ($a, $b)"; fi
        done
    else bad "default image: the base or this tree did not build (see $out/b-*.build.log)"; fi
    if build "$out/head" ctl -DATABOY_CTL=ON && build "$out/head" ctlj -DATABOY_CTL=ON -DATABOY_BOARD_JUMPERS=ON; then
        OBJDUMP=${OBJDUMP:-$tc/bin/arm-none-eabi-objdump.exe}
        [ -x "$OBJDUMP" ] || OBJDUMP=$tc/bin/arm-none-eabi-objdump
        OBJDUMP=$OBJDUMP python "$here/check_hotpath.py" "$out/b-base/ATAboy.elf" "$out/b-ctl/ATAboy.elf" "$out/b-ctlj/ATAboy.elf" >"$out/hotpath.log" 2>&1
        rc=$?
        if [ $rc = 0 ]; then ok "READ and log path of the API images: $(head -1 "$out/hotpath.log")"
        elif [ $rc = 98 ]; then cant "READ and log path of the API images: $(head -1 "$out/hotpath.log")"
        else bad "READ and log path of the API images: see $out/hotpath.log"; fi
    else bad "READ and log path: an API image did not build (see $out/b-*.build.log)"; fi
    # 5. The link check on those two images (review C LOW-4): nothing the API runs can reach a
    # settings save, a sector write, a command or a reset. Driven both ways: it must pass on the
    # images and fail with --forbid ide_read_reg (which taskfile.snapshot does reach), or it
    # is not a check.
    if [ -f "$out/b-ctl/ATAboy.elf" ] && [ -f "$out/b-ctlj/ATAboy.elf" ]; then
        python "$here/check_ctl.py" "$out/b-ctl/ATAboy.elf" "$OBJDUMP" >"$out/link-ctl.log" 2>&1; r1=$?
        python "$here/check_ctl.py" "$out/b-ctlj/ATAboy.elf" "$OBJDUMP" --jumpers >"$out/link-ctlj.log" 2>&1; r2=$?
        python "$here/check_ctl.py" "$out/b-ctlj/ATAboy.elf" "$OBJDUMP" --jumpers --forbid ide_read_reg >"$out/link-forbid.log" 2>&1; r3=$?
        if [ $r1 = 98 ] || [ $r2 = 98 ] || [ $r3 = 98 ]; then cant "link check: $(head -1 "$out/link-ctl.log")"
        elif [ $r1 = 0 ] && [ $r2 = 0 ] && [ $r3 = 1 ]; then ok "link check: $(tail -1 "$out/link-ctl.log") (CTL), $(tail -1 "$out/link-ctlj.log") (CTL+jumpers), and it fails with --forbid ide_read_reg: $(tail -1 "$out/link-forbid.log")"
        else bad "link check: exit codes $r1 $r2 (want 0 0) and $r3 with --forbid (want 1); see $out/link-*.log"; fi
    fi
fi

echo "identity gate: $pass passed, $fail failed, $nocan could not run (working files in $out)"
if [ "$fail" -gt 0 ]; then exit 1; fi
if [ "$nocan" -gt 0 ]; then exit 98; fi
exit 0
