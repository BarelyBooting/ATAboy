# ATAboy — palimpsest fork

This is a maintained fork of **[ATAboy](https://github.com/redruM0381/ATAboy)**,
the open-source legacy IDE-to-USB bridge, originally designed and built by
**redruM0381**. All credit for the hardware design, the RP2350 firmware
architecture, and the project as a whole belongs to the original author —
see [`README.md`](README.md) and the [upstream repository](https://github.com/redruM0381/ATAboy)
for the project itself, its hardware requirements, and usage instructions.
This file only describes what this fork adds on top of that work.

The original author has said they will have limited time for a while (on
[issue #13](https://github.com/redruM0381/ATAboy/issues/13), 2026-08-25, a fix
"may be as late as spring 2027"). This fork exists to keep the firmware
building and to carry a small number of targeted, tested fixes forward - each
one offered back upstream as its own small pull request. It is not a
replacement for the original project.

## What this fork is

- **Repository:** [BarelyBooting/ATAboy](https://github.com/BarelyBooting/ATAboy)
- **Upstream:** [redruM0381/ATAboy](https://github.com/redruM0381/ATAboy)
- **Fork branch:** `palimpsest/dev`
- **Fork firmware source:** [`FW source/v.6f3p1/`](FW%20source/v.6f3p1/), built
  from upstream's `v.6f3` source with the changes below layered on top.

## What `palimpsest/dev` changes relative to upstream `v.6f3`

Each change is small and separable, so it can be proposed upstream
individually if the original author returns:

1. **Strict media bounds** (`usb.c`, `ATABOY_STRICT_MEDIA_BOUNDS`, on in this
   build). A `READ(10)` that reaches the configured drive's maximum LBA no
   longer silently zero-fills the remainder of the transfer and reports
   success. It now returns only the sectors actually read, and the rest of
   the command fails. The firmware sets sense 5/21/00 (LBA out of range);
   note that the host currently *receives* 2/3A/00 instead, because TinyUSB
   rewrites the sense of any failed READ(10) - a separate TinyUSB issue.
   Upstream's original behaviour is kept under `#else`
   (`-DATABOY_STRICT_MEDIA_BOUNDS=0`).
   **Tested on hardware** (RP2350 ATAboy, 40 GB Maxtor donor, LBA mode):
   reads at non-existent sectors, which previously returned success + zeros,
   now fail with nothing written to the host buffer; real sectors read back
   byte-identical to the stock firmware.

2. **Esc aborts manual CHS entry** (`menus.c`). Pressing Esc while manually
   entering CHS geometry (in both the picker and the force-detect copies of
   that screen) used to fall through and apply whatever geometry happened to
   be sitting in the input buffer. It now aborts cleanly and applies nothing.
   *Built and flashed, but not yet exercised on hardware.*

3. **Media-changed on error-box dismissal** (`menus.c`). Dismissing the
   detect-error box used to clear `is_mounted` without also setting the
   media-changed flag that every other unmount path sets, which could leave
   the host with a stale view of the medium. It now behaves like every other
   unmount path. *Built and flashed, but not yet exercised on hardware.*

4. **`pico_sdk_import.cmake` supplied.** Upstream's published `v.6f3` source
   includes `include(pico_sdk_import.cmake)` in `CMakeLists.txt` but does not
   ship the file itself, so the published source tree does not build as-is.
   This fork's source folder (`v.6f3p1/`) carries a copy that is
   byte-identical to the Raspberry Pi Pico SDK's own
   `external/pico_sdk_import.cmake`. The CI workflow supplies the same file
   for upstream's own `v.6f3` folder at build time too (see below) — it is
   never edited or committed into that folder in the repository itself.

This fork's build identifies itself unambiguously: version string
`0.6f3p1-palimpsest` and banner `v0.6f3p1 (fork)`, so it can never be
mistaken for a stock upstream `v0.6f3` build.

## Getting a build

### CI builds (recommended)

Every push to `main`, to a `palimpsest/**` branch, every pull request, and
every `v*` tag triggers [`.github/workflows/build-firmware.yml`](.github/workflows/build-firmware.yml),
which builds **both** this fork's firmware (`v.6f3p1`) and upstream's latest
published source (`v.6f3`, patched only in the CI runner's own workspace to
supply the missing `pico_sdk_import.cmake`) for the RP2350 (`pico2`) board,
using the exact toolchain/SDK/picotool versions pinned in each source
folder's `CMakeLists.txt`:

- Arm GNU Toolchain 14.2.Rel1 (`arm-none-eabi`)
- Raspberry Pi Pico SDK 2.2.0
- picotool 2.2.0-a4

Each build produces `ATAboy.uf2`, `ATAboy.elf`, `ATAboy.bin`, a
`SHA256SUMS` file, and the output of `picotool info -a` (which shows the
program version string), uploaded as workflow artifacts. A separate,
informational `reproduce-upstream` job checks whether this Linux CI
toolchain can reproduce the vendor-shipped `FW binaries/ATAboy6f3.uf2` file
byte-for-byte; its result (MATCH/DIFFER) is written to that run's job
summary and never fails the workflow.

Tagging a commit `vX.Y.Z` additionally publishes a **pre-release** GitHub
Release with the UF2s, `SHA256SUMS`, and picotool info attached.

### Building locally

See the toolchain/SDK/picotool versions above, then:

```
cmake -S "FW source/v.6f3p1" -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPICO_BOARD=pico2 -DPICO_SDK_PATH=<sdk> -DPICO_TOOLCHAIN_PATH=<toolchain> \
  -Dpicotool_DIR=<picotool>
ninja -C build
```

## Contact, bugs and questions

Please [open an issue on this fork](https://github.com/BarelyBooting/ATAboy/issues)
or mention **@BarelyBooting** - GitHub notifies the maintainer. Problems with
the original hardware or stock firmware belong on the
[upstream issue tracker](https://github.com/redruM0381/ATAboy/issues).

## This is unofficial

Nothing built from this fork or its CI workflow is an official release from
the original ATAboy author. Every GitHub Release this repository publishes
is explicitly marked as a pre-release fork build. If you want the original,
canonical ATAboy firmware and hardware, go to
[redruM0381/ATAboy](https://github.com/redruM0381/ATAboy) or
[obsoletetech.us](https://obsoletetech.us/products/ataboy-an-open-source-legacy-ide-usb-bridge).

## License

ATAboy is licensed under the **GNU General Public License v3.0** (GPL-3.0) —
see [`LICENSE`](LICENSE). This fork is distributed under the same license.
