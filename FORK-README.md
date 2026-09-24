# ATAboy (BarelyBooting fork)

This is a fork of [ATAboy](https://github.com/redruM0381/ATAboy), the open-source IDE-to-USB bridge by redruM0381. The hardware, the firmware and the whole idea are theirs. For what ATAboy is and how to use it, start with the upstream [README](README.md) and repository.

I use ATAboy to image some old drives that can't be replaced, so I've been fixing a few things I ran into. The author mentioned on [issue #13](https://github.com/redruM0381/ATAboy/issues/13) (Aug 2026) that a fix there "may be as late as spring 2027", so this fork keeps the firmware building and carries a handful of small, tested fixes in the meantime. Each one is also sent upstream as its own pull request. It isn't meant to replace the original project.

## What's different

The fork's firmware lives in [`FW source/v.6f3p1/`](FW%20source/v.6f3p1/) on the `palimpsest/dev` branch. It's upstream v.6f3 with these changes:

1. **Reads past the end of the drive no longer come back as zeros with a success status.** Upstream zero-fills any part of a READ(10) at or beyond the configured sector count and reports success, which looks exactly like real data. Now it returns only the sectors it actually read and fails the rest. The firmware sets sense 5/21 (LBA out of range), though the host currently sees 2/3A instead because TinyUSB rewrites the sense on failed reads. That's a separate TinyUSB issue. You can get the old behaviour back with `-DATABOY_STRICT_MEDIA_BOUNDS=0`. I tested this one on hardware (40 GB Maxtor, LBA mode): reads past the end now fail with nothing written to the host buffer, and real sectors read back byte-identical to the stock firmware.

2. **MODE SENSE(10) can't overrun its buffer any more.** The TinyUSB in pico-sdk 2.2.0 passes the host's requested length through as the buffer size, and the MODE SENSE handler memsets that many bytes into a 4 KB buffer. It's now clamped. Built and checked in the disassembly, but deliberately not triggered on hardware.

3. **Esc actually cancels manual CHS entry.** It used to fall through and apply whatever geometry was in the input fields. Built and flashed, not yet exercised on hardware.

4. **Dismissing the detect error box flags a media change**, like every other unmount path already does. Built and flashed, not yet exercised on hardware.

5. **`pico_sdk_import.cmake` is included**, so the source builds as-is. Upstream v.6f3 references it but doesn't ship it.

Added in 0.6f3p4 (built and host-tested, not yet run on hardware):

6. **ATA PASS-THROUGH, read-only.** The bridge now answers ATA PASS-THROUGH(12) and (16) for IDENTIFY DEVICE and READ SECTORS / READ SECTORS EXT (PIO, 1 to 8 sectors per command, LBA mode), so tools can read the drive's own identity and read sectors with the ATA command they chose. Everything else is refused with ILLEGAL REQUEST before anything is sent to the drive: writes, DMA, SMART, other protocols. (Data-out direction can't be fully checked from inside the callback, but only the read-only commands above can ever run.) The allowlist is one small pure file, `sat_policy.c`, with an exhaustive host test and mutation tests in `test/`. Build with `-DATABOY_SAT=0` to leave it out. Built and host-tested, not yet tried on hardware.

7. **[Issue #13](https://github.com/redruM0381/ATAboy/issues/13): one bad sector no longer fails a whole multi-sector read.** The host gets every good sector before the bad one, then an error, and can read the rest one at a time. A failed sector is never filled in, and any flawed data the drive offers with the error is thrown away. The firmware no longer soft-resets the drive after an ordinary read error; it still resets on a timeout. The drive's error registers are saved before any reset, and Debug Mode's "E" shows them.

8. **Status polls wait for BSY to clear before looking at ERR or DRQ**, and wait 400 ns after each command before the first status read, as ATA requires. Before, an ERR bit seen while the drive was still busy could end a command early.

9. **[Issue #9](https://github.com/redruM0381/ATAboy/issues/9): the whole 80x24 screen is painted blue**, instead of relying on the terminal to erase in the current colour (GNU screen doesn't, by default). **Ctrl+L redraws the screen.**

10. **A soft reset now waits for the drive and checks the CHS geometry came back.** Before, the firmware waited 2 s after a soft reset and then sent INITIALIZE DEVICE PARAMETERS whether the drive was ready or not. ATA gives a drive up to 31 s. A slow drive ignored the command, fell back to its own default translation, and every later CHS read went to a different sector than the one asked for, with good status. Now the reset waits up to 31 s, selects the drive again (a reset always selects the master, which broke slave drives), and in CHS mode refuses reads and writes until the drive has accepted the geometry. Debug E shows "no geometry" or "reset FAILED" when that happens. The drive is also no longer handed a new command while it is still offering data from an earlier one; that check used to be part of item 6 only.

11. **READ(10) and WRITE(10) refuse a malformed request.** TinyUSB works the block size out from the host's transfer length and never checks it is 512, so a length that didn't match the block count could make the partial-sector code copy past a 512-byte buffer. A request that isn't whole 512-byte sectors is now refused before anything is read or written. Normal operating systems never send one.

12. **Host tests** in `tests/host/`: the real `ide.c` and `usb.c` run against a simulated drive (`run.sh`, `mutate.py`), and `tui_compare.py` checks the screen output against an older build.

The fork's build calls itself `0.6f3p4-palimpsest` and shows `v0.6f3p4 (fork)` in the setup screen (0.6f3p1 before items 6 to 12), so it can't be mistaken for a stock v0.6f3.

## Builds

Every push to `main` or a `palimpsest/**` branch, every pull request and every `v*` tag runs [`build-firmware.yml`](.github/workflows/build-firmware.yml). It builds the fork firmware and upstream's v.6f3 with the exact versions the sources ask for (Arm GNU Toolchain 14.2.Rel1, Pico SDK 2.2.0, picotool 2.2.0-a4), and uploads the `.uf2`, `.elf`, `.bin`, `SHA256SUMS` and `picotool info` output.

The builds are reproducible. The build date is pinned to the commit date, and the same commit gives the same bytes on Windows and on the Linux runners. There's also a job that rebuilds upstream's published `ATAboy6f3.uf2` from source and compares it byte for byte. It matches.

Tagging `vX.Y.Z` publishes a pre-release with the files attached.

To build locally with the same toolchain:

```
cmake -S "FW source/v.6f3p1" -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPICO_BOARD=pico2 -DPICO_SDK_PATH=<sdk> -DPICO_TOOLCHAIN_PATH=<toolchain> \
  -Dpicotool_DIR=<picotool>
ninja -C build
```

## Not official

Nothing built here is an official ATAboy release, and the releases are marked as pre-releases. For the real thing, go to [redruM0381/ATAboy](https://github.com/redruM0381/ATAboy) or [obsoletetech.us](https://obsoletetech.us/products/ataboy-an-open-source-legacy-ide-usb-bridge).

## Getting in touch

Hailing frequencies are open. [Open an issue on this fork](https://github.com/BarelyBooting/ATAboy/issues) or mention @BarelyBooting and I'll see it. Problems with the original hardware or stock firmware belong on the [upstream tracker](https://github.com/redruM0381/ATAboy/issues).

## License

GPL-3.0, same as upstream. See [LICENSE](LICENSE).
