#ifndef OBS_H
#define OBS_H

// Observers (owner decision F1, 2026-09-26, stage 1). Two records kept in
// RAM, read over USB with SCSI READ BUFFER, that change nothing the data
// path does:
//  - a log of every bus event: each ATA command with the task file written
//    for it, each Device Control write, each RESET- pulse, and the mount and
//    unmount markers. ide_write_reg() is the only way a command register is
//    written and ide_write_control() the only way Device Control is, so a
//    command missing from the log was not sent, as long as the host read
//    the log with no gap (below);
//  - a SHA-256 of each 512-byte block as it came off the bus, keyed by LBA
//    and by the log entry of the command that read it.
// Formats, the cursor and the retention policy are in obs.c and in the
// palimpsest PENDING-mchs-native-host document.

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define OBS_LOG_ENTRIES   4096      // 32 bytes each: 128 KB
#define OBS_HASH_ENTRIES  2048      // 48 bytes each: 96 KB

// SCSI READ BUFFER(10), mode 02h (data). 5Ah is the salvage capture (usb.c)
// and stays as it is; these two were never used before.
#define OBS_LOG_BUFFER_ID   0x4C    // 'L': the command log
#define OBS_HASH_BUFFER_ID  0x48    // 'H': the per-sector hashes
#define OBS_PAGE_HEADER     32

// Log entry kinds.
#define OBS_EV_CMD      1   // command register written (value: the opcode)
#define OBS_EV_CTRL     2   // Device Control written (value: the byte)
#define OBS_EV_RESET    3   // RESET- pulled low (value: OBS_RST_*)
#define OBS_EV_MOUNT    4   // the drive was mounted (value: OBS_MNT_*)
#define OBS_EV_UNMOUNT  5   // the drive was unmounted (value: OBS_UNM_*)
#define OBS_EV_BOOT     6   // the log began (power-up or reset of the ATAboy)
// 7 is kept for the API's own entries (palimpsest api/SPEC-v1.md, OBS_EV_API).
#define OBS_EV_REFUSED  8   // the tripwire refused a command: NOT sent (value: the opcode;
                            // tf: the task file as written for it). Always the last entry.

// Where a RESET- pulse came from.
#define OBS_RST_RESET_DRIVE 1   // ide_reset_drive: Debug R, auto-mount at boot
#define OBS_RST_PROBE       2   // ide_probe_devices: Auto Detect
#define OBS_RST_RECOVERY    3   // recovery: hardware reset after a soft reset failed
#define OBS_RST_MANUAL_CHS  4   // Ctrl+G (either mode)

// Mount and unmount markers.
#define OBS_MNT_AUTO        1   // auto-mount at boot
#define OBS_MNT_MENU        2   // Mount from the main menu
#define OBS_UNM_MENU        1   // Unmount from the main menu
#define OBS_UNM_DISMISS     2   // the detection result box dismissed
#define OBS_UNM_NOTHING     3   // Mount with nothing to mount
// A marker's task file bytes: features = geometry mode (OBS_MODE_*),
// count = sectors per track, sector = heads, cyl lo/hi = cylinders,
// device/head = the device base (A0h or B0h).
#define OBS_MODE_LBA        0
#define OBS_MODE_CHS        1   // CHS set up with IDENTIFY (Auto Detect, auto-mount)
#define OBS_MODE_MCHS_91    2   // Ctrl+G, 91h mode
#define OBS_MODE_MCHS_NATIVE 3  // Ctrl+G, power-up default (no 91h) mode

// Entry bits.
#define OBS_B_TF_WRITTEN    0x003F  // bit n: task file register n+1 written since the last command
#define OBS_B_HOB_WRITTEN   0x03C0  // bit 6+n: register n+2 written twice; hob[n] holds the first
#define OBS_B_CORE1         0x0400  // written by core 1 (menus), not core 0 (USB)
#define OBS_B_HOST          0x0800  // inside a USB host command
#define OBS_B_STATUS        0x1000  // status holds a Status read while this command was current
#define OBS_B_ERROR         0x2000  // error holds an Error read likewise
#define OBS_B_FINISHED      0x4000  // the path that sent it recorded its end (the counts)
#define OBS_B_FROZEN        0x8000  // status and error are final

// Outcome.
#define OBS_OUT_OPEN        0x00    // no end recorded (yet)
#define OBS_OUT_OK          0x01
#define OBS_OUT_FAILED      0xFF    // failed, no failure record written for it
// 0x80 | IDE_FAIL_*: failed, with that failure record (Debug E)

typedef struct {
    uint32_t seq;           // 1, 2, 3 ... since the log began; 0: never written
    uint32_t us_lo;         // microseconds since boot, bits 0-31
    uint8_t  us_hi;         //   bits 32-39 (12.7 days)
    uint8_t  kind;          // OBS_EV_*
    uint8_t  value;         // opcode, Device Control byte, OBS_RST_*, OBS_MNT_*, OBS_UNM_*
    uint8_t  outcome;       // OBS_OUT_* (commands)
    uint8_t  tf[6];         // features, count, sector, cyl lo, cyl hi, device/head
    uint8_t  hob[4];        // count, sector, cyl lo, cyl hi before their last write
    uint16_t bits;          // OBS_B_*
    uint8_t  status;        // Status at the end (commands)
    uint8_t  error;         // Error at the end (commands)
    uint16_t handed;        // blocks handed to the caller
    uint16_t witnessed;     // blocks taken with the drive's drop seen before them
    uint16_t unwitnessed;   // blocks taken with no drop seen (holistic review round 2, R-1)
} obs_log_entry_t;

// Hash entry flags.
#define OBS_H_UNWITNESSED   0x01    // the block was taken with no drop seen before it
#define OBS_H_READ_LONG     0x02    // READ LONG's 512 data bytes (not the ECC bytes)
#define OBS_H_SALVAGE       0x04    // data offered with ERR (salvage capture), never handed over
#define OBS_H_SAT           0x08    // an ATA PASS-THROUGH data-in block
#define OBS_H_PREFETCH      0x10    // read ahead by pipelined reads (stage 2)
#define OBS_H_NO_HASH       0x80    // the engine did not finish: digest is all zero

typedef struct {
    uint32_t seq;           // 1, 2, 3 ...; 0: never written
    uint32_t lba;
    uint32_t cmd_seq;       // the log entry of the command that read it
    uint16_t index;         // the block's place in that command, from 0
    uint8_t  flags;         // OBS_H_*
    uint8_t  reserved;
    uint8_t  digest[32];    // SHA-256, in the usual byte order
} obs_hash_entry_t;

#ifdef __cplusplus
static_assert(sizeof(obs_log_entry_t) == 32, "log entry is 32 bytes");
static_assert(sizeof(obs_hash_entry_t) == 48, "hash entry is 48 bytes");
#else
_Static_assert(sizeof(obs_log_entry_t) == 32, "log entry is 32 bytes");
_Static_assert(sizeof(obs_hash_entry_t) == 48, "hash entry is 48 bytes");
#endif

void     obs_init(void);                        // BOOT entry, hash engine self-test
void     obs_set_host(bool on);                 // ide.c: inside a USB host command or not
void     obs_set_read_ahead(bool on);           // usb.c: a pipelined read ahead (stage 2)
// Bus events (ide.c). Only the core using the bus calls these.
void     obs_tf_write(uint8_t reg, uint8_t val);    // registers 1 to 6
uint32_t obs_cmd(uint8_t opcode);                   // returns the entry's seq
uint32_t obs_refused(uint8_t opcode);               // the tripwire (ide.h); returns the entry's seq
void     obs_ctrl(uint8_t val);
void     obs_reset(uint8_t where);
void     obs_status(uint8_t st);                    // a Status read (register 7)
void     obs_error(uint8_t err);                    // an Error read (register 1)
void     obs_fail(uint8_t kind, uint8_t st);        // a failure record for the current command
void     obs_finish(uint32_t seq, uint32_t handed, uint32_t taken, uint32_t unwitnessed, bool ok);
uint32_t obs_current_cmd(void);                     // seq of the last command written, 0 if none
// Markers (menus.c, core 1).
void     obs_marker(uint8_t kind, uint8_t value, uint8_t mode, uint8_t spt,
                    uint8_t heads, uint16_t cyls, uint8_t dev);
// Hashes (ide.c, core 0). The data must stay as it is until obs_hash_flush()
// (the block is only queued; the engine may read it any time until then).
void     obs_hash_sector(uint32_t cmd_seq, uint32_t lba, uint32_t index, uint8_t flags,
                         const uint8_t *data);
void     obs_hash_step(void);                        // start or collect one (ide.c's BSY waits)
void     obs_hash_flush(void);                       // all queued, before the buffer is handed on
bool     obs_hash_engine_ok(void);
// READ BUFFER pages (usb.c): header and entries, at most cap bytes.
uint32_t obs_log_page(uint32_t cursor, uint8_t *out, uint32_t cap);
uint32_t obs_hash_page(uint32_t cursor, uint8_t *out, uint32_t cap);

#if ATABOY_CTL
// The ATAboy Control API's own entries (palimpsest api/SPEC-v1.md 5.4): kind
// 7, value = the op (ctl_policy.h, CTL_OP_*), tf[0..3] = the request id,
// little-endian, outcome = what the job did. Core 1 writes one when it picks
// a job up (API_BEGIN) and one when the job ends (API_END), so every bus
// event of a job lies between the two. They are not bus events: they never
// become the current command. A reader built before this kind existed must
// reject a page holding one rather than skip it.
#define OBS_EV_API          7
#define OBS_API_BEGIN       0x00
#define OBS_API_DONE        0x01
#define OBS_API_FAILED      0x80
#define OBS_API_UNKNOWN     0xFF
uint32_t obs_api(uint8_t op, uint32_t id, uint8_t outcome);     // returns the entry's seq
// The seq of the last mount or unmount marker (status.mount_log_seq).
uint32_t obs_last_marker_seq(void);
#endif

#endif
