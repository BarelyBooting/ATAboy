#ifndef IDE_H
#define IDE_H

#include <stdint.h>
#include <stdbool.h>

// --- Pin Definitions ---
// Data bus: GPIO 0-15   (PIO-managed)
// DIOR:     GPIO 26     (PIO-managed)
// DIOW:     GPIO 27     (PIO-managed)
// All others: SIO-managed from C

#define IDE_DIR         16
#define IDE_DIR1        17
#define IDE_OE          18
#define IDE_OE1         19
#define IDE_A0          20
#define IDE_A1          21
#define IDE_A2          22
#define IDE_RESET       23
#define IDE_CS0         24
#define IDE_CS1         25
#define IDE_DIOR        26      // PIO side-set (read SM)
#define IDE_DIOW        27      // PIO side-set (write SM)
#define IDE_IORDY       29      // 74LVC2G125 Buffer 1 output (1Y) → GPIO 29
#define IDE_INTRQ       28      // 74LVC2G125 Buffer 2 output (2Y) → GPIO 28

#define DATA_MASK       0x0000FFFF
#define ADDR_MASK       ((1 << IDE_A0) | (1 << IDE_A1) | (1 << IDE_A2))

// --- IDE Interface ---
void    ide_select_device(uint8_t base);   // 0xA0 = master, 0xB0 = slave
uint8_t ide_probe_devices(void);           // reset + scan master/slave, return dev_base or 0
void    ide_hw_init(void);
void    ide_reset_drive(void);
bool    ide_wait_until_ready(uint32_t timeout_ms);

void    ide_write_reg(uint8_t reg, uint8_t val);
uint8_t ide_read_reg(uint8_t reg);
uint8_t ide_read_alt_status(void);
void    ide_write_control(uint8_t val);
void    ide_set_iordy(bool enabled);

bool    ide_identify(uint16_t *buf);
bool    ide_set_geometry(uint8_t heads, uint8_t spt);
void    ide_drain_sector(void);

int32_t ide_read_sectors(uint32_t lba, uint32_t count, uint8_t *buf);
int32_t ide_write_sectors(uint32_t lba, uint32_t count, const uint8_t *buf);

// Same as ide_read_sectors, but on failure *done is set to the number of
// sectors that were read into buf before the failing one. Those sectors are
// good data from the drive. Nothing is written to buf for the failing sector
// or any sector after it.
int32_t ide_read_sectors_partial(uint32_t lba, uint32_t count, uint8_t *buf,
                                 uint32_t *done);

// What the drive reported for the last failed sector read or write, captured
// before any recovery step (drain or soft reset) could change it.
#define IDE_FAIL_NONE       0
#define IDE_FAIL_NOT_READY  1   // drive not ready before the command was sent
#define IDE_FAIL_ERR        2   // drive finished the command with ERR set
#define IDE_FAIL_TIMEOUT    3   // no DRQ / no completion in time
#define IDE_FAIL_STALE_DRQ  4   // drive still offering data from an earlier command
#define IDE_FAIL_NO_GEOMETRY 5  // CHS mode, and the drive would not take our geometry
typedef struct {
    uint8_t  kind;          // IDE_FAIL_*
    uint8_t  command;       // ATA command that failed
    uint8_t  status;        // status register when the failure was seen
    uint8_t  error;         // error register (meaningful when status has ERR)
    uint8_t  tf[5];         // registers 2..6: count, sector, cyl lo, cyl hi, dev/head
    bool     drained;       // drive offered data for the failed sector; discarded
    bool     reset;         // a soft reset was needed to get the drive back
    bool     reset_failed;  // ...and the drive did not come back ready, or in CHS
                            // mode did not take the geometry again
    uint32_t lba;           // first sector not transferred
    uint32_t done;          // sectors transferred before the failure
    uint32_t count;         // sectors requested
} ide_fail_t;

// Copy of the last failure record (kind == IDE_FAIL_NONE if none yet).
void    ide_last_failure(ide_fail_t *out);

// Read task file registers 1-7 into tf[1]..tf[7] (tf[0] unused).
void    ide_read_taskfile(uint8_t tf[8]);

// Issue a single-sector READ SECTORS to 'target' (LBA or cylinder depending
// on 'lba' flag), wait for completion, and drain the DRQ data.
// Returns the status register value after the operation.
uint8_t ide_seek_read_one(uint32_t target, bool lba);

#if ATABOY_SAT
#include "sat_policy.h"

// Result of ide_sat_pio_in() / ide_sat_nondata(). Only IDE_SAT_OK means the
// command completed without error (and, for PIO, that the buffer holds data).
#define IDE_SAT_OK          0
#define IDE_SAT_NOT_ISSUED  1   // drive busy or in an unexpected state; no command sent
#define IDE_SAT_ATA_ERROR   2   // the drive set ERR or DF; *regs holds what it reported
#define IDE_SAT_TIMEOUT     3   // BSY or DRQ never came; the command was aborted (SRST)
#define IDE_SAT_BAD_END     4   // the drive offered data it should not have; aborted (SRST)

// The drive's output registers after a SAT command, read before anything else
// could change them. Filled on IDE_SAT_ATA_ERROR, and on IDE_SAT_OK from
// ide_sat_nondata(). The HOB bytes are read (hob == true) only for a 48-bit
// command the drive did not abort: a drive that aborted it may predate 48-bit
// ATA, and the HOB bit in Device Control means nothing to it.
typedef struct {
    uint8_t error, count, lba_low, lba_mid, lba_high, device, status;
    uint8_t hob_count, hob_lba_low, hob_lba_mid, hob_lba_high;
    bool    hob;
} ide_sat_regs_t;

// Issue one PIO data-in command built by sat_policy_check() and read
// tf->sectors * 512 bytes into buf. No soft reset on an ATA error.
int ide_sat_pio_in(const sat_taskfile_t *tf, uint8_t *buf, ide_sat_regs_t *regs);

// Issue one non-data command built by sat_policy_check() and wait for it to
// end. Never touches the data register. No soft reset on an ATA error.
int ide_sat_nondata(const sat_taskfile_t *tf, ide_sat_regs_t *regs);

// The drive's own IDENTIFY DEVICE words 82, 83 and 84 (command sets and
// features supported), from the last ide_identify() on the selected device.
// valid is false when there is nothing to trust: no IDENTIFY since power-up,
// the last one failed, a probe (ide_probe_devices) has run since, or another
// device is selected now. Only the firmware's own IDENTIFY (detection,
// auto-mount, the debug screen) sets them; a host's pass-through IDENTIFY
// does not. sat.c hands them to the policy, which gates the optional
// commands on them (sat_policy.h, "Drive capability").
typedef struct {
    bool     valid;
    uint16_t w82, w83, w84;
} ide_id_words_t;
void ide_id_words(ide_id_words_t *out);
// Forget the captured IDENTIFY words. Called at unmount: the next drive on
// the cable may be a different one, and it must not be judged by this one's
// IDENTIFY. Until a detection captures new words, the gated SAT rows refuse.
void ide_id_words_forget(void);
// Re-IDENTIFY the drive and check it is the one the words came from (serial,
// model, words 82..84): 1 yes; 0 no or IDENTIFY failed (words forgotten), or
// no words held (nothing sent); -1 drive busy or offering stale data (nothing
// sent, words kept).
int ide_id_words_verify(void);
#endif

#endif
