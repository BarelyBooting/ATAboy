#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// 0x1DE45707 until 0.6f3p10's holistic review (2026-09-26, L-2), which added
// crc: settings saved by an older build fail the check and load as defaults.
// 0x1DE45708 until F1 stage 2 (2026-09-26) added pipelined_reads: settings
// saved before it load as defaults once (Pipelined Reads Off).
#define CONFIG_MAGIC 0x1DE45709

typedef struct {
    uint32_t magic;
    uint8_t  main_selected;
    uint8_t  feat_selected;
    bool     drive_write_protected;
    bool     auto_mount;
    bool     iordy_enabled;
    bool     intrq_enabled;
    bool     use_lba_mode;
    uint16_t cyls;
    uint8_t  heads;
    uint8_t  spt;
    uint64_t lba_sectors;
    uint8_t  dev_base;            // 0xA0 = master, 0xB0 = slave
    bool     pipelined_reads;     // F1 stage 2: Features, Pipelined Reads (usb.c); Off by default
    uint32_t crc;                 // config_crc() of everything above (L-2)
} config_t;

// Holistic review L-2: the settings sector used to be trusted on its magic
// alone, so a torn write or a flipped bit with the first 4 bytes intact
// could turn Auto Mount on (IDENTIFY and 91h at the next boot, with no key
// pressed), put the LBA bit into every device register write (dev_base),
// or give 17 heads. Now a CRC-32 over the fields, and every field in range,
// or the defaults (write protect on, Auto Mount off) are used instead.
static inline uint32_t config_crc32_add(uint32_t crc, const uint8_t *p, unsigned n) {
    for (unsigned i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
}
// Over each field's bytes as stored (not the padding between them).
#define CONFIG_CRC_FIELD(crc, c, f) crc = config_crc32_add(crc, (const uint8_t *)&(c)->f, sizeof((c)->f))
static inline uint32_t config_crc(const config_t *c) {
    uint32_t crc = 0xFFFFFFFFu;
    CONFIG_CRC_FIELD(crc, c, magic);
    CONFIG_CRC_FIELD(crc, c, main_selected);
    CONFIG_CRC_FIELD(crc, c, feat_selected);
    CONFIG_CRC_FIELD(crc, c, drive_write_protected);
    CONFIG_CRC_FIELD(crc, c, auto_mount);
    CONFIG_CRC_FIELD(crc, c, iordy_enabled);
    CONFIG_CRC_FIELD(crc, c, intrq_enabled);
    CONFIG_CRC_FIELD(crc, c, use_lba_mode);
    CONFIG_CRC_FIELD(crc, c, cyls);
    CONFIG_CRC_FIELD(crc, c, heads);
    CONFIG_CRC_FIELD(crc, c, spt);
    CONFIG_CRC_FIELD(crc, c, lba_sectors);
    CONFIG_CRC_FIELD(crc, c, dev_base);
    CONFIG_CRC_FIELD(crc, c, pipelined_reads);
    return ~crc;
}
static inline bool config_bool_ok(const bool *b) { uint8_t v; memcpy(&v, b, 1); return v <= 1; }
static inline bool config_valid(const config_t *c) {
    if (c->magic != CONFIG_MAGIC || c->crc != config_crc(c)) return false;
    if (!config_bool_ok(&c->drive_write_protected) || !config_bool_ok(&c->auto_mount) ||
        !config_bool_ok(&c->iordy_enabled) || !config_bool_ok(&c->intrq_enabled) ||
        !config_bool_ok(&c->use_lba_mode) || !config_bool_ok(&c->pipelined_reads)) return false;
    if (c->main_selected > 4 || c->feat_selected > 5) return false;
    if (c->dev_base != 0xA0 && c->dev_base != 0xB0) return false;
    if (c->heads > 16) return false;
    bool none = c->cyls == 0 && c->heads == 0 && c->spt == 0;
    bool chs  = c->cyls >= 1 && c->heads >= 1 && c->spt >= 1;
    if (!c->use_lba_mode && !none && !chs) return false;
    if (c->lba_sectors >= (1ull << 48)) return false;
    return true;
}

extern config_t config;

void config_load(void);
void config_save(void);
void config_defaults(void);
#if ATABOY_CTL
// config.get (palimpsest api/SPEC-v1.md 4.5): whether config_load() found a
// valid sector, and a copy of the sector as it is now, read through XIP.
// Reading it never writes anything; a save pauses core 0 (config_save).
bool config_loaded_from_flash(void);
void config_flash_copy(config_t *out);
#endif

#endif
