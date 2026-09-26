// Observers: the bus event log and the per-sector hashes (obs.h).
// Owner decision F1, 2026-09-26, stage 1. Nothing here reads or changes
// what the data path does with a block; it only watches.
//
// The log
// -------
// One entry per bus event, in the order they happened: every write of the
// command register (ide_write_reg, register 7), every Device Control write
// (ide_write_control), every RESET- pulse (the four places that pull it
// low), and the mount and unmount markers from the menus. A command's entry
// holds the task file registers as last written before it, which of them
// were written since the command before, the Status and Error read while it
// was the current command (frozen when its path records a failure or its
// end), and, from the path that sent it, the blocks handed to the caller,
// taken with the drop seen and taken without (R-1), and the outcome.
//
// Retention (the owner asked for a clear-on-mount policy): the log is NOT
// cleared on mount, or on anything but a reset of the ATAboy itself. A
// mount writes a MOUNT marker instead, with the geometry mode. Clearing at
// mount would throw away the very commands a proof needs: Ctrl+G sends
// RESET-, RECALIBRATE and (91h mode) 91h before the drive is mounted, and
// what came before a mount is what shows no ECh or 91h reached the drive.
// The log is a ring of OBS_LOG_ENTRIES; the oldest entries are overwritten.
// A host that reads it often enough (after every READ(10) of 128 sectors is
// far more than enough: that is 16 entries) never loses one, and a gap is
// always visible (the page header's oldest-held seq, and flag bit 0).
// So "no ECh and no 91h since the mount marker" is proven when the host has
// every seq from that marker to now; with any gap it proves nothing.
//
// Concurrency. Each writer claims its seq with one atomic add, so the two
// cores never write the same entry. An entry's seq is set to 0 while it is
// filled and to its number last, with a barrier between; a reader copies it
// and takes it only if seq was that number before and after the copy. A
// command's status, error and counts are filled in later, in place; a
// reader may see some of them from before (OBS_B_FINISHED is set last).
// Bus events come from whichever core owns the bus, which is one at a time
// (usb.c, msc_busy); only the markers come from core 1 while core 0 may be
// in a command, and they never become the "current" command.
//
// The hashes
// ----------
// SHA-256 of each 512-byte block as it came off the bus (READ SECTORS,
// READ LONG's data, the salvage capture, ATA PASS-THROUGH data-in), in a
// ring of OBS_HASH_ENTRIES keyed by LBA and by the command's log seq. On the
// RP2350 the SHA-256 accelerator does the work, fed by two chained DMA
// channels (the block, then a fixed padding block: every message is 512
// bytes, so the padding never changes), so the CPU only starts it and later
// collects the digest. Where it is started and collected is below
// (obs_hash_step): in the read loop's waits for a busy drive, and at the end
// of the command. Nothing is hashed from anywhere but the caller's buffer
// after the block is in it, and the buffer is not handed on before
// obs_hash_flush has run. At boot the
// engine hashes a known block through the same path and is compared with a
// known digest (the self-test); if that fails no hashes are kept, and the
// page header says so. Host test builds use a software SHA-256 instead.

#include "obs.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <string.h>

#if !ATABOY_HOST_TEST
#include "hardware/dma.h"
#include "hardware/sha256.h"
#include "hardware/sync.h"
#endif

static obs_log_entry_t  log_ring[OBS_LOG_ENTRIES];
static obs_hash_entry_t hash_ring[OBS_HASH_ENTRIES];
static uint32_t log_next = 1;           // the seq the next entry gets
static uint32_t hash_next = 1;
static uint32_t log_base = 1, hash_base = 1;    // the first seq (host tests may move it)

static inline void obs_barrier(void) { __atomic_thread_fence(__ATOMIC_SEQ_CST); }

// ---------------------------------------------------------------------------
//  The log
// ---------------------------------------------------------------------------

static uint8_t  tf_shadow[6];           // registers 1 to 6 as last written
static uint8_t  hob_shadow[4];
static uint16_t tf_written;             // OBS_B_TF_WRITTEN | OBS_B_HOB_WRITTEN since the last command
static obs_log_entry_t *cur;            // the current bus event
static uint32_t cur_seq;
static volatile bool host_on;           // ide.c: inside a USB host command

void obs_set_host(bool on) { host_on = on; }

static obs_log_entry_t *log_claim(uint32_t *seq_out, uint8_t kind, uint8_t value) {
    uint32_t seq = __atomic_fetch_add(&log_next, 1, __ATOMIC_SEQ_CST);
    obs_log_entry_t *e = &log_ring[seq % OBS_LOG_ENTRIES];
    e->seq = 0;
    obs_barrier();
    uint64_t us = to_us_since_boot(get_absolute_time());
    e->us_lo = (uint32_t)us;
    e->us_hi = (uint8_t)(us >> 32);
    e->kind = kind;
    e->value = value;
    e->outcome = OBS_OUT_OPEN;
    memset(e->tf, 0, sizeof e->tf);
    memset(e->hob, 0, sizeof e->hob);
    e->bits = (get_core_num() ? OBS_B_CORE1 : 0) | (host_on ? OBS_B_HOST : 0);
    e->status = 0;
    e->error = 0;
    e->handed = 0;
    e->witnessed = 0;
    e->unwitnessed = 0;
    *seq_out = seq;
    return e;
}

static void log_publish(obs_log_entry_t *e, uint32_t seq) {
    obs_barrier();
    e->seq = seq;
    obs_barrier();
}

// A bus event: it becomes the current one, which ends the one before (its
// status and error stop following the reads).
static void bus_event(obs_log_entry_t *e, uint32_t seq) {
    if (cur && cur->seq == cur_seq) cur->bits |= OBS_B_FROZEN;
    log_publish(e, seq);
    cur = e;
    cur_seq = seq;
}

void obs_tf_write(uint8_t reg, uint8_t val) {
    if (reg < 1 || reg > 6) return;
    uint8_t i = (uint8_t)(reg - 1);
    if (reg >= 2 && reg <= 5 && (tf_written & (1u << i))) {
        hob_shadow[reg - 2] = tf_shadow[i];         // written twice: the first is the HOB byte
        tf_written |= (uint16_t)(1u << (6 + reg - 2));
    }
    tf_shadow[i] = val;
    tf_written |= (uint16_t)(1u << i);
}

uint32_t obs_cmd(uint8_t opcode) {
    uint32_t seq;
    obs_log_entry_t *e = log_claim(&seq, OBS_EV_CMD, opcode);
    memcpy(e->tf, tf_shadow, sizeof e->tf);
    memcpy(e->hob, hob_shadow, sizeof e->hob);
    e->bits |= tf_written;
    tf_written = 0;
    bus_event(e, seq);
    return seq;
}

void obs_ctrl(uint8_t val) {
    uint32_t seq;
    obs_log_entry_t *e = log_claim(&seq, OBS_EV_CTRL, val);
    bus_event(e, seq);
}

void obs_reset(uint8_t where) {
    uint32_t seq;
    obs_log_entry_t *e = log_claim(&seq, OBS_EV_RESET, where);
    bus_event(e, seq);
}

static bool cur_open(void) {
    return cur && cur->seq == cur_seq && cur->kind == OBS_EV_CMD && !(cur->bits & OBS_B_FROZEN);
}

void obs_status(uint8_t st) {
    if (!cur_open()) return;
    cur->status = st;
    cur->bits |= OBS_B_STATUS;
}

void obs_error(uint8_t err) {
    if (!cur_open()) return;
    cur->error = err;
    cur->bits |= OBS_B_ERROR;
}

// record_failure (ide.c): the failure belongs to the current command only if
// that command is still open; a refusal before anything was sent finds the
// last command frozen already (its path recorded its end).
void obs_fail(uint8_t kind, uint8_t st) {
    if (!cur_open()) return;
    cur->outcome = (uint8_t)(0x80 | kind);
    cur->status = st;
    cur->bits |= OBS_B_STATUS | OBS_B_FROZEN;
}

static uint16_t sat16(uint32_t v) { return v > 0xFFFF ? 0xFFFF : (uint16_t)v; }

void obs_finish(uint32_t seq, uint32_t handed, uint32_t taken, uint32_t unwitnessed, bool ok) {
    if (seq == 0) return;
    obs_log_entry_t *e = &log_ring[seq % OBS_LOG_ENTRIES];
    if (e->seq != seq) return;                      // overwritten already
    e->handed = sat16(handed);
    e->unwitnessed = sat16(unwitnessed);
    e->witnessed = sat16(taken >= unwitnessed ? taken - unwitnessed : 0);
    if (e->outcome == OBS_OUT_OPEN) e->outcome = ok ? OBS_OUT_OK : OBS_OUT_FAILED;
    obs_barrier();
    e->bits |= OBS_B_FINISHED | OBS_B_FROZEN;
}

uint32_t obs_current_cmd(void) {
    return (cur && cur->seq == cur_seq && cur->kind == OBS_EV_CMD) ? cur_seq : 0;
}

void obs_marker(uint8_t kind, uint8_t value, uint8_t mode, uint8_t spt,
                uint8_t heads, uint16_t cyls, uint8_t dev) {
    uint32_t seq;
    obs_log_entry_t *e = log_claim(&seq, kind, value);
    e->tf[0] = mode;
    e->tf[1] = spt;
    e->tf[2] = heads;
    e->tf[3] = (uint8_t)cyls;
    e->tf[4] = (uint8_t)(cyls >> 8);
    e->tf[5] = dev;
    log_publish(e, seq);                            // never the current bus event
}

// ---------------------------------------------------------------------------
//  The hash engine
// ---------------------------------------------------------------------------

static bool engine_ok = false;          // the boot self-test passed
static bool engine_tested = false;

#if ATABOY_HOST_TEST
// Software SHA-256 (FIPS 180-4), for the host tests only.
static const uint32_t sha_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};
static inline uint32_t ror32(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
static void sha_block(uint32_t h[8], const uint8_t *p) {
    uint32_t w[64];
    for (int i = 0; i < 16; i++)
        w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) | ((uint32_t)p[4 * i + 2] << 8) | p[4 * i + 3];
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; i++) {
        uint32_t t1 = hh + (ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25)) + ((e & f) ^ (~e & g)) + sha_k[i] + w[i];
        uint32_t t2 = (ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}
// Tests can break the engine (the self-test must then fail) and give each
// hash a CPU cost on the mock clock (the throughput check).
inline bool obs_test_engine_broken = false;
inline uint64_t obs_test_hash_cpu_ns = 0;
extern uint64_t mock_now_ns;
static uint8_t sw_digest[32];
static bool sw_pending = false;
static void engine_start(const uint8_t *data) {
    static const uint32_t h0[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };
    uint32_t h[8];
    memcpy(h, h0, sizeof h);
    for (int b = 0; b < 8; b++) sha_block(h, data + 64 * b);
    uint8_t pad[64] = { 0x80 };
    pad[62] = 0x10;                                 // 4096 bits, big-endian
    sha_block(h, pad);
    for (int i = 0; i < 8; i++) {
        sw_digest[4 * i] = (uint8_t)(h[i] >> 24); sw_digest[4 * i + 1] = (uint8_t)(h[i] >> 16);
        sw_digest[4 * i + 2] = (uint8_t)(h[i] >> 8); sw_digest[4 * i + 3] = (uint8_t)h[i];
    }
    if (obs_test_engine_broken) sw_digest[0] ^= 1;
    sw_pending = true;
    mock_now_ns += obs_test_hash_cpu_ns;
}
static bool engine_result(uint8_t out[32]) {
    if (!sw_pending) return false;
    sw_pending = false;
    memcpy(out, sw_digest, 32);
    return true;
}
static bool engine_setup(void) { return true; }
#else
// The block, then the fixed padding: 0x80, zeros, and the length in bits
// (4096) big-endian in the last 8 bytes. With BSWAP set the engine takes the
// bytes of each word in memory order, as the SDK's pico_sha256 does.
static const uint8_t __attribute__((aligned(4))) pad_block[64] = { 0x80, [62] = 0x10 };
static int ch_data = -1, ch_pad = -1;
static bool pending = false;
// How long the engine may take over one block: nine 64-byte blocks through
// DMA. A few microseconds; anything past this is taken as a fault, the DMA
// is stopped and the entry says "no hash".
#define OBS_ENGINE_US 200
static bool engine_setup(void) {
    ch_data = dma_claim_unused_channel(false);
    ch_pad = dma_claim_unused_channel(false);
    if (ch_data < 0 || ch_pad < 0) return false;
    dma_channel_config c = dma_channel_get_default_config((uint)ch_data);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_SHA256);
    channel_config_set_chain_to(&c, (uint)ch_pad);
    dma_channel_configure((uint)ch_data, &c, sha256_get_write_addr(), NULL, 128, false);
    dma_channel_config p = dma_channel_get_default_config((uint)ch_pad);
    channel_config_set_transfer_data_size(&p, DMA_SIZE_32);
    channel_config_set_read_increment(&p, true);
    channel_config_set_write_increment(&p, false);
    channel_config_set_dreq(&p, DREQ_SHA256);
    dma_channel_configure((uint)ch_pad, &p, sha256_get_write_addr(), pad_block, 16, false);
    sha256_set_dma_size(4);
    sha256_set_bswap(true);
    return true;
}
static void engine_start(const uint8_t *data) {
    sha256_err_not_ready_clear();
    sha256_start();
    if (((uintptr_t)data & 3u) == 0) {
        dma_channel_set_read_addr((uint)ch_pad, pad_block, false);
        dma_channel_set_trans_count((uint)ch_pad, 16, false);
        dma_channel_set_read_addr((uint)ch_data, data, false);
        dma_channel_set_trans_count((uint)ch_data, 128, true);     // starts; chains to the padding
    } else {
        // Not word aligned (no caller does this today): fed by the CPU.
        for (int i = 0; i < 144; i++) {
            const uint8_t *p = i < 128 ? data + 4 * i : pad_block + 4 * (i - 128);
            uint32_t w = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            uint32_t t0 = time_us_32();
            while (!sha256_is_ready()) if (time_us_32() - t0 > OBS_ENGINE_US) break;
            sha256_put_word(w);
        }
    }
    pending = true;
}
static bool engine_result(uint8_t out[32]) {
    if (!pending) return false;
    pending = false;
    uint32_t t0 = time_us_32();
    while (dma_channel_is_busy((uint)ch_data) || dma_channel_is_busy((uint)ch_pad) || !sha256_is_sum_valid()) {
        if (time_us_32() - t0 > OBS_ENGINE_US) {
            dma_channel_abort((uint)ch_data);
            dma_channel_abort((uint)ch_pad);
            return false;
        }
    }
    if (sha256_err_not_ready()) return false;
    sha256_result_t r;
    sha256_get_result(&r, SHA256_BIG_ENDIAN);
    memcpy(out, r.bytes, 32);
    return true;
}
#endif

// SHA-256 of the 512 bytes i & FFh (python: hashlib.sha256(bytes(i & 0xFF
// for i in range(512)))).
static const uint8_t kat_digest[32] = {
    0x11, 0x00, 0x09, 0xdc, 0xee, 0x21, 0x62, 0x0b, 0x16, 0x6f, 0x3a, 0xbf, 0xec, 0xb5, 0xef, 0xf7,
    0xa8, 0x73, 0xbe, 0x72, 0x9d, 0x1c, 0x2d, 0x53, 0x82, 0x2e, 0x7a, 0xcc, 0x5f, 0x34, 0xeb, 0x9b,
};

static bool engine_self_test(void) {
    static uint8_t __attribute__((aligned(4))) block[512];
    for (int i = 0; i < 512; i++) block[i] = (uint8_t)i;
    uint8_t d[32];
    engine_start(block);
    return engine_result(d) && memcmp(d, kat_digest, 32) == 0;
}

void obs_init(void) {
    uint32_t seq;
    obs_log_entry_t *e = log_claim(&seq, OBS_EV_BOOT, 0);
    log_publish(e, seq);
    engine_ok = engine_setup() && engine_self_test();
    engine_tested = true;
}

bool obs_hash_engine_ok(void) { return engine_ok; }

// ---------------------------------------------------------------------------
//  The hashes
// ---------------------------------------------------------------------------

// Blocks queued (obs_hash_sector) and the one the engine is working on.
// A block is only queued where it is taken off the bus; the engine is
// started by obs_hash_step, which the read loop calls while the drive is
// busy between blocks (ide.c, read_bsy_wait), so the hashing costs that
// loop no time when the drive is slower than the bus, and by
// obs_hash_flush, at the end of each command, for whatever is left.
#define OBS_HASH_QUEUE 256              // a whole ATA command (count 0 is 256)
static struct { obs_hash_entry_t *e; uint32_t seq; const uint8_t *data; } hq[OBS_HASH_QUEUE];
static uint32_t hq_head, hq_tail;       // queued: hq_head up to hq_tail
static obs_hash_entry_t *hash_pend;     // the engine has it
static uint32_t hash_pend_seq;

static void hash_collect(void) {
    if (!hash_pend) return;
    if (!engine_result(hash_pend->digest)) {
        memset(hash_pend->digest, 0, sizeof hash_pend->digest);
        hash_pend->flags |= OBS_H_NO_HASH;
    }
    obs_barrier();
    hash_pend->seq = hash_pend_seq;
    obs_barrier();
    hash_pend = NULL;
}

void obs_hash_step(void) {
    hash_collect();
    if (hq_head == hq_tail) return;
    uint32_t i = hq_head % OBS_HASH_QUEUE;
    hash_pend = hq[i].e;
    hash_pend_seq = hq[i].seq;
    hq_head++;
    engine_start(hq[i].data);
}

void obs_hash_flush(void) {
    while (hash_pend || hq_head != hq_tail) obs_hash_step();
}

void obs_hash_sector(uint32_t cmd_seq, uint32_t lba, uint32_t index, uint8_t flags,
                     const uint8_t *data) {
    if (!engine_ok) return;
    while (hq_tail - hq_head >= OBS_HASH_QUEUE) obs_hash_step();
    uint32_t seq = __atomic_fetch_add(&hash_next, 1, __ATOMIC_SEQ_CST);
    obs_hash_entry_t *e = &hash_ring[seq % OBS_HASH_ENTRIES];
    e->seq = 0;
    obs_barrier();
    e->lba = lba;
    e->cmd_seq = cmd_seq;
    e->index = (uint16_t)index;
    e->flags = flags;
    e->reserved = 0;
    uint32_t i = hq_tail % OBS_HASH_QUEUE;
    hq[i].e = e;
    hq[i].seq = seq;
    hq[i].data = data;
    hq_tail++;
}

// ---------------------------------------------------------------------------
//  READ BUFFER pages
// ---------------------------------------------------------------------------
// Header, 32 bytes, little-endian:
//   0  8  magic: "ATBLOG01" (log) or "ATBSHA01" (hashes)
//   8  4  seq of the first entry in this page (0: none)
//  12  2  entries in this page
//  14  2  entry size (32 or 48)
//  16  4  the seq the next entry will get
//  20  4  the oldest seq still held (0: none yet)
//  24  4  ring size (entries)
//  28  1  flags: bit 0 the cursor asked for entries no longer held (lost);
//              bit 1 hash engine self-test passed; bit 2 it failed (no
//              hashes are kept); bit 3 software engine (host test build)
//  29  3  zero
// Then whole entries only. The cursor is BUFFER OFFSET, the low 24 bits of
// the first seq wanted: the page starts at the one seq held (or next) with
// those low bits; 0 starts at the oldest held. The ring is far smaller than
// 2^24, so the low bits name one seq. An entry being written stops the page
// short (ask again). A host keeps "last seq seen + 1" as its cursor.

static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

#if ATABOY_HOST_TEST
inline void (*obs_test_mid_copy)(uint32_t seq) = nullptr;
#endif
static uint32_t page(bool hashes, uint32_t cursor, uint8_t *out, uint32_t cap) {
    uint32_t next = __atomic_load_n(hashes ? &hash_next : &log_next, __ATOMIC_SEQ_CST);
    uint32_t ring = hashes ? OBS_HASH_ENTRIES : OBS_LOG_ENTRIES;
    uint32_t esize = hashes ? sizeof(obs_hash_entry_t) : sizeof(obs_log_entry_t);
    uint32_t base = hashes ? hash_base : log_base;
    uint32_t oldest = next - base > ring ? next - ring : base;
    if (next == base) oldest = 0;
    uint8_t flags = 0;
    uint32_t want;
    cursor &= 0xFFFFFFu;
    if (cursor == 0) {
        want = oldest ? oldest : next;
    } else {
        want = (next & ~0xFFFFFFu) | cursor;
        if (want > next) want -= 0x1000000u;
        if (want > next) want = next;                   // below seq 1: from the oldest
        if (oldest && want < oldest) { flags |= 1; want = oldest; }
        if (!oldest) want = next;
    }
    if (hashes) {
        if (engine_tested) flags |= engine_ok ? 2 : 4;
#if ATABOY_HOST_TEST
        flags |= 8;
#endif
    }
    uint8_t hdr[OBS_PAGE_HEADER];
    memset(hdr, 0, sizeof hdr);
    memcpy(hdr, hashes ? "ATBSHA01" : "ATBLOG01", 8);
    uint32_t n = 0, first = 0;
    uint32_t room = cap > OBS_PAGE_HEADER ? (cap - OBS_PAGE_HEADER) / esize : 0;
    for (uint32_t s = want; s < next && n < room; s++) {
        uint8_t tmp[48];
        const void *src = hashes ? (const void *)&hash_ring[s % ring] : (const void *)&log_ring[s % ring];
        uint32_t s1 = *(const volatile uint32_t *)src;
        obs_barrier();
        memcpy(tmp, src, esize);
#if ATABOY_HOST_TEST
        if (obs_test_mid_copy) obs_test_mid_copy(s);   // a writer on the other core, now
#endif
        obs_barrier();
        uint32_t s2 = *(const volatile uint32_t *)src;
        if (s1 != s || s2 != s) {
            if (s1 > s || s2 > s) { if (n == 0) flags |= 1; }   // overwritten while reading
            break;
        }
        if (n == 0) first = s;
        memcpy(out + OBS_PAGE_HEADER + n * esize, tmp, esize);
        n++;
    }
    put32(hdr + 8, first);
    hdr[12] = (uint8_t)n; hdr[13] = (uint8_t)(n >> 8);
    hdr[14] = (uint8_t)esize; hdr[15] = 0;
    put32(hdr + 16, next);
    put32(hdr + 20, oldest);
    put32(hdr + 24, ring);
    hdr[28] = flags;
    uint32_t hl = cap < OBS_PAGE_HEADER ? cap : OBS_PAGE_HEADER;
    memcpy(out, hdr, hl);
    return hl + n * esize;
}

uint32_t obs_log_page(uint32_t cursor, uint8_t *out, uint32_t cap) { return page(false, cursor, out, cap); }
uint32_t obs_hash_page(uint32_t cursor, uint8_t *out, uint32_t cap) { return page(true, cursor, out, cap); }

#if ATABOY_HOST_TEST
// Host tests: back to power-up (both rings empty, then the BOOT entry and
// the self-test, as obs_init), optionally with the next seqs set, to test
// the ring's wrap and the cursor's 24 bits.
void obs_test_reset(uint32_t log_first, uint32_t hash_first) {
    memset(log_ring, 0, sizeof log_ring);
    memset(hash_ring, 0, sizeof hash_ring);
    log_next = log_base = log_first ? log_first : 1;
    hash_next = hash_base = hash_first ? hash_first : 1;
    memset(tf_shadow, 0, sizeof tf_shadow);
    memset(hob_shadow, 0, sizeof hob_shadow);
    tf_written = 0;
    cur = NULL; cur_seq = 0;
    host_on = false;
    hash_pend = NULL; hash_pend_seq = 0;
    hq_head = hq_tail = 0;
    sw_pending = false;
    engine_ok = false; engine_tested = false;
    obs_init();
}
#endif
