// Exhaustive host test for sat_policy.c, the firmware's SAT allowlist.
//
// Build and run with run-sat-policy-tests.ps1, which also runs the mutants.
// Exit code 0 only if every assertion passed.
//
// The expected verdicts do NOT come from sat_policy.c. expect_allow() below is
// written a different way on purpose: it pulls out the few fields an allowed
// command may vary (command, SMART subcommand, count, LBA, obsolete device
// bits, CK_COND and the two data-phase bits of a non-data command),
// re-encodes the only CDB those fields are allowed to produce, and compares
// bytes. Any byte that differs from that canonical encoding is a refusal. The
// policy instead checks field by field. Two shapes of the same rule, so a
// mistake in one is unlikely to be repeated in the other.
//
// The allowed set is written out here by hand as well, not taken from the
// policy's header:
//   PIO data-in, 1..8 sectors unless stated:
//     IDENTIFY 0xEC (1), READ SECTORS 0x20 / 0x21 (LBA28), READ SECTORS EXT
//     0x24 (16-byte form, EXTEND=1), SMART 0xB0 with FEATURES D0 or D1 (1
//     sector, LBA low 0) or D5 (LBA low = any log address).
//   Non-data, CBW length 0:
//     READ VERIFY 0x40 (LBA28, count 0..255, 0 = 256, CK_COND 0 or 1),
//     SMART RETURN STATUS 0xB0/DA, READ NATIVE MAX 0xF8, READ NATIVE MAX EXT
//     0x27 (16-byte form, EXTEND=1); these three need CK_COND=1.
//   SMART needs 4F/C2 in LBA mid/high. Reads and verifies need LBA mode;
//   IDENTIFY, SMART and READ NATIVE MAX are allowed in CHS mode too.
//   The drive's captured IDENTIFY (words 82..84, valid only with 01b in bits
//   15:14 of words 83 and 84 and word 82 not 0000h/FFFFh; word 85 valid only
//   with 01b in bits 15:14 of word 87) must show: SMART 82.0 and 85.0 (READ
//   LOG also 84.0), READ NATIVE MAX 82.10, READ NATIVE MAX EXT 82.10 and
//   83.10, READ SECTORS EXT 83.10. IDENTIFY, READ SECTORS and READ VERIFY
//   need nothing. An allowed row that needed something must come back with
//   needs_identity set (sat.c then checks the drive is still that drive);
//   every other verdict must have it clear. Sections 0 to 10 run with a
//   drive that shows everything; section 11 sweeps every combination of
//   those bits.

#include "sat_policy.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define MAXSEC 8u      // this test's own copy of the sector cap

static long n_checks, n_fail, n_allow, n_refuse;
static const char *section = "";

// The drive's IDENTIFY words 82..85 and 87 as the firmware would pass them in.
typedef struct { bool captured; uint16_t w82, w83, w84, w85, w87; } idcaps_t;
// A drive that shows every feature the policy asks about: SMART and NOP (82),
// HPA (82.10), 48-bit (83.10), SMART error logging (84.0), SMART and HPA
// enabled (85), valid signatures (83, 84, 87).
static const idcaps_t FULL_CAPS = { true, 0x4401, 0x4400, 0x4001, 0x4401, 0x4000 };
static idcaps_t caps;          // what check() passes in; FULL_CAPS unless a section says otherwise

static void fail(const char *what, const uint8_t *cdb, int len, bool dir_in, uint32_t xfer,
                 bool mounted, bool lba_mode) {
    n_fail++;
    if (n_fail > 40) return;
    printf("  FAIL [%s] %s: len=%d dir_in=%d xfer=%u mounted=%d lba=%d id=%d/%04X/%04X/%04X/%04X/%04X cdb=",
           section, what, len, dir_in, (unsigned)xfer, mounted, lba_mode,
           caps.captured, caps.w82, caps.w83, caps.w84, caps.w85, caps.w87);
    for (int i = 0; i < 16; i++) printf("%02X%s", cdb[i], i == 15 ? "\n" : " ");
}

// ---------------------------------------------------------------------------
//  Encoders: register values in, CDB bytes out
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t proto, b2, feat, count, lo, mid, hi, hlo, hmid, hhi, dev, cmd;
    bool ext;
} fields_t;

static void put12(uint8_t o[16], const fields_t *f) {
    memset(o, 0, 16);
    o[0] = 0xA1; o[1] = (uint8_t)(f->proto << 1); o[2] = f->b2;
    o[3] = f->feat; o[4] = f->count;
    o[5] = f->lo; o[6] = f->mid; o[7] = f->hi;
    o[8] = f->dev; o[9] = f->cmd;
}

static void put16(uint8_t o[16], const fields_t *f) {
    memset(o, 0, 16);
    o[0] = 0x85; o[1] = (uint8_t)((f->proto << 1) | (f->ext ? 1 : 0)); o[2] = f->b2;
    o[4] = f->feat; o[6] = f->count;
    o[7] = f->hlo; o[8] = f->lo; o[9] = f->hmid; o[10] = f->mid; o[11] = f->hhi; o[12] = f->hi;
    o[13] = f->dev; o[14] = f->cmd;
}

// Stage 1 shapes: PIO data-in, FEATURES 0.
static void enc12(uint8_t o[16], uint8_t cmd, uint8_t count, uint32_t lba28, uint8_t devtop) {
    fields_t f = { 4, 0x0E, 0, count, (uint8_t)lba28, (uint8_t)(lba28 >> 8), (uint8_t)(lba28 >> 16),
                   0, 0, 0, (uint8_t)(devtop | ((lba28 >> 24) & 0x0F)), cmd, false };
    put12(o, &f);
}

static void enc16(uint8_t o[16], uint8_t cmd, bool ext, uint8_t count, uint64_t lba, uint8_t devtop) {
    fields_t f = { 4, 0x0E, 0, count, (uint8_t)lba, (uint8_t)(lba >> 8), (uint8_t)(lba >> 16),
                   0, 0, 0, devtop, cmd, ext };
    if (ext) { f.hlo = (uint8_t)(lba >> 24); f.hmid = (uint8_t)(lba >> 32); f.hhi = (uint8_t)(lba >> 40); }
    else f.dev = (uint8_t)(devtop | ((lba >> 24) & 0x0F));
    put16(o, &f);
}

// Non-data READ VERIFY.
static void encv(uint8_t o[16], bool is16, uint8_t b2, uint8_t count, uint32_t lba28, uint8_t devtop) {
    fields_t f = { 3, b2, 0, count, (uint8_t)lba28, (uint8_t)(lba28 >> 8), (uint8_t)(lba28 >> 16),
                   0, 0, 0, (uint8_t)(devtop | ((lba28 >> 24) & 0x0F)), 0x40, false };
    if (is16) put16(o, &f); else put12(o, &f);
}

// SMART, any subcommand.
static void encs(uint8_t o[16], bool is16, uint8_t proto, uint8_t b2, uint8_t feat, uint8_t count,
                 uint8_t lo, uint8_t dev) {
    fields_t f = { proto, b2, feat, count, lo, 0x4F, 0xC2, 0, 0, 0, dev, 0xB0, false };
    if (is16) put16(o, &f); else put12(o, &f);
}

// READ NATIVE MAX (0xF8, 28-bit) or READ NATIVE MAX EXT (0x27, 16-byte EXTEND=1).
static void encn(uint8_t o[16], bool is16, bool ext, uint8_t cmd, uint8_t b2, uint8_t dev) {
    fields_t f = { 3, b2, 0, 0, 0, 0, 0, 0, 0, 0, dev, cmd, ext };
    if (is16) put16(o, &f); else put12(o, &f);
}

// ---------------------------------------------------------------------------
//  The expectation
// ---------------------------------------------------------------------------

typedef struct {
    unsigned count;     // PIO data-in blocks (0 for non-data)
    bool     nondata;
    uint8_t  tdev;      // device bits the task file must carry
} expect_t;

// The IDENTIFY side of the expectation, written as (word, bit) pairs per row
// rather than the policy's capability mask.
static unsigned id_word(const idcaps_t *k, int word) {
    return word == 82 ? k->w82 : word == 83 ? k->w83 : word == 84 ? k->w84 : word == 85 ? k->w85 : k->w87;
}
static bool id_words_valid(const idcaps_t *k) {
    if (!k->captured) return false;
    if ((id_word(k, 83) >> 14) != 1u || (id_word(k, 84) >> 14) != 1u) return false;
    if (id_word(k, 82) == 0u || id_word(k, 82) == 0xFFFFu) return false;
    return true;
}
// The (word, bit) pairs a row needs. None for IDENTIFY, READ SECTORS and
// READ VERIFY.
static int id_reqs(uint8_t cmd, uint8_t feat, int req[3][2]) {
    int n = 0;
    if (cmd == 0x24) { req[n][0] = 83; req[n][1] = 10; n++; }
    if (cmd == 0xB0) {
        req[n][0] = 82; req[n][1] = 0; n++;
        req[n][0] = 85; req[n][1] = 0; n++;     // SMART enabled, not just supported
        if (feat == 0xD5) { req[n][0] = 84; req[n][1] = 0; n++; }
    }
    if (cmd == 0xF8) { req[n][0] = 82; req[n][1] = 10; n++; }
    if (cmd == 0x27) { req[n][0] = 82; req[n][1] = 10; n++; req[n][0] = 83; req[n][1] = 10; n++; }
    return n;
}
static bool id_shows(const idcaps_t *k, uint8_t cmd, uint8_t feat) {
    int req[3][2];
    int n = id_reqs(cmd, feat, req);
    if (n == 0) return true;
    if (!id_words_valid(k)) return false;
    for (int i = 0; i < n; i++) {
        // word 85 means something only with 01b in bits 15:14 of word 87
        if (req[i][0] == 85 && (id_word(k, 87) >> 14) != 1u) return false;
        if (!((id_word(k, req[i][0]) >> req[i][1]) & 1u)) return false;
    }
    return true;
}

// Would a correct policy allow this?
static bool expect_allow_caps(const uint8_t *c, int len, bool dir_in, uint32_t xfer,
                              bool mounted, bool lba_mode, const idcaps_t *k, expect_t *e_out) {
    bool is16;
    if (c[0] == 0xA1 && len == 12) is16 = false;
    else if (c[0] == 0x85 && len == 16) is16 = true;
    else return false;

    uint8_t cmd  = is16 ? c[14] : c[9];
    uint8_t dev  = is16 ? c[13] : c[8];
    uint8_t feat = is16 ? c[4] : c[3];
    uint8_t cnt  = is16 ? c[6] : c[4];
    uint8_t lo   = is16 ? c[8] : c[5];
    uint8_t mid  = is16 ? c[10] : c[6];
    uint8_t hi   = is16 ? c[12] : c[7];
    uint8_t b2   = c[2];

    fields_t f;
    memset(&f, 0, sizeof(f));
    f.cmd = cmd;
    expect_t e = { 0, false, 0 };
    bool user_data = false;
    // The non-data byte 2: CK_COND, T_DIR and BYTE_BLOCK may be anything.
    uint8_t nd_b2_free = (uint8_t)(b2 & 0x2C);
    uint8_t nd_b2_ck   = (uint8_t)(0x20 | (b2 & 0x0C));

    if (cmd == 0xEC) {
        e.count = 1;
        f.count = 1;
        f.dev = dev & 0xE0;                                         // bits 7,6,5 free
    } else if (cmd == 0x20 || cmd == 0x21 || cmd == 0x40) {
        uint32_t lba = lo | ((uint32_t)mid << 8) | ((uint32_t)hi << 16) | ((uint32_t)(dev & 0x0F) << 24);
        unsigned n;
        if (cmd == 0x40) { n = cnt ? cnt : 256; e.nondata = true; f.b2 = nd_b2_free; }
        else { if (cnt < 1 || cnt > MAXSEC) return false; n = cnt; e.count = cnt; }
        if ((uint64_t)lba + n > 0x10000000ull) return false;
        f.count = cnt; f.lo = lo; f.mid = mid; f.hi = hi;
        f.dev = (uint8_t)((dev & 0xA0) | 0x40 | (dev & 0x0F));
        e.tdev = (uint8_t)(0x40 | (dev & 0x0F));
        user_data = true;
    } else if (cmd == 0x24) {
        if (!is16) return false;
        uint64_t lba = lo | ((uint64_t)mid << 8) | ((uint64_t)hi << 16) |
                       ((uint64_t)c[7] << 24) | ((uint64_t)c[9] << 32) | ((uint64_t)c[11] << 40);
        if (cnt < 1 || cnt > MAXSEC) return false;
        if (lba + cnt > (1ull << 48)) return false;
        e.count = cnt;
        f.ext = true; f.count = cnt; f.lo = lo; f.mid = mid; f.hi = hi;
        f.hlo = c[7]; f.hmid = c[9]; f.hhi = c[11];
        f.dev = (uint8_t)((dev & 0xA0) | 0x40);
        e.tdev = 0x40;
        user_data = true;
    } else if (cmd == 0xB0) {
        f.feat = feat; f.mid = 0x4F; f.hi = 0xC2;
        f.dev = dev & 0xE0;
        // D0 and DA save attribute values to the drive (ATA-3): refused unless
        // built with ATABOY_SAT_SMART_SAVES=1 (sat_policy.h).
        if ((feat == 0xD0 && ATABOY_SAT_SMART_SAVES) || feat == 0xD1) {
            e.count = 1; f.count = 1;
        } else if (feat == 0xD5) {
            if (cnt < 1 || cnt > MAXSEC) return false;
            e.count = cnt; f.count = cnt; f.lo = lo;
        } else if (feat == 0xDA && ATABOY_SAT_SMART_SAVES) {
            e.nondata = true; f.b2 = nd_b2_ck;
        } else {
            return false;
        }
    } else if (cmd == 0xF8 || cmd == 0x27) {
        if (cmd == 0x27) { if (!is16) return false; f.ext = true; }
        e.nondata = true; f.b2 = nd_b2_ck;
        f.dev = (uint8_t)((dev & 0xA0) | 0x40);
        e.tdev = 0x40;
    } else {
        return false;
    }
    if (e.nondata) f.proto = 3;
    else { f.proto = 4; f.b2 = 0x0E; }

    uint8_t canon[16];
    if (is16) put16(canon, &f); else put12(canon, &f);
    if (memcmp(canon, c, is16 ? 16u : 12u) != 0) return false;

    if (e.nondata) { if (xfer != 0) return false; }
    else if (!dir_in || xfer != e.count * 512u) return false;
    if (!mounted) return false;
    if (user_data && !lba_mode) return false;
    if (!id_shows(k, cmd, feat)) return false;
    if (e_out) *e_out = e;
    return true;
}

static bool expect_allow(const uint8_t *c, int len, bool dir_in, uint32_t xfer,
                         bool mounted, bool lba_mode, expect_t *e_out) {
    return expect_allow_caps(c, len, dir_in, xfer, mounted, lba_mode, &caps, e_out);
}

static void set_caps(sat_input_t *in, const idcaps_t *k) {
    in->id_captured = k->captured;
    in->id_w82 = k->w82; in->id_w83 = k->w83; in->id_w84 = k->w84;
    in->id_w85 = k->w85; in->id_w87 = k->w87;
}

// ---------------------------------------------------------------------------
//  One assertion: run the policy, compare with the expectation
// ---------------------------------------------------------------------------

static bool check(const uint8_t cdb_in[16], int len, bool dir_in, uint32_t xfer,
                  bool mounted, bool lba_mode) {
    uint8_t cdb[16];
    memcpy(cdb, cdb_in, 16);
    sat_input_t in = { cdb, (uint8_t)len, dir_in, xfer, mounted, lba_mode, false, 0, 0, 0, 0, 0 };
    set_caps(&in, &caps);
    sat_taskfile_t tf;
    memset(&tf, 0xAA, sizeof(tf));
    sat_verdict_t v = sat_policy_check(&in, &tf);

    expect_t e;
    bool exp = expect_allow(cdb, len, dir_in, xfer, mounted, lba_mode, &e);
    n_checks++;

    if (memcmp(cdb, cdb_in, 16) != 0) fail("policy modified the CDB", cdb_in, len, dir_in, xfer, mounted, lba_mode);

    if (v.allow != exp) {
        fail(exp ? "expected ALLOW, got REFUSE" : "expected REFUSE, got ALLOW", cdb, len, dir_in, xfer, mounted, lba_mode);
        return v.allow;
    }

    if (v.allow) {
        n_allow++;
        {
            // needs_identity: exactly the rows that needed IDENTIFY words
            int req[3][2];
            bool is16c = cdb[0] == 0x85;
            bool want_id = id_reqs(is16c ? cdb[14] : cdb[9], is16c ? cdb[4] : cdb[3], req) > 0;
            if (v.needs_identity != want_id)
                fail(want_id ? "allowed row that needed IDENTIFY without needs_identity"
                             : "needs_identity on a row that needs no IDENTIFY", cdb, len, dir_in, xfer, mounted, lba_mode);
        }
        // The task file must be exactly what the CDB said, and only a read.
        bool is16 = cdb[0] == 0x85;
        uint8_t cmd = is16 ? cdb[14] : cdb[9];
        bool ext = is16 && (cdb[1] & 1);
        sat_taskfile_t w;
        memset(&w, 0, sizeof(w));
        w.command = cmd;
        w.feature = cmd == 0xB0 ? (is16 ? cdb[4] : cdb[3]) : 0;
        w.count = cmd == 0xEC ? 1 : (is16 ? cdb[6] : cdb[4]);
        w.sectors = (uint8_t)e.count;
        w.lba_low = is16 ? cdb[8] : cdb[5];
        w.lba_mid = is16 ? cdb[10] : cdb[6];
        w.lba_high = is16 ? cdb[12] : cdb[7];
        w.ext = ext;
        if (ext) { w.hob_lba_low = cdb[7]; w.hob_lba_mid = cdb[9]; w.hob_lba_high = cdb[11]; }
        w.device = e.tdev;
        w.protocol = e.nondata ? 3 : 4;
        w.ck_cond = e.nondata && (cdb[2] & 0x20);
        bool read_set = tf.command == 0xEC || tf.command == 0x20 || tf.command == 0x21 ||
                        tf.command == 0x24 || tf.command == 0x40 || tf.command == 0xF8 ||
                        tf.command == 0x27 ||
                        (tf.command == 0xB0 && (tf.feature == 0xD0 || tf.feature == 0xD1 ||
                                                tf.feature == 0xD5 || tf.feature == 0xDA));
        if (!read_set)
            fail("task file carries a command outside the read set", cdb, len, dir_in, xfer, mounted, lba_mode);
        if (tf.command != w.command || tf.count != w.count || tf.sectors != w.sectors ||
            tf.lba_low != w.lba_low || tf.lba_mid != w.lba_mid || tf.lba_high != w.lba_high ||
            tf.hob_lba_low != w.hob_lba_low || tf.hob_lba_mid != w.hob_lba_mid ||
            tf.hob_lba_high != w.hob_lba_high || tf.hob_count != 0 || tf.feature != w.feature ||
            tf.hob_feature != 0 || tf.ext != w.ext || tf.device != w.device ||
            tf.protocol != w.protocol || tf.ck_cond != w.ck_cond)
            fail("task file does not match the CDB", cdb, len, dir_in, xfer, mounted, lba_mode);
        if (e.nondata) {
            if (tf.sectors != 0 || xfer != 0)
                fail("non-data task file has a data phase", cdb, len, dir_in, xfer, mounted, lba_mode);
        } else if (tf.sectors < 1 || tf.sectors > MAXSEC || (uint32_t)tf.sectors * 512u != xfer) {
            fail("task file transfer does not match the CBW", cdb, len, dir_in, xfer, mounted, lba_mode);
        }
    } else {
        n_refuse++;
        if (v.needs_identity) fail("refusal with needs_identity set", cdb, len, dir_in, xfer, mounted, lba_mode);
        static const sat_taskfile_t zero;
        if (memcmp(&tf, &zero, sizeof(tf)) != 0)
            fail("refusal left data in the task file", cdb, len, dir_in, xfer, mounted, lba_mode);
        bool ok_sense =
            (v.sense_key == 0x05 && (v.asc == 0x20 || v.asc == 0x21 || v.asc == 0x24) && v.ascq == 0) ||
            (v.sense_key == 0x02 && v.asc == 0x04 && v.ascq == 0);
        if (!ok_sense) fail("refusal sense is not 5/20, 5/21, 5/24 or 2/04", cdb, len, dir_in, xfer, mounted, lba_mode);
        // NOT READY exactly when nothing is mounted and the CDB itself is
        // valid. (With nothing mounted, the LBA-mode setting and the drive's
        // IDENTIFY mean nothing: NOT READY comes first.)
        bool only_unmounted = !mounted && expect_allow_caps(cdb, len, dir_in, xfer, true, true, &FULL_CAPS, NULL);
        if (only_unmounted != (v.sense_key == 0x02))
            fail(only_unmounted ? "unmounted: expected NOT READY" : "NOT READY for a CDB that is invalid anyway",
                 cdb, len, dir_in, xfer, mounted, lba_mode);
    }
    return v.allow;
}

// Seeds: one canonical CDB for every allowed row, at a few LBAs and in the
// byte-2 and device-byte variants real tools send.
typedef struct { uint8_t cdb[16]; int len; bool dir_in; uint32_t xfer; } seed_t;
static seed_t seeds[600];
static int n_seeds;

static seed_t *new_seed(int len, bool dir_in, uint32_t xfer) {
    seed_t *s = &seeds[n_seeds++];
    s->len = len; s->dir_in = dir_in; s->xfer = xfer;
    return s;
}
static void add_seed12(uint8_t cmd, uint8_t count, uint32_t lba, uint8_t devtop) {
    enc12(new_seed(12, true, count * 512u)->cdb, cmd, count, lba, devtop);
}
static void add_seed16(uint8_t cmd, bool ext, uint8_t count, uint64_t lba, uint8_t devtop) {
    enc16(new_seed(16, true, count * 512u)->cdb, cmd, ext, count, lba, devtop);
}

static void build_seeds(void) {
    // IDENTIFY exactly as probe.ps1 and imagelba.ps1 send it: A1 08 0E 00 01 00 00 00 00 EC 00 00
    add_seed12(0xEC, 1, 0, 0x00);
    add_seed12(0xEC, 1, 0, 0xA0);
    add_seed12(0xEC, 1, 0, 0xE0);
    add_seed16(0xEC, false, 1, 0, 0x00);
    add_seed16(0xEC, false, 1, 0, 0xE0);
    static const uint32_t l28[] = { 0, 1, 0x1234, 0x04C92C7F, 0x0FFFFFF7 };
    for (unsigned i = 0; i < sizeof(l28) / sizeof(l28[0]); i++)
        for (uint8_t n = 1; n <= MAXSEC; n++) {
            // imagelba.ps1 Cdb-AtaLbaRead: device 0xE0 | LBA 27:24, command 0x20
            add_seed12(0x20, n, l28[i], 0xE0);
            if (n == 1 || n == MAXSEC) {
                add_seed12(0x21, n, l28[i], 0x40);
                add_seed16(0x20, false, n, l28[i], 0xE0);
                add_seed16(0x21, false, n, l28[i], 0x40);
            }
        }
    static const uint64_t l48[] = { 0, 0x0FFFFFFF, 0x10000000, 0x123456789ABull, 0xFFFFFFFFFFF8ull };
    for (unsigned i = 0; i < sizeof(l48) / sizeof(l48[0]); i++) {
        add_seed16(0x24, true, 1, l48[i], 0x40);
        add_seed16(0x24, true, MAXSEC, l48[i], 0xE0);
    }

    // READ VERIFY (non-data). hdparm sends byte 2 = 0x20 (CK_COND only),
    // smartctl 0x2C (T_DIR and BYTE_BLOCK as well).
    static const uint32_t v28[] = { 0, 1, 0x1234, 0x0950FA0F, 0x0FFFFF00 };
    static const uint8_t vcnt[] = { 1, 8, 255, 0 };
    static const uint8_t vb2[] = { 0x00, 0x20, 0x2C, 0x0C, 0x08, 0x04, 0x28, 0x24 };
    for (unsigned i = 0; i < sizeof(v28) / sizeof(v28[0]); i++)
        for (unsigned k = 0; k < sizeof(vcnt) / sizeof(vcnt[0]); k++)
            for (unsigned b = 0; b < sizeof(vb2) / sizeof(vb2[0]); b++) {
                if (b > 1 && k != 0 && i != 2) continue;          // every byte-2 form, at a few places
                encv(new_seed(12, false, 0)->cdb, false, vb2[b], vcnt[k], v28[i], 0xE0);
                encv(new_seed(16, false, 0)->cdb, true, vb2[b], vcnt[k], v28[i], 0x40);
            }

    // SMART. smartread.ps1: A1 08 0E D0 01 00 4F C2 A0 B0 00 00 (and D1).
    static const uint8_t sdev[] = { 0x00, 0xA0, 0xE0, 0x40 };
    for (unsigned d = 0; d < sizeof(sdev) / sizeof(sdev[0]); d++)
        for (int form = 0; form < 2; form++) {
            bool is16 = form == 1;
            int len = is16 ? 16 : 12;
            encs(new_seed(len, true, 512)->cdb, is16, 4, 0x0E, 0xD1, 1, 0, sdev[d]);
            if (ATABOY_SAT_SMART_SAVES) {   // D0 and DA: see sat_policy.h
                encs(new_seed(len, true, 512)->cdb, is16, 4, 0x0E, 0xD0, 1, 0, sdev[d]);
                encs(new_seed(len, false, 0)->cdb, is16, 3, 0x20, 0xDA, 0, 0, sdev[d]);
                encs(new_seed(len, false, 0)->cdb, is16, 3, 0x2C, 0xDA, 0, 0, sdev[d]);
            }
        }
    static const uint8_t logs[] = { 0x00, 0x01, 0x06, 0x80, 0xE0, 0xFF };
    for (unsigned g = 0; g < sizeof(logs) / sizeof(logs[0]); g++)
        for (uint8_t n = 1; n <= MAXSEC; n++) {
            if (n != 1 && n != MAXSEC && g != 1) continue;
            encs(new_seed(12, true, n * 512u)->cdb, false, 4, 0x0E, 0xD5, n, logs[g], 0xA0);
            encs(new_seed(16, true, n * 512u)->cdb, true, 4, 0x0E, 0xD5, n, logs[g], 0x00);
        }
    if (ATABOY_SAT_SMART_SAVES) {
        encs(new_seed(12, false, 0)->cdb, false, 3, 0x28, 0xDA, 0, 0, 0xA0);
        encs(new_seed(12, false, 0)->cdb, false, 3, 0x24, 0xDA, 0, 0, 0xA0);
    }

    // READ NATIVE MAX ADDRESS (0xF8) and READ NATIVE MAX ADDRESS EXT (0x27).
    static const uint8_t nb2[] = { 0x20, 0x2C, 0x28, 0x24 };
    static const uint8_t ndev[] = { 0x40, 0xE0, 0x60, 0xC0 };
    for (unsigned b = 0; b < sizeof(nb2) / sizeof(nb2[0]); b++)
        for (unsigned d = 0; d < sizeof(ndev) / sizeof(ndev[0]); d++) {
            encn(new_seed(12, false, 0)->cdb, false, false, 0xF8, nb2[b], ndev[d]);
            encn(new_seed(16, false, 0)->cdb, true, false, 0xF8, nb2[b], ndev[d]);
            encn(new_seed(16, false, 0)->cdb, true, true, 0x27, nb2[b], ndev[d]);
        }
}

// ---------------------------------------------------------------------------

int main(void) {
    build_seeds();
    caps = FULL_CAPS;

    // Every seed must itself be allowed, or the sweeps around it prove nothing.
    section = "0 seeds";
    for (int i = 0; i < n_seeds; i++)
        if (!check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, true))
            fail("seed is not allowed", seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, true);
    printf("[0] %d seeds, all allowed\n", n_seeds);

    // 1. The domain: {A1, 85 EXTEND=0, 85 EXTEND=1} x {IDENTIFY-, READ-,
    //    SMART-D0-, SMART-D5-, SMART-DA- and NATIVE-MAX-shaped} x every ATA
    //    command byte x every PROTOCOL x eight byte-2 values x counts 0..9 and
    //    255. The CBW agrees with byte 2 (a consistent host).
    section = "1 domain";
    long a0 = n_allow, c0 = n_checks;
    static const unsigned counts[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 255 };
    static const uint8_t b2s[] = { 0x0E, 0x06, 0x00, 0x20, 0x2C, 0x0C, 0x2E, 0x24 };
    for (int form = 0; form < 3; form++)
        for (int shape = 0; shape < 6; shape++)
            for (unsigned cmd = 0; cmd < 256; cmd++)
                for (unsigned proto = 0; proto < 16; proto++)
                    for (unsigned bi = 0; bi < sizeof(b2s); bi++)
                        for (unsigned ci = 0; ci < sizeof(counts) / sizeof(counts[0]); ci++) {
                            uint8_t c[16];
                            uint8_t cnt = (uint8_t)counts[ci];
                            bool is16 = form != 0, ext = form == 2;
                            fields_t f;
                            memset(&f, 0, sizeof(f));
                            f.cmd = (uint8_t)cmd; f.count = cnt; f.ext = ext;
                            switch (shape) {
                            case 0: break;                                              // all zero
                            case 1: f.lo = 0x67; f.mid = 0x45; f.hi = 0x23; f.dev = 0xE0;
                                    if (ext) { f.hlo = 0x89; f.hmid = 0x01; } else f.dev |= 0x02;
                                    break;
                            case 2: f.feat = 0xD0; f.mid = 0x4F; f.hi = 0xC2; f.dev = 0xA0; break;
                            case 3: f.feat = 0xD5; f.lo = 0x80; f.mid = 0x4F; f.hi = 0xC2; f.dev = 0xA0; break;
                            case 4: f.feat = 0xDA; f.mid = 0x4F; f.hi = 0xC2; f.dev = 0xA0; break;
                            case 5: f.dev = 0x40; break;
                            }
                            if (is16) put16(c, &f); else put12(c, &f);
                            c[1] = (uint8_t)((proto << 1) | (ext ? 1 : 0));
                            c[2] = b2s[bi];
                            bool tlen = (c[2] & 0x03) != 0, tdir = (c[2] & 0x08) != 0;
                            check(c, is16 ? 16 : 12, tlen && tdir, tlen ? cnt * 512u : 0, true, true);
                        }
    printf("[1] domain sweep: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 2. Every single-byte change to every allowed seed, at every position.
    section = "2 single-byte";
    a0 = n_allow; c0 = n_checks;
    for (int i = 0; i < n_seeds; i++)
        for (int pos = 0; pos < 16; pos++)
            for (unsigned val = 0; val < 256; val++) {
                uint8_t c[16];
                memcpy(c, seeds[i].cdb, 16);
                c[pos] = (uint8_t)val;
                check(c, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, true);
            }
    printf("[2] single-byte changes: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 3. CBW direction and transfer length.
    section = "3 cbw";
    a0 = n_allow; c0 = n_checks;
    static const uint32_t lens[] = { 0, 1, 511, 512, 513, 1024, 1536, 2048, 2560, 3072, 3584,
                                     4095, 4096, 4097, 4608, 8192, 65535, 65536, 65536 + 512,
                                     0x80000200u, 0xFFFFFFFFu };
    for (int i = 0; i < n_seeds; i++)
        for (int dir = 0; dir < 2; dir++)
            for (unsigned li = 0; li < sizeof(lens) / sizeof(lens[0]); li++)
                check(seeds[i].cdb, seeds[i].len, dir != 0, lens[li], true, true);
    printf("[3] CBW direction x length: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 4. CB length, including short and oversized.
    section = "4 cdb length";
    a0 = n_allow; c0 = n_checks;
    for (int i = 0; i < n_seeds; i++)
        for (int len = 0; len < 256; len++)
            check(seeds[i].cdb, len, seeds[i].dir_in, seeds[i].xfer, true, true);
    printf("[4] CB length 0..255: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 5. State: mounted x LBA mode, for every seed and for its single-byte changes.
    section = "5 state";
    a0 = n_allow; c0 = n_checks;
    for (int i = 0; i < n_seeds; i++)
        for (int m = 0; m < 2; m++)
            for (int l = 0; l < 2; l++) {
                check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, m != 0, l != 0);
                for (int pos = 0; pos < 16; pos++) {
                    uint8_t c[16];
                    memcpy(c, seeds[i].cdb, 16);
                    c[pos] ^= 0x01;
                    check(c, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, m != 0, l != 0);
                }
            }
    printf("[5] mounted x LBA mode: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 5b. The CHS-mode rule stated directly, not through expect_allow(): with
    //     the drive set up in CHS mode, a seed is allowed exactly when it names
    //     no user sector (IDENTIFY, SMART, READ NATIVE MAX).
    section = "5b chs rule";
    a0 = n_allow; c0 = n_checks;
    for (int i = 0; i < n_seeds; i++) {
        uint8_t cmd = seeds[i].len == 16 ? seeds[i].cdb[14] : seeds[i].cdb[9];
        bool want = cmd == 0xEC || cmd == 0xB0 || cmd == 0xF8 || cmd == 0x27;
        if (check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, false) != want) {
            n_fail++; printf("  FAIL CHS rule, command %02X\n", cmd);
        }
    }
    printf("[5b] CHS mode, direct: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 6. Every SCSI opcode in byte 0, with an otherwise valid body.
    section = "6 opcode";
    a0 = n_allow; c0 = n_checks;
    for (int i = 0; i < n_seeds; i++)
        for (unsigned op = 0; op < 256; op++) {
            uint8_t c[16];
            memcpy(c, seeds[i].cdb, 16);
            c[0] = (uint8_t)op;
            check(c, 12, seeds[i].dir_in, seeds[i].xfer, true, true);
            check(c, 16, seeds[i].dir_in, seeds[i].xfer, true, true);
        }
    printf("[6] SCSI opcode byte: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 7. LBA boundaries.
    section = "7 lba bounds";
    a0 = n_allow; c0 = n_checks;
    // Here the expectation is also stated directly: allowed exactly when the
    // last sector read is still below 2^28 (or 2^48). LBAs that would not fit
    // the field at all are skipped, since the encoding would wrap them.
    for (uint8_t n = 1; n <= MAXSEC; n++) {
        uint8_t c[16];
        for (int d = -2; d <= 2; d++) {
            uint64_t lba = 0x10000000ull - n + (uint64_t)(int64_t)d;
            bool want = lba + n <= 0x10000000ull;
            if (lba < 0x10000000ull) {
                enc12(c, 0x20, n, (uint32_t)lba, 0xE0);
                if (check(c, 12, true, n * 512u, true, true) != want) { n_fail++; printf("  FAIL 28-bit bound, 12-byte, n=%u d=%d\n", n, d); }
                enc16(c, 0x21, false, n, lba, 0xE0);
                if (check(c, 16, true, n * 512u, true, true) != want) { n_fail++; printf("  FAIL 28-bit bound, 16-byte, n=%u d=%d\n", n, d); }
            }
            uint64_t l48 = (1ull << 48) - n + (uint64_t)(int64_t)d;
            want = l48 + n <= (1ull << 48);
            if (l48 < (1ull << 48)) {
                enc16(c, 0x24, true, n, l48, 0xE0);
                if (check(c, 16, true, n * 512u, true, true) != want) { n_fail++; printf("  FAIL 48-bit bound, n=%u d=%d\n", n, d); }
            }
        }
    }
    // READ VERIFY: count 0 means 256 sectors, and the end check must know it.
    static const unsigned vn[] = { 1, 2, 8, 128, 255, 256 };
    for (unsigned k = 0; k < sizeof(vn) / sizeof(vn[0]); k++) {
        uint8_t c[16];
        unsigned n = vn[k];
        for (int d = -2; d <= 2; d++) {
            uint64_t lba = 0x10000000ull - n + (uint64_t)(int64_t)d;
            bool want = lba + n <= 0x10000000ull;
            if (lba >= 0x10000000ull) continue;
            encv(c, false, 0x20, (uint8_t)(n & 0xFF), (uint32_t)lba, 0x40);
            if (check(c, 12, false, 0, true, true) != want) { n_fail++; printf("  FAIL verify bound, 12-byte, n=%u d=%d\n", n, d); }
            encv(c, true, 0x00, (uint8_t)(n & 0xFF), (uint32_t)lba, 0xE0);
            if (check(c, 16, false, 0, true, true) != want) { n_fail++; printf("  FAIL verify bound, 16-byte, n=%u d=%d\n", n, d); }
        }
    }
    printf("[7] LBA boundaries: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 8. Named refusals: each must be refused. Most are already inside the
    //    sweeps; they are listed here so the report names them.
    section = "8 named";
    c0 = n_checks;
    int named_bad = 0;
    struct { const char *name; uint8_t c[16]; int len; bool dir_in; uint32_t xfer; bool lba_mode; } named[] = {
        { "SRST template (SAT protocol 1)",       { 0xA1, 0x02 }, 12, false, 0, true },
        { "Return Response Info (protocol 15)",   { 0xA1, 0x1E, 0x20 }, 12, false, 0, true },
        { "READ DMA 0xC8",                        { 0xA1, 0x0C, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0xC8 }, 12, true, 512, true },
        { "READ DMA 0xC8 as PIO",                 { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0xC8 }, 12, true, 512, true },
        { "READ DMA EXT 0x25",                    { 0x85, 0x0D, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x25 }, 16, true, 512, true },
        { "READ DMA EXT 0x25 as PIO",             { 0x85, 0x09, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x25 }, 16, true, 512, true },
        { "READ SECTORS EXT 0x24 in 12-byte CDB", { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x24 }, 12, true, 512, true },
        { "READ SECTORS EXT with EXTEND=0",       { 0x85, 0x08, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x24 }, 16, true, 512, true },
        { "READ SECTORS with EXTEND=1",           { 0x85, 0x09, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x20 }, 16, true, 512, true },
        { "READ SECTORS count 0 (ATA 256)",       { 0xA1, 0x08, 0x0E, 0, 0, 0, 0, 0, 0xE0, 0x20 }, 12, true, 0, true },
        { "READ SECTORS count 9",                 { 0xA1, 0x08, 0x0E, 0, 9, 0, 0, 0, 0xE0, 0x20 }, 12, true, 9 * 512, true },
        { "READ SECTORS count 128 (imagelba default chunk)", { 0xA1, 0x08, 0x0E, 0, 128, 0, 0, 0, 0xE0, 0x20 }, 12, true, 128 * 512, true },
        { "READ SECTORS in CHS form (LBA bit 0)", { 0xA1, 0x08, 0x0E, 0, 1, 1, 0, 0, 0xA0, 0x20 }, 12, true, 512, true },
        { "READ SECTORS to device 1 (DEV bit)",   { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xF0, 0x20 }, 12, true, 512, true },
        { "READ SECTORS, CK_COND=1",              { 0xA1, 0x08, 0x2E, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, true, 512, true },
        { "READ SECTORS, T_DIR=0",                { 0xA1, 0x08, 0x06, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, true, 512, true },
        { "READ SECTORS, BYTE_BLOCK=0",           { 0xA1, 0x08, 0x0A, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, true, 512, true },
        { "READ SECTORS, MULTIPLE_COUNT=1",       { 0xA1, 0x28, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, true, 512, true },
        { "READ SECTORS, PIO data-out protocol",  { 0xA1, 0x0A, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, true, 512, true },
        { "READ SECTORS as non-data",             { 0xA1, 0x06, 0x20, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, false, 0, true },
        { "READ SECTORS in CHS mode",             { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, true, 512, false },
        { "READ VERIFY in CHS mode",              { 0xA1, 0x06, 0x20, 0, 1, 0, 0, 0, 0xE0, 0x40 }, 12, false, 0, false },
        { "READ VERIFY in CHS form (LBA bit 0)",  { 0xA1, 0x06, 0x20, 0, 1, 1, 0, 0, 0xA0, 0x40 }, 12, false, 0, true },
        { "READ VERIFY as PIO data-in",           { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x40 }, 12, true, 512, true },
        { "READ VERIFY with a data-in CBW",       { 0xA1, 0x06, 0x20, 0, 1, 0, 0, 0, 0xE0, 0x40 }, 12, true, 512, true },
        { "READ VERIFY with a data-out CBW",      { 0xA1, 0x06, 0x20, 0, 1, 0, 0, 0, 0xE0, 0x40 }, 12, false, 512, true },
        { "READ VERIFY, T_LENGTH=2",              { 0xA1, 0x06, 0x22, 0, 1, 0, 0, 0, 0xE0, 0x40 }, 12, false, 0, true },
        { "READ VERIFY, OFF_LINE=1",              { 0xA1, 0x06, 0x60, 0, 1, 0, 0, 0, 0xE0, 0x40 }, 12, false, 0, true },
        { "READ VERIFY, T_TYPE=1",                { 0xA1, 0x06, 0x30, 0, 1, 0, 0, 0, 0xE0, 0x40 }, 12, false, 0, true },
        { "READ VERIFY with EXTEND=1",            { 0x85, 0x07, 0x20, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x40 }, 16, false, 0, true },
        { "READ VERIFY EXT 0x42",                 { 0x85, 0x07, 0x20, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0x40, 0x42 }, 16, false, 0, true },
        { "READ VERIFY past 2^28 (count 0)",      { 0xA1, 0x06, 0x20, 0, 0, 0x01, 0xFF, 0xFF, 0xEF, 0x40 }, 12, false, 0, true },
        { "SMART RETURN STATUS as PIO (interlock-test.ps1 shape)", { 0xA1, 0x08, 0x0E, 0xDA, 1, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, true, 512, true },
        { "SMART RETURN STATUS, CK_COND=0",       { 0xA1, 0x06, 0x0C, 0xDA, 0, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "SMART RETURN STATUS, count 1",         { 0xA1, 0x06, 0x20, 0xDA, 1, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "SMART READ DATA without 4F/C2",        { 0xA1, 0x08, 0x0E, 0xD0, 1, 0, 0, 0, 0xA0, 0xB0 }, 12, true, 512, true },
        { "SMART READ DATA as non-data",          { 0xA1, 0x06, 0x20, 0xD0, 1, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "SMART READ DATA, count 2",             { 0xA1, 0x08, 0x0E, 0xD0, 2, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, true, 1024, true },
        { "SMART READ DATA with EXTEND=1",        { 0x85, 0x09, 0x0E, 0, 0xD0, 0, 1, 0, 0, 0, 0x4F, 0, 0xC2, 0xA0, 0xB0 }, 16, true, 512, true },
        { "SMART READ LOG, 9 sectors",            { 0xA1, 0x08, 0x0E, 0xD5, 9, 1, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, true, 9 * 512, true },
        { "SMART READ LOG, 0 sectors",            { 0xA1, 0x08, 0x0E, 0xD5, 0, 1, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, true, 0, true },
        { "SMART ENABLE OPERATIONS D8",           { 0xA1, 0x06, 0x20, 0xD8, 0, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "SMART DISABLE OPERATIONS D9",          { 0xA1, 0x06, 0x20, 0xD9, 0, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "SMART ATTRIBUTE AUTOSAVE D2",          { 0xA1, 0x06, 0x20, 0xD2, 0xF1, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "SMART SAVE ATTRIBUTE VALUES D3",       { 0xA1, 0x06, 0x20, 0xD3, 0, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "SMART EXECUTE OFF-LINE D4",            { 0xA1, 0x06, 0x20, 0xD4, 0, 0x01, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "SMART WRITE LOG D6",                   { 0xA1, 0x0A, 0x06, 0xD6, 1, 0x80, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 512, true },
        { "SMART WRITE LOG D6 dressed as data-in",{ 0xA1, 0x08, 0x0E, 0xD6, 1, 0x80, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, true, 512, true },
        { "SMART ENABLE/DISABLE AUTO OFF-LINE DB",{ 0xA1, 0x06, 0x20, 0xDB, 0xF8, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, false, 0, true },
        { "READ NATIVE MAX, CK_COND=0",           { 0xA1, 0x06, 0x00, 0, 0, 0, 0, 0, 0x40, 0xF8 }, 12, false, 0, true },
        { "READ NATIVE MAX, LBA bit 0",           { 0xA1, 0x06, 0x20, 0, 0, 0, 0, 0, 0xA0, 0xF8 }, 12, false, 0, true },
        { "READ NATIVE MAX, count 1",             { 0xA1, 0x06, 0x20, 0, 1, 0, 0, 0, 0x40, 0xF8 }, 12, false, 0, true },
        { "READ NATIVE MAX with EXTEND=1",        { 0x85, 0x07, 0x20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x40, 0xF8 }, 16, false, 0, true },
        { "READ NATIVE MAX EXT in 12-byte CDB",   { 0xA1, 0x06, 0x20, 0, 0, 0, 0, 0, 0x40, 0x27 }, 12, false, 0, true },
        { "READ NATIVE MAX EXT with EXTEND=0",    { 0x85, 0x06, 0x20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x40, 0x27 }, 16, false, 0, true },
        { "READ NATIVE MAX EXT, CK_COND=0",       { 0x85, 0x07, 0x0C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x40, 0x27 }, 16, false, 0, true },
        { "READ NATIVE MAX EXT, HOB LBA set",     { 0x85, 0x07, 0x20, 0, 0, 0, 0, 0x01, 0, 0, 0, 0, 0, 0x40, 0x27 }, 16, false, 0, true },
        { "SET MAX ADDRESS 0xF9",                 { 0xA1, 0x06, 0x00, 0, 0, 0, 0, 0, 0xE0, 0xF9 }, 12, false, 0, true },
        { "SET MAX ADDRESS 0xF9 with CK_COND",    { 0xA1, 0x06, 0x20, 0, 0, 0, 0, 0, 0x40, 0xF9 }, 12, false, 0, true },
        { "SET MAX ADDRESS EXT 0x37",             { 0x85, 0x07, 0x20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x40, 0x37 }, 16, false, 0, true },
        { "DCO RESTORE 0xB1/C0",                  { 0xA1, 0x06, 0x20, 0xC0, 0, 0, 0, 0, 0xA0, 0xB1 }, 12, false, 0, true },
        { "DCO FREEZE LOCK 0xB1/C1",              { 0xA1, 0x06, 0x20, 0xC1, 0, 0, 0, 0, 0xA0, 0xB1 }, 12, false, 0, true },
        { "DCO IDENTIFY 0xB1/C2",                 { 0xA1, 0x08, 0x0E, 0xC2, 1, 0, 0, 0, 0xA0, 0xB1 }, 12, true, 512, true },
        { "DCO SET 0xB1/C3",                      { 0xA1, 0x0A, 0x06, 0xC3, 1, 0, 0, 0, 0xA0, 0xB1 }, 12, false, 512, true },
        { "DCO IDENTIFY dressed as SMART shape",  { 0xA1, 0x08, 0x0E, 0xC2, 1, 0, 0x4F, 0xC2, 0xA0, 0xB1 }, 12, true, 512, true },
        { "SECURITY SET PASSWORD 0xF1",           { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xA0, 0xF1 }, 12, false, 512, true },
        { "SECURITY UNLOCK 0xF2",                 { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xA0, 0xF2 }, 12, false, 512, true },
        { "SECURITY ERASE PREPARE 0xF3",          { 0xA1, 0x06, 0x20, 0, 0, 0, 0, 0, 0xA0, 0xF3 }, 12, false, 0, true },
        { "SECURITY ERASE UNIT 0xF4",             { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xA0, 0xF4 }, 12, true, 512, true },
        { "SECURITY FREEZE LOCK 0xF5",            { 0xA1, 0x06, 0x20, 0, 0, 0, 0, 0, 0xA0, 0xF5 }, 12, false, 0, true },
        { "SECURITY DISABLE PASSWORD 0xF6",       { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xA0, 0xF6 }, 12, false, 512, true },
        { "WRITE SECTORS 0x30",                   { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xE0, 0x30 }, 12, true, 512, true },
        { "WRITE SECTORS 0x30 dressed as data-in",{ 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x30 }, 12, true, 512, true },
        { "WRITE SECTORS 0x30 as non-data",       { 0xA1, 0x06, 0x20, 0, 1, 0, 0, 0, 0xE0, 0x30 }, 12, false, 0, true },
        { "WRITE SECTORS EXT 0x34",               { 0x85, 0x0B, 0x06, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x34 }, 16, true, 512, true },
        { "WRITE VERIFY 0x3C",                    { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xE0, 0x3C }, 12, false, 512, true },
        { "WRITE DMA 0xCA",                       { 0xA1, 0x0C, 0x06, 0, 1, 0, 0, 0, 0xE0, 0xCA }, 12, true, 512, true },
        { "FORMAT TRACK 0x50",                    { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xE0, 0x50 }, 12, false, 512, true },
        { "INITIALIZE DEVICE PARAMETERS 0x91",    { 0xA1, 0x06, 0x20, 0, 17, 0, 0, 0, 0xA9, 0x91 }, 12, false, 0, true },
        { "RECALIBRATE 0x10",                     { 0xA1, 0x06, 0x20, 0, 0, 0, 0, 0, 0xA0, 0x10 }, 12, false, 0, true },
        { "SET FEATURES 0xEF",                    { 0xA1, 0x06, 0x20, 0x82, 0, 0, 0, 0, 0xA0, 0xEF }, 12, false, 0, true },
        { "DOWNLOAD MICROCODE 0x92",              { 0xA1, 0x0A, 0x06, 0x07, 1, 0, 0, 0, 0xA0, 0x92 }, 12, true, 512, true },
        { "IDENTIFY with CB length 16",           { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0, 0xEC }, 16, true, 512, true },
        { "IDENTIFY, 85 form with CB length 12",  { 0x85, 0x08, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0xEC }, 12, true, 512, true },
        { "IDENTIFY as non-data with CK_COND",    { 0xA1, 0x06, 0x20, 0, 1, 0, 0, 0, 0, 0xEC }, 12, false, 0, true },
    };
    for (unsigned i = 0; i < sizeof(named) / sizeof(named[0]); i++) {
        bool a = check(named[i].c, named[i].len, named[i].dir_in, named[i].xfer, true, named[i].lba_mode);
        if (a) { named_bad++; printf("  FAIL named refusal allowed: %s\n", named[i].name); n_fail++; }
    }
    printf("[8] named refusals: %u cases, %d allowed\n", (unsigned)(sizeof(named) / sizeof(named[0])), named_bad);
    (void)c0;

    // 9. Degenerate calls.
    section = "9 null";
    {
        sat_taskfile_t tf;
        sat_verdict_t v = sat_policy_check(NULL, &tf);
        n_checks++; if (v.allow) { n_fail++; printf("  FAIL NULL input allowed\n"); }
        sat_input_t in = { NULL, 12, true, 512, true, true, false, 0, 0, 0, 0, 0 };
        set_caps(&in, &FULL_CAPS);
        v = sat_policy_check(&in, &tf);
        n_checks++; if (v.allow) { n_fail++; printf("  FAIL NULL cdb allowed\n"); }
        in.cdb = seeds[0].cdb;
        v = sat_policy_check(&in, NULL);
        n_checks++; if (v.allow) { n_fail++; printf("  FAIL NULL task file allowed\n"); }
    }
    printf("[9] NULL arguments: 3 cases\n");

    // 10. sat_cbw_parse(): a valid CBW image, then every byte changed to every value.
    section = "10 cbw parse";
    c0 = n_checks;
    {
        const uint8_t *cdb = seeds[0].cdb;           // IDENTIFY, 512 bytes in
        uint8_t img[31] = { 0x55, 0x53, 0x42, 0x43, 0x11, 0x22, 0x33, 0x44,
                            0x00, 0x02, 0x00, 0x00, 0x80, 0x00, 12 };
        memcpy(img + 15, cdb, 16);
        for (int pos = -1; pos < 31; pos++)
            for (unsigned val = 0; val < 256; val++) {
                if (pos < 0 && val > 0) break;
                uint8_t m[31];
                memcpy(m, img, 31);
                if (pos >= 0) m[pos] = (uint8_t)val;
                sat_cbw_t out;
                bool got = sat_cbw_parse(m, 0, cdb, 512, &out);
                // expectation, byte by byte
                bool exp = true;
                if (pos >= 0 && pos <= 3 && m[pos] != img[pos]) exp = false;      // signature
                if ((pos == 8 || pos == 9) && m[pos] != img[pos]) exp = false;    // low 16 bits of length
                if (pos == 13 && m[pos] != 0) exp = false;                        // LUN 0, reserved bits 0
                if (pos >= 15 && m[pos] != img[pos]) exp = false;                 // CB bytes
                n_checks++;
                if (got != exp) { fail(exp ? "cbw parse refused a valid image" : "cbw parse accepted a bad image", cdb, pos, 0, val, 0, 0); continue; }
                if (!got) continue;
                uint32_t len = (uint32_t)m[8] | ((uint32_t)m[9] << 8) | ((uint32_t)m[10] << 16) | ((uint32_t)m[11] << 24);
                if (out.xfer_len != len || out.dir_in != (m[12] == 0x80) || out.cb_len != m[14])
                    fail("cbw parse fields wrong", cdb, pos, 0, val, 0, 0);
                // and through the policy: only the untouched image (or a changed tag) may be allowed
                sat_input_t in = { cdb, out.cb_len, out.dir_in, out.xfer_len, true, true, false, 0, 0, 0, 0, 0 };
                set_caps(&in, &FULL_CAPS);
                sat_taskfile_t tf;
                bool allowed = sat_policy_check(&in, &tf).allow;
                bool exp_allow = (pos < 0) || (pos >= 4 && pos <= 7) || m[pos] == img[pos];
                n_checks++;
                if (allowed != exp_allow) fail("cbw -> policy verdict wrong", cdb, pos, 0, val, 0, 0);
            }
        // LUN 1 must be refused when the callback is for LUN 0, and the other way round
        n_checks++;
        { sat_cbw_t out; if (sat_cbw_parse(img, 1, cdb, 512, &out)) { n_fail++; printf("  FAIL cbw parse ignored LUN\n"); } }
        // bufsize16 must match the image
        n_checks++;
        { sat_cbw_t out; if (sat_cbw_parse(img, 0, cdb, 1024, &out)) { n_fail++; printf("  FAIL cbw parse ignored length\n"); } }
        // A CBW that DECLARES data-out, with a valid read CDB, must be refused.
        // This checks the policy only. It does not prove the firmware refuses
        // real data-out commands: on that path TinyUSB puts the host's payload
        // in the buffer, not the CBW, and a payload can mimic a data-in CBW.
        // See the note in sat.c (review finding M1).
        uint8_t out_img[31];
        memcpy(out_img, img, 31);
        out_img[12] = 0x00;
        sat_cbw_t o;
        n_checks++;
        if (!sat_cbw_parse(out_img, 0, cdb, 512, &o) || o.dir_in) { n_fail++; printf("  FAIL data-out CBW parse\n"); }
        sat_input_t in = { cdb, o.cb_len, o.dir_in, o.xfer_len, true, true, false, 0, 0, 0, 0, 0 };
        set_caps(&in, &FULL_CAPS);
        sat_taskfile_t tf;
        n_checks++;
        if (sat_policy_check(&in, &tf).allow) { n_fail++; printf("  FAIL data-out CBW allowed\n"); }
        // Non-data rows through a data-out CBW. On the data-out path TinyUSB
        // passes the real CBW length (1..4096) as bufsize, and a mimic image
        // must match it in its low 16 bits, so its length is never 0: every
        // non-data row is refused, whatever the mimic says.
        {
            uint8_t v[16];
            encs(v, false, 3, 0x20, 0xDA, 0, 0, 0xA0);
            for (uint32_t real = 1; real <= 4096; real++) {
                uint8_t mimic[31] = { 0x55, 0x53, 0x42, 0x43, 0, 0, 0, 0,
                                      (uint8_t)real, (uint8_t)(real >> 8), 0, 0, 0x00, 0x00, 12 };
                memcpy(mimic + 15, v, 16);
                for (int hi = 0; hi < 2; hi++) {
                    mimic[10] = (uint8_t)hi;           // try 0x0001xxxx as well
                    sat_cbw_t mo;
                    if (!sat_cbw_parse(mimic, 0, v, (uint16_t)real, &mo)) continue;
                    sat_input_t mi = { v, mo.cb_len, mo.dir_in, mo.xfer_len, true, true, false, 0, 0, 0, 0, 0 };
                    set_caps(&mi, &FULL_CAPS);
                    n_checks++;
                    if (sat_policy_check(&mi, &tf).allow) { n_fail++; printf("  FAIL non-data row allowed through a data-out mimic, len %u\n", real); }
                }
            }
        }
        // NULL arguments
        n_checks++;
        if (sat_cbw_parse(NULL, 0, cdb, 512, &o) || sat_cbw_parse(img, 0, NULL, 512, &o) ||
            sat_cbw_parse(img, 0, cdb, 512, NULL)) { n_fail++; printf("  FAIL cbw parse NULL\n"); }
    }
    printf("[10] CBW parse: %ld cases\n", n_checks - c0);

    // 11. The drive's IDENTIFY. Every seed (so every row) against every
    //     combination of: IDENTIFY captured or not; bits 15:14 of words 83,
    //     84 and 87 (00, 01, 10, 11); the five feature bits the policy reads
    //     (82.0, 82.10, 83.10, 84.0, 85.0); and the other bits of the five
    //     words all clear or all set (so a check on the wrong bit shows up);
    //     in LBA and CHS mode, and unmounted (NOT READY must still come
    //     first). Word 82 is kept non-zero by bit 14 (NOP) when its other bits
    //     are clear. Then word 82 = 0000h and FFFFh with valid 83/84, which
    //     must count as "not reported".
    section = "11 identify";
    a0 = n_allow; c0 = n_checks;
    for (int i = 0; i < n_seeds; i++)
        for (unsigned combo = 0; combo < 8192u; combo++) {
            idcaps_t k;
            bool other = (combo & 1u) != 0;
            k.captured = (combo & 2u) != 0;
            unsigned s83 = (combo >> 2) & 3u, s84 = (combo >> 4) & 3u;
            bool b82_0 = ((combo >> 6) & 1u) != 0, b82_10 = ((combo >> 7) & 1u) != 0;
            bool b83_10 = ((combo >> 8) & 1u) != 0, b84_0 = ((combo >> 9) & 1u) != 0;
            k.w82 = (uint16_t)((b82_0 ? 0x0001u : 0u) | (b82_10 ? 0x0400u : 0u) | (other ? 0xFBFEu : 0x4000u));
            k.w83 = (uint16_t)((s83 << 14) | (b83_10 ? 0x0400u : 0u) | (other ? 0x3BFFu : 0u));
            k.w84 = (uint16_t)((s84 << 14) | (b84_0 ? 0x0001u : 0u) | (other ? 0x3FFEu : 0u));
            bool b85_0 = ((combo >> 10) & 1u) != 0;
            unsigned s87 = (combo >> 11) & 3u;
            k.w85 = (uint16_t)((b85_0 ? 0x0001u : 0u) | (other ? 0xFFFEu : 0u));
            k.w87 = (uint16_t)((s87 << 14) | (other ? 0x3FFFu : 0u));
            caps = k;
            check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, true);
            check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, false);
            check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, false, true);
        }
    for (int i = 0; i < n_seeds; i++) {
        static const uint16_t w82s[] = { 0x0000, 0xFFFF };
        for (unsigned j = 0; j < 2; j++) {
            caps = FULL_CAPS; caps.w82 = w82s[j];
            check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, true);
        }
    }
    printf("[11] IDENTIFY words 82..85, 87: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 11b. The same rule stated directly: with no IDENTIFY captured, or one
    //      from a drive older than ATA-4 (words 83/84 without the 01b
    //      signature, even with every feature bit set), a seed is allowed
    //      exactly when it is IDENTIFY, READ SECTORS or READ VERIFY.
    section = "11b ungated";
    a0 = n_allow; c0 = n_checks;
    {
        static const idcaps_t none[] = {
            { false, 0x4401, 0x4400, 0x4001, 0x4401, 0x4000 },   // good words, but not captured for this drive
            { true,  0x0401, 0x0400, 0x0001, 0x0401, 0x0000 },   // pre-ATA-4: no signature
            { true,  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF },   // floating bus
            { true,  0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
        };
        for (unsigned j = 0; j < sizeof(none) / sizeof(none[0]); j++) {
            caps = none[j];
            for (int i = 0; i < n_seeds; i++) {
                uint8_t cmd = seeds[i].len == 16 ? seeds[i].cdb[14] : seeds[i].cdb[9];
                bool want = cmd == 0xEC || cmd == 0x20 || cmd == 0x21 || cmd == 0x40;
                if (check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, true) != want) {
                    n_fail++; printf("  FAIL ungated rule, IDENTIFY case %u, command %02X\n", j, cmd);
                }
            }
        }
    }
    printf("[11b] no usable IDENTIFY, direct: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 11c. SMART supported but not enabled, stated directly: the ST380011A
    //      donor's words 82 and 85 (346Bh, 3468h), and a full drive whose
    //      word 87 does not vouch for word 85. Every SMART row refused, every
    //      other row as with a full drive.
    section = "11c smart disabled";
    a0 = n_allow; c0 = n_checks;
    {
        static const idcaps_t off[] = {
            { true, 0x346B, 0x4400, 0x4001, 0x3468, 0x4000 },
            { true, 0x4401, 0x4400, 0x4001, 0x4401, 0x0000 },
            { true, 0x4401, 0x4400, 0x4001, 0x4401, 0xC000 },
        };
        for (unsigned j = 0; j < sizeof(off) / sizeof(off[0]); j++) {
            caps = off[j];
            for (int i = 0; i < n_seeds; i++) {
                uint8_t cmd = seeds[i].len == 16 ? seeds[i].cdb[14] : seeds[i].cdb[9];
                bool want = cmd != 0xB0;
                if (check(seeds[i].cdb, seeds[i].len, seeds[i].dir_in, seeds[i].xfer, true, true) != want) {
                    n_fail++; printf("  FAIL SMART-disabled rule, case %u, command %02X\n", j, cmd);
                }
            }
        }
    }
    printf("[11c] SMART supported, not enabled, direct: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);
    caps = FULL_CAPS;

    printf("\n%ld assertions, %ld allowed, %ld refused, %ld failures\n", n_checks, n_allow, n_refuse, n_fail);
    if (n_fail) { printf("SAT POLICY TEST: FAIL\n"); return 1; }
    printf("SAT POLICY TEST: PASS\n");
    return 0;
}
