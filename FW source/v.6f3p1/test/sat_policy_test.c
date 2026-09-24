// Exhaustive host test for sat_policy.c, the firmware's SAT allowlist.
//
// Build and run with run-sat-policy-tests.ps1, which also runs the mutants.
// Exit code 0 only if every assertion passed.
//
// The expected verdicts do NOT come from sat_policy.c. expect_allow() below is
// written a different way on purpose: it pulls out the few fields an allowed
// command may vary (command, count, LBA, obsolete device bits), re-encodes the
// only CDB those fields are allowed to produce, and compares bytes. Any byte
// that differs from that canonical encoding is a refusal. The policy instead
// checks field by field. Two shapes of the same rule, so a mistake in one is
// unlikely to be repeated in the other.
//
// The allowed set is written out here by hand as well, not taken from the
// policy's header: IDENTIFY (0xEC), READ SECTORS (0x20, 0x21) and READ
// SECTORS EXT (0x24, 16-byte form only), PIO data-in, 1..8 sectors.

#include "sat_policy.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define MAXSEC 8u      // this test's own copy of the sector cap

static long n_checks, n_fail, n_allow, n_refuse;
static const char *section = "";

static void fail(const char *what, const uint8_t *cdb, int len, bool dir_in, uint32_t xfer,
                 bool mounted, bool lba_mode) {
    n_fail++;
    if (n_fail > 40) return;
    printf("  FAIL [%s] %s: len=%d dir_in=%d xfer=%u mounted=%d lba=%d cdb=",
           section, what, len, dir_in, (unsigned)xfer, mounted, lba_mode);
    for (int i = 0; i < 16; i++) printf("%02X%s", cdb[i], i == 15 ? "\n" : " ");
}

// ---------------------------------------------------------------------------
//  The expectation
// ---------------------------------------------------------------------------

static void enc12(uint8_t o[16], uint8_t cmd, uint8_t count, uint32_t lba28, uint8_t devtop) {
    memset(o, 0, 16);
    o[0] = 0xA1; o[1] = 4 << 1; o[2] = 0x0E;
    o[4] = count;
    o[5] = (uint8_t)lba28; o[6] = (uint8_t)(lba28 >> 8); o[7] = (uint8_t)(lba28 >> 16);
    o[8] = (uint8_t)(devtop | ((lba28 >> 24) & 0x0F));
    o[9] = cmd;
}

static void enc16(uint8_t o[16], uint8_t cmd, bool ext, uint8_t count, uint64_t lba, uint8_t devtop) {
    memset(o, 0, 16);
    o[0] = 0x85; o[1] = (uint8_t)((4 << 1) | (ext ? 1 : 0)); o[2] = 0x0E;
    o[6] = count;
    o[8] = (uint8_t)lba; o[10] = (uint8_t)(lba >> 8); o[12] = (uint8_t)(lba >> 16);
    if (ext) {
        o[7] = (uint8_t)(lba >> 24); o[9] = (uint8_t)(lba >> 32); o[11] = (uint8_t)(lba >> 40);
        o[13] = devtop;
    } else {
        o[13] = (uint8_t)(devtop | ((lba >> 24) & 0x0F));
    }
    o[14] = cmd;
}

// Would a correct policy allow this? Also reports the sector count it implies.
static bool expect_allow(const uint8_t *c, int len, bool dir_in, uint32_t xfer,
                         bool mounted, bool lba_mode, unsigned *count_out) {
    uint8_t canon[16];
    unsigned count = 0;
    int n;
    if (c[0] == 0xA1 && len == 12) {
        uint8_t cmd = c[9], dev = c[8];
        uint32_t lba = c[5] | ((uint32_t)c[6] << 8) | ((uint32_t)c[7] << 16) | ((uint32_t)(dev & 0x0F) << 24);
        count = c[4];
        if (cmd == 0xEC) {
            count = 1;
            enc12(canon, 0xEC, 1, 0, dev & 0xE0);                     // bits 7,6,5 free
        } else if (cmd == 0x20 || cmd == 0x21) {
            if (!lba_mode || count < 1 || count > MAXSEC) return false;
            if ((uint64_t)lba + count > 0x10000000ull) return false;
            enc12(canon, cmd, (uint8_t)count, lba, (uint8_t)((dev & 0xA0) | 0x40));
        } else {
            return false;
        }
        n = 12;
    } else if (c[0] == 0x85 && len == 16) {
        uint8_t cmd = c[14], dev = c[13];
        count = c[6];
        if (cmd == 0xEC) {
            count = 1;
            enc16(canon, 0xEC, false, 1, 0, dev & 0xE0);
        } else if (cmd == 0x20 || cmd == 0x21) {
            uint32_t lba = c[8] | ((uint32_t)c[10] << 8) | ((uint32_t)c[12] << 16) | ((uint32_t)(dev & 0x0F) << 24);
            if (!lba_mode || count < 1 || count > MAXSEC) return false;
            if ((uint64_t)lba + count > 0x10000000ull) return false;
            enc16(canon, cmd, false, (uint8_t)count, lba, (uint8_t)((dev & 0xA0) | 0x40));
        } else if (cmd == 0x24) {
            uint64_t lba = c[8] | ((uint64_t)c[10] << 8) | ((uint64_t)c[12] << 16) |
                           ((uint64_t)c[7] << 24) | ((uint64_t)c[9] << 32) | ((uint64_t)c[11] << 40);
            if (!lba_mode || count < 1 || count > MAXSEC) return false;
            if (lba + count > (1ull << 48)) return false;
            enc16(canon, 0x24, true, (uint8_t)count, lba, (uint8_t)((dev & 0xA0) | 0x40));
        } else {
            return false;
        }
        n = 16;
    } else {
        return false;
    }
    if (memcmp(canon, c, (size_t)n) != 0) return false;
    if (!dir_in || xfer != count * 512u) return false;
    if (!mounted) return false;
    if (count_out) *count_out = count;
    return true;
}

// ---------------------------------------------------------------------------
//  One assertion: run the policy, compare with the expectation
// ---------------------------------------------------------------------------

static bool check(const uint8_t cdb_in[16], int len, bool dir_in, uint32_t xfer,
                  bool mounted, bool lba_mode) {
    uint8_t cdb[16];
    memcpy(cdb, cdb_in, 16);
    sat_input_t in = { cdb, (uint8_t)len, dir_in, xfer, mounted, lba_mode };
    sat_taskfile_t tf;
    memset(&tf, 0xAA, sizeof(tf));
    sat_verdict_t v = sat_policy_check(&in, &tf);

    unsigned count = 0;
    bool exp = expect_allow(cdb, len, dir_in, xfer, mounted, lba_mode, &count);
    n_checks++;

    if (memcmp(cdb, cdb_in, 16) != 0) fail("policy modified the CDB", cdb_in, len, dir_in, xfer, mounted, lba_mode);

    if (v.allow != exp) {
        fail(exp ? "expected ALLOW, got REFUSE" : "expected REFUSE, got ALLOW", cdb, len, dir_in, xfer, mounted, lba_mode);
        return v.allow;
    }

    if (v.allow) {
        n_allow++;
        // The task file must be exactly what the CDB said, and only a read.
        bool is16 = cdb[0] == 0x85;
        uint8_t cmd = is16 ? cdb[14] : cdb[9];
        uint8_t dev = is16 ? cdb[13] : cdb[8];
        bool ext = is16 && (cdb[1] & 1);
        sat_taskfile_t w;
        memset(&w, 0, sizeof(w));
        w.command = cmd;
        w.count = (uint8_t)count;
        w.sectors = (uint8_t)count;
        w.lba_low = is16 ? cdb[8] : cdb[5];
        w.lba_mid = is16 ? cdb[10] : cdb[6];
        w.lba_high = is16 ? cdb[12] : cdb[7];
        w.ext = ext;
        if (ext) { w.hob_lba_low = cdb[7]; w.hob_lba_mid = cdb[9]; w.hob_lba_high = cdb[11]; }
        w.device = cmd == 0xEC ? 0x00 : (ext ? 0x40 : (uint8_t)(0x40 | (dev & 0x0F)));
        if (tf.command != 0xEC && tf.command != 0x20 && tf.command != 0x21 && tf.command != 0x24)
            fail("task file carries a command outside the read set", cdb, len, dir_in, xfer, mounted, lba_mode);
        if (tf.command != w.command || tf.count != w.count || tf.sectors != w.sectors ||
            tf.lba_low != w.lba_low || tf.lba_mid != w.lba_mid || tf.lba_high != w.lba_high ||
            tf.hob_lba_low != w.hob_lba_low || tf.hob_lba_mid != w.hob_lba_mid ||
            tf.hob_lba_high != w.hob_lba_high || tf.hob_count != 0 || tf.feature != 0 ||
            tf.hob_feature != 0 || tf.ext != w.ext || tf.device != w.device)
            fail("task file does not match the CDB", cdb, len, dir_in, xfer, mounted, lba_mode);
        if (tf.sectors < 1 || tf.sectors > MAXSEC || (uint32_t)tf.sectors * 512u != xfer)
            fail("task file transfer does not match the CBW", cdb, len, dir_in, xfer, mounted, lba_mode);
    } else {
        n_refuse++;
        static const sat_taskfile_t zero;
        if (memcmp(&tf, &zero, sizeof(tf)) != 0)
            fail("refusal left data in the task file", cdb, len, dir_in, xfer, mounted, lba_mode);
        bool ok_sense =
            (v.sense_key == 0x05 && (v.asc == 0x20 || v.asc == 0x21 || v.asc == 0x24) && v.ascq == 0) ||
            (v.sense_key == 0x02 && v.asc == 0x04 && v.ascq == 0);
        if (!ok_sense) fail("refusal sense is not 5/20, 5/21, 5/24 or 2/04", cdb, len, dir_in, xfer, mounted, lba_mode);
        // NOT READY exactly when nothing is mounted and the CDB itself is
        // valid. (With nothing mounted, the LBA-mode setting means nothing.)
        bool only_unmounted = !mounted && expect_allow(cdb, len, dir_in, xfer, true, true, NULL);
        if (only_unmounted != (v.sense_key == 0x02))
            fail(only_unmounted ? "unmounted: expected NOT READY" : "NOT READY for a CDB that is invalid anyway",
                 cdb, len, dir_in, xfer, mounted, lba_mode);
    }
    return v.allow;
}

// Seeds: one canonical CDB for every allowed row, at a few LBAs.
typedef struct { uint8_t cdb[16]; int len; unsigned count; } seed_t;
static seed_t seeds[200];
static int n_seeds;

static void add_seed12(uint8_t cmd, uint8_t count, uint32_t lba, uint8_t devtop) {
    seed_t *s = &seeds[n_seeds++];
    enc12(s->cdb, cmd, count, lba, devtop); s->len = 12; s->count = count;
}
static void add_seed16(uint8_t cmd, bool ext, uint8_t count, uint64_t lba, uint8_t devtop) {
    seed_t *s = &seeds[n_seeds++];
    enc16(s->cdb, cmd, ext, count, lba, devtop); s->len = 16; s->count = count;
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
}

// ---------------------------------------------------------------------------

int main(void) {
    build_seeds();

    // Every seed must itself be allowed, or the sweeps around it prove nothing.
    section = "0 seeds";
    for (int i = 0; i < n_seeds; i++)
        if (!check(seeds[i].cdb, seeds[i].len, true, seeds[i].count * 512u, true, true))
            fail("seed is not allowed", seeds[i].cdb, seeds[i].len, true, seeds[i].count * 512u, true, true);
    printf("[0] %d seeds, all allowed\n", n_seeds);

    // 1. The domain: {A1, 85 EXTEND=0, 85 EXTEND=1} x {IDENTIFY-shaped,
    //    READ-shaped} x every ATA command byte x every PROTOCOL x both T_DIR
    //    x counts 0..9 and 255. The CBW agrees with the CDB (a consistent host).
    section = "1 domain";
    long a0 = n_allow, c0 = n_checks;
    static const unsigned counts[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 255 };
    for (int form = 0; form < 3; form++)
        for (int shape = 0; shape < 2; shape++)
            for (unsigned cmd = 0; cmd < 256; cmd++)
                for (unsigned proto = 0; proto < 16; proto++)
                    for (int tdir = 0; tdir < 2; tdir++)
                        for (unsigned ci = 0; ci < sizeof(counts) / sizeof(counts[0]); ci++) {
                            uint8_t c[16];
                            uint8_t cnt = (uint8_t)counts[ci];
                            if (form == 0) {
                                if (shape == 0) enc12(c, (uint8_t)cmd, cnt, 0, 0x00);
                                else            enc12(c, (uint8_t)cmd, cnt, 0x0234567, 0xE0);
                                c[1] = (uint8_t)(proto << 1);
                            } else {
                                bool ext = form == 2;
                                if (shape == 0) enc16(c, (uint8_t)cmd, ext, cnt, 0, 0x00);
                                else            enc16(c, (uint8_t)cmd, ext, cnt, ext ? 0x0123456789Aull : 0x0234567, 0xE0);
                                c[1] = (uint8_t)((proto << 1) | (ext ? 1 : 0));
                            }
                            c[2] = tdir ? 0x0E : 0x06;
                            check(c, form == 0 ? 12 : 16, tdir != 0, cnt * 512u, true, true);
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
                check(c, seeds[i].len, true, seeds[i].count * 512u, true, true);
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
            check(seeds[i].cdb, len, true, seeds[i].count * 512u, true, true);
    printf("[4] CB length 0..255: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 5. State: mounted x LBA mode, for every seed and for its single-byte changes.
    section = "5 state";
    a0 = n_allow; c0 = n_checks;
    for (int i = 0; i < n_seeds; i++)
        for (int m = 0; m < 2; m++)
            for (int l = 0; l < 2; l++) {
                check(seeds[i].cdb, seeds[i].len, true, seeds[i].count * 512u, m != 0, l != 0);
                for (int pos = 0; pos < 16; pos++) {
                    uint8_t c[16];
                    memcpy(c, seeds[i].cdb, 16);
                    c[pos] ^= 0x01;
                    check(c, seeds[i].len, true, seeds[i].count * 512u, m != 0, l != 0);
                }
            }
    printf("[5] mounted x LBA mode: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 6. Every SCSI opcode in byte 0, with an otherwise valid body.
    section = "6 opcode";
    a0 = n_allow; c0 = n_checks;
    for (int i = 0; i < n_seeds; i++)
        for (unsigned op = 0; op < 256; op++) {
            uint8_t c[16];
            memcpy(c, seeds[i].cdb, 16);
            c[0] = (uint8_t)op;
            check(c, 12, true, seeds[i].count * 512u, true, true);
            check(c, 16, true, seeds[i].count * 512u, true, true);
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
    printf("[7] LBA boundaries: %ld cases, %ld allowed\n", n_checks - c0, n_allow - a0);

    // 8. Named refusals: each must be refused. Most are already inside the
    //    sweeps; they are listed here so the report names them.
    section = "8 named";
    c0 = n_checks;
    int named_bad = 0;
    struct { const char *name; uint8_t c[16]; int len; uint32_t xfer; } named[] = {
        { "SRST template (SAT protocol 1)",       { 0xA1, 0x02 }, 12, 0 },
        { "Return Response Info (protocol 15)",   { 0xA1, 0x1E, 0x20 }, 12, 0 },
        { "READ DMA 0xC8",                        { 0xA1, 0x0C, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0xC8 }, 12, 512 },
        { "READ DMA 0xC8 as PIO",                 { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0xC8 }, 12, 512 },
        { "READ DMA EXT 0x25",                    { 0x85, 0x0D, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x25 }, 16, 512 },
        { "READ DMA EXT 0x25 as PIO",             { 0x85, 0x09, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x25 }, 16, 512 },
        { "READ SECTORS EXT 0x24 in 12-byte CDB", { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x24 }, 12, 512 },
        { "READ SECTORS EXT with EXTEND=0",       { 0x85, 0x08, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x24 }, 16, 512 },
        { "READ SECTORS with EXTEND=1",           { 0x85, 0x09, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x20 }, 16, 512 },
        { "READ SECTORS count 0 (ATA 256)",       { 0xA1, 0x08, 0x0E, 0, 0, 0, 0, 0, 0xE0, 0x20 }, 12, 0 },
        { "READ SECTORS count 9",                 { 0xA1, 0x08, 0x0E, 0, 9, 0, 0, 0, 0xE0, 0x20 }, 12, 9 * 512 },
        { "READ SECTORS count 128 (imagelba default chunk)", { 0xA1, 0x08, 0x0E, 0, 128, 0, 0, 0, 0xE0, 0x20 }, 12, 128 * 512 },
        { "READ SECTORS in CHS form (LBA bit 0)", { 0xA1, 0x08, 0x0E, 0, 1, 1, 0, 0, 0xA0, 0x20 }, 12, 512 },
        { "READ SECTORS to device 1 (DEV bit)",   { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xF0, 0x20 }, 12, 512 },
        { "READ SECTORS, CK_COND=1",              { 0xA1, 0x08, 0x2E, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, 512 },
        { "READ SECTORS, T_DIR=0",                { 0xA1, 0x08, 0x06, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, 512 },
        { "READ SECTORS, BYTE_BLOCK=0",           { 0xA1, 0x08, 0x0A, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, 512 },
        { "READ SECTORS, MULTIPLE_COUNT=1",       { 0xA1, 0x28, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, 512 },
        { "READ SECTORS, PIO data-out protocol",  { 0xA1, 0x0A, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x20 }, 12, 512 },
        { "READ VERIFY 0x40 (not in stage 1)",    { 0xA1, 0x06, 0x00, 0, 1, 0, 0, 0, 0xE0, 0x40 }, 12, 0 },
        { "SMART READ DATA (not in stage 1)",     { 0xA1, 0x08, 0x0E, 0xD0, 1, 0, 0x4F, 0xC2, 0xA0, 0xB0 }, 12, 512 },
        { "SMART READ LOG (not in stage 1)",      { 0x85, 0x08, 0x0E, 0, 0xD5, 0, 1, 0, 0, 0, 0x4F, 0, 0xC2, 0xA0, 0xB0 }, 16, 512 },
        { "WRITE SECTORS 0x30",                   { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xE0, 0x30 }, 12, 512 },
        { "WRITE SECTORS 0x30 dressed as data-in",{ 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0xE0, 0x30 }, 12, 512 },
        { "WRITE SECTORS EXT 0x34",               { 0x85, 0x0B, 0x06, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xE0, 0x34 }, 16, 512 },
        { "WRITE DMA 0xCA",                       { 0xA1, 0x0C, 0x06, 0, 1, 0, 0, 0, 0xE0, 0xCA }, 12, 512 },
        { "SECURITY ERASE UNIT 0xF4",             { 0xA1, 0x0A, 0x06, 0, 1, 0, 0, 0, 0xA0, 0xF4 }, 12, 512 },
        { "SET MAX ADDRESS 0xF9",                 { 0xA1, 0x06, 0x00, 0, 0, 0, 0, 0, 0xE0, 0xF9 }, 12, 0 },
        { "DOWNLOAD MICROCODE 0x92",              { 0xA1, 0x0A, 0x06, 0x07, 1, 0, 0, 0, 0xA0, 0x92 }, 12, 512 },
        { "IDENTIFY with CB length 16",           { 0xA1, 0x08, 0x0E, 0, 1, 0, 0, 0, 0, 0xEC }, 16, 512 },
        { "IDENTIFY, 85 form with CB length 12",  { 0x85, 0x08, 0x0E, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0xEC }, 12, 512 },
    };
    for (unsigned i = 0; i < sizeof(named) / sizeof(named[0]); i++) {
        bool a = check(named[i].c, named[i].len, true, named[i].xfer, true, true);
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
        sat_input_t in = { NULL, 12, true, 512, true, true };
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
                sat_input_t in = { cdb, out.cb_len, out.dir_in, out.xfer_len, true, true };
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
        // data-out CBW with a valid read CDB, end to end
        uint8_t out_img[31];
        memcpy(out_img, img, 31);
        out_img[12] = 0x00;
        sat_cbw_t o;
        n_checks++;
        if (!sat_cbw_parse(out_img, 0, cdb, 512, &o) || o.dir_in) { n_fail++; printf("  FAIL data-out CBW parse\n"); }
        sat_input_t in = { cdb, o.cb_len, o.dir_in, o.xfer_len, true, true };
        sat_taskfile_t tf;
        n_checks++;
        if (sat_policy_check(&in, &tf).allow) { n_fail++; printf("  FAIL data-out CBW allowed\n"); }
        // NULL arguments
        n_checks++;
        if (sat_cbw_parse(NULL, 0, cdb, 512, &o) || sat_cbw_parse(img, 0, NULL, 512, &o) ||
            sat_cbw_parse(img, 0, cdb, 512, NULL)) { n_fail++; printf("  FAIL cbw parse NULL\n"); }
    }
    printf("[10] CBW parse: %ld cases\n", n_checks - c0);

    printf("\n%ld assertions, %ld allowed, %ld refused, %ld failures\n", n_checks, n_allow, n_refuse, n_fail);
    if (n_fail) { printf("SAT POLICY TEST: FAIL\n"); return 1; }
    printf("SAT POLICY TEST: PASS\n");
    return 0;
}
