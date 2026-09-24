// Restricted SAT policy. See sat_policy.h for the allowed set.
//
// Every check below is a refusal point. The function reads the CDB once into
// a local copy, validates the copy, and builds the task file from the same
// copy, so the bytes checked are the bytes that get sent to the drive.

#include "sat_policy.h"
#include <string.h>

#define ATA_IDENTIFY             0xEC
#define ATA_READ_SECTORS         0x20
#define ATA_READ_SECTORS_NR      0x21
#define ATA_READ_SECTORS_EXT     0x24
#define ATA_READ_VERIFY          0x40
#define ATA_SMART                0xB0
#define ATA_READ_NATIVE_MAX      0xF8
#define ATA_READ_NATIVE_MAX_EXT  0x27

// SMART subcommands (FEATURES) and the signature SMART needs in LBA mid/high.
#define SMART_READ_DATA          0xD0
#define SMART_READ_THRESHOLDS    0xD1
#define SMART_READ_LOG           0xD5
#define SMART_RETURN_STATUS      0xDA
#define SMART_LBA_MID            0x4F
#define SMART_LBA_HIGH           0xC2

// Byte 2 of both CDB forms for PIO data-in. OFF_LINE=0, CK_COND=0, T_TYPE=0,
// T_DIR=1, BYTE_BLOCK=1, T_LENGTH=2. Nothing else is accepted.
#define SAT_BYTE2_PIO_IN         0x0E
// Byte 2 for non-data: CK_COND may be set, and T_DIR / BYTE_BLOCK, which only
// describe a data phase, are ignored. OFF_LINE, T_TYPE and T_LENGTH must be 0.
#define SAT_BYTE2_CK_COND        0x20
#define SAT_BYTE2_NO_DATA_PHASE  0x0C

// What the drive's IDENTIFY says it supports (sat_policy.h, "Drive
// capability"). A row lists the capabilities it needs; any missing is a
// refusal.
#define CAP_SMART      0x01u    // word 82 bit 0: SMART feature set
#define CAP_SMART_LOG  0x02u    // word 84 bit 0: SMART error logging (SMART READ LOG)
#define CAP_HPA        0x04u    // word 82 bit 10: Host Protected Area feature set
#define CAP_LBA48      0x08u    // word 83 bit 10: 48-bit Address feature set
#define CAP_SMART_ON   0x10u    // word 85 bit 0: SMART feature set enabled (valid per word 87)

// Nothing counts without a captured IDENTIFY whose words 82..84 are valid:
// the 01b signature in bits 15:14 of words 83 and 84, and a word 82 that is
// not 0000h or FFFFh. (83 and 84 of 0000h or FFFFh fail the signature.)
// Word 85 (what is enabled) counts only when bits 15:14 of word 87 are 01b,
// which is how ATA marks words 85..87 valid (sat_policy.h).
static unsigned drive_caps(const sat_input_t *in) {
    if (!in->id_captured) return 0;
    if ((in->id_w83 & 0xC000u) != 0x4000u) return 0;
    if ((in->id_w84 & 0xC000u) != 0x4000u) return 0;
    if (in->id_w82 == 0x0000u || in->id_w82 == 0xFFFFu) return 0;
    unsigned caps = 0;
    if (in->id_w82 & (1u << 0))  caps |= CAP_SMART;
    if (in->id_w84 & (1u << 0))  caps |= CAP_SMART_LOG;
    if (in->id_w82 & (1u << 10)) caps |= CAP_HPA;
    if (in->id_w83 & (1u << 10)) caps |= CAP_LBA48;
    if ((in->id_w87 & 0xC000u) == 0x4000u && (in->id_w85 & (1u << 0))) caps |= CAP_SMART_ON;
    return caps;
}

static sat_verdict_t refuse(uint8_t sk, uint8_t asc) {
    sat_verdict_t v = { false, sk, asc, 0x00, false };
    return v;
}

sat_verdict_t sat_policy_check(const sat_input_t *in, sat_taskfile_t *tf) {
    if (tf) memset(tf, 0, sizeof(*tf));
    if (!in || !in->cdb || !tf)
        return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_OPCODE);

    uint8_t c[16];
    memcpy(c, in->cdb, sizeof(c));

    // ---- form and length ------------------------------------------------
    bool is16;
    if (c[0] == 0xA1 && in->cdb_len == 12) is16 = false;
    else if (c[0] == 0x85 && in->cdb_len == 16) is16 = true;
    else return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_OPCODE);

    // ---- map the register fields out of the form ------------------------
    uint8_t b1 = c[1], b2 = c[2];
    uint8_t feat, count, lba0, lba1, lba2, dev, cmd, ctrl;
    uint8_t hfeat = 0, hcount = 0, lba3 = 0, lba4 = 0, lba5 = 0;
    bool ext = false;
    if (!is16) {
        feat = c[3]; count = c[4];
        lba0 = c[5]; lba1 = c[6]; lba2 = c[7];
        dev  = c[8]; cmd  = c[9]; ctrl = c[11];
        if (b1 & 0x01) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // reserved
        if (c[10] != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // reserved
    } else {
        ext  = (b1 & 0x01) != 0;
        hfeat = c[3];  feat  = c[4];
        hcount = c[5]; count = c[6];
        lba3 = c[7];  lba0 = c[8];
        lba4 = c[9];  lba1 = c[10];
        lba5 = c[11]; lba2 = c[12];
        dev  = c[13]; cmd  = c[14]; ctrl = c[15];
    }

    // ---- fields that are the same for every allowed command -------------
    if ((b1 >> 5) != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);                  // MULTIPLE_COUNT
    if (ctrl != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    if (hfeat != 0 || hcount != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);      // no row uses them
    if (dev & 0x10) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // DEV: the bridge picks the device

    // ---- protocol and byte 2 --------------------------------------------
    uint8_t proto = (uint8_t)((b1 >> 1) & 0x0F);
    bool ck = false;
    if (proto == SAT_PROTO_PIO_IN) {
        if (b2 != SAT_BYTE2_PIO_IN) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    } else if (proto == SAT_PROTO_NON_DATA) {
        if ((b2 & (uint8_t)~(SAT_BYTE2_CK_COND | SAT_BYTE2_NO_DATA_PHASE)) != 0)
            return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);                              // OFF_LINE, T_TYPE, T_LENGTH
        ck = (b2 & SAT_BYTE2_CK_COND) != 0;
    } else {
        return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    }

    // ---- the closed command list ----------------------------------------
    uint64_t lba = 0;
    uint8_t tdev = 0;
    uint8_t want_proto = 0;     // the only protocol this row may use
    uint8_t sectors = 0;        // PIO data-in blocks
    bool need_ck = false;       // the row exists to return registers
    bool user_data = false;     // carries a user-data address: LBA mode only
    unsigned need = 0;          // CAP_* the drive's IDENTIFY must show
    switch (cmd) {
    case ATA_IDENTIFY:
        want_proto = SAT_PROTO_PIO_IN;
        if (ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (feat != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (count != 1) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (lba0 | lba1 | lba2 | lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (dev & 0x0F) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        tdev = 0x00;
        sectors = 1;
        break;

    case ATA_READ_SECTORS:
    case ATA_READ_SECTORS_NR:
        want_proto = SAT_PROTO_PIO_IN;
        user_data = true;
        if (ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (feat != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (!(dev & 0x40)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // LBA addressing only
        if (count < 1 || count > SAT_MAX_SECTORS) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        lba = (uint64_t)lba0 | ((uint64_t)lba1 << 8) | ((uint64_t)lba2 << 16) | ((uint64_t)(dev & 0x0F) << 24);
        if (lba + count > (1ull << 28)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_LBA_RANGE);
        tdev = (uint8_t)(0x40 | (dev & 0x0F));
        sectors = count;
        break;

    case ATA_READ_SECTORS_EXT:
        want_proto = SAT_PROTO_PIO_IN;
        user_data = true;
        need = CAP_LBA48;       // read ext: a pre-48-bit drive does not know 0x24
        if (!is16 || !ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (feat != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if ((dev & 0x4F) != 0x40) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // LBA=1, low nibble reserved
        if (count < 1 || count > SAT_MAX_SECTORS) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        lba = (uint64_t)lba0 | ((uint64_t)lba1 << 8) | ((uint64_t)lba2 << 16) |
              ((uint64_t)lba3 << 24) | ((uint64_t)lba4 << 32) | ((uint64_t)lba5 << 40);
        if (lba + count > (1ull << 48)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_LBA_RANGE);
        tdev = 0x40;
        sectors = count;
        break;

    case ATA_READ_VERIFY:
        // The drive reads the sectors and checks them, and sends nothing back.
        // Same addressing rules as READ SECTORS; the count is not capped by the
        // MSC buffer because no data moves (0 means 256, as in ATA).
        want_proto = SAT_PROTO_NON_DATA;
        user_data = true;
        if (ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (feat != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (!(dev & 0x40)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // verify: LBA addressing only
        lba = (uint64_t)lba0 | ((uint64_t)lba1 << 8) | ((uint64_t)lba2 << 16) | ((uint64_t)(dev & 0x0F) << 24);
        {
            uint32_t n = count ? count : 256u;
            if (lba + n > (1ull << 28)) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_LBA_RANGE);
        }
        tdev = (uint8_t)(0x40 | (dev & 0x0F));
        break;

    case ATA_SMART:
        // SMART is 28-bit only, and every SMART command carries 4F/C2 in LBA
        // mid/high. The subcommand in FEATURES is the real verb: only the four
        // below read anything, the rest change drive state and are refused.
        if (ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (lba1 != SMART_LBA_MID || lba2 != SMART_LBA_HIGH) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (dev & 0x0F) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);   // smart: low nibble reserved
        need = CAP_SMART | CAP_SMART_ON;    // smart: the drive has SMART, and it is enabled
        switch (feat) {
        case SMART_READ_DATA:
        case SMART_READ_THRESHOLDS:
            if (feat == SMART_READ_DATA && !ATABOY_SAT_SMART_SAVES)   // smart d0 saves to the drive
                return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
            want_proto = SAT_PROTO_PIO_IN;
            if (count != 1) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
            if (lba0 != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
            sectors = 1;
            break;
        case SMART_READ_LOG:
            // Any log address (LBA low); reading a log changes nothing.
            want_proto = SAT_PROTO_PIO_IN;
            need = CAP_SMART | CAP_SMART_ON | CAP_SMART_LOG;   // smart read log
            if (count == 0 || count > SAT_MAX_SECTORS) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
            sectors = count;
            break;
        case SMART_RETURN_STATUS:
            if (!ATABOY_SAT_SMART_SAVES)                                // smart da saves to the drive
                return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
            want_proto = SAT_PROTO_NON_DATA;
            need_ck = true;
            if (count != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
            if (lba0 != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
            break;
        default:
            return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        }
        tdev = 0x00;
        break;

    case ATA_READ_NATIVE_MAX:
        // A query: the drive reports its native maximum LBA in the registers.
        // Nothing the firmware does follows it with SET MAX ADDRESS, and SET
        // MAX is refused here, so the query cannot become half of a change.
        want_proto = SAT_PROTO_NON_DATA;
        need_ck = true;
        need = CAP_HPA;         // native max
        if (ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (feat != 0 || count != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (lba0 | lba1 | lba2 | lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if ((dev & 0x4F) != 0x40) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max: LBA=1, low nibble 0
        tdev = 0x40;
        break;

    case ATA_READ_NATIVE_MAX_EXT:
        want_proto = SAT_PROTO_NON_DATA;
        need_ck = true;
        need = CAP_HPA | CAP_LBA48;     // native max ext
        if (!is16 || !ext) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max ext: 16-byte, EXTEND=1
        if (feat != 0 || count != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (lba0 | lba1 | lba2 | lba3 | lba4 | lba5) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if ((dev & 0x4F) != 0x40) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);  // native max ext: LBA=1, low nibble 0
        tdev = 0x40;
        break;

    default:
        return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    }

    // ---- the row decides the protocol and whether CK_COND is required ---
    if (proto != want_proto) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    if (need_ck && !ck) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);

    // ---- the CBW must ask for exactly the data the CDB describes --------
    if (want_proto == SAT_PROTO_PIO_IN) {
        if (!in->dir_in) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
        if (in->xfer_len != (uint32_t)count * SAT_SECTOR_SIZE) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    } else {
        // Non-data: no data phase at all. BOT says the direction bit means
        // nothing when the length is 0, so only the length is checked.
        if (in->xfer_len != 0) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    }

    // ---- state ----------------------------------------------------------
    if (!in->mounted) return refuse(SAT_SK_NOT_READY, SAT_ASC_NOT_READY);
    // Set up by Ctrl+G, never asked IDENTIFY: nothing at all goes through,
    // IDENTIFY first among them (sat_policy.h, review of 0.6f3p8 L-2).
    if (in->manual_chs) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    // An LBA address sent to a drive that was set up in CHS mode may not be
    // understood as LBA at all; an old CHS-only drive would act on some other
    // sector and report success. So reads and verifies need LBA mode.
    // IDENTIFY, SMART and READ NATIVE MAX name no user sector and are allowed
    // in CHS mode as well.
    if (user_data && !in->lba_mode) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);
    // The drive's own IDENTIFY must show every optional feature the row uses
    // (sat_policy.h, "Drive capability"). No captured IDENTIFY: refused.
    if ((drive_caps(in) & need) != need) return refuse(SAT_SK_ILLEGAL_REQUEST, SAT_ASC_INVALID_FIELD);

    // Allowed. The task file is written only now, so a refusal never leaves
    // anything in it but zeros.
    tf->command = cmd;
    tf->feature = feat;
    tf->device = tdev;
    tf->count = count;
    tf->lba_low = lba0; tf->lba_mid = lba1; tf->lba_high = lba2;
    tf->ext = ext;
    if (ext) { tf->hob_lba_low = lba3; tf->hob_lba_mid = lba4; tf->hob_lba_high = lba5; }
    tf->protocol = want_proto;
    tf->ck_cond = ck;
    tf->sectors = sectors;

    // A row that needed anything from IDENTIFY needs the drive to be the one
    // that IDENTIFY came from: sat.c checks that before sending (review L-1:
    // derived here, so the list of rows cannot drift from the policy's).
    sat_verdict_t ok = { true, 0, 0, 0, need != 0 };
    return ok;
}

bool sat_cbw_parse(const uint8_t *img, uint8_t lun, const uint8_t cdb[16],
                   uint16_t bufsize16, sat_cbw_t *out) {
    if (!img || !cdb || !out) return false;
    memset(out, 0, sizeof(*out));
    // dCBWSignature "USBC", little-endian
    if (img[0] != 0x55 || img[1] != 0x53 || img[2] != 0x42 || img[3] != 0x43) return false;
    uint32_t len = (uint32_t)img[8] | ((uint32_t)img[9] << 8) |
                   ((uint32_t)img[10] << 16) | ((uint32_t)img[11] << 24);
    if (img[13] != lun) return false;                 // reserved bits 7..4 must be 0
    if (memcmp(img + 15, cdb, 16) != 0) return false;
    if ((uint16_t)len != bufsize16) return false;
    // bmCBWFlags: bit 7 is the direction, bits 6..0 are reserved. Anything
    // but exactly 0x80 counts as "not data-in", which the policy refuses for
    // every data-in row. (Non-data rows need a length of 0 and ignore it.)
    out->dir_in = (img[12] == 0x80);
    out->xfer_len = len;
    out->cb_len = img[14];                            // reserved bits kept, so they fail the length check
    return true;
}
