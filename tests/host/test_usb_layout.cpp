// Host test for the USB configuration descriptor's layout (palimpsest
// api/SPEC-v1.md 11.1, closing its finding F-3): usb_descriptors.c compiled
// against TinyUSB's REAL descriptor headers (pico-sdk 2.2.0's copy), so the
// TUD_*_DESCRIPTOR macros are the ones the firmware gets. test_usb_desc.cpp
// replaces them with 0 and so never saw the layout.
//
// Run twice by run.sh: with ATABOY_CTL=0 the bytes must be today's (one CDC,
// MSC; written out below by hand from the USB and CDC specifications, not
// captured from a build), and with ATABOY_CTL=1 the Control function must be
// appended as interfaces 3 and 4 with its own IAD, endpoints 84h (interrupt,
// 8 bytes), 05h and 85h (bulk, 64), string 6 "ATAboy Control", bcdDevice
// 0101h, and the device class kept at EF/02/01.
//
// Build: g++ -I <this dir's layout stand-ins> -I <tinyusb/src> -I <firmware>.
// Exit status is the number of failed checks.
#define CFG_TUSB_MCU OPT_MCU_NONE
#include "usb_descriptors.c"
#include <stdio.h>
#include <set>
#include <string>
#include <vector>

void pico_get_unique_board_id_string(char *id_out, unsigned len) { snprintf(id_out, len, "0123456789ABCDEF"); }

static int failures = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

// Written out by hand: configuration, then per CDC function its IAD, the
// communication interface with its header, call management, ACM and union
// functional descriptors and notification endpoint, the data interface with
// its two bulk endpoints; then MSC's interface and two bulk endpoints.
static const uint8_t cdc_tui[] = {
    0x08, 0x0B, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,             // IAD: interfaces 0-1, CDC ACM, no string
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x00, 0x04,       // interface 0, 1 endpoint, string 4
    0x05, 0x24, 0x00, 0x20, 0x01,                               // header, CDC 1.20
    0x05, 0x24, 0x01, 0x00, 0x01,                               // call management: data interface 1
    0x04, 0x24, 0x02, 0x06,                                     // ACM: line coding, break
    0x05, 0x24, 0x06, 0x00, 0x01,                               // union: 0 controls 1
    0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x10,                   // 81h interrupt, 8 bytes, 16 ms
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,       // interface 1, CDC data, 2 endpoints
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,                   // 02h bulk out, 64
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,                   // 82h bulk in, 64
};
static const uint8_t msc[] = {
    0x09, 0x04, 0x02, 0x00, 0x02, 0x08, 0x06, 0x50, 0x05,       // interface 2, MSC SCSI BOT, string 5
    0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00,
};
static const uint8_t cdc_ctl[] = {
    0x08, 0x0B, 0x03, 0x02, 0x02, 0x02, 0x00, 0x00,             // IAD: interfaces 3-4
    0x09, 0x04, 0x03, 0x00, 0x01, 0x02, 0x02, 0x00, 0x06,       // interface 3, string 6
    0x05, 0x24, 0x00, 0x20, 0x01,
    0x05, 0x24, 0x01, 0x00, 0x04,                               // call management: data interface 4
    0x04, 0x24, 0x02, 0x06,
    0x05, 0x24, 0x06, 0x03, 0x04,                               // union: 3 controls 4
    0x07, 0x05, 0x84, 0x03, 0x08, 0x00, 0x10,                   // 84h interrupt, 8 bytes
    0x09, 0x04, 0x04, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,       // interface 4, CDC data
    0x07, 0x05, 0x05, 0x02, 0x40, 0x00, 0x00,                   // 05h bulk out, 64
    0x07, 0x05, 0x85, 0x02, 0x40, 0x00, 0x00,                   // 85h bulk in, 64
};

static std::string string_desc(uint8_t index) {
    const uint16_t *d = tud_descriptor_string_cb(index, 0x0409);
    if (!d) return "<none>";
    std::string s;
    for (unsigned i = 1; i < (d[0] & 0xFF) / 2u; i++) s.push_back((char)d[i]);
    return s;
}

int main() {
    const uint8_t *cfg = tud_descriptor_configuration_cb(0);
    unsigned total = cfg[2] | (cfg[3] << 8);
    std::vector<uint8_t> want = { 0x09, 0x02, 0, 0, 0, 0x01, 0x00, 0x80, 0x32 };
    want.insert(want.end(), cdc_tui, cdc_tui + sizeof cdc_tui);
    want.insert(want.end(), msc, msc + sizeof msc);
#if ATABOY_CTL
    want.insert(want.end(), cdc_ctl, cdc_ctl + sizeof cdc_ctl);
    want[4] = 5;
#else
    (void)cdc_ctl;
    want[4] = 3;
#endif
    want[2] = (uint8_t)want.size(); want[3] = (uint8_t)(want.size() >> 8);
    CHECK(total == sizeof desc_configuration && total == want.size(), "total length %u, array %u, expected %zu",
          total, (unsigned)sizeof desc_configuration, want.size());
    CHECK(sizeof desc_configuration == want.size() && memcmp(desc_configuration, want.data(), want.size()) == 0,
          "the bytes differ from the ones written out");
    // Walk it as a host would.
    unsigned itfs = 0, iads = 0;
    std::vector<std::pair<int, int>> iad_ranges;
    std::vector<uint8_t> eps;
    for (unsigned i = cfg[0]; i + 1 < total && cfg[i] >= 2; i += cfg[i]) {
        if (cfg[i + 1] == TUSB_DESC_INTERFACE) itfs++;
        if (cfg[i + 1] == TUSB_DESC_INTERFACE_ASSOCIATION) { iads++; iad_ranges.push_back({ cfg[i + 2], cfg[i + 3] }); }
        if (cfg[i + 1] == TUSB_DESC_ENDPOINT) eps.push_back(cfg[i + 2]);
    }
    std::set<uint8_t> ep_set(eps.begin(), eps.end());
    CHECK(ep_set.size() == eps.size(), "an endpoint address used twice");
    CHECK(cfg[4] == itfs, "bNumInterfaces %u, %u interfaces", cfg[4], itfs);
    CHECK(desc_device.bDeviceClass == 0xEF && desc_device.bDeviceSubClass == 0x02 && desc_device.bDeviceProtocol == 0x01,
          "device class %02X/%02X/%02X", desc_device.bDeviceClass, desc_device.bDeviceSubClass, desc_device.bDeviceProtocol);
    CHECK(desc_device.idVendor == 0xCAFE && desc_device.idProduct == 0x4005, "VID:PID");
#if ATABOY_CTL
    CHECK(CFG_TUD_CDC == 2, "CFG_TUD_CDC %d", CFG_TUD_CDC);
    CHECK(itfs == 5 && iads == 2 && iad_ranges[0] == std::make_pair(0, 2) && iad_ranges[1] == std::make_pair(3, 2),
          "%u interfaces, %u IADs", itfs, iads);
    CHECK(ep_set == std::set<uint8_t>({ 0x81, 0x02, 0x82, 0x03, 0x83, 0x84, 0x05, 0x85 }), "%zu endpoints", ep_set.size());
    CHECK(string_desc(6) == "ATAboy Control" && string_desc(7) == "<none>", "string 6 \"%s\"", string_desc(6).c_str());
    CHECK(desc_device.bcdDevice == 0x0101, "bcdDevice %04X", desc_device.bcdDevice);
#else
    CHECK(CFG_TUD_CDC == 1, "CFG_TUD_CDC %d", CFG_TUD_CDC);
    CHECK(itfs == 3 && iads == 1 && iad_ranges[0] == std::make_pair(0, 2), "%u interfaces, %u IADs", itfs, iads);
    CHECK(ep_set == std::set<uint8_t>({ 0x81, 0x02, 0x82, 0x03, 0x83 }), "%zu endpoints", ep_set.size());
    CHECK(string_desc(6) == "<none>", "string 6 \"%s\"", string_desc(6).c_str());
    CHECK(desc_device.bcdDevice == 0x0100, "bcdDevice %04X", desc_device.bcdDevice);
#endif
    CHECK(string_desc(4) == "ATAboy CDC" && string_desc(5) == "ATAboy Storage" && string_desc(2) == "ATAboy", "strings 2, 4, 5");
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
