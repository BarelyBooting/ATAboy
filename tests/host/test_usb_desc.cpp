// Host tests for the USB string descriptors (usb_descriptors.c), 0.6f3p9:
// the serial number (string 3) is the unit's own RP2350 board id, not
// upstream's constant "654321". The board id comes from a stand-in for
// pico_unique_id that formats a chosen 8-byte id as the SDK does, so the
// test can give the firmware two different units.
//
// Build and run: see run.sh. Exit status is the number of failed checks.
#include "mock_pico.h"
#include <stdio.h>
#include <string>

// ---- the parts of TinyUSB's descriptor headers usb_descriptors.c uses -------
typedef struct {
    uint8_t  bLength, bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0;
    uint16_t idVendor, idProduct, bcdDevice;
    uint8_t  iManufacturer, iProduct, iSerialNumber, bNumConfigurations;
} tusb_desc_device_t;
#define TUSB_DESC_DEVICE        0x01
#define TUSB_DESC_STRING        0x03
#define TUSB_CLASS_MISC         0xEF
#define MISC_SUBCLASS_COMMON    0x02
#define MISC_PROTOCOL_IAD       0x01
#define CFG_TUD_ENDPOINT0_SIZE  64
#define TUD_CONFIG_DESC_LEN     9
#define TUD_CDC_DESC_LEN        66
#define TUD_MSC_DESC_LEN        23
#define TUD_CONFIG_DESCRIPTOR(...) 0
#define TUD_CDC_DESCRIPTOR(...)    0
#define TUD_MSC_DESCRIPTOR(...)    0

#include "usb_descriptors.c"

uint64_t mock_now_ns = 0;
uint32_t mock_gpio_out = 0;
MockSio mock_sio;
bool mock_intrq(void) { return false; }
void mock_reset_line(bool) {}
bool tud_msc_set_sense(uint8_t, uint8_t, uint8_t, uint8_t) { return true; }
int32_t tud_msc_request_sense_cb(uint8_t, void *, uint16_t) { return 18; }

static uint8_t board_id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
static int id_calls = 0;
// As pico-sdk 2.2.0 src/rp2_common/pico_unique_id/unique_id.c formats it.
void pico_get_unique_board_id_string(char *id_out, unsigned len) {
    id_calls++;
    unsigned i;
    for (i = 0; (i < len - 1) && (i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2); i++) {
        int nibble = (board_id[i / 2] >> (4 - 4 * (i & 1))) & 0xf;
        id_out[i] = (char)(nibble < 10 ? nibble + '0' : nibble + 'A' - 10);
    }
    id_out[i] = 0;
}

static int failures = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

// String descriptor `index` as ASCII; "" with ok false if it is malformed.
static std::string string_desc(uint8_t index, bool *ok) {
    const uint16_t *d = tud_descriptor_string_cb(index, 0x0409);
    *ok = false;
    if (!d) return "";
    unsigned len = d[0] & 0xFF, type = d[0] >> 8;
    if (type != TUSB_DESC_STRING || len < 2 || (len & 1)) return "";
    std::string s;
    for (unsigned i = 1; i < len / 2; i++) {
        if (d[i] > 0x7F) return "";
        s.push_back((char)d[i]);
    }
    *ok = true;
    return s;
}

static void set_id(const uint8_t (&id)[8]) { memcpy(board_id, id, 8); }

int main() {
    bool ok;
    CHECK(desc_device.iSerialNumber == 3, "iSerialNumber %u", desc_device.iSerialNumber);
    const uint8_t unit_a[8] = { 0xE6, 0x61, 0x41, 0x03, 0xE7, 0x45, 0x2D, 0x2F };
    const uint8_t unit_b[8] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    set_id(unit_a);
    id_calls = 0;
    std::string a = string_desc(3, &ok);
    CHECK(ok && a == "E6614103E7452D2F", "unit A serial \"%s\"", a.c_str());
    CHECK(id_calls > 0, "the board id was never asked for");
    const uint16_t *d = tud_descriptor_string_cb(3, 0x0409);
    CHECK(d && d[0] == ((TUSB_DESC_STRING << 8) | (2 * 16 + 2)), "descriptor header %04X", d ? d[0] : 0);
    set_id(unit_b);
    std::string b = string_desc(3, &ok);
    CHECK(ok && b == "0123456789ABCDEF", "unit B serial \"%s\"", b.c_str());
    CHECK(a != b, "two units, one serial");
    CHECK(a.find("654321") == std::string::npos && b != "654321", "the upstream constant");
    // The same unit keeps its serial.
    CHECK(string_desc(3, &ok) == b && ok, "serial changed between two requests");
    // The other strings are as they were.
    CHECK(string_desc(1, &ok) == "Obsolete Tech" && ok, "manufacturer");
    CHECK(string_desc(2, &ok) == "ATAboy" && ok, "product");
    CHECK(string_desc(4, &ok) == "ATAboy CDC" && ok, "CDC interface");
    CHECK(string_desc(5, &ok) == "ATAboy Storage" && ok, "MSC interface");
    const uint16_t *lang = tud_descriptor_string_cb(0, 0);
    CHECK(lang && lang[0] == ((TUSB_DESC_STRING << 8) | 4) && lang[1] == 0x0409, "language %04X", lang ? lang[1] : 0);
    CHECK(tud_descriptor_string_cb(6, 0x0409) == nullptr, "string 6 exists");
    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
