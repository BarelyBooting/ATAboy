// ATAboy Menu System — runs on core 1
// All terminal I/O goes through cdc_printf/cdc_putchar/cdc_getchar_timeout_us
// which use pico SDK queues. Core 0 drains/fills these queues in cdc_task().
// NEVER call tud_task(), tud_cdc_*(), or any TinyUSB function from here.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "ide.h"
#include "config.h"
#include "fwupdate.h"
#include "manualchs.h"
#include "pico/util/queue.h"

// ---------------------------------------------------------------------------
//  CDC queue I/O (queues owned by ATAboy.c)
// ---------------------------------------------------------------------------

extern queue_t cdc_tx_queue;
extern queue_t cdc_rx_queue;
extern volatile bool cdc_connected;
extern volatile bool is_mounted;
extern volatile bool media_changed_waiting;
extern bool usb_msc_ide_busy(void);     // usb.c: an MSC command is using the IDE bus

static void cdc_putchar(char c) {
    queue_add_blocking(&cdc_tx_queue, &c);
}

static void cdc_puts(const char *s) {
    while (*s) cdc_putchar(*s++);
}

static void cdc_printf(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    cdc_puts(buf);
}

static int cdc_getchar_timeout_us(uint64_t timeout_us) {
    char c;
    if (queue_try_remove(&cdc_rx_queue, &c)) return (uint8_t)c;
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (!time_reached(deadline)) {
        if (queue_try_remove(&cdc_rx_queue, &c)) return (uint8_t)c;
        busy_wait_us_32(100);
    }
    return PICO_ERROR_TIMEOUT;
}

static void cdc_flush(void) {
    sleep_ms(5);
}

// ---------------------------------------------------------------------------
//  Box-drawing helper — emit a UTF-8 box char repeated N times
// ---------------------------------------------------------------------------

static void emit_n(const char *utf8_char, int n) {
    for (int i = 0; i < n; i++) cdc_puts(utf8_char);
}

// Box-drawing character constants (UTF-8, 3 bytes each)
#define BOX_HH   "\xe2\x95\x90"  /* double horizontal */
#define BOX_HL   "\xe2\x94\x80"  /* single horizontal */
#define BOX_VH   "\xe2\x95\x91"  /* double vertical   */
#define BOX_VL   "\xe2\x94\x82"  /* single vertical   */
#define BOX_TL   "\xe2\x95\x94"  /* top-left double   */
#define BOX_TR   "\xe2\x95\x97"  /* top-right double  */
#define BOX_BL   "\xe2\x95\x9a"  /* bot-left double   */
#define BOX_BR   "\xe2\x95\x9d"  /* bot-right double  */
#define BOX_TD   "\xe2\x95\xa4"  /* T-down mixed      */
#define BOX_TU   "\xe2\x94\xb4"  /* T-up single       */
#define BOX_ML   "\xe2\x95\x9f"  /* left-T mixed      */
#define BOX_MR   "\xe2\x95\xa2"  /* right-T mixed     */
#define BOX_MLD  "\xe2\x95\xa0"  /* left-T double     */
#define BOX_MRD  "\xe2\x95\xa3"  /* right-T double    */
#define BOX_LTD  "\xe2\x94\x9c"  /* left-T single     */
#define BOX_ARROW "\xe2\x96\xba" /* right arrow       */
#define BOX_ARRU "\xe2\x86\x91"  /* up arrow          */
#define BOX_ARRD "\xe2\x86\x93"  /* down arrow        */
#define BOX_ARRR "\xe2\x86\x92"  /* right arrow thin  */
#define BOX_ARRL "\xe2\x86\x90"  /* left arrow thin   */

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------

static char hdd_model_raw[48] = "";
static char hdd_status_text[80] = "";
static uint16_t detect_cyls = 0;
static uint8_t  detect_heads = 0;
static uint8_t  detect_spt = 0;
static uint16_t cur_cyls = 0;
static uint8_t  cur_heads = 0;
static uint8_t  cur_spt = 0;
static uint64_t total_lba_sectors = 0;
static bool     use_lba_mode = false;

static bool show_detect_result = false;
static bool force_detect = false;
static int  confirm_type = 0;

// IDENTIFY word 49 of the drive Auto Detect or auto-mount last identified
// (0.6f3p9, IORDY advisory below). Auto Detect forgets it when it starts,
// and Ctrl+G, which never sends IDENTIFY, forgets it too.
static bool     id_w49_known = false;
static uint16_t id_w49 = 0;

#define RESET       "\033[0m"
#define BG_BLUE     "\033[44m"
#define FG_WHITE    "\033[37;1m"
#define FG_RED      "\033[91m"
#define FG_GREEN    "\033[92m"
#define FG_YELLOW   "\033[33;1m"
#define HIDE_CUR    "\033[?25l"
#define CLR_SCR     "\033[2J"
#define SEL_RED     "\033[41;37;1m"

#define KEY_UP    1001
#define KEY_DOWN  1002
#define KEY_LEFT  1003
#define KEY_RIGHT 1004
#define KEY_ENTER 13
#define KEY_ESC   27
#define KEY_F10   1010

typedef enum {
    SCREEN_MAIN,
    SCREEN_FEATURES,
    SCREEN_CONFIRM,
    SCREEN_MOUNTED,
    SCREEN_DEBUG
} screen_t;

static screen_t current_screen = SCREEN_MAIN;
static screen_t confirm_return_screen = SCREEN_MAIN;
static bool needs_full_redraw = true;
static bool last_cdc_connected = false;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

static void draw_at(int x, int y, const char *text) {
    cdc_printf("\033[%d;%dH%s", y, x, text);
}

static int visible_strlen(const char *s) {
    int len = 0;
    while (*s) {
        if (*s == '\033') {
            s++;
            if (*s == '[') { s++; while (*s && (*s < '@' || *s > '~')) s++; if (*s) s++; }
        } else { len++; s++; }
    }
    return len;
}

static void sanitize_identify_model(char *s) {
    int len = strlen(s);
    while (len > 0 && s[len - 1] == ' ') s[--len] = '\0';
}

static void decode_ata_string(uint16_t *buf, int offset, int len_words, char *out) {
    int idx = 0;
    for (int i = 0; i < len_words; i++) {
        uint16_t w = buf[offset + i];
        out[idx++] = (char)(w >> 8);
        out[idx++] = (char)(w & 0xFF);
    }
    out[idx] = '\0';
    while (idx > 0 && out[idx - 1] == ' ') out[--idx] = '\0';
}

static void get_large_geometry(uint16_t n_cyl, uint8_t n_hd, uint8_t n_spt,
                               uint16_t *l_cyl, uint8_t *l_hd) {
    uint32_t total = (uint32_t)n_cyl * n_hd;
    uint8_t heads = n_hd;
    while (heads < 255 && (total / heads) > 65535) heads++;
    *l_hd = heads;
    *l_cyl = (uint16_t)(total / heads);
}

// ---------------------------------------------------------------------------
//  IORDY advisory (0.6f3p9)
// ---------------------------------------------------------------------------
// IDENTIFY word 49 bit 11 set means the drive says it supports IORDY. With
// IORDY switched on in Features and a drive whose IDENTIFY does not say so,
// one line says so: on the geometry picker, and on the main and Features
// screens while that drive is the one detected. Display only: the setting is
// not changed, and nothing is sent to the drive for it. A drive older than
// ATA-2 has 0000h in word 49, so the line says the drive does not declare
// IORDY, not that it cannot do it.
#define IORDY_NOTE      "IORDY on; drive does not declare IORDY"
#define IORDY_NOTE_LEN  38          // strlen(IORDY_NOTE): the left panel's width, cols 2 to 39
#define IORDY_NOTE_ROW  16

static bool iordy_note_due(bool known, uint16_t w49) {
    return known && config.iordy_enabled && !(w49 & 0x0800);
}

// The note at (col, row) in yellow, or the same cells blank in the frame's
// colours, so a redraw after IORDY is switched off clears it.
static void draw_iordy_note(int col, int row) {
    cdc_printf("\033[%d;%dH", row, col);
    if (iordy_note_due(id_w49_known, id_w49)) cdc_puts(FG_YELLOW IORDY_NOTE RESET BG_BLUE FG_WHITE);
    else cdc_printf(BG_BLUE FG_WHITE "%*s", IORDY_NOTE_LEN, "");
}

// ---------------------------------------------------------------------------
//  Help text display (right panel)
// ---------------------------------------------------------------------------

static void print_help(const char *text) {
    int start_row = 6, col = 57, width = 21, max_rows = 11;
    for (int i = 0; i < max_rows; i++)
        cdc_printf("\033[%d;%dH%*s", start_row + i, col, width, "");
    cdc_printf("\033[%d;%dH", start_row, col);
    int current_row = start_row, line_pos = 0, i = 0;
    while (text[i] && (current_row - start_row) < max_rows) {
        int word_len = 0;
        while (text[i + word_len] && text[i + word_len] != ' ' && text[i + word_len] != '\n') word_len++;
        if (line_pos + word_len > width && line_pos > 0) {
            current_row++;
            if ((current_row - start_row) >= max_rows) break;
            line_pos = 0;
            cdc_printf("\033[%d;%dH", current_row, col);
        }
        for (int j = 0; j < word_len; j++) { cdc_putchar(text[i++]); line_pos++; }
        if (text[i] == '\n') {
            current_row++;
            if ((current_row - start_row) >= max_rows) break;
            line_pos = 0;
            cdc_printf("\033[%d;%dH", current_row, col);
            i++;
        } else if (text[i] == ' ') {
            if (line_pos < width) { cdc_putchar(' '); line_pos++; }
            i++;
        }
    }
}

// ---------------------------------------------------------------------------
//  BIOS Frame — 80 columns
//  Row 2 top, rows 3-17 content+divider, row 18 mid, 19-20 keys,
//  row 21 mid, 22-23 status, row 24 bottom
// ---------------------------------------------------------------------------

// The top row. A build with SMART READ DATA and RETURN STATUS let through
// (ATABOY_SAT_SMART_SAVES, sat_policy.h) says so, so it cannot be mistaken on
// the unit for the shipping build (review L-2). That line is 11 characters
// longer, so it starts further left to stay inside 80 columns.
#if ATABOY_SAT && ATABOY_SAT_SMART_SAVES
#define BANNER "\033[1;4H" FG_WHITE "ATAboy Setup Utility v0.6f3p9 (fork+smartsaves) - (C) 2026 obsoletetech.us"
#else
#define BANNER "\033[1;10H" FG_WHITE "ATAboy Setup Utility v0.6f3p9 (fork) - (C) 2026 obsoletetech.us"
#endif

static void draw_bios_frame(void) {
    cdc_printf(BG_BLUE FG_WHITE CLR_SCR HIDE_CUR "\033[H");
    // Paint every cell of the 80x24 screen blue. ESC[2J alone is not enough:
    // whether an erase uses the current background colour depends on the
    // terminal (screen with bce off, for one, erases to the default colour).
    for (int row = 1; row <= 24; row++) cdc_printf("\033[%d;1H%80s", row, "");
    cdc_printf(BANNER);

    bool feat = (current_screen == SCREEN_FEATURES);

    // Row 2: top border (80 visible chars)
    // Main:     1 + 38 + 1 + 39 + 1 = 80
    // Features: 1 + 53 + 1 + 24 + 1 = 80
    cdc_puts("\033[2;1H");
    cdc_puts(BOX_TL);
    if (!feat) { emit_n(BOX_HH, 38); cdc_puts(BOX_TD); emit_n(BOX_HH, 39); }
    else       { emit_n(BOX_HH, 53); cdc_puts(BOX_TD); emit_n(BOX_HH, 24); }
    cdc_puts(BOX_TR);

    // Rows 3-23: side pipes + vertical divider
    for (int i = 3; i < 24; i++) {
        cdc_printf("\033[%d;1H" BOX_VH, i);
        cdc_printf("\033[%d;80H" BOX_VH, i);
        if (i < 18) {
            if (!feat) cdc_printf("\033[%d;40H" BOX_VL, i);
            else       cdc_printf("\033[%d;55H" BOX_VL, i);
        }
    }

    // Row 18: mid border (80 visible chars)
    // Main:     1 + 38 + 1 + 39 + 1 = 80
    // Features: 1 + 53 + 1 + 24 + 1 = 80
    cdc_puts("\033[18;1H");
    cdc_puts(BOX_ML);
    if (!feat) { emit_n(BOX_HL, 38); cdc_puts(BOX_TU); emit_n(BOX_HL, 39); }
    else       { emit_n(BOX_HL, 53); cdc_puts(BOX_TU); emit_n(BOX_HL, 24); }
    cdc_puts(BOX_MR);

    // Row 21: full-width mid border  1 + 78 + 1 = 80
    cdc_puts("\033[21;1H");
    cdc_puts(BOX_ML); emit_n(BOX_HL, 78); cdc_puts(BOX_MR);

    // Row 24: bottom border  1 + 78 + 1 = 80
    cdc_puts("\033[24;1H");
    cdc_puts(BOX_BL); emit_n(BOX_HH, 78); cdc_puts(BOX_BR);
}

// ---------------------------------------------------------------------------
//  HDD Status (rows 22-23) — shared by main + features screens
// ---------------------------------------------------------------------------

static void draw_hdd_status(void) {
    const char *display;
    bool is_error = false;
    if (show_detect_result && hdd_status_text[0]) { display = hdd_status_text; is_error = true; }
    else if (hdd_model_raw[0]) { display = hdd_model_raw; }
    else { display = "IDENT not run yet"; }

    const char *dev_label = (config.dev_base == 0xB0) ? " Slave" : " Master";
    int label_len = 13;
    int text_len = visible_strlen(display);
    int dev_len = (hdd_model_raw[0] && !is_error) ? (int)strlen(dev_label) : 0;
    int start_x = (80 - (label_len + text_len + dev_len)) / 2;

    draw_at(start_x, 22, FG_WHITE "Current HDD: ");
    if (is_error)        cdc_printf("\033[91;1m%s" RESET BG_BLUE FG_WHITE, display);
    else if (hdd_model_raw[0]) cdc_printf("\033[32;1m%s" FG_YELLOW "%s" RESET BG_BLUE FG_WHITE, display, dev_label);
    else                 cdc_printf(FG_YELLOW "%s" RESET BG_BLUE FG_WHITE, display);

    // Blank the rest of the row with blue spaces rather than ESC[K (see
    // draw_bios_frame): columns up to 77, then the right border as before.
    cdc_puts(RESET BG_BLUE FG_WHITE);
    int hdd_end = start_x + label_len + text_len + dev_len;
    if (hdd_end < 78) cdc_printf("%*s", 78 - hdd_end, "");
    cdc_puts("\033[22;78H  " BOX_VH);

    char geo_vals[64];
    if (use_lba_mode) {
        uint32_t mb = (uint32_t)((uint64_t)total_lba_sectors * 512 / 1048576);
        snprintf(geo_vals, sizeof(geo_vals), "LBA Mode Active (%lu MB)", (unsigned long)mb);
    } else {
        snprintf(geo_vals, sizeof(geo_vals), "%d Cyl / %d Hd / %d SPT", cur_cyls, cur_heads, cur_spt);
    }

    int geo_total_len = 18 + strlen(geo_vals);
    int geo_x = (80 - geo_total_len) / 2;
    draw_at(geo_x, 23, FG_WHITE "Current Geometry: ");

    bool is_valid = (use_lba_mode && total_lba_sectors > 0) ||
                    (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);
    cdc_printf(is_valid ? "\033[92;1m%s" : "\033[91;1m%s", geo_vals);

    cdc_puts(RESET BG_BLUE FG_WHITE);
    int geo_end = geo_x + geo_total_len;
    if (geo_end < 78) cdc_printf("%*s", 78 - geo_end, "");
    cdc_puts("\033[23;78H  " BOX_VH);
    cdc_puts("\033[24;79H");
    cdc_flush();
}

// ---------------------------------------------------------------------------
//  Main Menu
// ---------------------------------------------------------------------------

static void update_main_menu(void) {
    const char *items[] = {
        "  Auto Detect & Set Geometry",        // 0 — left,  row 6
        "  Mount HDD to USB Mass Storage",     // 1 — left,  row 8
        BOX_ARROW " ATAboy Features Setup",    // 2 — right, row 6
        "  Load Setup Defaults",               // 3 — right, row 8
        "  Save Setup to EEPROM"               // 4 — right, row 10
    };

    for (int i = 0; i < 5; i++) {
        int col = (i < 2) ? 4 : 43;
        int row = 6 + (i < 2 ? i : (i - 2)) * 2;
        cdc_printf("\033[%d;%dH", row, col);
        if (i == config.main_selected)
            cdc_printf(SEL_RED " %-28s " RESET BG_BLUE FG_WHITE, items[i]);
        else
            cdc_printf(FG_YELLOW " %-28s  " FG_WHITE, items[i]);
    }

    // Ctrl+G (manual CHS) and Ctrl+F (firmware update) only work with
    // nothing mounted, so they are only offered then.
    cdc_printf("\033[19;3H ESC: Quit to Main Menu%s"
               BOX_ARRU " " BOX_ARRD " " BOX_ARRR " " BOX_ARRL ": Select Item",
               is_mounted ? "                         " : "  Ctrl+G: Manual CHS     ");
    cdc_puts(is_mounted ? "\033[20;3H F10: Save Current Setup to EEPROM               Enter: Select"
                        : "\033[20;3H F10: Save Current Setup to EEPROM  Ctrl+F: Firmware Update  Enter: Select");

    draw_iordy_note(2, IORDY_NOTE_ROW);
    draw_hdd_status();
}

// ---------------------------------------------------------------------------
//  Features Menu
// ---------------------------------------------------------------------------

static void update_features_menu(void) {
    const char *labels[] = {"Write Protect", "Auto Mount at Start", "IORDY", "INTRQ", "Debug Mode"};
    const char *helps[] = {
        "Prevents any write commands from reaching the HDD.",
        "Automatically mounts the drive to USB on power-up sequence.",
        "Enables hardware IORDY (pin 27) flow control on the IDE bus.  Toggling this may help with picky drives.",
        "Enables hardware INTRQ (pin 28) for faster IDE command completion.  Toggling this may help with picky drives.",
        "Open low-level drive diagnostics and register status screen."
    };

    cdc_puts("\033[3;63H" FG_WHITE "Item Help");
    // col 55: single-T left + 24 dashes + mixed-T right = 26 chars
    cdc_puts("\033[4;55H" BOX_LTD);
    emit_n(BOX_HL, 24);
    cdc_puts(BOX_MR);

    for (int i = 0; i < 5; i++) {
        int row = 4 + i;
        cdc_printf("\033[%d;4H" FG_WHITE "%-25s", row, labels[i]);
        cdc_printf("\033[%d;35H" FG_YELLOW "[", row);
        cdc_puts(i == config.feat_selected ? SEL_RED : FG_YELLOW);

        if (i == 0)      cdc_printf("%-8s", config.drive_write_protected ? "Enabled" : "Disabled");
        else if (i == 1) cdc_printf("%-8s", config.auto_mount ? "Enabled" : "Disabled");
        else if (i == 2) cdc_printf("%-8s", config.iordy_enabled ? "Enabled" : "Disabled");
        else if (i == 3) cdc_printf("%-8s", config.intrq_enabled ? "Enabled" : "Disabled");
        else if (i == 4) cdc_printf("%-8s", "Enter");

        cdc_puts(RESET BG_BLUE FG_WHITE "]");
        if (i == config.feat_selected) print_help(helps[i]);
    }
    cdc_printf("\033[19;3H ESC: Quit to Main Menu                         "
               BOX_ARRU " " BOX_ARRD ": Select Item");
    cdc_puts("\033[20;3H F10: Save Current Setup to EEPROM               Enter: Select");

    draw_iordy_note(2, IORDY_NOTE_ROW);
    draw_hdd_status();
}

// ---------------------------------------------------------------------------
//  Input handler
// ---------------------------------------------------------------------------

static int get_input(void) {
    int c = cdc_getchar_timeout_us(100000);
    if (c == PICO_ERROR_TIMEOUT) return -1;
    if (c == 27) {
        int n1 = cdc_getchar_timeout_us(10000);
        if (n1 == '[') {
            int n2 = cdc_getchar_timeout_us(10000);
            if (n2 == 'A') return KEY_UP;
            if (n2 == 'B') return KEY_DOWN;
            if (n2 == 'C') return KEY_RIGHT;
            if (n2 == 'D') return KEY_LEFT;
            if (n2 == '2') { if (cdc_getchar_timeout_us(10000) == '1') if (cdc_getchar_timeout_us(10000) == '~') return KEY_F10; }
            // Consume unrecognized CSI sequences (e.g. PgUp/PgDn) so they don't trigger ESC
            if (n2 >= '0' && n2 <= '9') {
                int ch;
                do { ch = cdc_getchar_timeout_us(10000); } while (ch != '~' && ch != PICO_ERROR_TIMEOUT);
                return -1;
            }
        }
        return KEY_ESC;
    }
    return c;
}

// ---------------------------------------------------------------------------
//  Confirm box — 52 wide, red bg + shadow
// ---------------------------------------------------------------------------

static void draw_confirm_box(const char *message) {
    int box_w = 52, start_col = 14, start_row = 10;

    // Shadow
    for (int i = 0; i < 5; i++) cdc_printf("\033[%d;%dH\033[40m%*s", start_row+1+i, start_col+2, box_w, "");
    // Red bg
    for (int i = 0; i < 5; i++) cdc_printf("\033[%d;%dH\033[41m%*s", start_row+i, start_col, box_w, "");

    cdc_puts(SEL_RED);
    cdc_printf("\033[%d;%dH", start_row, start_col);
    cdc_puts(BOX_TL); emit_n(BOX_HH, 50); cdc_puts(BOX_TR);

    cdc_printf("\033[%d;%dH" BOX_VH "%50s" BOX_VH, start_row+1, start_col, "");

    int msg_len = visible_strlen(message);
    if (msg_len > 50) msg_len = 50;
    int pl = (50 - msg_len) / 2, pr = 50 - msg_len - pl;
    cdc_printf("\033[%d;%dH" BOX_VH "%*s%s%*s" BOX_VH, start_row+2, start_col, pl, "", message, pr, "");

    cdc_printf("\033[%d;%dH" BOX_VH "%50s" BOX_VH, start_row+3, start_col, "");

    cdc_printf("\033[%d;%dH", start_row+4, start_col);
    cdc_puts(BOX_BL); emit_n(BOX_HH, 50); cdc_puts(BOX_BR);
    cdc_puts(RESET);
}

// ---------------------------------------------------------------------------
//  Error box — 52 wide, 7 tall
// ---------------------------------------------------------------------------

static void draw_error_box(const char *error_msg, bool show_force) {
    int box_w = 52, start_col = 14, start_row = 9;

    for (int i = 0; i < 7; i++) cdc_printf("\033[%d;%dH\033[40m%*s", start_row+1+i, start_col+2, box_w, "");
    for (int i = 0; i < 7; i++) cdc_printf("\033[%d;%dH\033[41m%*s", start_row+i, start_col, box_w, "");

    cdc_puts(SEL_RED);
    cdc_printf("\033[%d;%dH", start_row, start_col);
    cdc_puts(BOX_TL); emit_n(BOX_HH, 50); cdc_puts(BOX_TR);

    cdc_printf("\033[%d;%dH" BOX_VH "                      ERROR!                      " BOX_VH, start_row+1, start_col);

    cdc_printf("\033[%d;%dH", start_row+2, start_col);
    cdc_puts(BOX_MLD); emit_n(BOX_HH, 50); cdc_puts(BOX_MRD);

    cdc_printf("\033[%d;%dH" BOX_VH "%50s" BOX_VH, start_row+3, start_col, "");
    int vl = visible_strlen(error_msg);
    int xo = (50 - vl) / 2;
    cdc_printf("\033[%d;%dH\033[41;97m%s", start_row+3, start_col+1+xo, error_msg);

    cdc_puts(SEL_RED);
    cdc_printf("\033[%d;%dH" BOX_VH "%50s" BOX_VH, start_row+4, start_col, "");

    if (show_force) {
        cdc_printf("\033[%d;%dH" BOX_VH "          \033[103;30m[OK]\033[0m" SEL_RED "                  " FG_YELLOW "[F: Force]" SEL_RED "        " BOX_VH, start_row+5, start_col);
    } else {
        cdc_printf("\033[%d;%dH" BOX_VH "                       \033[103;30m[OK]\033[0m" SEL_RED "                       " BOX_VH, start_row+5, start_col);
    }

    cdc_printf("\033[%d;%dH", start_row+6, start_col);
    cdc_puts(BOX_BL); emit_n(BOX_HH, 50); cdc_puts(BOX_BR);
    cdc_puts(RESET);
    cdc_flush();
}

// ---------------------------------------------------------------------------
//  Geometry selection overlay — 64 wide, 14 tall
// ---------------------------------------------------------------------------

static void draw_selection_menu_ex(uint16_t *id, int selected_idx, bool force_mode) {
    int box_w = 64, start_col = 9, start_row = 5;

    uint16_t n_cyl = id[1];
    uint8_t  n_hd  = (uint8_t)id[3];
    uint8_t  n_spt = (uint8_t)id[6];

    bool lba_supp = (id[49] & 0x0200);
    uint32_t lba28 = id[60] | ((uint32_t)id[61] << 16);
    uint64_t lba48 = ((uint64_t)id[103] << 48) | ((uint64_t)id[102] << 32) |
                     ((uint64_t)id[101] << 16) | ((uint64_t)id[100]);
    uint64_t lba_total = lba48 ? lba48 : (uint64_t)lba28;

    uint16_t l_cyl; uint8_t l_hd;
    get_large_geometry(n_cyl, n_hd, n_spt, &l_cyl, &l_hd);

    uint32_t n_size = (uint32_t)(((uint64_t)n_cyl * n_hd * n_spt * 512) / 1048576);
    uint32_t l_size = (uint32_t)(((uint64_t)l_cyl * l_hd * n_spt * 512) / 1048576);
    uint32_t lba_size = (uint32_t)((lba_total * 512ULL) / 1048576ULL);
    bool lba48_supp = ((id[83] & (1 << 10)) != 0) || (lba_total > 0x0FFFFFFFULL);

    // Shadow + bg
    for (int i = 0; i < 14; i++) cdc_printf("\033[%d;%dH\033[40m%*s", start_row+1+i, start_col+2, box_w, "");
    for (int i = 0; i < 14; i++) cdc_printf("\033[%d;%dH\033[41m%*s", start_row+i, start_col, box_w, "");

    cdc_puts(SEL_RED);

    // Row 0: 1+62+1 = 64
    cdc_printf("\033[%d;%dH", start_row, start_col);
    cdc_puts(BOX_TL); emit_n(BOX_HH, 62); cdc_puts(BOX_TR);

    // Row 1: model name centered
    int interior_w = 62;
    const char *det_dev_label = (config.dev_base == 0xB0) ? " Slave" : " Master";
    int vis_model = strlen(hdd_model_raw);
    int vis_total = 14 + vis_model + (int)strlen(det_dev_label);
    int title_x = start_col + 1 + ((interior_w - vis_total) / 2);
    cdc_printf("\033[%d;%dH%*s", start_row+1, start_col+1, interior_w, "");
    cdc_printf("\033[%d;%dH" BOX_VH, start_row+1, start_col);
    cdc_printf("\033[%d;%dHDetected HDD: \033[32;1m%s" FG_YELLOW "%s\033[0m", start_row+1, title_x, hdd_model_raw, det_dev_label);
    cdc_printf(SEL_RED "\033[%d;%dH" BOX_VH, start_row+1, start_col+63);

    cdc_printf("\033[%d;%dH" BOX_VH "                Select Drive Geometry Option:                 " BOX_VH, start_row+2, start_col);

    // Row 3: double separator
    cdc_printf("\033[%d;%dH", start_row+3, start_col);
    cdc_puts(BOX_MLD); emit_n(BOX_HH, 62); cdc_puts(BOX_MRD);

    cdc_printf("\033[%d;%dH" BOX_VH "  MODE         SIZE       CYLS      HEADS      SPT            " BOX_VH, start_row+4, start_col);

    // Row 5: single-line separator
    cdc_printf("\033[%d;%dH" BOX_VH " ", start_row+5, start_col);
    emit_n(BOX_HL, 60);
    cdc_puts(" " BOX_VH);

    const char *modes[] = {"NORMAL  ", "LARGE   ", "LBA     ", "MANUAL  "};
    const char *lba_label = lba48_supp ? "LBA48  " : "LBA     ";

    for (int i = 0; i < 4; i++) {
        cdc_printf("\033[%d;%dH" BOX_VH " ", start_row+6+i, start_col);
        bool is_lba = (i == 2), lba_na = (is_lba && !lba_supp);
        bool row_na = lba_na || (force_mode && (i == 0 || i == 1 || i == 2));

        if (i == selected_idx) {
            if (row_na) cdc_printf("\033[41;33m %-8s \033[0m" SEL_RED, is_lba ? lba_label : modes[i]);
            else        cdc_printf("\033[103;30m %-8s \033[0m" SEL_RED, is_lba ? lba_label : modes[i]);
        } else {
            if (row_na) cdc_printf("\033[90m %-8s ", is_lba ? lba_label : modes[i]);
            else        cdc_printf(FG_WHITE " %-8s ", is_lba ? lba_label : modes[i]);
        }

        if (force_mode && (i == 0 || i == 1 || i == 2)) {
            cdc_puts("\033[90m      --- MB    (No IDENTIFY Data)                 \033[0m" SEL_RED);
        } else if (lba_na) {
            cdc_puts("\033[90m      --- MB    (Not Supported by Drive)           \033[0m" SEL_RED);
        } else if (i == 0) {
            cdc_printf("   %4lu MB    %-5u       %-3u       %-3u            ", (unsigned long)n_size, n_cyl, n_hd, n_spt);
        } else if (i == 1) {
            cdc_printf("   %4lu MB    %-5u       %-3u       %-3u            ", (unsigned long)l_size, l_cyl, l_hd, n_spt);
        } else if (i == 2) {
            const char *bits = lba48_supp ? "48-bit" : "28-bit";
            if (lba_size > 9999) cdc_printf("   %4lu GB           LBA (%s)                  ", (unsigned long)(lba_size/1024), bits);
            else                 cdc_printf("   %4lu MB           LBA (%s)                  ", (unsigned long)lba_size, bits);
        } else if (i == 3) {
            if (detect_cyls > 0) {
                uint32_t ms = (uint32_t)((uint64_t)detect_cyls * detect_heads * detect_spt * 512 / 1048576);
                cdc_printf("   %4lu MB    %-5u       %-3u       %-3u            ", (unsigned long)ms, detect_cyls, detect_heads, detect_spt);
            } else cdc_puts("   ---- MB    -----       ---       ---            ");
        }
        cdc_puts(BOX_VH);
    }

    // Row 10
    cdc_printf("\033[%d;%dH", start_row+10, start_col);
    cdc_puts(BOX_MLD); emit_n(BOX_HH, 62); cdc_puts(BOX_MRD);

    cdc_printf("\033[%d;%dH" BOX_VH "  " BOX_ARRU " " BOX_ARRD ": Mode    TAB: Change CHS     Enter: Select    Esc: Quit " BOX_VH, start_row+11, start_col);
    if (iordy_note_due(true, id[49]))     // 12 + 38 + 12: the 62 columns inside
        cdc_printf("\033[%d;%dH" BOX_VH "            \033[33m" IORDY_NOTE "\033[0m" SEL_RED "            " BOX_VH, start_row+12, start_col);
    else
        cdc_printf("\033[%d;%dH" BOX_VH "  \033[33m   LBA Recommended for modern drives; NORMAL for legacy.    \033[0m" SEL_RED BOX_VH, start_row+12, start_col);

    // Row 13: bottom
    cdc_printf("\033[%d;%dH", start_row+13, start_col);
    cdc_puts(BOX_BL); emit_n(BOX_HH, 62); cdc_puts(BOX_BR);

    cdc_puts(RESET);
    cdc_flush();
}

static void draw_selection_menu(uint16_t *id, int selected_idx) {
    draw_selection_menu_ex(id, selected_idx, false);
}

// ---------------------------------------------------------------------------
//  Debug mode — 72 wide, 20 tall overlay
// ---------------------------------------------------------------------------

static void debug_print(int line_idx, const char *color, const char *fmt, ...) {
    if (line_idx > 16) return;
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    cdc_printf("\033[%d;%dH\033[40m%s%-*s", 4 + line_idx, 7, color, 68, buf);
    cdc_puts(RESET);
}

static void debug_cls(void) { for (int i = 0; i < 17; i++) debug_print(i, FG_WHITE, ""); }

static void draw_debug_overlay(void) {
    int box_w = 72, box_h = 20, sc = 5, sr = 3;
    int ix = sc+2, iy = sr+1, iw = box_w-4, ih = box_h-3;

    cdc_puts(RESET);
    for (int i = 0; i < box_h; i++) cdc_printf("\033[%d;%dH\033[40m%*s", sr+1+i, sc+2, box_w, "");
    for (int i = 0; i < box_h; i++) cdc_printf("\033[%d;%dH\033[44m%*s", sr+i, sc, box_w, "");
    for (int i = 0; i < ih; i++)    cdc_printf("\033[%d;%dH\033[40m%*s", iy+i, ix, iw, "");

    cdc_puts(FG_WHITE BG_BLUE);
    // Top: 1+70+1=72
    cdc_printf("\033[%d;%dH", sr, sc);
    cdc_puts(BOX_TL); emit_n(BOX_HH, 70); cdc_puts(BOX_TR);
    for (int i = 1; i < box_h-1; i++) {
        cdc_printf("\033[%d;%dH" BOX_VH, sr+i, sc);
        cdc_printf("\033[%d;%dH" BOX_VH, sr+i, sc+box_w-1);
    }
    // Bottom
    cdc_printf("\033[%d;%dH", sr+box_h-1, sc);
    cdc_puts(BOX_BL); emit_n(BOX_HH, 70); cdc_puts(BOX_BR);

    draw_at(sc+29, sr, "[ Debug Mode ]");
    cdc_printf("\033[%d;%dH" FG_WHITE BG_BLUE " ESC: Return  I: IDENT  T: Task File  E: Errors  S: Seek  R: Reset", sr+box_h-2, sc+3);
    cdc_puts(RESET);
    cdc_flush();
}

static void run_debug_identify(void) {
    debug_cls();
    debug_print(0, FG_YELLOW, "Sending IDENTIFY DEVICE (0xEC)...");

    uint16_t id[256];
    if (!ide_identify(id)) {
        debug_print(1, "\033[91;1m", "ERROR: Drive not ready or no data from IDENTIFY.");
        debug_print(2, FG_WHITE, "Check cabling, Master/Slave jumper, or power.");
        return;
    }

    char model[41], serial[21], fw[9];
    decode_ata_string(id, 27, 20, model);
    decode_ata_string(id, 10, 10, serial);
    decode_ata_string(id, 23, 4, fw);

    debug_print(0, "\033[92;1m", "IDENTIFY SUCCESSFUL");
    debug_print(2, FG_WHITE, "Model:  \033[96m%s", model);
    debug_print(3, FG_WHITE, "Serial: \033[96m%s", serial);
    debug_print(4, FG_WHITE, "FW Rev: \033[96m%s", fw);
    debug_print(5, FG_WHITE, "--------------------------------------------------");
    debug_print(6, FG_YELLOW, "[CHS Geometry]");
    debug_print(7, FG_WHITE, "Cyls: %-5d  Heads: %-3d  Sectors: %-3d", id[1], id[3], id[6]);

    bool lba_supp = (id[49] & 0x0200), lba48_supp = (id[83] & (1<<10));
    uint32_t lba28_cap = id[60] | ((uint32_t)id[61] << 16);
    uint64_t lba48_cap = ((uint64_t)id[103]<<48)|((uint64_t)id[102]<<32)|((uint64_t)id[101]<<16)|((uint64_t)id[100]);

    debug_print(9, FG_YELLOW, "[Capabilities]");
    debug_print(10, FG_WHITE, "LBA Supported:    %s", lba_supp ? "\033[92mYes" : "\033[91mNo");
    debug_print(11, FG_WHITE, "LBA48 Supported:  %s", lba48_supp ? "\033[92mYes" : "\033[91mNo");
    if (lba48_supp) {
        uint32_t gb = (uint32_t)(((uint64_t)lba48_cap*512)/1000000000);
        debug_print(12, FG_WHITE, "Capacity:         %lu Sectors (~%lu GB)", (unsigned long)lba48_cap, (unsigned long)gb);
    } else if (lba_supp) {
        uint32_t mb = (uint32_t)(((uint64_t)lba28_cap*512)/1000000);
        debug_print(12, FG_WHITE, "Capacity:         %lu Sectors (~%lu MB)", (unsigned long)lba28_cap, (unsigned long)mb);
    }
    debug_print(14, FG_YELLOW, "[Advanced]");
    debug_print(15, FG_WHITE, "DMA Support: %04X  PIO Support: %04X", id[49], id[64]);
    debug_print(16, FG_WHITE, "ATA Major Ver: %04X", id[80]);
}

static void run_debug_taskfile(void) {
    debug_cls();
    uint8_t tf[8];
    ide_read_taskfile(tf);
    const char *sc = (tf[7] & 0x81) ? "\033[91;1m" : "\033[92m";
    debug_print(0, FG_RED, "[Task File] " FG_WHITE "ERR:%02X SEC:%02X SN:%02X CL:%02X CH:%02X DH:%02X ST:%s%02X" RESET,
                tf[1], tf[2], tf[3], tf[4], tf[5], tf[6], sc, tf[7]);
}

static void run_debug_errors(void) {
    debug_cls();
    uint8_t tf[8];
    ide_read_taskfile(tf);
    uint8_t err = tf[1];
    char eb[64] = {0};
    if (!err) snprintf(eb, sizeof(eb), "\033[92mNo Errors Reported");  // no reset: keeps the box black
    else {
        if (err&0x80) strcat(eb,"BBK "); if (err&0x40) strcat(eb,"UNC ");
        if (err&0x20) strcat(eb,"MC ");  if (err&0x10) strcat(eb,"IDNF ");
        if (err&0x08) strcat(eb,"MCR "); if (err&0x04) strcat(eb,"ABRT ");
        if (err&0x02) strcat(eb,"TK0 "); if (err&0x01) strcat(eb,"AMNF ");
    }
    debug_print(0, FG_RED, "[Error Bits] %s", eb);

    // Registers saved by the last failed sector read/write, before any drain
    // or reset (the live ones above may since have been changed by a reset).
    ide_fail_t f;
    ide_last_failure(&f);
    if (f.kind == IDE_FAIL_NONE) {
        debug_print(2, FG_WHITE, "[Last Failed I/O] none since power-up");
    } else {
        static const char *kinds[] = {"?", "not ready", "ERR", "timeout", "stale data", "no geometry",
                                      "no time", "bad end"};
        if (f.kind == IDE_FAIL_MANUAL_CHS)      // Ctrl+G: no sector was asked for
            debug_print(2, FG_YELLOW, "[Last Failed I/O] cmd %02X, manual CHS: %s", f.command,
                        f.command == 0x91 ? "drive refused geometry" : "RECALIBRATE did not end");
        else
            debug_print(2, FG_YELLOW, "[Last Failed I/O] cmd %02X %s at LBA %lu, %lu of %lu done",
                        f.command, kinds[f.kind < 8 ? f.kind : 0], (unsigned long)f.lba,
                        (unsigned long)f.done, (unsigned long)f.count);
        // "HW reset": the soft reset did not bring the drive back, so RESET-
        // was used as well (both devices on the cable). The longest row,
        // drained and "HW reset FAILED", is exactly the 68 columns there are.
        const char *rs = !f.reset ? "" : f.hw_reset ? (f.reset_failed ? "  HW reset FAILED" : "  HW reset")
                                                    : (f.reset_failed ? "  reset FAILED" : "  reset");
        debug_print(3, FG_WHITE, "ST:%02X ERR:%02X SC:%02X SN:%02X CL:%02X CH:%02X DH:%02X%s%s",
                    f.status, f.error, f.tf[0], f.tf[1], f.tf[2], f.tf[3], f.tf[4],
                    f.drained ? "  drained" : "", rs);
        // The drive was not back when the USB command's time ran out (review
        // M-1): the next USB command carries on with the reset first.
        if (f.pending)
            debug_print(4, FG_YELLOW, "Reset still running: the next USB command waits for it");
    }
}

static void run_seek_test(void) {
    debug_cls();
    debug_print(0, FG_YELLOW, "Seek Test Running...");

    uint16_t id[256];
    if (!ide_identify(id)) { debug_print(1, FG_RED, "ERROR: Drive not ready."); sleep_ms(1500); return; }

    bool lba = (id[49] & 0x0200);
    uint32_t max_range = lba ? (id[60]|((uint32_t)id[61]<<16)) : id[1];
    if (!max_range) max_range = 1024;

    double x = 0;
    while (true) {
        if (cdc_getchar_timeout_us(0) == 27) break;
        double pos = (sin(x)*0.45)+0.5;
        uint32_t target = (uint32_t)(pos*(max_range-1));

        uint8_t st = ide_seek_read_one(target, lba);

        char bar[61]; memset(bar, '-', 60); bar[(int)(pos*59)] = '#'; bar[60] = '\0';
        debug_print(14, FG_GREEN, "[%s]", bar);
        debug_print(15, FG_WHITE, "Target %s: %lu (ST:%02X)", lba?"LBA":"CYL", (unsigned long)target, st);
        x += 0.12;
    }
    debug_print(14, FG_WHITE, "%*s", 60, ""); debug_print(15, FG_WHITE, "%*s", 60, "");
    sleep_ms(50);
}

// ---------------------------------------------------------------------------
//  Core 1 and a USB command still running (review L-5, L-1)
// ---------------------------------------------------------------------------
// Unmounting stops new USB commands from reaching the drive, but one that
// started before the unmount can still be running on core 0 (usb.c,
// usb_msc_ide_busy), for up to the host command's budget (IDE_HOST_BUDGET_MS,
// ide.h) and a small margin. Auto Detect and the debug commands drive the bus
// from core 1, and two cores on the bus at once can corrupt either one's
// command. So they wait for it first, up to IDE_BUS_WAIT_MS.
// Review L-1: while core 0 is inside that callback it does not run cdc_task(),
// which carries console bytes both ways. A message drawn during the wait only
// reaches the screen once the wait is over, and a key pressed during it only
// reaches core 1 then too. The first 0.6f3p7 draft printed "Waiting ... (Esc:
// cancel)" and read Esc meanwhile; neither could work. So the wait is silent
// and cannot be cancelled, and because it could not be, what was asked for is
// NOT then done: the screen says a USB command was running and nothing was
// sent, and the key can simply be pressed again (by then the drive may also
// have been reset by that command, which is worth a look first). If the
// command is somehow still running after IDE_BUS_WAIT_MS, it says that.
// Once the flag reads clear with nothing mounted it stays clear for bus
// purposes: a callback that starts later sees "not mounted" and never touches
// the drive (usb.c).
#define IDE_BUS_WAIT_MS (IDE_HOST_BUDGET_MS + 5000u)

// True if no USB command was using the bus: go ahead. False: one was (or
// still is), nothing may be sent, and the screen now says so. on_debug: where
// (the debug box, or a confirm box that waits for a key).
static bool bus_free_wait(bool on_debug) {
    if (!usb_msc_ide_busy()) return true;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (usb_msc_ide_busy() && to_ms_since_boot(get_absolute_time()) - t0 < IDE_BUS_WAIT_MS)
        sleep_ms(10);
    bool still = usb_msc_ide_busy();
    const char *no = still ? "USB still busy, nothing sent to the drive. Press a key"
                           : on_debug ? "A USB command was running, nothing sent. Try again"
                                      : "A USB command was running, nothing sent. Press a key";
    if (on_debug) debug_print(0, FG_RED, "%s", no); else draw_confirm_box(no);
    if (!on_debug) while (get_input() == -1) tight_loop_contents();
    return false;
}

// Features, IORDY: Enter switches the setting. Review of 0.6f3p7 (LOW): this
// used to put it on the pin at once, which undid ide.c holding IORDY ignored
// while a drive is still coming back from a hardware reset (a drive in its
// power-on diagnostics holds IORDY low, and a register read that believes it
// would never end). ide.c now decides when it reaches the pin
// (ide_iordy_follow_config): at once if nothing is mounted and no USB command
// is running, else at the next USB command, on core 0; in both cases not
// before the drive is back if IORDY is held.
static void features_toggle_iordy(void) {
    config.iordy_enabled = !config.iordy_enabled;
    if (!is_mounted && !usb_msc_ide_busy()) ide_iordy_follow_config();
}

// Debug screen keys other than Esc. Every one of them reads or drives the
// bus, so each waits for a running USB command first (bus_free_wait).
static void debug_key(int k) {
    bool known = k == 'i' || k == 'I' || k == 't' || k == 'T' || k == 'e' || k == 'E' ||
                 k == 's' || k == 'S' || k == 'r' || k == 'R';
    if (!known || !bus_free_wait(true)) return;
    if (k == 'i' || k == 'I') run_debug_identify();
    else if (k == 't' || k == 'T') run_debug_taskfile();
    else if (k == 'e' || k == 'E') run_debug_errors();
    else if (k == 's' || k == 'S') { run_seek_test(); current_screen = SCREEN_DEBUG; needs_full_redraw = true; }
    else {
        debug_cls();
        debug_print(0, FG_YELLOW, "Resetting drive...");
        ide_reset_drive();
        bool rdy = ide_wait_until_ready(5000);
        debug_print(1, rdy ? FG_GREEN : FG_RED, rdy ? "Drive ready." : "Drive not responding.");
    }
}

// ---------------------------------------------------------------------------
//  Sync local state to/from config_t
// ---------------------------------------------------------------------------

static void sync_from_config(void) {
    cur_cyls = config.cyls; cur_heads = config.heads; cur_spt = config.spt;
    use_lba_mode = config.use_lba_mode; total_lba_sectors = config.lba_sectors;
    ide_select_device(config.dev_base);
}

static void sync_to_config(void) {
    config.cyls = cur_cyls; config.heads = cur_heads; config.spt = cur_spt;
    config.use_lba_mode = use_lba_mode; config.lba_sectors = total_lba_sectors;
}

// ---------------------------------------------------------------------------
//  Auto-mount — runs on core 1 so IDE ops never block USB on core 0
// ---------------------------------------------------------------------------

// It resets the drive and sends IDENTIFY whenever a geometry is saved, so a
// Ctrl+G geometry is not saved with Auto Mount on (save_setup, review of
// 0.6f3p8 L-1).
static void try_auto_mount(void) {
    if (!config.auto_mount) return;

    bool has_geo = (config.use_lba_mode && config.lba_sectors > 0) ||
                   (!config.use_lba_mode && config.cyls > 0 && config.heads > 0 && config.spt > 0);
    if (!has_geo) return;

    // Wait for drive to spin up
    sleep_ms(5000);

    ide_select_device(config.dev_base);
    ide_reset_drive();

    // Poll drive ready
    uint32_t start = to_ms_since_boot(get_absolute_time());
    bool ready = false;
    while (to_ms_since_boot(get_absolute_time()) - start < 5000) {
        uint8_t st = ide_read_reg(7);
        if (!(st & 0x80) && (st & 0x40)) { ready = true; break; }
        sleep_ms(10);
    }
    if (!ready) return;

    uint16_t id_buf[256];
    if (!ide_identify(id_buf)) return;
    id_w49 = id_buf[49]; id_w49_known = true;   // IORDY advisory

    // Fill model string for display
    for (int i = 0; i < 20; i++) {
        uint16_t val = id_buf[27+i];
        hdd_model_raw[i*2] = (char)(val>>8); hdd_model_raw[i*2+1] = (char)(val&0xFF);
    }
    hdd_model_raw[40] = '\0'; sanitize_identify_model(hdd_model_raw);

    if (!config.use_lba_mode)
        ide_set_geometry(config.heads, config.spt);

    is_mounted = true;
    media_changed_waiting = true;
}

// ---------------------------------------------------------------------------
//  Firmware update mode (decisions in fwupdate.h)
// ---------------------------------------------------------------------------

// Ctrl+F on the main menu with nothing mounted opens the prompt. True if it did.
static bool fwupdate_key(int k) {
    if (!fwupdate_key_opens_prompt(current_screen == SCREEN_MAIN, is_mounted, k)) return false;
    current_screen = SCREEN_CONFIRM;
    confirm_type = 5;
    return true;
}

// Unmount. Also forgets the drive's captured IDENTIFY words: with the drive
// released, it can be swapped on the cable, and a later mount without a new
// detection must not let the SAT policy judge the new drive by the old one's
// IDENTIFY (a donor's SMART support would then admit SMART on a specimen).
static void menu_unmount(void) {
    is_mounted = false;
    media_changed_waiting = true;
#if ATABOY_SAT
    ide_id_words_forget();          // only SAT builds keep the words
#endif
}

// Y at the prompt: reboot into the RP2350 ROM bootloader (USB drive and
// picotool, the mode BOOTSEL gives), or come back with a message saying why
// not. A USB command that started before the drive was unmounted may still
// be running on core 0; wait for it, up to FWUPDATE_WAIT_MS, Esc cancels.
static void fwupdate_confirmed(void) {
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    bool told = false;
    for (;;) {
        uint32_t waited = to_ms_since_boot(get_absolute_time()) - t0;
        fwupdate_step_t step = fwupdate_step(is_mounted, usb_msc_ide_busy(), waited);
        if (step == FWUPDATE_GO) {
            cdc_puts("\033[2J\033[1;1H" RESET "Rebooting into firmware update mode...\r\n");
            sleep_ms(300);                  // let core 0 send it before the reset
            reset_usb_boot(0, 0);           // does not return
            return;
        }
        if (step == FWUPDATE_WAIT) {
            // No "Esc: cancel" here (review L-1): while the command runs on
            // core 0 neither this box nor a key gets through. An Esc typed
            // before it started is still honoured.
            if (!told) { draw_confirm_box("Waiting for a USB command to finish"); told = true; }
            if (cdc_getchar_timeout_us(20000) == KEY_ESC) return;
            continue;
        }
        draw_confirm_box(step == FWUPDATE_REFUSE_MOUNTED ? "Drive mounted, update not started. Press a key"
                                                         : "USB still busy, update not started. Press a key");
        while (get_input() == -1) tight_loop_contents();
        return;
    }
}

// Auto Detect (main menu, Enter on the first item). Waits first for a USB
// command that is still running on core 0 (bus_free_wait, review L-5).
// True when the result box should be drawn (core1_entry's trigger_overlay).
static bool run_auto_detect(void) {
    if (!bus_free_wait(false)) { needs_full_redraw = true; return false; }
    bool overlay = false;
    // Auto Detect: single reset, probe master then slave
    uint16_t id_buf[256];
    bool detected = false;
    uint8_t found = ide_probe_devices();
    // Review of 0.6f3p8 (M-1): the probe's RESET- has dropped whatever
    // geometry the drive had, and the probe may have found another device
    // than the one it was set for (a Ctrl+G geometry for the slave, say, with
    // a master now found first). So the old geometry is cleared here, not
    // kept: leaving the picker with Esc (or the forced entry after a failed
    // IDENTIFY) sets nothing, and there is nothing to mount until a geometry
    // is chosen. Kept, it would only have been sent again as 0x91 at the
    // first CHS transfer (ide.c marks it lost), possibly to the wrong drive.
    // The EEPROM is not touched; F10 still saves only what is set.
    cur_cyls = 0; cur_heads = 0; cur_spt = 0; use_lba_mode = false; total_lba_sectors = 0;
    sync_to_config();
    id_w49_known = false;           // IORDY advisory: only this detection's IDENTIFY counts
    if (found) {
        ide_select_device(found);
        config.dev_base = found;
        if (ide_identify(id_buf)) detected = true;
    }
    if (detected) { id_w49 = id_buf[49]; id_w49_known = true; }
    if (!detected) ide_select_device(config.dev_base);
    if (detected) {
            for (int i = 0; i < 20; i++) {
                uint16_t val = id_buf[27+i];
                hdd_model_raw[i*2] = (char)(val>>8); hdd_model_raw[i*2+1] = (char)(val&0xFF);
            }
            hdd_model_raw[40] = '\0'; sanitize_identify_model(hdd_model_raw);

            bool ls = (id_buf[49] & 0x0200);
            int geo_idx = ls ? 2 : 0;
            bool waiting = true, sd = true;

            while (waiting) {
                if (sd) { draw_selection_menu(id_buf, geo_idx); sd = false; }
                int ch = get_input();
                if (ch == -1) { tight_loop_contents(); continue; }

                if (ch == KEY_UP && geo_idx > 0) { geo_idx--; if (!ls && geo_idx == 2) geo_idx--; sd = true; }
                else if (ch == KEY_DOWN && geo_idx < 3) { geo_idx++; if (!ls && geo_idx == 2) geo_idx++; sd = true; }
                else if (ch == KEY_ESC) { waiting = false; }
                else if (ch == '\t' || (ch == KEY_ENTER && geo_idx == 3)) {
                    geo_idx = 3; draw_selection_menu(id_buf, geo_idx);
                    int mf[3] = {detect_cyls, detect_heads, detect_spt};
                    int mc[3] = {9+26, 9+38, 9+48};
                    bool manual_aborted = false;
                    for (int f = 0; f < 3; f++) {
                        char ib[7] = {0}; int p = 0;
                        draw_at(mc[f], 14, "\033[103;30m     \033[0m");
                        cdc_printf("\033[%d;%dH", 14, mc[f]);
                        while (true) {
                            int c = get_input();
                            if (c >= '0' && c <= '9' && p < 5) { ib[p++] = (char)c; cdc_putchar(c); }
                            else if ((c == 8 || c == 127) && p > 0) { p--; cdc_printf("\b \b\033[%d;%dH", 14, mc[f]+p); }
                            else if (c == KEY_ENTER || c == '\t') { ib[p] = '\0'; if (p > 0) mf[f] = atoi(ib); draw_at(mc[f], 14, RESET "\033[41;37;1m"); cdc_printf("%-5d", mf[f]); break; }
                            else if (c == KEY_ESC) { manual_aborted = true; f = 3; break; }
                            tight_loop_contents();
                        }
                    }
                    // Esc used to fall through to here and APPLY geometry (0x91)
                    // from whatever mf[] held. An abort must apply nothing.
                    if (manual_aborted) { sd = true; }
                    else {
                    detect_cyls = (uint16_t)mf[0]; detect_heads = (uint8_t)mf[1]; detect_spt = (uint8_t)mf[2];
                    if (detect_cyls > 0 && detect_heads > 0 && detect_spt > 0) {
                        use_lba_mode = false;
                        cur_cyls = detect_cyls; cur_heads = detect_heads; cur_spt = detect_spt;
                        ide_set_geometry(cur_heads, cur_spt);
                        sync_to_config(); waiting = false;
                    } else { sd = true; }
                    }   /* !manual_aborted */
                } else if (ch == KEY_ENTER) {
                    bool valid = false;
                    if (geo_idx == 0) { use_lba_mode = false; cur_cyls = id_buf[1]; cur_heads = (uint8_t)id_buf[3]; cur_spt = (uint8_t)id_buf[6]; valid = true; }
                    else if (geo_idx == 1) { use_lba_mode = false; get_large_geometry(id_buf[1],(uint8_t)id_buf[3],(uint8_t)id_buf[6],&cur_cyls,&cur_heads); cur_spt = (uint8_t)id_buf[6]; valid = true; }
                    else if (geo_idx == 2 && ls) {
                        use_lba_mode = true; cur_cyls = id_buf[1]; cur_heads = (uint8_t)id_buf[3]; cur_spt = (uint8_t)id_buf[6];
                        uint64_t l48 = ((uint64_t)id_buf[103]<<48)|((uint64_t)id_buf[102]<<32)|((uint64_t)id_buf[101]<<16)|((uint64_t)id_buf[100]);
                        total_lba_sectors = l48; if (!total_lba_sectors) total_lba_sectors = id_buf[60]|((uint32_t)id_buf[61]<<16);
                        valid = true;
                    }

                    if (valid) { if (!use_lba_mode) ide_set_geometry(cur_heads, cur_spt); sync_to_config(); waiting = false; }
                }
            }
    } else {
        if (found) {
            strcpy(hdd_status_text, "\033[91;1mIDENTIFY failed");
            force_detect = true;
        } else {
            strcpy(hdd_status_text, "\033[91;1mNo drive detected");
            force_detect = false;
        }
        show_detect_result = true; overlay = true; hdd_model_raw[0] = '\0';
    }
    needs_full_redraw = true;
    return overlay;
}

// ---------------------------------------------------------------------------
//  Manual CHS with no IDENTIFY (main menu, Ctrl+G; decisions in manualchs.h)
// ---------------------------------------------------------------------------
// The operator types cylinders, heads and sectors per track into a box of its
// own. Nothing reaches the drive until the entry is in range and Y is
// pressed; Esc at any point before that leaves everything as it was. Then:
// wait for a USB command still running (bus_free_wait), check again that no
// reset is pending, and ide_manual_chs() sends RESET-, RECALIBRATE and 0x91
// (ide.h), never IDENTIFY. Only a 0x91 the drive accepted sets the geometry;
// otherwise the error box says why, the geometry is 0 (nothing to mount),
// and Debug E has the registers. Nothing here saves to EEPROM (F10 still can,
// on purpose, later, but not with Auto Mount on: save_setup).
#define MCHS_COL   9                    // the box: 64 wide, as the picker
#define MCHS_ROW   5
#define MCHS_W     64
#define MCHS_H     11
#define MCHS_TITLE "Manual CHS (no IDENTIFY)"
#define MCHS_MODEL "Manual CHS (no IDENTIFY)"   // Current HDD once the drive took it
static const int mchs_field_col[3] = {18, 34, 50};

// One line of the box's interior (62 columns), text centred, in `color`.
static void mchs_line(int row, const char *color, const char *text) {
    int len = visible_strlen(text);
    if (len > MCHS_W - 2) len = MCHS_W - 2;
    int pl = (MCHS_W - 2 - len) / 2, pr = MCHS_W - 2 - len - pl;
    cdc_printf(SEL_RED "\033[%d;%dH" BOX_VH "%*s%s%s" SEL_RED "%*s" BOX_VH, row, MCHS_COL, pl, "", color, text, pr, "");
}

// fields: what was typed (digits, "" when empty). active: the field being
// typed into, or -1 (nothing is being typed). msg: the message row, "" for
// none; ask draws it as the Y/N question.
static void draw_manual_chs(char fields[3][6], int active, const char *msg, bool ask) {
    int c = MCHS_COL, r = MCHS_ROW;
    for (int i = 0; i < MCHS_H; i++) cdc_printf("\033[%d;%dH\033[40m%*s", r+1+i, c+2, MCHS_W, "");
    cdc_puts(SEL_RED);
    cdc_printf("\033[%d;%dH", r, c);
    cdc_puts(BOX_TL); emit_n(BOX_HH, MCHS_W - 2); cdc_puts(BOX_TR);
    mchs_line(r+1, SEL_RED, MCHS_TITLE);
    char who[64];
    snprintf(who, sizeof(who), "%s drive. Nothing is sent to it until you press Y.",
             config.dev_base == 0xB0 ? "Slave" : "Master");
    mchs_line(r+2, SEL_RED, who);
    cdc_printf("\033[%d;%dH", r+3, c);
    cdc_puts(BOX_MLD); emit_n(BOX_HH, MCHS_W - 2); cdc_puts(BOX_MRD);
    for (int i = 4; i <= 6; i++) mchs_line(r+i, SEL_RED, "");
    static const char *labels[3] = {"CYLS", "HEADS", "SPT"};
    static const char *ranges[3] = {"1-65535", "1-16", "1-255"};
    for (int f = 0; f < 3; f++) {
        cdc_printf(SEL_RED "\033[%d;%dH%s", r+4, mchs_field_col[f], labels[f]);
        cdc_printf("\033[%d;%dH%s%-5s" SEL_RED, r+5, mchs_field_col[f],
                   f == active ? "\033[103;30m" : "\033[40;37;1m", fields[f]);
        cdc_printf(FG_YELLOW "\033[%d;%dH%s" SEL_RED, r+6, mchs_field_col[f], ranges[f]);
    }
    mchs_line(r+7, ask ? "\033[103;30m" : FG_YELLOW, msg);
    cdc_printf("\033[%d;%dH", r+8, c);
    cdc_puts(BOX_MLD); emit_n(BOX_HH, MCHS_W - 2); cdc_puts(BOX_MRD);
    mchs_line(r+9, SEL_RED, "0-9: Digits   Enter/Tab: Next field   Esc: Cancel");
    cdc_printf("\033[%d;%dH", r+10, c);
    cdc_puts(BOX_BL); emit_n(BOX_HH, MCHS_W - 2); cdc_puts(BOX_BR);
    cdc_puts(RESET);
    if (active >= 0) cdc_printf("\033[%d;%dH", r+5, mchs_field_col[active] + (int)strlen(fields[active]));
    cdc_flush();
}

// Ctrl+G on the main menu with nothing mounted opens the entry. True if it did.
static bool manual_chs_key(int k) {
    return manual_chs_key_opens(current_screen == SCREEN_MAIN, is_mounted, k);
}

// A reset a USB command started has not finished (ide.c, recovery): the next
// USB command carries on with it, so nothing else may reset the drive now.
// Debug R or Auto Detect end it, and so does the drive answering the next
// USB command once mounted. True: refused, and the screen said so.
static bool mchs_pending_refused(void) {
    if (!ide_recovery_pending()) return false;
    draw_confirm_box("Reset still pending, nothing sent. Press a key");
    while (get_input() == -1) tight_loop_contents();
    return true;
}

// The entry, from Ctrl+G to the drive's answer. True when the result box
// should be drawn (core1_entry's trigger_overlay), as run_auto_detect.
static bool run_manual_chs(void) {
    needs_full_redraw = true;
    if (mchs_pending_refused()) return false;
    char fields[3][6] = {"", "", ""};
    int active = 0;
    const char *msg = "";
    uint32_t v[3] = {0, 0, 0};
    for (;;) {
        draw_manual_chs(fields, active, msg, false);
        int ch;
        while ((ch = get_input()) == -1) tight_loop_contents();
        int p = (int)strlen(fields[active]);
        if (ch == KEY_ESC) return false;                      // nothing sent, nothing changed
        if (ch >= '0' && ch <= '9') { if (p < 5) { fields[active][p] = (char)ch; fields[active][p+1] = '\0'; } continue; }
        if ((ch == 8 || ch == 127) && p > 0) { fields[active][p-1] = '\0'; continue; }
        if (ch != KEY_ENTER && ch != '\t') continue;
        if (active < 2) { active++; continue; }
        // Enter on the last field: all three are checked before anything else.
        for (int f = 0; f < 3; f++) v[f] = (uint32_t)strtoul(fields[f], NULL, 10);
        int bad = manual_chs_bad_field(v[0], v[1], v[2]);
        if (bad >= 0) { msg = manual_chs_refusal(bad); active = bad; continue; }
        char q[64];
        snprintf(q, sizeof(q), "Y: reset %s, send %lu/%lu/%lu (91h)   N: edit",
                 config.dev_base == 0xB0 ? "Slave" : "Master",
                 (unsigned long)v[0], (unsigned long)v[1], (unsigned long)v[2]);
        draw_manual_chs(fields, -1, q, true);
        int k;
        do { while ((k = get_input()) == -1) tight_loop_contents(); }
        while (k != 'y' && k != 'Y' && k != 'n' && k != 'N' && k != KEY_ESC);
        if (k == KEY_ESC) return false;
        if (k == 'n' || k == 'N') { msg = ""; continue; }
        break;
    }
    // Y. Nothing has been sent yet. A USB command still running, or a reset
    // it has left pending, stops it here too.
    if (!bus_free_wait(false) || mchs_pending_refused()) return false;
    draw_manual_chs(fields, -1, "Resetting the drive, then 10h and 91h. Up to a minute.", false);
    id_w49_known = false;           // no IDENTIFY: nothing to say about IORDY
    uint8_t st = 0;
    int r = ide_manual_chs((uint8_t)v[1], (uint8_t)v[2], &st);
    if (r == IDE_MCHS_OK) {
        use_lba_mode = false; total_lba_sectors = 0;
        cur_cyls = (uint16_t)v[0]; cur_heads = (uint8_t)v[1]; cur_spt = (uint8_t)v[2];
        strcpy(hdd_model_raw, MCHS_MODEL);
        sync_to_config();
        return false;
    }
    // The drive was reset, so whatever geometry it had is gone: nothing is
    // shown as set, and there is nothing to mount.
    const char *why = r == IDE_MCHS_REFUSED  ? "drive refused geometry" :
                      r == IDE_MCHS_RECAL    ? "RECALIBRATE did not end" :
                      r == IDE_MCHS_NO_DEVICE ? "no drive answered" :
                      r == IDE_MCHS_BUSY     ? "drive still busy" : "nothing sent";
    snprintf(hdd_status_text, sizeof(hdd_status_text), "\033[91;1mManual CHS: %s (ST:%02X)", why, st);
    hdd_model_raw[0] = '\0';
    cur_cyls = 0; cur_heads = 0; cur_spt = 0; use_lba_mode = false; total_lba_sectors = 0;
    sync_to_config();
    force_detect = false;
    show_detect_result = true;
    return true;
}

// Y at "Save Current Setup to EEPROM" (F10, or the main menu item). Review of
// 0.6f3p8 (L-1): with Auto Mount on, every power-up resets the drive and
// sends it IDENTIFY before anything else (try_auto_mount), whenever a
// geometry is saved. A geometry from Ctrl+G is for a drive that must never
// get IDENTIFY, so while it is the one in use (ide_manual_chs_active) it is
// not saved with Auto Mount on: the box says so and nothing is written. With
// Auto Mount off it is saved as before. The EEPROM layout is unchanged, so a
// saved geometry does not say where it came from: after a power-up, Auto
// Mount switched on and F10 again will save it (FORK-README item 22).
// True if saved.
#define SAVE_REFUSED_MCHS "Not saved: Auto Mount would IDENTIFY. Press a key"
static bool save_setup(void) {
    if (config.auto_mount && ide_manual_chs_active()) {
        draw_confirm_box(SAVE_REFUSED_MCHS);
        while (get_input() == -1) tight_loop_contents();
        return false;
    }
    sync_to_config();
    config_save();
    return true;
}


// ---------------------------------------------------------------------------
//  Core 1 entry point
// ---------------------------------------------------------------------------

void core1_entry(void) {
    sync_from_config();
    try_auto_mount();
    bool trigger_overlay = false;

    // If auto-mount succeeded, start on the mounted screen
    if (is_mounted) {
        current_screen = SCREEN_MOUNTED;
        trigger_overlay = true;
        needs_full_redraw = true;
    }

    while (true) {
        bool connected = cdc_connected;
        if (connected && !last_cdc_connected) { sleep_ms(200); needs_full_redraw = true; }
        last_cdc_connected = connected;
        if (!connected) { sleep_ms(100); continue; }

        if (needs_full_redraw) {
            draw_bios_frame();
            if (current_screen == SCREEN_MAIN || current_screen == SCREEN_CONFIRM || current_screen == SCREEN_MOUNTED)
                update_main_menu();
            else if (current_screen == SCREEN_FEATURES) update_features_menu();
            else if (current_screen == SCREEN_DEBUG) draw_debug_overlay();

            if (show_detect_result || current_screen == SCREEN_CONFIRM ||
                current_screen == SCREEN_MOUNTED || current_screen == SCREEN_DEBUG)
                trigger_overlay = true;
            needs_full_redraw = false;
        }

        if (!show_detect_result && current_screen != SCREEN_CONFIRM &&
            current_screen != SCREEN_MOUNTED && current_screen != SCREEN_DEBUG) {
            if (current_screen == SCREEN_MAIN) update_main_menu();
            else if (current_screen == SCREEN_FEATURES) update_features_menu();
        }

        if (show_detect_result) {
            if (trigger_overlay) { draw_error_box(hdd_status_text, force_detect); trigger_overlay = false; }
        } else if (current_screen == SCREEN_CONFIRM) {
            if (trigger_overlay) {
                if (confirm_type == 0)      draw_confirm_box("Load Defaults and Save to EEPROM (Y/N)?");
                else if (confirm_type == 1) draw_confirm_box("Save Current Setup to EEPROM (Y/N)?");
                else if (confirm_type == 3) draw_confirm_box("Are you sure you want to mount the drive (Y/N)?");
                else if (confirm_type == 4) draw_confirm_box("Are you sure you want to unmount (Y/N)?");
                else if (confirm_type == 5) draw_confirm_box("Enter firmware update mode (Y/N)?");
                trigger_overlay = false;
            }
        } else if (current_screen == SCREEN_MOUNTED) {
            if (trigger_overlay) {
                draw_confirm_box("Drive mounted!  Press 'U' to Unmount");
                trigger_overlay = false;
            }
        } else if (current_screen == SCREEN_DEBUG) {
            if (trigger_overlay) { draw_debug_overlay(); trigger_overlay = false; }
        }

        cdc_puts("\033[24;79H"); cdc_flush();
        int k = get_input();
        if (k == -1) { tight_loop_contents(); continue; }

        // Ctrl+L: redraw the whole screen, e.g. after a terminal resize (issue #9)
        if (k == 12) { needs_full_redraw = true; continue; }

        if (show_detect_result) {
            if (k == KEY_ENTER || k == KEY_ESC) {
                show_detect_result = false; hdd_status_text[0] = '\0'; hdd_model_raw[0] = '\0';
                cur_cyls = 0; cur_heads = 0; cur_spt = 0;
                total_lba_sectors = 0; use_lba_mode = false; is_mounted = false;
                media_changed_waiting = true;   // as every other unmount path does
                sync_to_config(); needs_full_redraw = true;
            } else if (force_detect && (k == 'f' || k == 'F')) {
                show_detect_result = false; hdd_status_text[0] = '\0';
                force_detect = false;
                strcpy(hdd_model_raw, "Manually Forced Drive");

                // Force geometry — manual entry only, no IDENTIFY data
                uint16_t dummy_id[256];
                memset(dummy_id, 0, sizeof(dummy_id));
                int geo_idx = 3;
                bool waiting = true, sd = true;

                while (waiting) {
                    if (sd) { draw_selection_menu_ex(dummy_id, geo_idx, true); sd = false; }
                    int ch = get_input();
                    if (ch == -1) { tight_loop_contents(); continue; }
                    if (ch == KEY_ESC) { waiting = false; }
                    else if (ch == '\t' || ch == KEY_ENTER) {
                        geo_idx = 3; draw_selection_menu_ex(dummy_id, geo_idx, true);
                        int mf[3] = {detect_cyls, detect_heads, detect_spt};
                        int mc[3] = {9+26, 9+38, 9+48};
                        bool f_aborted = false;
                        for (int f = 0; f < 3; f++) {
                            char ib[7] = {0}; int p = 0;
                            draw_at(mc[f], 14, "\033[103;30m     \033[0m");
                            cdc_printf("\033[%d;%dH", 14, mc[f]);
                            while (true) {
                                int c = get_input();
                                if (c >= '0' && c <= '9' && p < 5) { ib[p++] = (char)c; cdc_putchar(c); }
                                else if ((c == 8 || c == 127) && p > 0) { p--; cdc_printf("\b \b\033[%d;%dH", 14, mc[f]+p); }
                                else if (c == KEY_ENTER || c == '\t') { ib[p] = '\0'; if (p > 0) mf[f] = atoi(ib); draw_at(mc[f], 14, RESET "\033[41;37;1m"); cdc_printf("%-5d", mf[f]); break; }
                                else if (c == KEY_ESC) { f_aborted = true; f = 3; break; }
                                tight_loop_contents();
                            }
                        }
                        // Same defect as the picker copy: Esc must apply nothing.
                        if (f_aborted) { sd = true; }
                        else {
                        detect_cyls = (uint16_t)mf[0]; detect_heads = (uint8_t)mf[1]; detect_spt = (uint8_t)mf[2];
                        if (detect_cyls > 0 && detect_heads > 0 && detect_spt > 0) {
                            use_lba_mode = false;
                            cur_cyls = detect_cyls; cur_heads = detect_heads; cur_spt = detect_spt;
                            ide_set_geometry(cur_heads, cur_spt);
                            sync_to_config(); waiting = false;
                        } else { sd = true; }
                        }   /* !f_aborted */
                    }
                }
                needs_full_redraw = true;
            }
            continue;
        }

        if (current_screen == SCREEN_MOUNTED) {
            if (k == 'u' || k == 'U') { current_screen = SCREEN_CONFIRM; confirm_type = 4; trigger_overlay = true; needs_full_redraw = true; }
            continue;
        }

        if (current_screen == SCREEN_DEBUG) {
            if (k == KEY_ESC) { current_screen = SCREEN_FEATURES; needs_full_redraw = true; }
            else debug_key(k);
            continue;
        }

        if (current_screen == SCREEN_CONFIRM) {
            if (k == 'y' || k == 'Y') {
                if (confirm_type == 0) { config_defaults(); sync_from_config(); config_save(); current_screen = SCREEN_MAIN; }
                else if (confirm_type == 1) { (void)save_setup(); current_screen = confirm_return_screen; }
                else if (confirm_type == 3) { is_mounted = true; media_changed_waiting = true; current_screen = SCREEN_MOUNTED; }
                else if (confirm_type == 4) { menu_unmount(); current_screen = SCREEN_MAIN; }
                else if (confirm_type == 5) {
                    // Reboot into the ROM bootloader, so new firmware can be
                    // loaded without holding BOOTSEL. Mounted state and any
                    // USB command still running are checked again here.
                    fwupdate_confirmed();
                    current_screen = SCREEN_MAIN;
                }
                needs_full_redraw = true;
            } else if (k == 'n' || k == 'N' || k == KEY_ESC) {
                current_screen = (confirm_type == 1) ? confirm_return_screen :
                                 (confirm_type == 4) ? SCREEN_MOUNTED : SCREEN_MAIN;
                needs_full_redraw = true;
            }
            continue;
        }

        if (k == KEY_F10) { confirm_return_screen = current_screen; current_screen = SCREEN_CONFIRM; confirm_type = 1; trigger_overlay = true; needs_full_redraw = true; continue; }
        // Ctrl+F on the main menu, nothing mounted: ask before rebooting into
        // the ROM bootloader for a firmware update.
        if (fwupdate_key(k)) { trigger_overlay = true; needs_full_redraw = true; continue; }
        // Ctrl+G on the main menu, nothing mounted: manual CHS, no IDENTIFY.
        if (manual_chs_key(k)) { if (run_manual_chs()) trigger_overlay = true; continue; }

        if (current_screen == SCREEN_MAIN) {
            if (k == KEY_UP) {
                if (config.main_selected == 1 || config.main_selected == 3 || config.main_selected == 4) config.main_selected--;
            } else if (k == KEY_DOWN) {
                if (config.main_selected == 0 || config.main_selected == 2 || config.main_selected == 3) config.main_selected++;
            } else if (k == KEY_RIGHT) {
                if (config.main_selected == 0) config.main_selected = 2;
                else if (config.main_selected == 1) config.main_selected = 3;
            } else if (k == KEY_LEFT) {
                if (config.main_selected == 2) config.main_selected = 0;
                else if (config.main_selected == 3) config.main_selected = 1;
                else if (config.main_selected == 4) config.main_selected = 1;
            }
            else if (k == KEY_ENTER) {
                if (config.main_selected == 0) {
                    if (run_auto_detect()) trigger_overlay = true;
                } else if (config.main_selected == 1) {
                    bool dv = (hdd_model_raw[0] != '\0');
                    bool gv = (use_lba_mode && total_lba_sectors > 0) || (!use_lba_mode && cur_cyls > 0 && cur_heads > 0 && cur_spt > 0);
                    if (!dv || !gv) {
                        snprintf(hdd_status_text, sizeof(hdd_status_text), "\033[91;1mDetect drive and set geometry first!");
                        hdd_model_raw[0] = '\0'; cur_cyls = 0; cur_heads = 0; cur_spt = 0;
                        total_lba_sectors = 0; use_lba_mode = false; is_mounted = false;
                        show_detect_result = true; trigger_overlay = true; needs_full_redraw = true;
                    } else { current_screen = SCREEN_CONFIRM; confirm_type = 3; trigger_overlay = true; needs_full_redraw = true; }
                } else if (config.main_selected == 2) {
                    current_screen = SCREEN_FEATURES; config.feat_selected = 0; needs_full_redraw = true;
                } else if (config.main_selected == 3) { current_screen = SCREEN_CONFIRM; confirm_type = 0; trigger_overlay = true; needs_full_redraw = true; }
                else if (config.main_selected == 4) { current_screen = SCREEN_CONFIRM; confirm_type = 1; trigger_overlay = true; needs_full_redraw = true; }
            }
        } else if (current_screen == SCREEN_FEATURES) {
            if (k == KEY_UP && config.feat_selected > 0) config.feat_selected--;
            else if (k == KEY_DOWN && config.feat_selected < 4) config.feat_selected++;
            else if (k == KEY_ESC) current_screen = SCREEN_MAIN;
            else if (k == KEY_ENTER) {
                if (config.feat_selected == 0) config.drive_write_protected = !config.drive_write_protected;
                else if (config.feat_selected == 1) config.auto_mount = !config.auto_mount;
                else if (config.feat_selected == 2) features_toggle_iordy();
                else if (config.feat_selected == 3) config.intrq_enabled = !config.intrq_enabled;
                else if (config.feat_selected == 4) current_screen = SCREEN_DEBUG;
            }
            needs_full_redraw = true;
        }
    }
}
