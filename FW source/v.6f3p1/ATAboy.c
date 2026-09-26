// ATAboy Firmware — Main Entry Point
// Core 0: TinyUSB task loop + CDC queue bridge
// Core 1: Menu system (menus.c)
// NEVER call tud_task() from callbacks, IDE I/O, or core 1.

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "tusb.h"
#include "class/cdc/cdc_device.h"
#include "ide.h"
#include "obs.h"
#include "config.h"
#include "fault.h"

// Holistic review (the guide's L1): nothing may drive a board LED, UART,
// I2C, SPI or power-sense pin: on this board those pin numbers are the IDE
// bus (boards/ataboy_rev6.h).
#if defined(PICO_DEFAULT_LED_PIN) || defined(PICO_DEFAULT_WS2812_PIN) || defined(PICO_DEFAULT_UART_TX_PIN) ||     defined(PICO_DEFAULT_I2C_SDA_PIN) || defined(PICO_DEFAULT_SPI_SCK_PIN) || defined(PICO_SMPS_MODE_PIN) ||     defined(PICO_VBUS_PIN) || defined(PICO_VSYS_PIN)
#error "a board header with LED, bus or power pins: this board's pins are the IDE bus (boards/ataboy_rev6.h)"
#endif
#include "pico/util/queue.h"

// ---------------------------------------------------------------------------
//  Shared state (accessed by usb.c callbacks and menus.c)
// ---------------------------------------------------------------------------

volatile bool is_mounted = false;
volatile bool media_changed_waiting = false;
volatile bool cdc_connected = false;

// ---------------------------------------------------------------------------
//  CDC queues — core 1 writes TX, reads RX; core 0 drains TX, fills RX
// ---------------------------------------------------------------------------

queue_t cdc_tx_queue;
queue_t cdc_rx_queue;

#define CDC_TX_QUEUE_SIZE 4096
#define CDC_RX_QUEUE_SIZE 256

// ---------------------------------------------------------------------------
//  cdc_task() — called from core 0 main loop only
// ---------------------------------------------------------------------------

static void cdc_task(void) {
    cdc_connected = tud_cdc_connected();

    // Drain TX queue -> USB CDC
    if (cdc_connected) {
        uint32_t avail = tud_cdc_write_available();
        while (avail > 0) {
            char c;
            if (!queue_try_remove(&cdc_tx_queue, &c)) break;
            tud_cdc_write_char(c);
            avail--;
        }
        tud_cdc_write_flush();
    }

    // Fill RX queue <- USB CDC
    while (tud_cdc_available()) {
        int c = tud_cdc_read_char();
        if (c >= 0) {
            char ch = (char)c;
            queue_try_add(&cdc_rx_queue, &ch);  // drop if full
        }
    }
}

// ---------------------------------------------------------------------------
//  Core 1 entry (defined in menus.c)
// ---------------------------------------------------------------------------

extern void core1_entry(void);

// Holistic review L-4: core 1's stack. The SDK default was 2 KB and core 1's
// deepest path measured 2048 bytes (-fstack-usage: the menu loop, Auto
// Detect, IDENTIFY, a drain), so it lived off an unused array by accident.
// Now 8 KB of its own, with the stack limit (PICO_USE_STACK_GUARDS: MSPLIM
// at the bottom, a fault below it) and a canary just above the limit that
// core 1 checks on every pass of its loop (fault.h).
#define CORE1_STACK_WORDS  2048
#define CORE1_CANARY_WORDS 16
#define CORE1_CANARY       0xC0DE1C0Du
static uint32_t core1_stack[CORE1_STACK_WORDS] __attribute__((aligned(32)));

bool core1_stack_intact(void) {
    for (int i = 0; i < CORE1_CANARY_WORDS; i++)
        if (core1_stack[i] != CORE1_CANARY) return false;
    return true;
}

// ---------------------------------------------------------------------------
//  main — core 0 (USB only — no IDE operations here)
// ---------------------------------------------------------------------------

int main(void) {
    // No pico_stdio — all console I/O goes through TinyUSB CDC via queues
    config_load();
    obs_init();             // F1 stage 1: before the first bus event
    ide_hw_init();

    // Init CDC queues
    queue_init(&cdc_tx_queue, sizeof(char), CDC_TX_QUEUE_SIZE);
    queue_init(&cdc_rx_queue, sizeof(char), CDC_RX_QUEUE_SIZE);

    // Init TinyUSB
    tusb_init();

    // Allow core 1 to be paused for flash writes (config_save)
    multicore_lockout_victim_init();

    // Launch menu UI on core 1 (auto-mount runs there too), on its own stack
    for (int i = 0; i < CORE1_CANARY_WORDS; i++) core1_stack[i] = CORE1_CANARY;
    multicore_launch_core1_with_stack(core1_entry, core1_stack, sizeof core1_stack);

    // Core 0 loop: service USB + CDC bridge
    while (true) {
        tud_task();
        cdc_task();
    }
}
