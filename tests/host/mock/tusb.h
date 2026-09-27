// Host test stand-in for the Pico SDK / TinyUSB header of the same name.
#include "mock_pico.h"

// The ATAboy Control API's port, CDC instance 1 (ctl.c). Defined by the
// tests that build ctl.c (ctl_harness.hpp); declared here for the rest.
bool     tud_cdc_n_connected(uint8_t itf);
uint32_t tud_cdc_n_available(uint8_t itf);
uint32_t tud_cdc_n_read(uint8_t itf, void *buffer, uint32_t bufsize);
void     tud_cdc_n_read_flush(uint8_t itf);
uint32_t tud_cdc_n_write(uint8_t itf, void const *buffer, uint32_t bufsize);
uint32_t tud_cdc_n_write_flush(uint8_t itf);
uint32_t tud_cdc_n_write_available(uint8_t itf);
bool     tud_cdc_n_write_clear(uint8_t itf);
