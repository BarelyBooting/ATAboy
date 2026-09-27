// Stand-in for the Pico SDK header, for test_usb_layout.cpp only: that test
// builds usb_descriptors.c against TinyUSB's real headers, and mock/'s
// stand-ins would clash with them.
#pragma once
#define PICO_UNIQUE_BOARD_ID_SIZE_BYTES 8
void pico_get_unique_board_id_string(char *id_out, unsigned len);
