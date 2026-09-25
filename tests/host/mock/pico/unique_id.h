// Host test stand-in for the Pico SDK header of the same name.
#pragma once
#include "mock_pico.h"
#define PICO_UNIQUE_BOARD_ID_SIZE_BYTES 8
// The board id the test chose, as 16 hex digits (test_usb_desc.cpp does it
// the way pico_unique_id's unique_id.c does).
void pico_get_unique_board_id_string(char *id_out, unsigned len);
