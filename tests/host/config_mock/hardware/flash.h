// Host test stand-in (test_config_save.cpp): the flash calls config_save()
// makes, recorded instead of done.
#pragma once
#include <stdint.h>
#include <stddef.h>
#define FLASH_PAGE_SIZE   256u
#define FLASH_SECTOR_SIZE 4096u
#define XIP_BASE          0x10000000u
void flash_range_erase(uint32_t offset, size_t count);
void flash_range_program(uint32_t offset, const uint8_t *data, size_t count);
