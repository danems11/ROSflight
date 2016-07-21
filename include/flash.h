#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }

// define this symbol to increase or decrease flash size. not rely on flash_size_register.
#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
// if sizeof(_params) is over this number, compile-time error will occur. so, need to add another page to config data.
#define CONFIG_SIZE                     (FLASH_PAGE_SIZE * 2)

//static uint32_t enabledSensors = 0;
//static void resetConf(void);
static const uint32_t FLASH_WRITE_ADDR = 0x08000000 + (FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - (CONFIG_SIZE / 1024)));


/**
 * @brief Initialize Flash
 * @param capacity the maximum amount of memory that will be needed
 */
bool initEEPROM(uint16_t capacity);

/**
 * @brief Read the _param struct from Flash
 * @param dst the destination memory address
 * @param bytes the number of bytes to read
 * @returns true if the read was successful, false otherwise
 */
bool readEEPROM(void *dst, uint16_t bytes);

/**
 * @brief write the _param struct to Flash
 * @param src the source memory address
 * @param bytes the number of bytes to write
 * @returns true if the write was successful, false otherwise
 */
bool writeEEPROM(void *src, uint16_t bytes);
