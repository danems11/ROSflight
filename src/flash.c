#include <string.h>
#include <breezystm32/breezystm32.h>

#include "flash.h"

bool initEEPROM(uint16_t capacity)
{
  // make sure (at compile time) that config struct doesn't overflow allocated flash pages
//  ct_assert(capacity < CONFIG_SIZE);

  return capacity < CONFIG_SIZE;
}

bool readEEPROM(void *dst, uint16_t bytes)
{
  memcpy(dst, (char *)FLASH_WRITE_ADDR, bytes);
  return true;
}

bool writeEEPROM(void *src, uint16_t bytes)
{
  FLASH_Status status;

  // write it
  FLASH_Unlock();
  for (unsigned int tries = 3; tries; tries--)
  {
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    FLASH_ErasePage(FLASH_WRITE_ADDR);
    status = FLASH_ErasePage(FLASH_WRITE_ADDR + FLASH_PAGE_SIZE);
    for (unsigned int i = 0; i < bytes && status == FLASH_COMPLETE; i += 4)
      status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *)((char *)src + i));
    if (status == FLASH_COMPLETE)
      break;
  }
  FLASH_Lock();

  // Flash write failed - just die now
//  if (status != FLASH_COMPLETE || !validEEPROM())
  if (status != FLASH_COMPLETE) //! \todo data integrity check
  {
    return false;
  }

  return true;
}
