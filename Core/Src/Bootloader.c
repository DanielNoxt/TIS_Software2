/*
 * Bootloader.c
 *
 *  Created on: 11.10.24
 *      Author: M.Micka
 */


#include <Bootloader.h>
#include "string.h"
#include <FLASH.h>

void BOOT_JumpToBootloader(void)
{
  NVIC_SystemReset();
}
/**
  * @brief Sets delete flag in Constants flash space so DATA is deleted after update
  * @param None
  * @retval APP_ERROR_OK on success, APP_ERROR_INVARG on error
  */
AppError_t BOOT_SetDeleteFlag(uint32_t flag)
{
  CONSTANTS_TypeDef newData;
  AppError_t e_ret_status = APP_ERROR_INVARG;

  //Copy old data
  newData = *ConstantsData;

  //add new data
  newData.deleteFlag.delete_flag = flag;

  if(FLASH_eraseSector(CONSTANTS_SECTOR) == APP_ERROR_OK)
  {
    e_ret_status = FLASH_writeData((uint8_t *)ConstantsData, (uint8_t *)&newData, sizeof(CONSTANTS_TypeDef));
  }
  return e_ret_status;
}

/**
  * @brief Updates CanConfig in Constants flash space
  * @param None
  * @retval APP_ERROR_OK on success, APP_ERROR_INVARG on error
  */
AppError_t BOOT_UpdateConfig(CAN_Config_TypeDef newConfig)
{
  CONSTANTS_TypeDef newData;
  AppError_t e_ret_status = APP_ERROR_INVARG;

  //Copy old data
  newData = *ConstantsData;

  //add new data
  newData.canConfig = newConfig;

  if(FLASH_eraseSector(CONSTANTS_SECTOR) == APP_ERROR_OK)
  {
    e_ret_status = FLASH_writeData((uint8_t *)ConstantsData, (uint8_t *)&newData, sizeof(CONSTANTS_TypeDef));
  }
  return e_ret_status;
}

