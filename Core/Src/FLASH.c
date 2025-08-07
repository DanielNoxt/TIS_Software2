/*
 * FLASH.c
 *
 *  Created on: Oct 14, 2024
 *      Author: Matej
 */
#include <stdint.h>
#include "FLASH.h"
#include "stm32f4xx_hal.h"

AppError_t FLASH_Clear_Error()
{
  AppError_t e_ret_status = APP_ERROR_INVARG;

  /* Unlock the Program memory */
  if (HAL_FLASH_Unlock() == HAL_OK)
  {
    /* Clear all FLASH flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR | FLASH_SR_RDERR);

    /* Lock the Program memory */
    if (HAL_FLASH_Lock() == HAL_OK)
    {
      e_ret_status = APP_ERROR_OK;
    }
  }

  return e_ret_status;
}

AppError_t FLASH_eraseSector(uint8_t sector)
{
  FLASH_EraseInitTypeDef Erase;
  AppError_t e_ret_status = APP_ERROR_INVARG;
  uint32_t SError = 0;

  //check if sector is valid
  if(sector != DATA_SECTOR && sector != CONSTANTS_SECTOR) return APP_ERROR_INVARG;

  Erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  Erase.Sector = sector;
  Erase.NbSectors = 1;
  Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  //clear previous errors
  e_ret_status = FLASH_Clear_Error();

  if(e_ret_status == APP_ERROR_OK)
  {
    /* Unlock the Program memory */
    if (HAL_FLASH_Unlock() == HAL_OK)
    {
      if(HAL_FLASHEx_Erase(&Erase, &SError) == HAL_OK) e_ret_status = APP_ERROR_OK; //set return to ok on success of erasing
    }
    /* Lock the Program memory */
    if (HAL_FLASH_Lock() != HAL_OK) e_ret_status = APP_ERROR_INVARG;
  }
  return e_ret_status;
}

AppError_t FLASH_writeData(uint8_t *pDestination, const uint8_t *pSource, uint32_t Length)
{
  AppError_t e_ret_status = APP_ERROR_INVARG;

  //clear previous errors
  e_ret_status = FLASH_Clear_Error();

  if(e_ret_status == APP_ERROR_OK)
  {
    /* Unlock the Program memory */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
      e_ret_status = APP_ERROR_INVARG;
    }

    //start writing byte by byte
    for (uint32_t i = 0; (i < Length) && e_ret_status == APP_ERROR_OK; i++)
    {
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, ((uint32_t) pDestination) + i, *(pSource + i)) == HAL_OK)
      {
	if (*(pDestination+ i) != *(pSource + i))
	{
	  e_ret_status = APP_ERROR_INVARG; /* Flash content doesn't match SRAM content */
	}
      }
    }
    /* Lock the Program memory */
    if (HAL_FLASH_Lock() != HAL_OK) e_ret_status = APP_ERROR_INVARG;
  }
  return e_ret_status;
}
