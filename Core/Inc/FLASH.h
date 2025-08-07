/*
 * FLASH.h
 *
 *  Created on: Oct 14, 2024
 *      Author: Matej
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "AppErrorHandling.h"
#include "main.h"

AppError_t FLASH_eraseSector(uint8_t sector);
AppError_t FLASH_writeData(uint8_t *pDestination, const uint8_t *pSource, uint32_t Length);


#endif /* INC_FLASH_H_ */
