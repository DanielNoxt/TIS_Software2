/*
 * CAN_Bootloader.h
 *
 *  Created on: 13.05.2023
 *      Author: D.Claassen
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include "main.h"
#include "AppErrorHandling.h"

void BOOT_JumpToBootloader(void);
AppError_t BOOT_SetDeleteFlag(uint32_t flag);
AppError_t BOOT_UpdateConfig(CAN_Config_TypeDef newConfig);

#endif /* INC_BOOTLOADER_H_ */
