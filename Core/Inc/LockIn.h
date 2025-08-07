/*
 * LockIn.h
 *
 *  Created on: 11.03.2023
 *      Author: D.Claassen
 */

#ifndef INC_LOCKIN_H_
#define INC_LOCKIN_H_

#include "main.h"

//AppError_t LockIn_Set_Freq(uint8_t Ch, float freq);

AppError_t LockIn_LUT_Gen(uint8_t Ch);
AppError_t LockIn_CalcAdd(uint8_t Preset, LockIn_Results_TypeDef *hLockIn_Results);
void Lock_In_Results_Reset(LockIn_Results_TypeDef *hLockIn_Results);

void Lock_In_FillADC(uint8_t Ch, uint32_t length);

#endif /* INC_LOCKIN_H_ */
