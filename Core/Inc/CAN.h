/*
 * CAN.h
 *
 *  Created on: Mar 3, 2023
 *      Author: D.Claassen
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include "AppErrorHandling.h"

typedef union
{
	uint8_t u8[8];
	uint16_t u16[4];
	uint32_t u32[2];
	int8_t i8[8];
	int16_t i16[4];
	int32_t i32[2];
    float f32[2];
}CAN_TRx_Data_TypeDef;

//BitTimings von http://www.bittiming.can-wiki.info/

//SN65HVD235QDRQ1
#define CAN_TRX_SET_POWER_ON 		0		//Slope-Control-Mode
#define CAN_TRX_SET_POWER_LOW_POWER 1		//low-power-Mode
#define CAN_TRX_SET_LOOPBACK_OFF 0
#define CAN_TRX_SET_LOOPBACK_ON 1
#define CAN_SET_DISABLE 	0
#define CAN_SET_ENABLE 		1
#define CAN_SET_SLEEP		2


void CAN_Init_Tx_Header (void);
AppError_t CAN_Send_MeasMode_Normal(CAN_HandleTypeDef *hCAN, uint8_t Preset, const ElecTypeDef Sample_Elec[], LockIn_Results_TypeDef *hLockIn_Results);
AppError_t CAN_Send_MeasMode_Single(CAN_HandleTypeDef *hCAN, uint8_t Preset, const ElecTypeDef Sample_Elec, LockIn_Results_TypeDef *hLockIn_Results);
AppError_t CAN_Send_MeasMode_Sweep(CAN_HandleTypeDef *hCAN, uint8_t Preset, const ElecTypeDef Sample_Elec[], LockIn_Results_TypeDef *hLockIn_Results);
AppError_t CAN_Send_ADC_RawData(CAN_HandleTypeDef *hCAN, uint8_t Preset, const ElecTypeDef Sample_Elec[]);
AppError_t CAN_SendUID(CAN_HandleTypeDef *hCAN);
AppError_t CAN_SendTemp_Vbat(CAN_HandleTypeDef *hCAN, float *hTemp_Sensor, float *hVbat);
AppError_t CAN_SendError(CAN_HandleTypeDef *hCAN, AppError_t Error);
AppError_t CAN_SendGain(CAN_HandleTypeDef *hCAN, uint8_t uPreset);
AppError_t CAN_SendPhase(CAN_HandleTypeDef *hCAN, uint8_t uPreset);
AppError_t CAN_SendGainPhase_All(CAN_HandleTypeDef *hCAN);
AppError_t CAN_SendFreq(CAN_HandleTypeDef *hCAN, uint8_t uPreset);
AppError_t CAN_SendFreq_All(CAN_HandleTypeDef *hCAN);
AppError_t CAN_SendSensorInfo(CAN_HandleTypeDef *hCAN);
AppError_t CAN_Send_SensorID(CAN_HandleTypeDef *hCAN);
AppError_t CAN_Send_HW_Rev(CAN_HandleTypeDef *hCAN);
AppError_t CAN_Send_SW_Rev(CAN_HandleTypeDef *hCAN);
AppError_t CAN_Send_Systick(CAN_HandleTypeDef *hCAN);
AppError_t CAN_Send_Averages(CAN_HandleTypeDef *hCAN);

AppError_t CAN_ReadCMD(void);
AppError_t CAN_Set_Freq_Preset(CAN_HandleTypeDef *hCAN, CAN_FREQ_TypeDef Preset);
AppError_t CAN_Set_Mode(CAN_HandleTypeDef *hCAN, uint8_t CAN_Mode);
AppError_t CAN2_TRX_Set_Power(uint8_t LowPower);
AppError_t CAN2_TRX_Set_Loopback(uint8_t AB);




#endif /* INC_CAN_H_ */
