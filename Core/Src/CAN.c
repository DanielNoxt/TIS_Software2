/*
 * CAN.c
 *
 *  Created on: Mar 3, 2023
 *      Author: D.Claassen
 */

#include "CAN.h"
#include "Bootloader.h"
#include "LockIn.h"

CAN_TxHeaderTypeDef CAN_Tx_Header;
CAN_TRx_Data_TypeDef CAN_Tx_Data;
uint32_t CAN_Tx_Mailbox;
CAN_RxHeaderTypeDef CAN_Rx_Header;
CAN_TRx_Data_TypeDef CAN_Rx_Data;

//Todo: CAN-BUS Rücksetzfunktion für Fehlerfall

AppError_t CAN_SendMessage(CAN_HandleTypeDef *hcan);

/**
 * @brief  Initialisiere den Tx-Header mit Standardwerten
 * @retval None
 */
void CAN_Init_Tx_Header(void) {
	CAN_Tx_Header.ExtId = can_sensor_id;
	CAN_Tx_Header.DLC = 0;
	//Werden danach nicht mehr verändert!
	CAN_Tx_Header.StdId = 0;
	CAN_Tx_Header.IDE = CAN_ID_EXT;
	CAN_Tx_Header.RTR = CAN_RTR_DATA;
	CAN_Tx_Header.TransmitGlobalTime = DISABLE;
}

/**
 * @brief  Sende Aplitude und Phase vom MeasMode_Normal aller drei ADCs
 * @param  hCAN pointer zur CAN_HandleTypeDef structure für den verwendeten CAN-Port
 * @param  uint8_t Preset: Auswahl, welcher der Frequenzen verwendet wurde
 * @retval APP_ERROR_OK
 */
AppError_t CAN_Send_MeasMode_Normal(CAN_HandleTypeDef *hCAN, uint8_t Preset, const ElecTypeDef Sample_Elec[], LockIn_Results_TypeDef *hLockIn_Results) {

	for (uint8_t i = 0; i < 3; i++) {

		CAN_Tx_Data.f32[0] = hLockIn_Results->Ampl[i];
		CAN_Tx_Data.f32[1] = hLockIn_Results->Phase[i];

		CAN_Tx_Header.ExtId = can_sensor_id
				+ ((CAN_DATA_ID_MEASMODE_NORMAL_OFFSET
						+ CAN_DATA_ID_NUMBER_OF_DATA * Preset
						+ Sample_Elec[i] * 2) << 8);
		CAN_Tx_Header.DLC = 8;
		CAN_SendMessage(hCAN);
	}
	return APP_ERROR_OK;
}

AppError_t CAN_Send_MeasMode_Single(CAN_HandleTypeDef *hCAN, uint8_t Preset, const ElecTypeDef Sample_Elec, LockIn_Results_TypeDef *hLockIn_Results) {

	for (uint8_t i = 0; i < 3; i++) {

		CAN_Tx_Data.f32[0] = hLockIn_Results->Ampl[i];
		CAN_Tx_Data.f32[1] = hLockIn_Results->Phase[i];

		CAN_Tx_Header.ExtId = can_sensor_id
				+ ((CAN_DATA_ID_MEASMODE_NORMAL_OFFSET
						+ CAN_DATA_ID_NUMBER_OF_DATA * Preset
						+ Sample_Elec * 2) << 8);
		CAN_Tx_Header.DLC = 8;
		CAN_SendMessage(hCAN);
	}
	return APP_ERROR_OK;
}

/**
 * @brief  Sende Aplitude und Phase vom MeasMode_Sweep aller drei ADCs
 * @param  hCAN pointer zur CAN_HandleTypeDef structure für den verwendeten CAN-Port
 * @param  uint8_t Preset: Auswahl, welcher der Frequenzen verwendet wurde
 * @retval APP_ERROR_OK
 */
AppError_t CAN_Send_MeasMode_Sweep(CAN_HandleTypeDef *hCAN, uint8_t Preset, const ElecTypeDef Sample_Elec[], LockIn_Results_TypeDef *hLockIn_Results) {

	for (uint8_t i = 0; i < 3; i++) {
		CAN_Tx_Data.f32[0] = TIS_Sensor.Preset[0].fFreq_ist;
		CAN_Tx_Data.f32[1] = hLockIn_Results->Ampl[i];

		CAN_Tx_Header.ExtId = can_sensor_id
				+ ((CAN_DATA_ID_MEASMODE_SWEEP_OFFSET
						+ Sample_Elec[i] * 2) << 8);
		CAN_Tx_Header.DLC = 8;
		CAN_SendMessage(hCAN);

		CAN_Tx_Data.f32[0] = TIS_Sensor.Preset[0].fFreq_ist;
		CAN_Tx_Data.f32[1] = hLockIn_Results->Phase[i];

		CAN_Tx_Header.ExtId = can_sensor_id
				+ ((CAN_DATA_ID_MEASMODE_SWEEP_OFFSET
						+ Sample_Elec[i] * 2 + 1) << 8);
		CAN_Tx_Header.DLC = 8;
		CAN_SendMessage(hCAN);
	}
	return APP_ERROR_OK;
}

/**
 * @brief  Sende Aplitude und Phase vom LockIn aller drei ADCs
 * @param  hCAN pointer zur CAN_HandleTypeDef structure für den verwendeten CAN-Port
 * @param  uint8_t Preset: Auswahl, welcher der Frequenzen verwendet wurde
 * @retval APP_ERROR_OK
 */
AppError_t CAN_Send_ADC_RawData(CAN_HandleTypeDef *hCAN, uint8_t Preset, const ElecTypeDef Sample_Elec[]) {
	//Fülle CAN-Mailboxen bis voll:
	while (HAL_CAN_GetTxMailboxesFreeLevel(hCAN) > 0) {	//Fülle Mailbox, Falls eine Mailbox Frei
		uint8_t i = 0;
		while (i < 4 && ADC_Sampling.readIndex.Pos < ADC_Sampling.Length) { //Fülle die Mailbox, bis diese Voll ist oder keine Werte zum Senden verbleiben

			CAN_Tx_Data.u16[i] =
					ADC_Sampling.Data_DMA[ADC_Sampling.readIndex.Channel
							+ 3 * ADC_Sampling.readIndex.Pos];
			i++;

			ADC_Sampling.readIndex.Pos++;
		}
		CAN_Tx_Header.ExtId = can_sensor_id
				+ ((CAN_DATA_ID_MEASMODE_NORMAL_OFFSET
						+ CAN_DATA_ID_NUMBER_OF_DATA * Preset
						+ Sample_Elec[ADC_Sampling.readIndex.Channel]
								* 2 + 1) << 8);
		CAN_Tx_Header.DLC = 2 * i;
		CAN_SendMessage(hCAN);

		if (ADC_Sampling.readIndex.Pos == ADC_Sampling.Length) {
			ADC_Sampling.readIndex.Channel++;
			ADC_Sampling.readIndex.Pos = 0;
		}
	}

	return APP_ERROR_OK;
}

//Privat
AppError_t CAN_SendMessage(CAN_HandleTypeDef *hCAN) {
	while (HAL_CAN_GetTxMailboxesFreeLevel(hCAN) == 0)
		;
	if (HAL_CAN_AddTxMessage(hCAN, &CAN_Tx_Header, CAN_Tx_Data.u8,
			&CAN_Tx_Mailbox) != HAL_OK) {
		Error_Handler();
		return APP_ERROR_CAN_TX;
	}
	return APP_ERROR_OK;
}

AppError_t CAN_ReadCMD(void) {
	AppError_t Error = APP_ERROR_OK;
	if (CAN_Rx_Header.ExtId == can_sensor_id) {
		switch (CAN_Rx_Data.u8[0]) {
		case CAN_CMD_IDLE:
//			if (CAN_Rx_Header.DLC > 2) {//DBC kann scheinbar keine Multiplexed Botschaften mit Länge 1
//				Error = APP_ERROR_INVARG;
//				break;
//			}
			state_t_next = stateIdle;
			break;
		case CAN_CMD_RX_SAVE_DATA2FLASH:
//			if (CAN_Rx_Header.DLC > 2) {//DBC kann scheinbar keine Multiplexed Botschaften mit Länge 1
//				Error = APP_ERROR_INVARG;
//				break;
//			}
			DataWrite();
			break;
		case CAN_CMD_RX_PWM_FREQ:
			if (CAN_Rx_Header.DLC != 8
					|| CAN_Rx_Data.u8[1] >= 3
					|| CAN_Rx_Data.f32[1] < PWM_FREQ_MIN
					|| CAN_Rx_Data.f32[1] > PWM_FREQ_MAX) {
				Error = APP_ERROR_INVARG;
				break;
			}
			Preset_Calc(CAN_Rx_Data.u8[1], CAN_Rx_Data.f32[1]);
			CAN_SendFreq(&hcan2,CAN_Rx_Data.u8[1]);					//Echo der errechneten Frequenz
			break;
		case CAN_CMD_RX_ADC_LENGTH:
			if (CAN_Rx_Header.DLC != 8
					|| CAN_Rx_Data.u32[1] < ADC_C_DATA_DMA_MIN_LENGTH
					|| CAN_Rx_Data.u32[1] > ADC_C_DATA_DMA_MAX_LENGTH) {
				Error = APP_ERROR_INVARG;
				break;
			}
			ADC_Sampling.Length = CAN_Rx_Data.u32[1];
			break;
		case CAN_CMD_RX_CAN_PRESET:
//			if (CAN_Rx_Header.DLC != 2 || CAN_Rx_Data.u8[1] >= CAN_FREQ_Count) {
//				Error = APP_ERROR_INVARG;
//				break;
//			}
			TIS_Sensor.CAN_Freq_Preset = CAN_Rx_Data.u8[1];
			state_t_next = stateSetCanFreq;
			break;
		case CAN_CMD_RX_TIMING_LOCK_IN_DATA:
			if (CAN_Rx_Header.DLC != 8) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.Timing_LockInData = CAN_Rx_Data.u32[1];
			break;
		case CAN_CMD_RX_TIMING_ADDITIONAL_DATA:
			if (CAN_Rx_Header.DLC != 8) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.Timing_AdditionalData = CAN_Rx_Data.u32[1];
			break;
		case CAN_CMD_RX_TIMING_RAW_DATA:
			if (CAN_Rx_Header.DLC != 8) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.Timing_RawData = CAN_Rx_Data.u32[1];
			break;
		case CAN_CMD_RX_GAIN_PHASE:
			if (CAN_Rx_Header.DLC != 8
					|| CAN_Rx_Data.u8[1] > 2	//3 Preset
					|| CAN_Rx_Data.u8[2] > 3//4 DDS Channel
					|| CAN_Rx_Data.i16[2] < DDS_GAIN_MIN
					|| CAN_Rx_Data.i16[2] > DDS_GAIN_MAX
					|| CAN_Rx_Data.u16[3] < DDS_PHASE_MIN
					|| CAN_Rx_Data.u16[3] > DDS_PHASE_MAX) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.Preset[CAN_Rx_Data.u8[1]].DDS_Preset.iDACx_Dig_Gain[CAN_Rx_Data.u8[2]] =
					CAN_Rx_Data.i16[2];
			TIS_Sensor.Preset[CAN_Rx_Data.u8[1]].DDS_Preset.uDDSx_Phase[CAN_Rx_Data.u8[2]] =
					CAN_Rx_Data.u16[3];
			CAN_SendGain(&hcan2, CAN_Rx_Data.u8[1]);
			CAN_SendPhase(&hcan2, CAN_Rx_Data.u8[1]);
			break;
		case CAN_CMD_RX_FACTORY_RESET:
//			if (CAN_Rx_Header.DLC > 2) {//DBC kann scheinbar keine Multiplexed Botschaften mit Länge 1
//				Error = APP_ERROR_INVARG;
//				break;
//			}
			FactoryReset();
			break;
		case CAN_CMD_RX_SWEEP_PARAMETER:
			if (//CAN_Rx_Header.DLC != 4 ||
					   CAN_Rx_Data.u8[1] >= SWEEP_MODE_COUNT) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.SweepParameter.uParameter[0] = CAN_Rx_Data.u8[1];
			TIS_Sensor.SweepParameter.uParameter[1] = CAN_Rx_Data.u8[2];
			TIS_Sensor.SweepParameter.uParameter[2] = CAN_Rx_Data.u8[3];
			break;
		case CAN_CMD_RX_SWEEP_START:
			if (CAN_Rx_Header.DLC != 8) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.SweepParameter.fStart = CAN_Rx_Data.f32[1];
			break;
		case CAN_CMD_RX_SWEEP_STOP:
			if (CAN_Rx_Header.DLC != 8) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.SweepParameter.fStop = CAN_Rx_Data.f32[1];
			break;
		case CAN_CMD_RX_SWEEP_STEPSIZE:
			if (CAN_Rx_Header.DLC != 8 || CAN_Rx_Data.f32[1] == 0) {
				Error = APP_ERROR_INVARG;
				break;
			}
			//Check whether the sign of the stepsize matches the start and stop conditions
			if ((TIS_Sensor.SweepParameter.fStop - TIS_Sensor.SweepParameter.fStart) / CAN_Rx_Data.f32[1] <= 0) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.SweepParameter.fStep = CAN_Rx_Data.f32[1];
			//state_t_next = stateSweepParameter;
			break;
		case CAN_CMD_RX_SEND_SENSOR_INFO:
//			if (CAN_Rx_Header.DLC > 2) {//DBC kann scheinbar keine Multiplexed Botschaften mit Länge 1
//				Error = APP_ERROR_INVARG;
//				break;
//			}
			CAN_SendSensorInfo(&hcan2);
			break;
		case CAN_CMD_RX_AUTO_ZERO:
//			if (CAN_Rx_Header.DLC > 2) {//DBC kann scheinbar keine Multiplexed Botschaften mit Länge 1
//				Error = APP_ERROR_INVARG;
//				break;
//			}
			state_t_next = stateAutoZero;
			break;
		case CAN_CMD_RX_AVERAGES:
			if (//CAN_Rx_Header.DLC != 6 ||
					   CAN_Rx_Data.u16[2] < AVERAGES_LOCK_IN_MIN
					|| CAN_Rx_Data.u16[2] > AVERAGES_LOCK_IN_MAX) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.Averages_LockIn = CAN_Rx_Data.u16[2];
			break;
		case CAN_CMD_RX_TIMING_META_DATA:
			if (CAN_Rx_Header.DLC != 8) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.Timing_MetaData = CAN_Rx_Data.u32[1];
			break;
		case CAN_CMD_RX_MEAS_MODE:
			if (//CAN_Rx_Header.DLC != 6 ||
						CAN_Rx_Data.u16[2] >= MEASMODE_COUNT) {
				Error = APP_ERROR_INVARG;
				break;
			}
			TIS_Sensor.MeasMode = CAN_Rx_Data.u16[2];

			break;
		case CAN_CMD_START_BOOTLOADER:
			// Set DATA to be deleted if user selected reset
			if(CAN_Rx_Data.u8[1]) BOOT_SetDeleteFlag(1);
			BOOT_JumpToBootloader();
			break;

		default:
			Error = APP_ERROR_INVARG;
			break;
		}
		SoftError_Handler(Error);
		return Error;
	}
	SoftError_Handler(APP_ERROR_INVARG);
	return APP_ERROR_INVARG;
}

AppError_t CAN_SendError(CAN_HandleTypeDef *hCAN, AppError_t Error) {
	CAN_Tx_Data.u8[0] = Error;
	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_ERROR << 8);
	CAN_Tx_Header.DLC = 1;
	CAN_SendMessage(hCAN);
	return APP_ERROR_OK;
}

AppError_t CAN_SendUID(CAN_HandleTypeDef *hCAN) {
	AppError_t AppError = APP_ERROR_OK;
	CAN_Tx_Data.u32[0] = HAL_GetUIDw0();
	CAN_Tx_Data.u32[1] = HAL_GetUIDw1();

	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_UNIQUE_ID_LOW << 8);
	CAN_Tx_Header.DLC = 8;
	AppError = CAN_SendMessage(hCAN);

	CAN_Tx_Data.u32[0] = HAL_GetUIDw2();
	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_UNIQUE_ID_HIGH << 8);
	CAN_Tx_Header.DLC = 4;
	AppError = CAN_SendMessage(hCAN);
	return AppError;
}

AppError_t CAN_SendTemp_Vbat(CAN_HandleTypeDef *hCAN, float *hTemp_Sensor,
		float *hVbat) {
	AppError_t AppError = APP_ERROR_OK;
	CAN_Tx_Data.f32[0] = hTemp_Sensor[0];
	CAN_Tx_Data.f32[1] = hTemp_Sensor[1];
	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_TEMP1_TEMP2 << 8);
	CAN_Tx_Header.DLC = 8;
	AppError = CAN_SendMessage(hCAN);

	CAN_Tx_Data.f32[0] = hTemp_Sensor[2];
	CAN_Tx_Data.f32[1] = hTemp_Sensor[3];
	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_TEMP3_TEMP4 << 8);
	CAN_Tx_Header.DLC = 8;
	AppError = CAN_SendMessage(hCAN);

	CAN_Tx_Data.f32[0] = *hVbat;
	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_VBAT << 8);
	CAN_Tx_Header.DLC = 4;
	AppError = CAN_SendMessage(hCAN);

	return AppError;
}

AppError_t CAN_SendGain(CAN_HandleTypeDef *hCAN, uint8_t uPreset) {
	AppError_t AppError = APP_ERROR_OK;

	CAN_Tx_Header.DLC = 8;	//Float übertragen
	CAN_Tx_Header.ExtId = can_sensor_id
			+ ((CAN_DATA_ID_MEASMODE_NORMAL_OFFSET
					+ ((uPreset + 1) * CAN_DATA_ID_NUMBER_OF_DATA) - 3) << 8);
	for (uint8_t ch = 0; ch < 4; ch++) {
		CAN_Tx_Data.i16[ch] =
				TIS_Sensor.Preset[uPreset].DDS_Preset.iDACx_Dig_Gain[ch];
	}

	AppError = CAN_SendMessage(hCAN);

	return AppError;
}

AppError_t CAN_SendPhase(CAN_HandleTypeDef *hCAN, uint8_t uPreset) {
	AppError_t AppError = APP_ERROR_OK;

	CAN_Tx_Header.DLC = 8;	//Float übertragen
	CAN_Tx_Header.ExtId = can_sensor_id
			+ ((CAN_DATA_ID_MEASMODE_NORMAL_OFFSET
					+ ((uPreset + 1) * CAN_DATA_ID_NUMBER_OF_DATA) - 2) << 8);
	for (uint8_t ch = 0; ch < 4; ch++) {
		CAN_Tx_Data.u16[ch] =
				TIS_Sensor.Preset[uPreset].DDS_Preset.uDDSx_Phase[ch];
	}

	AppError = CAN_SendMessage(hCAN);

	return AppError;
}

AppError_t CAN_SendGainPhase_All(CAN_HandleTypeDef *hCAN) {
	AppError_t AppError = APP_ERROR_OK;

	for (uint8_t uPreset = 0; uPreset < 3; uPreset++) {
		AppError = CAN_SendGain(hCAN, uPreset);
		AppError = CAN_SendPhase(hCAN, uPreset);
	}
	return AppError;
}

AppError_t CAN_SendFreq(CAN_HandleTypeDef *hCAN, uint8_t uPreset) {
	AppError_t AppError = APP_ERROR_OK;

	CAN_Tx_Header.DLC = 8;	//Float übertragen
	CAN_Tx_Header.ExtId = can_sensor_id
			+ ((CAN_DATA_ID_MEASMODE_NORMAL_OFFSET
					+ ((uPreset + 1) * CAN_DATA_ID_NUMBER_OF_DATA) - 1) << 8);
	CAN_Tx_Data.u32[0] = 0;
	CAN_Tx_Data.f32[1] = TIS_Sensor.Preset[uPreset].fFreq_ist;

	AppError = CAN_SendMessage(hCAN);

	return AppError;
}

AppError_t CAN_SendFreq_All(CAN_HandleTypeDef *hCAN) {
	AppError_t AppError = APP_ERROR_OK;

	for (uint8_t uPreset = 0; uPreset < 3; uPreset++) {
		AppError = CAN_SendFreq(hCAN, uPreset);
	}
	return AppError;
}

AppError_t CAN_SendSensorInfo(CAN_HandleTypeDef *hCAN) {
	AppError_t AppError = APP_ERROR_OK;
	AppError |= CAN_Send_SensorID(hCAN);
	AppError |= CAN_Send_HW_Rev(hCAN);
	AppError |= CAN_Send_SW_Rev(hCAN);
	return AppError;
}

AppError_t CAN_Send_SensorID(CAN_HandleTypeDef *hCAN) {
	AppError_t AppError = APP_ERROR_OK;
	CAN_Tx_Data.u32[0] = TIS_Sensor.SensorSN;

	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_SENSOR_DATA << 8);
	CAN_Tx_Header.DLC = 4;

	AppError = CAN_SendMessage(hCAN);
	return AppError;
}

AppError_t CAN_Send_HW_Rev(CAN_HandleTypeDef *hCAN) {
	AppError_t AppError = APP_ERROR_OK;
	CAN_Tx_Data.u16[0] = HW_VERSION_MAJOR;
	CAN_Tx_Data.u16[1] = HW_VERSION_MINOR;
	CAN_Tx_Data.u16[2] = HW_VERSION_PATCH;

	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_HW_REV << 8);
	CAN_Tx_Header.DLC = 6;

	AppError = CAN_SendMessage(hCAN);
	return AppError;
}

AppError_t CAN_Send_SW_Rev(CAN_HandleTypeDef *hCAN) {
	AppError_t AppError = APP_ERROR_OK;
	CAN_Tx_Data.u16[0] = SW_VERSION_MAJOR;
	CAN_Tx_Data.u16[1] = SW_VERSION_MINOR;
	CAN_Tx_Data.u16[2] = SW_VERSION_PATCH;

	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_SW_REV << 8);
	CAN_Tx_Header.DLC = 6;

	AppError = CAN_SendMessage(hCAN);
	return AppError;
}

AppError_t CAN_Send_Systick(CAN_HandleTypeDef *hCAN) {
	AppError_t AppError = APP_ERROR_OK;
	CAN_Tx_Data.u32[0] = HAL_GetTick();

	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_SYSTICK << 8);
	CAN_Tx_Header.DLC = 4;

	AppError = CAN_SendMessage(hCAN);
	return AppError;
}

AppError_t CAN_Send_Averages(CAN_HandleTypeDef *hCAN){
	AppError_t AppError = APP_ERROR_OK;
	CAN_Tx_Data.u16[0] = TIS_Sensor.Averages_LockIn;

	CAN_Tx_Header.ExtId = can_sensor_id + (CAN_DATA_ID_AVERAGES << 8);
	CAN_Tx_Header.DLC = 2;

	AppError = CAN_SendMessage(hCAN);
	return AppError;
}

AppError_t CAN2_TRX_Set_Power(uint8_t Power) {
	if (Power == CAN_TRX_SET_POWER_ON || Power == CAN_TRX_SET_POWER_LOW_POWER) {
		HAL_GPIO_WritePin(CAN2_RS_GPIO_Port, CAN2_RS_Pin, Power);
		return APP_ERROR_OK;
	}
	return APP_ERROR_INVARG;
}

AppError_t CAN2_TRX_Set_Loopback(uint8_t AB) {
	if (AB == CAN_TRX_SET_LOOPBACK_OFF || AB == CAN_TRX_SET_LOOPBACK_ON) {
		HAL_GPIO_WritePin(CAN2_AB_GPIO_Port, CAN2_AB_Pin, AB);
		return APP_ERROR_OK;
	}
	return APP_ERROR_INVARG;
}

AppError_t CAN_Set_Mode(CAN_HandleTypeDef *hCAN, uint8_t CAN_Mode) {
	switch (CAN_Mode) {
	case CAN_SET_DISABLE:
		HAL_CAN_DeactivateNotification(hCAN, CAN_IT_ERROR);
		HAL_CAN_DeactivateNotification(hCAN, CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_AbortTxRequest(hCAN, CAN_TX_MAILBOX0);
		HAL_CAN_AbortTxRequest(hCAN, CAN_TX_MAILBOX1);
		HAL_CAN_AbortTxRequest(hCAN, CAN_TX_MAILBOX2);
		HAL_CAN_Stop(hCAN);
		return APP_ERROR_OK;
	case CAN_SET_ENABLE:
		HAL_CAN_ResetError(hCAN);
		HAL_CAN_Start(hCAN);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_ERROR);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_RX_FIFO0_MSG_PENDING);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_RX_FIFO0_FULL);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_RX_FIFO0_OVERRUN);
		return APP_ERROR_OK;
	case CAN_SET_SLEEP:
		return APP_ERROR_OK;
	default:
		return APP_ERROR_INVARG;
	}
}

AppError_t CAN_Set_Freq_Preset(CAN_HandleTypeDef *hCAN, CAN_FREQ_TypeDef Preset) {
	uint32_t BuffTimingLockIn = TIS_Sensor.Timing_LockInData;
	uint32_t BuffTimingRawData = TIS_Sensor.Timing_AdditionalData;

	TIS_Sensor.Timing_LockInData = 0;
	TIS_Sensor.Timing_AdditionalData = 0;

	uint32_t CAN_Prescaler = 0;
	switch (Preset) {
	case (CAN_FREQ_100KHZ):
		CAN_Prescaler = 30;
		break;
	case (CAN_FREQ_125KHZ):
		CAN_Prescaler = 24;
		break;
	case (CAN_FREQ_200KHZ):
		CAN_Prescaler = 15;
		break;
	case (CAN_FREQ_250KHZ):
		CAN_Prescaler = 12;
		break;
	case (CAN_FREQ_500KHZ):
		CAN_Prescaler = 6;
		break;
	case (CAN_FREQ_1000KHZ):
		CAN_Prescaler = 3;
		break;
	default:
		CAN_Prescaler = 30;
		break;
	}

	hCAN->Init.Prescaler = CAN_Prescaler;
	if (HAL_CAN_Init(hCAN) != HAL_OK) {
		Error_Handler();
	}
	HAL_CAN_Start(hCAN);
	TIS_Sensor.CAN_Freq_Preset = Preset;

	TIS_Sensor.Timing_LockInData = BuffTimingLockIn;
	TIS_Sensor.Timing_AdditionalData = BuffTimingRawData;
	return APP_ERROR_OK;
}

//DeviceID uint32_t HAL_GetDEVID(void)
