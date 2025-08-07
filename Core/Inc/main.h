/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SensorInfo.h"
#include "AppErrorHandling.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SoftError_Handler(AppError_t Error);
void Preset_Calc(uint8_t Preset, float fFreq);
void FactoryReset(void);
void InitGlobalVar(void);
void DataWrite(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LSI_CLOCK 32000
#define IWDG_TIMEOUT 30
#define Status_LED_Pin GPIO_PIN_3
#define Status_LED_GPIO_Port GPIOE
#define SPI4_CS_DDS_Pin GPIO_PIN_4
#define SPI4_CS_DDS_GPIO_Port GPIOE
#define Elec3_Pin GPIO_PIN_0
#define Elec3_GPIO_Port GPIOA
#define Elec2_Pin GPIO_PIN_1
#define Elec2_GPIO_Port GPIOA
#define Elec1_Pin GPIO_PIN_2
#define Elec1_GPIO_Port GPIOA
#define MicroElec_Pin GPIO_PIN_3
#define MicroElec_GPIO_Port GPIOA
#define MeasSin_Pin GPIO_PIN_5
#define MeasSin_GPIO_Port GPIOC
#define Eref_Pin GPIO_PIN_0
#define Eref_GPIO_Port GPIOB
#define V_batt_FB_Pin GPIO_PIN_1
#define V_batt_FB_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define CAN2_AB_Pin GPIO_PIN_6
#define CAN2_AB_GPIO_Port GPIOC
#define DDS_Trigger_Pin GPIO_PIN_7
#define DDS_Trigger_GPIO_Port GPIOC
#define CAN2_RS_Pin GPIO_PIN_10
#define CAN2_RS_GPIO_Port GPIOC
#define EN_PSU_A_Pin GPIO_PIN_12
#define EN_PSU_A_GPIO_Port GPIOC
#define CAN2_RX_Pin GPIO_PIN_5
#define CAN2_RX_GPIO_Port GPIOB
#define DDS_Reset_Pin GPIO_PIN_1
#define DDS_Reset_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
typedef enum {
	stateIdle,
	stateSendLockIn,
	stateSendAdditionalData,
	stateSendRawData,
	stateSendMetaData,
	stateSetCanFreq,
	stateSweepParameter,
	stateAutoZero,
	stateCount
} state_t_TypeDef;

typedef enum {
	CAN_FREQ_100KHZ,
	CAN_FREQ_125KHZ,
	CAN_FREQ_200KHZ,
	CAN_FREQ_250KHZ,
	CAN_FREQ_500KHZ,
	CAN_FREQ_1000KHZ,
	CAN_FREQ_Count
} CAN_FREQ_TypeDef;

typedef struct
{
  uint32_t		CanReceiveID;
  uint32_t		CanSendID;	//only used for bootloader
  CAN_FREQ_TypeDef 	CAN_Freq_Preset;
} CAN_Config_TypeDef;

typedef struct
{
  uint32_t	delete_flag;
} Delete_Flag_TypeDef;


// WARNING: Can_Config_TypeDef darf niemals verschoben werden, wegen kompatibilität zum bootloader!!
// WARNING: Das selbe gilt für den Delete_Flag
typedef struct
{
  CAN_Config_TypeDef  canConfig;
  Delete_Flag_TypeDef deleteFlag;
} CONSTANTS_TypeDef;


//Sektoren wo variablen gespeichert werden
#define CONSTANTS_SECTOR FLASH_SECTOR_3
#define DATA_SECTOR FLASH_SECTOR_4

//Sensorinfo

#define SENSOR_SN 2014
#if SENSOR_SN < 1999 || SENSOR_SN > 2031
  #error "SENSOR_SN muss zwischen 2000 und 2030 liegen."
#endif

#define	CAN_FREQ_PRESET	CAN_FREQ_500KHZ

//CAN-Adress:
#define CAN_SENSOR_ID_DEFAULT		((SENSOR_SN-2000)*0x00010000+0x1CD000EE)
#define CAN_SENSOR_MASK  		0x1FFFFFFF
#define REMOTE_FRAME 			0		//0: the frame will be either remote or data frame, 1: remote frame
#define EXTID 				1		//0: the frame should be a frame with standard ID, 1: extended ID
#define CAN_SENSOR_FILTER_ID(sensor_id)	((sensor_id << 3) | (EXTID <<2) | (REMOTE_FRAME<<1))
#define CAN_SENSOR_FILTER_MASK  	((CAN_SENSOR_MASK << 3) | (EXTID <<2) | (REMOTE_FRAME<<1))

//CAN Data-ID
#define CAN_DATA_ID_SENSOR_DATA		0x01
#define CAN_DATA_ID_ERROR			0x02
#define CAN_DATA_ID_UNIQUE_ID_LOW	0x03
#define CAN_DATA_ID_UNIQUE_ID_HIGH	0x04
#define CAN_DATA_ID_TEMP1_TEMP2		0x05
#define CAN_DATA_ID_TEMP3_TEMP4		0x06
#define CAN_DATA_ID_HW_REV			0x07
#define CAN_DATA_ID_SW_REV			0x08
#define CAN_DATA_ID_SYSTICK			0x09
#define CAN_DATA_ID_VBAT			0x0A
#define CAN_DATA_ID_AVERAGES		0x0B

#define CAN_DATA_ID_MEASMODE_SWEEP_OFFSET 	0x26
#define CAN_DATA_ID_MEASMODE_NORMAL_OFFSET 	0x32
#define CAN_DATA_ID_NUMBER_OF_DATA	0x0F

//CAN-Befehlssätze für CAN_BASE_ADR und CAN_SENSOR_ADR
#define CAN_CMD_IDLE	 					0x00
#define CAN_CMD_RX_SAVE_DATA2FLASH			0x01
#define CAN_CMD_RX_PWM_FREQ					0x02
#define CAN_CMD_RX_ADC_LENGTH 				0x03
#define CAN_CMD_RX_CAN_PRESET 				0x04
#define CAN_CMD_RX_TIMING_LOCK_IN_DATA		0x05
#define CAN_CMD_RX_TIMING_ADDITIONAL_DATA	0x06
#define CAN_CMD_RX_TIMING_RAW_DATA			0x07
#define CAN_CMD_RX_GAIN_PHASE				0x08
#define CAN_CMD_RX_FACTORY_RESET			0x09
#define CAN_CMD_RX_SWEEP_PARAMETER			0x0A
#define CAN_CMD_RX_SWEEP_START				0x0B
#define CAN_CMD_RX_SWEEP_STOP				0x0C
#define CAN_CMD_RX_SWEEP_STEPSIZE			0x0D
#define CAN_CMD_RX_SEND_SENSOR_INFO			0x0E
#define CAN_CMD_RX_AUTO_ZERO				0x0F
#define CAN_CMD_RX_AVERAGES					0x10
#define CAN_CMD_RX_TIMING_META_DATA			0x11
#define CAN_CMD_RX_MEAS_MODE				0x12

#define CAN_CMD_START_BOOTLOADER 			0xAA

//Clocks, Timings
#define PCLK2_FREQ 90E6
#define APB2_PERIPHERAL_FREQ PCLK2_FREQ
#define APB2_TIMER_FREQ PCLK2_FREQ*2
#define PWM_FREQ_MIN			(float)1E3
#define PWM_FREQ_MAX 			(float)1E6
#define DDS_GAIN_MIN 			(int16_t)-2048
#define DDS_GAIN_MAX 			(int16_t)2047
#define DDS_PHASE_MIN 			(uint16_t) 0
#define DDS_PHASE_MAX 			(uint16_t) 65535
#define AVERAGES_LOCK_IN_MIN 	(uint16_t) 1
#define AVERAGES_LOCK_IN_MAX 	(uint16_t) 65535

//AutoZero Defines
#define AUTOZERO_INITIAL_STEP_PHASE 	256
#define AUTOZERO_REFINEMENT_FACTOR 		8
#define AUTOZERO_MAX_ITERATIONS			5
#define AUTOZERO_MOVING_AVERAGE_SIZE	5

#define AUTOZERO_STEP_GAIN				1

//ADCs
#define ADC_VRef (float) 2.5
#define ADC_FREQ PCLK2_FREQ/4
#define ADC_SYNC_DELAY_CYCLES	5
#define ADC_SAMPLING_CYCLES		3
#define ADC_CONV_CYCLES			12
#define ADC_RES					12
#define ADC_RANGE_MIN 			(int16_t)(-(1<<ADC_RES)/2)        // ADC Measrange
#define ADC_RANGE_MAX 			(int16_t)((1<<ADC_RES)/2-1)
#define LOCK_IN_AMP_REF 		(float)0.1
#define ADC_SAMPLE_FREQ 		(float)(ADC_FREQ/(float)(ADC_SAMPLING_CYCLES+ADC_CONV_CYCLES))
#define ADC_SAMPLE_TIME			(float)(1/ADC_SAMPLE_FREQ)
#define ADC_C_DATA_DMA_LENGTH		1000
#define ADC_C_DATA_DMA_MIN_LENGTH	50		//Minimale Anzahl an Werten für den LockIn, bevor dieser nur noch Quatsch rechnet.
#define ADC_C_DATA_DMA_MAX_LENGTH	5000
#define ADC_C_DATA_DMA_SIZE		3*ADC_C_DATA_DMA_MAX_LENGTH
#define ADC_V_BATT_R1	(float)1e3
#define ADC_V_BATT_R2	(float)10e3

typedef enum{
	EREF,
	EMICRO,
	E1,
	E2,
	E3,
	EA,
	V_BATT_FB,
	ELEC_COUNT
}ElecTypeDef;

typedef struct {
	uint8_t Channel;
	uint32_t Pos;
}Data_DMA_Index_TypeDef;

typedef struct {
	volatile uint16_t Data_DMA[ADC_C_DATA_DMA_SIZE];
	volatile Data_DMA_Index_TypeDef readIndex;
	volatile uint8_t ConCplet;
	volatile uint8_t TimerSync;		//Statusbit, welches zum Synchronisieren auf den nächsten Timerinterrupt dient
	volatile uint32_t Length;
} ADC_Sampling_TypeDef;

typedef struct {
	float Temp;
	float Slope;
	float Offset;
} ADC_Temp_ResultsTypeDef;

typedef struct {
	int16_t Vr[ADC_C_DATA_DMA_MAX_LENGTH];
	int16_t Vr_90[ADC_C_DATA_DMA_MAX_LENGTH];
} LockIn_TypeDef;

typedef struct {
	float Ampl[3];
	float Phase[3];
}LockIn_Results_TypeDef;

typedef struct {
	uint32_t u32_DDS_TW;
	uint16_t uDDSx_Phase[4];
	int16_t iDACx_Dig_Gain[4];
} AD9106_SinusSettings_TypeDef;

typedef struct {
	volatile float fFreq_soll;
	float fFreq_ist;
	AD9106_SinusSettings_TypeDef DDS_Preset;
} Preset_TypeDef;

typedef struct {
	volatile float fStart;
	volatile float fStop;
	volatile float fStep;
	volatile uint8_t uParameter[3];
} SweepParameter_TypeDef;

//DDS Channel Assignment
typedef enum{
	E2_E3_Comp,
	E1_Comp,
	EA_SinGen,
	EMICRO_Comp,
	E_COMP_COUNT
}DDSChannelTypeDef;

#ifdef __MAIN_C
//// Mapping-Tabelle: Gibt die Elektrode zu einem DDS-Channel aus.
//const ElecTypeDef DDS_to_Elec_mapping[E_COMP_COUNT] = {
//    E3,    // E2_E3_Comp auf E3
//    E1,    // E1_Comp auf E1
//	  EA,    // EA_SinGen auf EA
//    EMICRO // EMICRO_CompEA_SinGen
//};
// Mapping-Tabelle: Gibt den DDS-Channel zu einer Elektrode aus.
const DDSChannelTypeDef Elec_to_DDS_mapping[ELEC_COUNT] = {
	E_COMP_COUNT,	//Eref, no compensation Channel
	EMICRO_Comp,	//EMikro Compensation
	E1_Comp,		//E1 Compensation
	E2_E3_Comp,		//E2 Compensation
	E_COMP_COUNT,	//E3 Compensation, kann ausgewählt werden
	E_COMP_COUNT,	//EA, no compensation Channel
	E_COMP_COUNT,	//V_BATT_FB, no compensation Channel
};
#endif

//MeasModes
enum{
	MEASMODE_NORMAL,
	MEASMODE_SWEEP_FREQ,
	MEASMODE_SINGLE,
	MEASMODE_COUNT
};

//SweepModes
enum{
	SWEEP_MODE_FREQ,
	SWEEP_MODE_GAIN,
	SWEEP_MODE_PHASE,
	SWEEP_MODE_COUNT
};

typedef struct {
	uint16_t SensorSN;
	volatile uint32_t Timing_LockInData;
	volatile uint32_t Timing_AdditionalData;
	volatile uint32_t Timing_RawData;
	volatile uint32_t Timing_MetaData;
	volatile uint16_t Averages_LockIn;
	volatile uint16_t MeasMode;
	SweepParameter_TypeDef SweepParameter;
	CAN_FREQ_TypeDef CAN_Freq_Preset;

	Preset_TypeDef Preset[3];
}TIS_Sensor_TypeDef;

typedef struct
{
	TIS_Sensor_TypeDef TIS_Sensor;
	uint32_t ADC_Length;
}Flash_Data_TypeDef;


#define ADC_CHANNEL_EREF		ADC_CHANNEL_8		//ADC12_IN8
#define ADC_CHANNEL_EMICRO		ADC_CHANNEL_3		//ADC123_IN3
#define ADC_CHANNEL_E1			ADC_CHANNEL_2		//ADC123_IN2
#define ADC_CHANNEL_E2			ADC_CHANNEL_1		//ADC123_IN1
#define ADC_CHANNEL_E3			ADC_CHANNEL_0		//ADC123_IN0
#define ADC_CHANNEL_V_SINGEN	ADC_CHANNEL_15		//ADC12_IN15
#define ADC_CHANNEL_V_BATT_FB	ADC_CHANNEL_9		//ADC12_IN9

//Globale Variablen, werden in der Main reserviert und überall anders als extern gehandhabt.
#ifdef __MAIN_C
CONSTANTS_TypeDef *ConstantsData = (CONSTANTS_TypeDef*)0x0800C000;
Flash_Data_TypeDef *FlashData = (Flash_Data_TypeDef*)  0x08010000;
uint32_t	can_sensor_id;
TIS_Sensor_TypeDef TIS_Sensor = {SENSOR_SN};
ADC_Sampling_TypeDef ADC_Sampling;
const ElecTypeDef SampleElec[2][3] = {
		{EMICRO, E1, E3},
		{EA, EREF, E2}
};
LockIn_Results_TypeDef LockIn_Results = {
        .Ampl = {0.0, 0.0, 0.0},
        .Phase = {0.0, 0.0, 0.0}
};
float fTemp_Sensor[4] = {0};

LockIn_TypeDef LockIn_Preset[3];

float VBat = 0;
state_t_TypeDef state_t_cur = stateIdle;
volatile state_t_TypeDef state_t_next = stateIdle;

const uint32_t ADCx_CHANNEL[ELEC_COUNT] = {ADC_CHANNEL_EREF,ADC_CHANNEL_EMICRO,ADC_CHANNEL_E1,ADC_CHANNEL_E2,ADC_CHANNEL_E3,ADC_CHANNEL_V_SINGEN, ADC_CHANNEL_V_BATT_FB};


#else
	extern CONSTANTS_TypeDef *ConstantsData;
	extern uint32_t can_sensor_id;
	extern TIS_Sensor_TypeDef TIS_Sensor;
	extern ADC_Sampling_TypeDef ADC_Sampling;
	extern const ElecTypeDef SampleElec[2][3];
	extern float fTemp_Sensor[3];
	extern LockIn_TypeDef LockIn_Preset[3];
	extern AD9106_SinusSettings_TypeDef DDS_Preset[3];

	extern state_t_TypeDef state_t_cur;
	extern volatile state_t_TypeDef state_t_next;

	extern const uint32_t ADCx_CHANNEL[ELEC_COUNT];

	extern ADC_HandleTypeDef hadc1;
	extern ADC_HandleTypeDef hadc2;
	extern ADC_HandleTypeDef hadc3;
	extern DMA_HandleTypeDef hdma_adc1;
	extern DMA_HandleTypeDef hdma_adc2;
	extern DMA_HandleTypeDef hdma_adc3;

	extern CAN_HandleTypeDef hcan2;
	extern FMPI2C_HandleTypeDef hfmpi2c1;
	extern SPI_HandleTypeDef hspi4;

	extern CRC_HandleTypeDef hcrc;

	extern TIM_HandleTypeDef htim8;
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
