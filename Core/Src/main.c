/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

#define __MAIN_C
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "FLASH.h"
#include "ADC.h"
#include "Bootloader.h"
#include "CAN.h"
#include "LockIn.h"
#include "AS6214.h"
#include "DDS.h"
#include "MovingAverage.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

FMPI2C_HandleTypeDef hfmpi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
extern CAN_RxHeaderTypeDef CAN_Rx_Header;
extern CAN_TRx_Data_TypeDef CAN_Rx_Data;

AD9106_HandleTypeDef DDS = {&hspi4, SPI4_CS_DDS_GPIO_Port, SPI4_CS_DDS_Pin, DDS_Reset_GPIO_Port, DDS_Reset_Pin, DDS_Trigger_GPIO_Port, DDS_Trigger_Pin};

AS6212_HandleTypeDef Temp_links = {&hfmpi2c1,0x48};
AS6212_HandleTypeDef Temp_mitte = {&hfmpi2c1,0x49};
AS6212_HandleTypeDef Temp_rechts = {&hfmpi2c1,0x4A};
AS6212_HandleTypeDef Temp_Elektrode = {&hfmpi2c1,0x4B};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI4_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
static void InitFactory(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
AppError_t SystemCheck(void);
AppError_t WakeUp(void);

void AS6214_Init(void);
void nextstate(state_t_TypeDef nstate_t);
void MeasMode_Normal(LockIn_Results_TypeDef *hLockIn_Results);
void MeasMode_Sweep_Freq(LockIn_Results_TypeDef *hLockIn_Results);
void MeasMode_Single(LockIn_Results_TypeDef *hLockIn_Results);
void MeasMode_Sweep_Gain(LockIn_Results_TypeDef *hLockIn_Results);
void MeasMode_Sweep_Phase(LockIn_Results_TypeDef *hLockIn_Results);
void SweepParameter(LockIn_Results_TypeDef *hLockIn_Results);
void AutoZero(LockIn_Results_TypeDef *hLockIn_Results);
void SampleCalc(uint8_t Preset, uint16_t uAverages, const ElecTypeDef Sample_Elec[], LockIn_Results_TypeDef *hLockIn_Results);
void SampleCalcSingle(uint8_t Preset, uint16_t uAverages, const ElecTypeDef Sample_Elec, LockIn_Results_TypeDef *hLockIn_Results);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//JumpToBootloader();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //Refresh watchdog, can't use HAL function because IWDG handle is not initialized yet
  WRITE_REG(IWDG->KR, IWDG_KEY_RELOAD);
  //Initialisiert variablen für CANBUS
  InitFactory();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_CAN2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_SPI4_Init();
  MX_FMPI2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	//Initialisieren aller Parameter
	InitGlobalVar();

	//  SystemCheck();
	WakeUp();

	__enable_irq();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	state_t_cur = stateIdle;
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_IWDG_Refresh(&hiwdg);
		switch (state_t_cur) {
		case stateIdle:
			//nextstate(stateIdle);
			break;

		case stateSendLockIn:
			switch(TIS_Sensor.MeasMode){
				case MEASMODE_NORMAL:
					MeasMode_Normal(&LockIn_Results);
					break;

				case MEASMODE_SWEEP_FREQ:
					MeasMode_Sweep_Freq(&LockIn_Results);
					break;

				case MEASMODE_SINGLE:
					MeasMode_Single(&LockIn_Results);
					break;
//				case MEASMODE_SWEEP_GAIN:
//					MeasMode_Sweep_Gain(&LockIn_Results);
//					break;
//
//				case MEASMODE_SWEEP_PHASE:
//					MeasMode_Sweep_Phase(&LockIn_Results);
//					break;

				default:
					break;
			}
			nextstate(stateIdle);
			break;

		case stateSendAdditionalData:
				ADC_Vbat(&VBat);
				fTemp_Sensor[0] = AS6212_getTempC(&Temp_links);
				fTemp_Sensor[1] = AS6212_getTempC(&Temp_mitte);
				fTemp_Sensor[2] = AS6212_getTempC(&Temp_rechts);
				fTemp_Sensor[3] = AS6212_getTempC(&Temp_Elektrode);
				CAN_SendTemp_Vbat(&hcan2, fTemp_Sensor, &VBat);
				nextstate(stateIdle);
			break;

		case stateSendRawData:
			for (uint8_t uPre = 0; uPre < 3; uPre++) {	//Toggle alle 3 Frequenzen durch
				DDS_set_Preset(&DDS, &TIS_Sensor.Preset[uPre].DDS_Preset);

				SampleCalc(uPre, 1, &SampleElec[0][0], &LockIn_Results);
				while (ADC_Sampling.readIndex.Channel < 3
						&& ADC_Sampling.readIndex.Pos < ADC_Sampling.Length) {
					CAN_Send_ADC_RawData(&hcan2, uPre, &SampleElec[0][0]);	//Sende Rohwerte
				}

				SampleCalc(uPre, 1, &SampleElec[1][0], &LockIn_Results);
				while (ADC_Sampling.readIndex.Channel < 3
						&& ADC_Sampling.readIndex.Pos < ADC_Sampling.Length) {
					CAN_Send_ADC_RawData(&hcan2, uPre, &SampleElec[1][0]);	//Sende Rohwerte
				}
			}
			CAN_SendSensorInfo(&hcan2);
			CAN_SendError(&hcan2, APP_ERROR_OK);
			CAN_SendFreq_All(&hcan2);
			nextstate(stateIdle);
			break;

		case stateSendMetaData:
			CAN_Send_Systick(&hcan2);
			CAN_SendFreq_All(&hcan2);
			CAN_SendGainPhase_All(&hcan2);
			CAN_Send_Averages(&hcan2);
			nextstate(stateIdle);
			break;

		case stateSweepParameter:
			//Achtung: Möglicherweise Lange Laufzeit des Programms, je nach Parameter
			SweepParameter(&LockIn_Results);
			nextstate(stateIdle);
			break;

		case stateAutoZero:
			AutoZero(&LockIn_Results);
			nextstate(stateIdle);
			break;

		case stateSetCanFreq:
			CAN_Set_Freq_Preset(&hcan2, TIS_Sensor.CAN_Freq_Preset);
			nextstate(stateIdle);
			break;


		default:
			nextstate(stateIdle);
			break;
		}
		state_t_cur = state_t_next;
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
	CAN_Init_Tx_Header();
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */
	//Bittimings: http://www.bittiming.can-wiki.info/
  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	//Filtersetup für allgemein eingehende Nachrichten auf CAN_SENSOR_ADR
  	//Wenn das Ergebnis der bitweisen AND-Operation zwischen dem empfangenen Identifier und der Maske gleich der Filter-Id ist, dann wird die Nachricht vom Filter akzeptiert.
	CAN_FilterTypeDef CAN_FILTER_CONFIG;
	CAN_FILTER_CONFIG.FilterActivation = CAN_FILTER_ENABLE;
	CAN_FILTER_CONFIG.SlaveStartFilterBank = 1;	//insgesamt 28 Filter, geteilt zwischen CAN1 (Master) CAN2(Slave) aktuell Can1: 0 Can2: 1-27
	CAN_FILTER_CONFIG.FilterBank = 1;		//Alles ab SlaveStartFilterBank
	CAN_FILTER_CONFIG.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_FILTER_CONFIG.FilterIdHigh = (CAN_SENSOR_FILTER_ID(can_sensor_id) >> 16) & 0xFFFF;	//Shift um 5, da EXID die ersten 5 Bits belegt. [RM0390 Rev 6 S.1057]
	CAN_FILTER_CONFIG.FilterIdLow = CAN_SENSOR_FILTER_ID(can_sensor_id) & 0xFFFF;
	CAN_FILTER_CONFIG.FilterMaskIdHigh = (CAN_SENSOR_FILTER_MASK >> 16) & 0xFFFF;
	CAN_FILTER_CONFIG.FilterMaskIdLow = CAN_SENSOR_FILTER_MASK & 0xFFFF;
	CAN_FILTER_CONFIG.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FILTER_CONFIG.FilterScale = CAN_FILTERSCALE_32BIT;
	if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FILTER_CONFIG) != HAL_OK) {
		Error_Handler();
	}
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FMPI2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMPI2C1_Init(void)
{

  /* USER CODE BEGIN FMPI2C1_Init 0 */

  /* USER CODE END FMPI2C1_Init 0 */

  /* USER CODE BEGIN FMPI2C1_Init 1 */

  /* USER CODE END FMPI2C1_Init 1 */
  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0x00401650;
  hfmpi2c1.Init.OwnAddress1 = 2;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMPI2C1_Init 2 */

  /* USER CODE END FMPI2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = (IWDG_TIMEOUT*LSI_CLOCK)/256;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 179;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPI4_CS_DDS_Pin|DDS_Reset_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAN2_AB_Pin|CAN2_RS_Pin|EN_PSU_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DDS_Trigger_GPIO_Port, DDS_Trigger_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Status_LED_Pin DDS_Reset_Pin */
  GPIO_InitStruct.Pin = Status_LED_Pin|DDS_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI4_CS_DDS_Pin */
  GPIO_InitStruct.Pin = SPI4_CS_DDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI4_CS_DDS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN2_AB_Pin CAN2_RS_Pin EN_PSU_A_Pin */
  GPIO_InitStruct.Pin = CAN2_AB_Pin|CAN2_RS_Pin|EN_PSU_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DDS_Trigger_Pin */
  GPIO_InitStruct.Pin = DDS_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DDS_Trigger_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ADC_Sampling.ConCplet = 0;
	//HAL_ADCEx_MultiModeStop_DMA(&hadc1);

}

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	ADC_Sampling.TimerSync = 0;
}

AppError_t SystemCheck(void) {
	// Spannungen Überprüfen, insbesondere Vref, und 5 V
	return APP_ERROR_OK;
}

AppError_t WakeUp(void) {
	HAL_GPIO_WritePin(EN_PSU_A_GPIO_Port, EN_PSU_A_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);

//	HAL_TIM_Base_Start(&htim8);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

	CAN2_TRX_Set_Power(CAN_TRX_SET_POWER_ON);
	CAN2_TRX_Set_Loopback(CAN_TRX_SET_LOOPBACK_OFF);
	CAN_Set_Mode(&hcan2, CAN_SET_ENABLE);

	AS6214_Init();
	DDS_Init(&DDS);
	return APP_ERROR_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_Header, CAN_Rx_Data.u8);
	CAN_ReadCMD();
}

void nextstate(state_t_TypeDef nstate_t) {
	if (state_t_cur == state_t_next) {//Dann wurde dieser nicht vom Host geändert.
		state_t_next = nstate_t;
	}
}

void SampleCalc(uint8_t Preset, uint16_t uAverages, const ElecTypeDef Sample_Elec[], LockIn_Results_TypeDef *hLockIn_Results){
	Lock_In_Results_Reset(hLockIn_Results);
	ADC_Sample_Elec_DDS(&Sample_Elec[0]);
	LockIn_CalcAdd(Preset, hLockIn_Results);
	if(uAverages>1){
		for(uint16_t i = 1; i<uAverages; i++){
			ADC_Sample_Elec_DDS(&Sample_Elec[0]);
			LockIn_CalcAdd(Preset, hLockIn_Results);
		}
		for(uint8_t i= 0; i<3 ; i++){
			hLockIn_Results->Ampl[i] /= uAverages;
			hLockIn_Results->Phase[i] /= uAverages;
		}
	}
}

void SampleCalcSingle(uint8_t Preset, uint16_t uAverages, const ElecTypeDef Sample_Elec, LockIn_Results_TypeDef *hLockIn_Results){
	Lock_In_Results_Reset(hLockIn_Results);
	ADC_Sample_Elec_DDS_Single(Sample_Elec);
	LockIn_CalcAdd(Preset, hLockIn_Results);
	if(uAverages>1){
		for(uint16_t i = 1; i<uAverages; i++){
			ADC_Sample_Elec_DDS_Single(Sample_Elec);
			LockIn_CalcAdd(Preset, hLockIn_Results);
		}
		for(uint8_t i= 0; i<3 ; i++){
			hLockIn_Results->Ampl[i] /= uAverages;
			hLockIn_Results->Phase[i] /= uAverages;
		}
	}
}

void MeasMode_Normal(LockIn_Results_TypeDef *hLockIn_Results){
	for (uint8_t uPre = 0; uPre < 3; uPre++) {
		HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
		DDS_set_Preset(&DDS, &TIS_Sensor.Preset[uPre].DDS_Preset);

		SampleCalc(uPre, TIS_Sensor.Averages_LockIn, &SampleElec[0][0], hLockIn_Results);
		CAN_Send_MeasMode_Normal(&hcan2, uPre, &SampleElec[0][0], hLockIn_Results);
		SampleCalc(uPre, TIS_Sensor.Averages_LockIn, &SampleElec[1][0], hLockIn_Results);
		CAN_Send_MeasMode_Normal(&hcan2, uPre, &SampleElec[1][0], hLockIn_Results);
	}
}

void MeasMode_Sweep_Freq(LockIn_Results_TypeDef *hLockIn_Results){
	static uint32_t iSweep = 0;  // Fortschritt speichern
	float fSweep = TIS_Sensor.SweepParameter.fStart + iSweep * TIS_Sensor.SweepParameter.fStep;
	//Berechne Bestmögliche Frequenz des DDS
	fSweep = DDS_Calc_Freq(fSweep);

	// Überprüfen, ob der Sweep innerhalb seiner Grenzen ist
	if (fSweep < TIS_Sensor.SweepParameter.fStart ||
	    		fSweep > TIS_Sensor.SweepParameter.fStop) {
	        fSweep = TIS_Sensor.SweepParameter.fStart;  // Reset für neuen Zyklus
	        iSweep = 0;
	    }
				if(fSweep < PWM_FREQ_MIN
						|| fSweep > PWM_FREQ_MAX
						|| fSweep == TIS_Sensor.Preset[0].fFreq_ist){
					iSweep++;
					return;
				}
				TIS_Sensor.Preset[0].fFreq_soll = fSweep;
				Preset_Calc(0, TIS_Sensor.Preset[0].fFreq_soll);
				DDS_set_Preset(&DDS, &TIS_Sensor.Preset[0].DDS_Preset);

				//Starte Run:
				HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
				SampleCalc(0, TIS_Sensor.Averages_LockIn, &SampleElec[0][0], hLockIn_Results);
				CAN_Send_MeasMode_Sweep(&hcan2, 0, &SampleElec[0][0], hLockIn_Results);
				SampleCalc(0, TIS_Sensor.Averages_LockIn, &SampleElec[1][0], hLockIn_Results);
				CAN_Send_MeasMode_Sweep(&hcan2, 0, &SampleElec[1][0], hLockIn_Results);

				iSweep++;
}

void MeasMode_Single(LockIn_Results_TypeDef *hLockIn_Results){
	ElecTypeDef Electrode = EMICRO;
	for (uint8_t uPre = 0; uPre < 3; uPre++) {
		HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
		DDS_set_Preset(&DDS, &TIS_Sensor.Preset[uPre].DDS_Preset);

		SampleCalcSingle(uPre, TIS_Sensor.Averages_LockIn, Electrode, hLockIn_Results);
		CAN_Send_MeasMode_Single(&hcan2, uPre, Electrode, hLockIn_Results);
	}
}

void MeasMode_Sweep_Gain(LockIn_Results_TypeDef *hLockIn_Results){

}

void MeasMode_Sweep_Phase(LockIn_Results_TypeDef *hLockIn_Results){

}

void SweepParameter(LockIn_Results_TypeDef *hLockIn_Results){
	//Sichere altes Preset:
	Preset_TypeDef Preset_Backup = TIS_Sensor.Preset[0];

	switch(TIS_Sensor.SweepParameter.uParameter[0]){
	case SWEEP_MODE_FREQ:
		for(float fSweep=TIS_Sensor.SweepParameter.fStart; fSweep<TIS_Sensor.SweepParameter.fStop;fSweep+=TIS_Sensor.SweepParameter.fStep){
			if(fSweep < PWM_FREQ_MIN
					|| fSweep > PWM_FREQ_MAX){
				break;
			}
			TIS_Sensor.Preset[0].fFreq_soll = fSweep;
			Preset_Calc(0, TIS_Sensor.Preset[0].fFreq_soll);
			DDS_set_Preset(&DDS, &TIS_Sensor.Preset[0].DDS_Preset);

			//Starte Run:
			HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
			CAN_SendFreq(&hcan2, 0);
			SampleCalc(0, TIS_Sensor.Averages_LockIn, &SampleElec[0][0], hLockIn_Results);
			SampleCalc(0, TIS_Sensor.Averages_LockIn, &SampleElec[1][0], hLockIn_Results);
		}
		break;
	case SWEEP_MODE_GAIN:
		if(TIS_Sensor.SweepParameter.uParameter[1] > 3){ //4 DDS Channel
			break;
		}
		for(float fSweep=TIS_Sensor.SweepParameter.fStart; fSweep<TIS_Sensor.SweepParameter.fStop;fSweep+=TIS_Sensor.SweepParameter.fStep){
			int16_t iGain = (int16_t)roundf(fSweep);
			if(iGain < DDS_GAIN_MIN
					|| iGain > DDS_GAIN_MAX){
				break;
			}
			TIS_Sensor.Preset[0].DDS_Preset.iDACx_Dig_Gain[TIS_Sensor.SweepParameter.uParameter[1]] = iGain;
			DDS_set_Preset(&DDS, &TIS_Sensor.Preset[0].DDS_Preset);

			//Starte Run:
			HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
			CAN_SendGain(&hcan2, 0);
			SampleCalc(0, TIS_Sensor.Averages_LockIn, &SampleElec[0][0], hLockIn_Results);
			SampleCalc(0, TIS_Sensor.Averages_LockIn, &SampleElec[1][0], hLockIn_Results);
		}
		break;
	case SWEEP_MODE_PHASE:
		if(TIS_Sensor.SweepParameter.uParameter[1] > 3){ //4 DDS Channel
			break;
		}
		for(float fSweep=TIS_Sensor.SweepParameter.fStart; fSweep<TIS_Sensor.SweepParameter.fStop;fSweep+=TIS_Sensor.SweepParameter.fStep){
			uint16_t uPhase = (uint16_t)roundf(fSweep);
			if(uPhase < DDS_PHASE_MIN
					|| uPhase > DDS_PHASE_MAX){
				break;
			}
			TIS_Sensor.Preset[0].DDS_Preset.uDDSx_Phase[TIS_Sensor.SweepParameter.uParameter[1]] = uPhase;
			DDS_set_Preset(&DDS, &TIS_Sensor.Preset[0].DDS_Preset);

			//Starte Run:
			HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
			CAN_SendPhase(&hcan2, 0);
			SampleCalc(0, TIS_Sensor.Averages_LockIn, &SampleElec[0][0], hLockIn_Results);
			SampleCalc(0, TIS_Sensor.Averages_LockIn, &SampleElec[1][0], hLockIn_Results);
		}
		break;
	default:
		break;
	}
	//Stelle altes Preset wieder her
	TIS_Sensor.Preset[0] = Preset_Backup;
	Preset_Calc(0, TIS_Sensor.Preset[0].fFreq_soll);
}

uint16_t AutoZero_FindGlobalMin_Phase(uint8_t uPreset, ElecTypeDef AutoZero_Elec[], ElecTypeDef uElec, LockIn_Results_TypeDef *hLockIn_Results, int32_t istart, int32_t istop, uint16_t istep){
	circular_buf_t buffer;
	CircularBuf_init(&buffer, 2048.0f);

	uint16_t uMin_index = 0;
	float fMinAverage = 2048;
	for(int32_t i = istart; i<= istop; i+= istep){
		TIS_Sensor.Preset[uPreset].DDS_Preset.uDDSx_Phase[Elec_to_DDS_mapping[uElec]] = i;
		DDS_set_Preset(&DDS, &TIS_Sensor.Preset[uPreset].DDS_Preset);
		CAN_SendPhase(&hcan2, uPreset);

		//Starte Messwertaufnahme:
		HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
		SampleCalc(uPreset, TIS_Sensor.Averages_LockIn, &AutoZero_Elec[0], hLockIn_Results);
		switch(uElec){				//Achtung: Auslesen der Daten ist hartgecoded.
			case EMICRO:
				CircularBuf_addValue(&buffer, hLockIn_Results->Ampl[0]);
				break;
			case E1:
				CircularBuf_addValue(&buffer, hLockIn_Results->Ampl[1]);
				break;
			case E2:
				CircularBuf_addValue(&buffer, hLockIn_Results->Ampl[2]);
				break;
			default:
				CircularBuf_addValue(&buffer, 2048);
				break;
		}

		float current_average = CircularBuf_average(&buffer);
        if (current_average < fMinAverage) {
			if(i>=istep*(AUTOZERO_MOVING_AVERAGE_SIZE/2)){
				fMinAverage = current_average;
				uMin_index = i-istep*(AUTOZERO_MOVING_AVERAGE_SIZE/2);
        	}
        }
	}

	return uMin_index;
}

void AutoZero_Phase_Refinement_Logic(uint8_t uPreset, ElecTypeDef AutoZero_Elec[], ElecTypeDef uElec, LockIn_Results_TypeDef *hLockIn_Results){
	int32_t istart = DDS_PHASE_MIN;
	int32_t istop = DDS_PHASE_MAX;
	uint16_t ustep = AUTOZERO_INITIAL_STEP_PHASE;

	uint16_t uMin_index = 0;
	uint16_t iterations = 0;

	while (ustep > 1  && iterations < AUTOZERO_MAX_ITERATIONS) {
		uMin_index = AutoZero_FindGlobalMin_Phase(uPreset, &AutoZero_Elec[0], uElec, hLockIn_Results, istart, istop, ustep);




		// Definiere neuen Bereich um das gefundene Minimum
		istart = uMin_index - 100 * ustep;
		if (istart < DDS_PHASE_MIN) istart = DDS_PHASE_MIN;
		istop = uMin_index + 100 * ustep;
		if (istop > DDS_PHASE_MAX) istop = DDS_PHASE_MAX;

		// Verkleinere den Schritt für die nächste Verfeinerung
		ustep /= AUTOZERO_REFINEMENT_FACTOR;
		// Inkrementiere die Iterationszähler
		iterations++;
	}
	//
	//	    // Letzte Verfeinerung mit Schrittweite 1, falls maximale Iterationen erreicht oder Schrittweite 1
	uMin_index = AutoZero_FindGlobalMin_Phase(uPreset, &AutoZero_Elec[0], uElec, hLockIn_Results, istart, istop, 1);
	TIS_Sensor.Preset[uPreset].DDS_Preset.uDDSx_Phase[Elec_to_DDS_mapping[uElec]] = uMin_index;
	DDS_set_Preset(&DDS, &TIS_Sensor.Preset[uPreset].DDS_Preset);
}

int16_t AutoZero_FindGlobalMin_Gain(uint8_t uPreset, ElecTypeDef AutoZero_Elec[], ElecTypeDef uElec, LockIn_Results_TypeDef *hLockIn_Results){
	circular_buf_t buffer;
	CircularBuf_init(&buffer, 2048.0f);
	float fMinAverage = 2048;

	int16_t iMin_index = 0;

	for(int16_t i = 0; i<= DDS_GAIN_MAX; i+= AUTOZERO_STEP_GAIN){
		TIS_Sensor.Preset[uPreset].DDS_Preset.iDACx_Dig_Gain[Elec_to_DDS_mapping[uElec]] = i;
		DDS_set_Preset(&DDS, &TIS_Sensor.Preset[uPreset].DDS_Preset);
		CAN_SendGain(&hcan2, uPreset);

		//Starte Messwertaufnahme:
		HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
		SampleCalc(uPreset, TIS_Sensor.Averages_LockIn, &AutoZero_Elec[0], hLockIn_Results);
		switch(uElec){				//Achtung: Auslesen der Daten ist hartgecoded.
			case EMICRO:
				CircularBuf_addValue(&buffer, hLockIn_Results->Ampl[0]);
				break;
			case E1:
				CircularBuf_addValue(&buffer, hLockIn_Results->Ampl[1]);
				break;
			case E2:
				CircularBuf_addValue(&buffer, hLockIn_Results->Ampl[2]);
				break;
			default:
				CircularBuf_addValue(&buffer, 2048);
				break;
		}

		float current_average = CircularBuf_average(&buffer);
        if (current_average < fMinAverage) {
			if(i>=AUTOZERO_STEP_GAIN*(AUTOZERO_MOVING_AVERAGE_SIZE/2)){
				fMinAverage = current_average;
				iMin_index = i-AUTOZERO_STEP_GAIN*(AUTOZERO_MOVING_AVERAGE_SIZE/2);
        	}
        }
	}
	TIS_Sensor.Preset[uPreset].DDS_Preset.iDACx_Dig_Gain[Elec_to_DDS_mapping[uElec]] = iMin_index;
	DDS_set_Preset(&DDS, &TIS_Sensor.Preset[uPreset].DDS_Preset);
	return iMin_index;
}

void AutoZero(LockIn_Results_TypeDef *hLockIn_Results){
	ElecTypeDef AutoZero_Elec[3] = {EMICRO, E1, E2};

	for (uint8_t uPreset = 0; uPreset < 3; uPreset++) {	//AutoZero für alle 3 Presets
		TIS_Sensor.Preset[uPreset].DDS_Preset.iDACx_Dig_Gain[Elec_to_DDS_mapping[AutoZero_Elec[0]]] = 2;
		TIS_Sensor.Preset[uPreset].DDS_Preset.iDACx_Dig_Gain[Elec_to_DDS_mapping[AutoZero_Elec[1]]] = 50;
		TIS_Sensor.Preset[uPreset].DDS_Preset.iDACx_Dig_Gain[Elec_to_DDS_mapping[AutoZero_Elec[2]]] = 50;
		DDS_set_Preset(&DDS, &TIS_Sensor.Preset[uPreset].DDS_Preset);


		for(ElecTypeDef uEl = EMICRO; uEl < EA; uEl++){	//AutoZero für die Elektroden EMikro,E1,E2,E3
			if (uEl == E3) {
				continue; // E3 auslassen, nicht belegt aktuell
			}
		AutoZero_Phase_Refinement_Logic(uPreset, &AutoZero_Elec[0], uEl, hLockIn_Results);
		AutoZero_FindGlobalMin_Gain(uPreset, &AutoZero_Elec[0], uEl, hLockIn_Results);
		}
	}
}

/**
  * @brief Can send and receive ID and frequency initialization
  * @param None
  * @note If no CanID is found then default ID is used which is defined in main.h
  * @retval None
  */
void InitFactory(void)
{
	//Check if Can basis ID is in Flash and use it, if not use default ID
	if(ConstantsData->canConfig.CanReceiveID != 0x00 && ConstantsData->canConfig.CanReceiveID != 0xFFFFFFFF)
	{
	  can_sensor_id = ConstantsData->canConfig.CanReceiveID;
	}
	else can_sensor_id = CAN_SENSOR_ID_DEFAULT;

	//Check if sendId and Frequency are initialized if not initialize and write to flash
	if(ConstantsData->canConfig.CanSendID == 0x00 || ConstantsData->canConfig.CanSendID == 0xFFFFFFFF)
	{
	  CAN_Config_TypeDef newConfig;
	  newConfig.CanReceiveID = can_sensor_id;
	  newConfig.CanSendID	 = can_sensor_id + 0xFF00; //only used for bootloader
	  newConfig.CAN_Freq_Preset = CAN_FREQ_PRESET;
	  BOOT_UpdateConfig(newConfig);
	}
	//Check if Delete flag is still 1 and reset
	if(ConstantsData->deleteFlag.delete_flag) BOOT_SetDeleteFlag(0);
}

void FactoryReset(void){
	//Preset 0
	TIS_Sensor.Preset[0].fFreq_soll = PRESET0_FREQ;
	int16_t preset0_gain[4] = PRESET0_GAIN;
	uint16_t preset0_phase[4] = PRESET0_PHASE;
	for(uint8_t i = 0; i<4; i++){
		TIS_Sensor.Preset[0].DDS_Preset.iDACx_Dig_Gain[i] = preset0_gain[i];
		TIS_Sensor.Preset[0].DDS_Preset.uDDSx_Phase[i] = preset0_phase[i];
	}

	//Preset 1
	TIS_Sensor.Preset[1].fFreq_soll = PRESET1_FREQ;
	int16_t preset1_gain[4] = PRESET1_GAIN;
	uint16_t preset1_phase[4] = PRESET1_PHASE;
	for(uint8_t i = 0; i<4; i++){
		TIS_Sensor.Preset[1].DDS_Preset.iDACx_Dig_Gain[i] = preset1_gain[i];
		TIS_Sensor.Preset[1].DDS_Preset.uDDSx_Phase[i] = preset1_phase[i];
	}

	//Preset 2
	TIS_Sensor.Preset[2].fFreq_soll = PRESET2_FREQ;
	int16_t preset2_gain[4] = PRESET2_GAIN;
	uint16_t preset2_phase[4] = PRESET2_PHASE;
	for(uint8_t i = 0; i<4; i++){
		TIS_Sensor.Preset[2].DDS_Preset.iDACx_Dig_Gain[i] = preset2_gain[i];
		TIS_Sensor.Preset[2].DDS_Preset.uDDSx_Phase[i] = preset2_phase[i];
	}

	TIS_Sensor.Timing_LockInData = TIME_SEND_LOCK_IN_DATA;
	TIS_Sensor.Timing_AdditionalData = TIME_SEND_ADDITIONAL_DATA;
	TIS_Sensor.Timing_MetaData = TIME_SEND_META_DATA;
	TIS_Sensor.Timing_RawData = TIME_SEND_RAW_DATA;
	TIS_Sensor.Averages_LockIn = AVERAGES_LOCK_IN;
	TIS_Sensor.MeasMode = MEASMODE_NORMAL;
	TIS_Sensor.SweepParameter.uParameter[0] = 0;
	TIS_Sensor.SweepParameter.uParameter[1] = 0;
	TIS_Sensor.SweepParameter.uParameter[2] = 0;
	TIS_Sensor.SweepParameter.fStart = 20e3;
	TIS_Sensor.SweepParameter.fStop = 1e6;
	TIS_Sensor.SweepParameter.fStep = 2e3;
	TIS_Sensor.CAN_Freq_Preset = ConstantsData->canConfig.CAN_Freq_Preset;
	ADC_Sampling.Length = ADC_C_DATA_DMA_LENGTH;
	DataWrite();

}

void InitGlobalVar(void) {

	//überprüfe, ob der Flashspeicher gesetzt wurde
	//Ungeschriebener Flash = 0x00
	uint8_t Flash_Unwritten = 1;
	for (uint32_t i = 0; i < sizeof(Flash_Data_TypeDef); i++) {
		if (*((uint8_t*) FlashData + i) != 0x00 && *((uint8_t*) FlashData + i) != 0xFF) {
			Flash_Unwritten = 0;
			break;
		}
	}

	//Falls erster Start nach Reset: Beschreibe Flash
	if (Flash_Unwritten) {
		FactoryReset();
	}

	TIS_Sensor = FlashData->TIS_Sensor;
	ADC_Sampling.Length = FlashData->ADC_Length;
	TIS_Sensor.CAN_Freq_Preset = ConstantsData->canConfig.CAN_Freq_Preset;

	//ADC_Temp_Calc_Slope_Offset(&ADC_uC_Temp_Results);

	Preset_Calc(0, TIS_Sensor.Preset[0].fFreq_soll);
	Preset_Calc(1, TIS_Sensor.Preset[1].fFreq_soll);
	Preset_Calc(2, TIS_Sensor.Preset[2].fFreq_soll);

	CAN_Set_Freq_Preset(&hcan2, TIS_Sensor.CAN_Freq_Preset);

	state_t_next = stateIdle;
}

void DataWrite(void) {

	//Erase datasector
	FLASH_eraseSector(DATA_SECTOR);

	//write TIS_sensor data
	FLASH_writeData((uint8_t *)(&FlashData->TIS_Sensor), (uint8_t*)&TIS_Sensor, sizeof(TIS_Sensor_TypeDef));

	//write ADC length
	FLASH_writeData((uint8_t *)(&FlashData->ADC_Length), (uint8_t*)(&ADC_Sampling.Length), sizeof(((ADC_Sampling_TypeDef){}).Length));
	//erklärung (ADC_Sampling_TypeDef){}).Length -> gibt die länge des members Length aus, falls sich das mal ändert

	//Update baud for bootloader
	CAN_Config_TypeDef newConfig = ConstantsData->canConfig;
	newConfig.CAN_Freq_Preset = TIS_Sensor.CAN_Freq_Preset;
	BOOT_UpdateConfig(newConfig);
}

void SoftError_Handler(AppError_t Error){
	CAN_SendError(&hcan2, Error);
}

void AS6214_Init(void) {
	AS6212_ConfigTypeDef AS6212_Config = {0};
	AS6212_Config.CR = AS6212_CR_250MS;
	AS6212_Config.SM = AS6212_SM_CONTINUOUS_CONVERSION_MODE;
	AS6212_Config.IM = AS6212_IM_COMPARATOR;
	AS6212_Config.POL = AS6212_POL_ACTIVE_LOW;
	AS6212_Config.CF = AS6212_CF_1;
	AS6212_Config.SS = AS6212_SS_NoConversionOngoing_ConversionFinished;

	AS6212_setConfig(&Temp_links, &AS6212_Config);
	AS6212_setConfig(&Temp_mitte, &AS6212_Config);
	AS6212_setConfig(&Temp_rechts, &AS6212_Config);

	AS6212_getConfig(&Temp_links, &AS6212_Config);
}

void Preset_Calc(uint8_t Preset, float fFreq){
	TIS_Sensor.Preset[Preset].fFreq_soll= fFreq;
	//Berechne Tuning Word für AD9106, Rückgabe ist-Frequenz
	TIS_Sensor.Preset[Preset].fFreq_ist = DDS_Preset_Calc(&TIS_Sensor.Preset[Preset].DDS_Preset, fFreq);
	//Erzeuge LUT für Ist-Frequenz
	LockIn_LUT_Gen(Preset);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);
	HAL_NVIC_SystemReset();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
