/*
 * AS6214.h
 *
 *  Created on: Apr 28, 2024
 *      Author: D.Claassen
 */

#ifndef INC_AS6214_H_
#define INC_AS6214_H_

#include "main.h"
#include "AppErrorHandling.h"

#define AS6212_I2C_TIMEOUT 100

typedef struct __AS6212_HandleTypeDef
{
	FMPI2C_HandleTypeDef *hFMPI2C;		/* I2C Interface */
	uint8_t I2C_ADR;					/* I2C Adresse */
} AS6212_HandleTypeDef;


typedef struct __AS6212_ConfigTypeDef
{
	uint8_t AL;			//Alert Bit
	uint8_t CR;			//Conversion RATE
	uint8_t SM;			//Sleep Mode
	uint8_t IM;			//Interrupt Mode
	uint8_t POL;		//Polarity
	uint8_t CF;			//Consecutive Faults
	uint8_t SS;			//Single Shot
} AS6212_ConfigTypeDef;



float AS6212_getTempC(AS6212_HandleTypeDef *hAS6212);
float AS6212_getTLowC(AS6212_HandleTypeDef *hAS6212); //ungetestet
void AS6212_setTLowC(AS6212_HandleTypeDef *hAS6212, float TlowLimit); //ungetestet
float AS6212_getTHighC(AS6212_HandleTypeDef *hAS6212); //ungetestet
void AS6212_setTHighC(AS6212_HandleTypeDef *hAS6212, float ThighLimit); //ungetestet
void AS6212_getConfig(AS6212_HandleTypeDef *hAS6212,AS6212_ConfigTypeDef *AS6212_Config); //ungetestet
void AS6212_setConfig(AS6212_HandleTypeDef *hAS6212,AS6212_ConfigTypeDef *AS6212_Config); //ungetestet

//Register kopiert von https://github.com/sparkfun/SparkFun_AS6212_Qwiic_Arduino_Library

typedef enum{

  //Internal Register Addresses
  TVAL      =       0x0,    //Temperature Register
  CONFIG    =       0x1,    //Configuration Register
  TLOW      =       0x2,    //Low Temperature Threshold
  THIGH     =       0x3,    //High Temperature Threshold

  //Helpful preset definitions for configuration register
  DEFAULTM  =       0x40A0,   //Default state
  SLEEPMODE =       0x41A0,   //Sleep Mode
  SLEEPSS   =       0xC1A0,   //Sleep Mode Single Shot
}AS6212_RegAddress;


//16 Bit Config Register
//0-4 bit: Reserved (Read Only)
#define AS6212_CONFIG_BIT_ALERT 5
#define AS6212_CONFIG_BIT_CONVERSION_RATE_0 6
#define AS6212_CONFIG_BIT_CONVERSION_RATE_1 7
#define AS6212_CONFIG_BIT_SLEEP_MODE 8
#define AS6212_CONFIG_BIT_INTERRUPT_MODE 9
#define AS6212_CONFIG_BIT_ALERT_POL 10
#define AS6212_CONFIG_BIT_CONSECUTIVE_FAULTS_0 11
#define AS6212_CONFIG_BIT_CONSECUTIVE_FAULTS_1 12
#define AS6212_CONFIG_BIT_SINGLE_SHOT 15

//Conversion Rate Bits in ms
#define AS6212_CR_125MS 3
#define AS6212_CR_250MS 2
#define AS6212_CR_1000MS 1
#define AS6212_CR_4000MS 0

//Sleep Mode Configuration
#define AS6212_SM_CONTINUOUS_CONVERSION_MODE 0
#define AS6212_SM_SLEEP_MODE 1

//Interrupt Mode Configuration
#define AS6212_IM_COMPARATOR 0
#define AS6212_IM_INTERRUPT 1

//Polarity Bit Configuration
#define AS6212_POL_ACTIVE_HIGH 1
#define AS6212_POL_ACTIVE_LOW 0

//Consecutive Faults Bit Settings
#define AS6212_CF_1 0
#define AS6212_CF_2 1
#define AS6212_CF_3 2
#define AS6212_CF_4 3

//Single Shot Conversion Bit Settings
#define AS6212_SS_NoConversionOngoing_ConversionFinished 0
#define AS6212_SS_StartSingleShot_ConversionOngoing 1

#endif /* INC_AS6214_H_ */
