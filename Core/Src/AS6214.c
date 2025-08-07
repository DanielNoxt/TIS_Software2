/*
 * AS6214.c
 *
 *  Created on: Apr 28, 2024
 *      Author: D.Claassen
 */


#include "AS6214.h"


//Privat Includes
uint16_t readRegister(AS6212_HandleTypeDef *hAS6212, AS6212_RegAddress reg_addr);
void writeRegister(AS6212_HandleTypeDef *hAS6212, AS6212_RegAddress reg_addr, uint16_t data);

float AS6212_getTempC(AS6212_HandleTypeDef *hAS6212)
{

  int16_t digitalTempC = (int16_t)readRegister(hAS6212, TVAL);

  float TempC = digitalTempC * 0.0078125;

  return TempC;
}

float AS6212_getTLowC(AS6212_HandleTypeDef *hAS6212)
{
	int16_t lowTemp = (int16_t)readRegister(hAS6212, TLOW);

	float TempC_Low = lowTemp * 0.0078125;

	return TempC_Low;
}

void AS6212_setTLowC(AS6212_HandleTypeDef *hAS6212, float TlowLimit)
{
    int16_t TlowTemp = TlowLimit / 0.0078125;
    writeRegister(hAS6212, TLOW, (uint16_t)TlowTemp);
}

float AS6212_getTHighC(AS6212_HandleTypeDef *hAS6212)
{
	int16_t HighTemp = (int16_t)readRegister(hAS6212, THIGH);

	float TempC_High = HighTemp * 0.0078125;

	return TempC_High;
}

void AS6212_setTHighC(AS6212_HandleTypeDef *hAS6212, float ThighLimit)
{
    int16_t ThighTemp = ThighLimit / 0.0078125;
    writeRegister(hAS6212,THIGH, (uint16_t)ThighTemp);
}

void AS6212_getConfig(AS6212_HandleTypeDef *hAS6212, AS6212_ConfigTypeDef *hAS6212_Config)			//ungetestet
{
	// Lesen des Konfigurationsregisters
	    uint16_t configReg = readRegister(hAS6212, CONFIG);

	    // Extraktion des Bits für Alert
	    hAS6212_Config->AL = (configReg >> AS6212_CONFIG_BIT_ALERT) & 0x01;  // 1 Bit extrahieren

	    // Extraktion der Bits für Conversion Rate
	    hAS6212_Config->CR = (configReg >> AS6212_CONFIG_BIT_CONVERSION_RATE_0) & 0x03;  // 2 Bits extrahieren

	    // Extraktion des Bits für Sleep Mode
	    hAS6212_Config->SM = (configReg >> AS6212_CONFIG_BIT_SLEEP_MODE) & 0x01;  // 1 Bit extrahieren

	    // Extraktion des Bits für Interrupt Mode
	    hAS6212_Config->IM = (configReg >> AS6212_CONFIG_BIT_INTERRUPT_MODE) & 0x01;  // 1 Bit extrahieren

	    // Extraktion des Bits für Alert Polarity
	    hAS6212_Config->POL = (configReg >> AS6212_CONFIG_BIT_ALERT_POL) & 0x01;  // 1 Bit extrahieren

	    // Extraktion der Bits für Consecutive Faults
	    hAS6212_Config->CF = (configReg >> AS6212_CONFIG_BIT_CONSECUTIVE_FAULTS_0) & 0x03;  // 2 Bits extrahieren

	    // Extraktion des Bits für Single Shot
	    hAS6212_Config->SS = (configReg >> AS6212_CONFIG_BIT_SINGLE_SHOT) & 0x01;  // 1 Bit extrahieren
}

void AS6212_setConfig(AS6212_HandleTypeDef *hAS6212,AS6212_ConfigTypeDef *hAS6212_Config)		//ungetestet
{
	// Initialisiere das Konfigurationsregister
	    uint16_t configReg = 0;

	    // Setze das Bit für Alert
	    configReg |= (hAS6212_Config->AL & 0x01) << AS6212_CONFIG_BIT_ALERT;

	    // Setze die Bits für Conversion Rate
	    configReg |= (hAS6212_Config->CR & 0x03) << AS6212_CONFIG_BIT_CONVERSION_RATE_0;

	    // Setze das Bit für Sleep Mode
	    configReg |= (hAS6212_Config->SM & 0x01) << AS6212_CONFIG_BIT_SLEEP_MODE;

	    // Setze das Bit für Interrupt Mode
	    configReg |= (hAS6212_Config->IM & 0x01) << AS6212_CONFIG_BIT_INTERRUPT_MODE;

	    // Setze das Bit für Alert Polarity
	    configReg |= (hAS6212_Config->POL & 0x01) << AS6212_CONFIG_BIT_ALERT_POL;

	    // Setze die Bits für Consecutive Faults
	    configReg |= (hAS6212_Config->CF & 0x03) << AS6212_CONFIG_BIT_CONSECUTIVE_FAULTS_0;

	    // Setze das Bit für Single Shot
	    configReg |= (hAS6212_Config->SS & 0x01) << AS6212_CONFIG_BIT_SINGLE_SHOT;

	writeRegister(hAS6212, CONFIG, configReg);
}


uint16_t readRegister(AS6212_HandleTypeDef *hAS6212, AS6212_RegAddress reg_addr)
{
	//Datasheet: Figure 33: Timing Diagram for Word Read
	//16 bit registers
	uint8_t u8reg_addr = reg_addr;

	uint8_t dataBuffer[2]={0,0};
	uint16_t dataReg = 0;

	HAL_FMPI2C_Master_Transmit(hAS6212->hFMPI2C, hAS6212->I2C_ADR<<1, &u8reg_addr, 1, AS6212_I2C_TIMEOUT);
	HAL_FMPI2C_Master_Receive(hAS6212->hFMPI2C, hAS6212->I2C_ADR<<1, dataBuffer, 2, AS6212_I2C_TIMEOUT);


	dataReg = ((dataBuffer[0] << 8) | dataBuffer[1]);

	return dataReg;
}

void writeRegister(AS6212_HandleTypeDef *hAS6212, AS6212_RegAddress reg_addr, uint16_t data)
{
	//Datasheet: Figure 32: Timing Diagram for Word Write
	//16 bit registers

	uint8_t dataBuffer[3];
	dataBuffer[0] = reg_addr;
	dataBuffer[1] = (data >> 8) & 0xFF;   // Höherwertiges Byte von data
	dataBuffer[2] = data & 0xFF;          // Niederwertiges Byte von data

	HAL_FMPI2C_Master_Transmit(hAS6212->hFMPI2C, hAS6212->I2C_ADR<<1, dataBuffer, 3, AS6212_I2C_TIMEOUT);
}
