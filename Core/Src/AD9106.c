/*
 * AD9106.c
 *
 *  Created on: Apr 29, 2024
 *      Author: D.Claassen
 */


#include "AD9106.h"

#include <math.h>

/* Most SPI accessible registers are double buffered. An active register set controls operation of the
 * AD9106 during pattern generation. A set of shadow registers stores updated register values.
 * Register updates can be written at any time, and when the configuration update is complete,
 * a 1 is written to the update bit in the RAMUPDATE register.
 * The update bit arms the register set for transfer from shadow registers to active registers.
 * The AD9106 performs this transfer automatically the next time the pattern generator is off.
 * This procedure does not apply to the 4096 × 12-bit SRAM.
 */

/* Transfer:
 * 1. Set CS
 * 2. 16 bit Command Word
 * 		2. a) DB15 (first transfered Bit): read/write indicator (high for read, low for write)
 * 		2. b) DB14 - DB0; Adress-Bits A14 - A0
 * 3. 16 bit Data Word
 * 4. Multiple consecutive register write/read operations if CS stays low beyond the first data-word:
 * 		4. a) 0x00 to 0x60 address space, SPI port automatically increments the register
 * 		4. b) 0x6FFF address space, the address autodecrements
 */

//			https://os.mbed.com/teams/AnalogDevices/code/EVAL-AD910x/
//			uint16_t reg_add[66]=				{0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x000e, 0x001f, 0x0020, 0x0022, 0x0023, 0x0024, 0x0025, 0x0026, 0x0027, 0x0028, 0x0029, 0x002a, 0x002b, 0x002c, 0x002d, 0x002e, 0x002f, 0x0030, 0x0031, 0x0032, 0x0033, 0x0034, 0x0035, 0x0036, 0x0037, 0x003e, 0x003f, 0x0040, 0x0041, 0x0042, 0x0043, 0x0044, 0x0045, 0x0047, 0x0050, 0x0051, 0x0052, 0x0053, 0x0054, 0x0055, 0x0056, 0x0057, 0x0058, 0x0059, 0x005a, 0x005b, 0x005c, 0x005d, 0x005e, 0x005f, 0x001e, 0x001d};
//		4 Gaussian Pulses with Different Start Delays and Digital Gain Settings
//			uint16_t AD9106_example1_regval[66]={0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x3030, 0x3030, 0x0111, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x2000, 0x2000, 0x4000, 0x0001, 0x0200, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07d0, 0x0000, 0xfff0, 0x0100, 0x03e8, 0x0000, 0xfff0, 0x0100, 0x0bb8, 0x0000, 0xfff0, 0x0100, 0x0fa0, 0x0000, 0xfff0, 0x0100, 0x0001, 0x0001};
//		4 Pulses Generated from an SRAM Vector
//			uint16_t AD9106_example2_regval[66]={0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x3030, 0x3030, 0x0111, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x2000, 0x2000, 0x4000, 0x0001, 0x0200, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07d0, 0xc000, 0xfff0, 0x0100, 0x03e8, 0x8000, 0xbff0, 0x0100, 0x0bb8, 0x3ff0, 0x7ff0, 0x0100, 0x0fa0, 0x0000, 0x3ff0, 0x0100, 0x0001, 0x0001};
//		4 Pulsed DDS-Generated Sine Waves with Different Start Delays and Digital Gain Settings
//			uint16_t AD9106_example3_regval[66]={0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x3232, 0x3232, 0x0111, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x2000, 0x2000, 0x4000, 0x0001, 0x0200, 0x0a3d, 0x7100, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07d0, 0x0000, 0x0000, 0x0100, 0x03e8, 0x0000, 0x0000, 0x0100, 0x0bb8, 0x0000, 0x0000, 0x0100, 0x0fa0, 0x0000, 0x0000, 0x0100, 0x0001, 0x0001};
//		Pulsed DDS-Generated Sine Wave and 3 Sawtooth Generator Waveforms
//			uint16_t AD9106_example4_regval[66]={0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x1212, 0x1232, 0x0121, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x1011, 0x0600, 0x1999, 0x9a00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07d0, 0x0000, 0x0000, 0x0001, 0x03e8, 0x0000, 0x0000, 0x0001, 0x03e8, 0x0000, 0x0000, 0x0001, 0x0fa0, 0x0000, 0x0000, 0x16ff, 0x0001, 0x0001};
//		Pulsed DDS-Generated Sine Waves Amplitude-Modulated by an SRAM Vector
//			uint16_t AD9106_example5_regval[66]={0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x3333, 0x3333, 0x0111, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0001, 0x0200, 0x0750, 0x7500, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07d0, 0x0000, 0xfff0, 0x0100, 0x03e8, 0x0000, 0xfff0, 0x0100, 0x0bb8, 0x0000, 0xfff0, 0x0100, 0x0fa0, 0x0000, 0xfff0, 0x0100, 0x0001, 0x0001};
//		DDS-Generated Sine Wave and 3 Sawtooth Waveforms
//			uint16_t AD9106_example6_regval[66]={0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x1212, 0x1232, 0x0111, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0001, 0x7e00, 0x0750, 0x7500, 0x0000, 0x0000, 0x0000, 0x0000, 0x0002, 0x0000, 0x0000, 0x2710, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000, 0x0000, 0x0001, 0x1770, 0x0000, 0x0000, 0x0001, 0x0fa0, 0x0000, 0x0000, 0x7fff, 0x0001, 0x0001};
//
//		Write Example:
//			for(uint16_t i=0;i<64;i++){
//				if(reg_add[i]>=0x00){
//					AD9106_writeRegister(&DDS, reg_add[i], AD9106_example4_regval[i]);
//				}
//			}


//		//Reading all Reg for debugging
//		uint16_t reg[100];
//		for(uint8_t i = 0; i<100; i++){
//			reg[i]=0xFFFF;
//		}
//		AD9106_read_n_Registers(&DDS, 99, 100, reg);
//		// Array umdrehen
//		uint16_t start = 0;
//		uint16_t end = 99;
//		uint16_t temp = 0;
//		while (start < end) {
//			// Elemente tauschen
//			temp = reg[start];
//			reg[start] = reg[end];
//			reg[end] = temp;
//
//			// Indizes aktualisieren
//			start++;
//			end--;
//		}





uint16_t AD9106_readRegister(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr);
void AD9106_writeRegister(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data);
void AD9106_read_n_Registers(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data_size, uint16_t* reg_data);
void AD9106_write_n_Registers(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data_size, uint16_t* reg_data);


void AD9106_Init(AD9106_HandleTypeDef *hAD9106){
	//Initial values of IOs
	HAL_GPIO_WritePin(hAD9106->Reset_Port, hAD9106->Reset_Pin, GPIO_PIN_RESET);		//Reset Chip

	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(hAD9106->Trigger_Port, hAD9106->Trigger_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(hAD9106->Reset_Port, hAD9106->Reset_Pin, GPIO_PIN_SET);
}

void AD9106_set_waveform_sinus(AD9106_HandleTypeDef *hAD9106){
	uint16_t config[2];

	//Wave1/Wave2 Select Register (WAV2_1CONFIG, Address 0x27)
	config[0] = (3<<PRESTORE_SEL2_POS)|(1<<WAVE_SEL2_POS)|(3<<PRESTORE_SEL1_POS)|(1<<WAVE_SEL1_POS); //DDSx output, restored waveform modulated by waveform from RAM

	//Wave3/Wave4 Select Register (WAV4_3CONFIG, Address 0x26)
	config[1] = (3<<PRESTORE_SEL4_POS)|(1<<WAVE_SEL4_POS)|(3<<PRESTORE_SEL3_POS)|(1<<WAVE_SEL3_POS); //DDSx output, restored waveform modulated by waveform from RAM

	AD9106_write_n_Registers(hAD9106, WAV2_1CONFIG, 2, config);
}

void AD9106_set_DACx_digital_gain(AD9106_HandleTypeDef *hAD9106, int16_t* iDACx_Dig_Gain){
	//12 Bit Register --> Values 2047 to -2048
	uint16_t DGain[4];
	for(uint8_t i=0;i<4;i++){
		DGain[i] = (uint16_t)(iDACx_Dig_Gain[i]<<DACx_DIG_GAIN_POS);
	}
	AD9106_write_n_Registers(hAD9106, DAC1_DGAIN, 4, DGain);
}

void AD9106_set_DACx_digital_gain_float(AD9106_HandleTypeDef *hAD9106, float* fDACx_Dig_Gain){
	//12 Bit Register --> Values +2 to -2
	int16_t iGain[4];

	for(uint8_t i=0;i<4;i++){
		iGain[i] = (int16_t)roundf(fDACx_Dig_Gain[i]*1024);

	    if (iGain[i] > 2047) iGain[i] = 2047;
	    if (iGain[i] < -2048) iGain[i] = -2048;
	}

	AD9106_set_DACx_digital_gain(hAD9106, iGain);
}

void AD9106_set_DSS_Freq_in_Hz(AD9106_HandleTypeDef *hAD9106, float fDSS_Freq){
	//The DDS output frequency is DDS_TW × fCLKP/CLKN/2^24.
	union{
		uint32_t u32_DDS_TW;
		uint16_t u16_DDS_TW[2];
	}u32_u16;

	float fDDS_TW = (fDSS_Freq/(float)AD9106_F_CLK)*(1<<24);
	u32_u16.u32_DDS_TW = (uint32_t)roundf(fDDS_TW);

	if(u32_u16.u32_DDS_TW<=0xFFFFFF){
		u32_u16.u32_DDS_TW <<= DDSTW_LSB_POS;
		AD9106_write_n_Registers(hAD9106, DDS_TW1, 2, u32_u16.u16_DDS_TW);
	}else{
		//Error, Zahlt passt nicht ins Register
	}
}

void AD9106_set_DSS_Freq(AD9106_HandleTypeDef *hAD9106, uint32_t uDSS_Freq){
	//The DDS output frequency is DDS_TW × fCLKP/CLKN/2^24.
	union{
		uint32_t u32_DDS_TW;
		uint16_t u16_DDS_TW[2];
	}u32_u16;

	u32_u16.u32_DDS_TW = uDSS_Freq;

	if(u32_u16.u32_DDS_TW<=0xFFFFFF){
		u32_u16.u32_DDS_TW <<= DDSTW_LSB_POS;
		AD9106_write_n_Registers(hAD9106, DDS_TW1, 2, u32_u16.u16_DDS_TW);
	}else{
		//Error, Zahlt passt nicht ins Register
	}
}

void AD9106_set_DSS_Phase_in_degrees(AD9106_HandleTypeDef *hAD9106, float* fDSS_Phase){
	//DDSx phase offset for each DACx data path has a range of 360° and a resolution of 360°/(2^16 − 1).
	uint16_t uDDSx_Phase[4];
	for(uint8_t i=0;i<4;i++){
		uDDSx_Phase[i] = (uint16_t) roundf(((1<<16)-1) * fDSS_Phase[i]/360);
	}
	AD9106_write_n_Registers(hAD9106, DDS1_PW, 4, uDDSx_Phase);
}

void AD9106_set_DSS_Phase(AD9106_HandleTypeDef *hAD9106, uint16_t* uDSS_Phase){
	AD9106_write_n_Registers(hAD9106, DDS1_PW, 4, uDSS_Phase);
}

void AD9106_Reset_IOUTFSx_calibration(AD9106_HandleTypeDef *hAD9106){
	//To reset the calibration, pulse the CAL_RESET bit in Register 0x0D (CALCONFIG) to Logic 1 and Logic 0
	uint16_t data = CAL_RESET;
	AD9106_writeRegister(hAD9106, CALCONFIG, data);
	data = 0;
	AD9106_writeRegister(hAD9106, CALCONFIG, data);
}

void AD9106_IOUTFSx_calibration(AD9106_HandleTypeDef *hAD9106){
	uint16_t uCALCONFIG = 0;
	uint8_t uCOMP_CAL_RNG = 0;	//Offset calibration range. 2 bit

	//DEBUG
	//uCALCONFIG = AD9106_readRegister(hAD9106, CALCONFIG);

	//	1. Set the calibration ranges in Register 0x08 (DACxRANGE), Bits[7:0], and Register 0x0D (CALCONFIG), Bits[5:4] to their minimum values to allow best calibration.
	AD9106_writeRegister(hAD9106, DACxRANGE, (uint16_t) 0);
	AD9106_writeRegister(hAD9106, CALCONFIG, uCALCONFIG);

	//	2.Enable the calibration clock bit, CAL_CLK_EN, inRegister 0x0D.
	uCALCONFIG |= CAL_CLK_EN;
	AD9106_writeRegister(hAD9106, CALCONFIG, uCALCONFIG);

	//	3.Set the divider ratio for the calibration clock by setting CAL_CLK_DIV bits in Register 0x0D. The default is 512. The frequency of the calibration clock must be less than 500 kHz.
	// https://ez.analog.com/data_converters/high-speed_dacs/f/q-a/164308/ad9106-cal_clk_div-and-bgdr-bits
	//uCALCONFIG |= CAL_CLK_DIV;		//Set CAL_CLK_DIV to 7: calibration clock = (2^CAL_CLK_DIV)*4

//	4.Set the CAL_MODE_EN bit in Register 0x0D (CALCONFIG) to Logic 1.
	uCALCONFIG |= CAL_MODE_EN;
	AD9106_writeRegister(hAD9106, CALCONFIG, uCALCONFIG);

	uint8_t i = 0;
	do{
		//	5.Set the START_CAL bit in Register 0x0E to Logic 1 to begin the calibration of the comparator, RSETx, and gain.
		AD9106_writeRegister(hAD9106, COMPOFFSET, (uint16_t)START_CAL);

		//	6.The CAL_MODE flag in Register 0x0D goes to Logic 1 while the device is calibrating. The CAL_FIN flag in Register 0x0E (COMPOFFSET) goes to Logic 1 when the calibration is complete.
		uint16_t calibrationComplete = 0, timeout = 0;
		do{
			HAL_Delay(1);
			timeout++;
			calibrationComplete = AD9106_readRegister(hAD9106, COMPOFFSET) & CAL_FIN;

		}while(timeout<500 && !calibrationComplete);

		//	7.Set the START_CAL bit in Register 0x0E (COMPOFFSET) to Logic 0.
		//AD9106_readRegister(hAD9106, COMPOFFSET);
		AD9106_writeRegister(hAD9106, COMPOFFSET, (uint16_t) 0);
		//AD9106_readRegister(hAD9106, COMPOFFSET);

		//	8.After calibration, verify that the overflow and underflowflags in Register 0x0D are not set (Bits[14:9]). If they are,change the corresponding calibration range to the nextlarger range and begin again at Step 5.
		uCALCONFIG = AD9106_readRegister(hAD9106, CALCONFIG);
		if(uCALCONFIG & (GAIN_CAL_UF | GAIN_CAL_OF | RSET_CAL_UF | RSET_CAL_OF | COMP_OFFSET_UF | COMP_OFFSET_OF)){		//Gain calibration value overflow or underflow.
			uCOMP_CAL_RNG++;
			uCALCONFIG &= ~COMP_CAL_RNG;
			uCALCONFIG |= uCOMP_CAL_RNG<<COMP_CAL_RNG_POS;
			AD9106_writeRegister(hAD9106, CALCONFIG, uCALCONFIG);
			i++;
		}
		else{
			//Leave do While:
			i=100;
		}
	}while(i<4);


	//	9.If no flag is set, read the DACx_RSET_CAL, Bits[12:8], and DACx_GAIN_CAL values in the DACxRSET and DACxAGAIN registers, respectively, and write them into their corresponding DACxRSET and DACxAGAIN registers.
	//Store DACx_RSET_CAL to DACxRSET in Register DACxRSET
	uint16_t uDACx_val[4] = {0};
	AD9106_read_n_Registers(hAD9106, DAC1RSET,4,uDACx_val);
	for(uint8_t i=0;i<4;i++){
		uDACx_val[i] = (uDACx_val[i] & DACx_RSET_CAL)>>DACx_RSET_CAL_POS;
		//uDACx_val[i] |= (1<<DACx_RSET_EN_POS);
	}
	AD9106_write_n_Registers(hAD9106, DAC1RSET,4,uDACx_val);

	//Store DACx_GAIN_CAL to DACxAGAIN in Register DACxAGAIN
	AD9106_read_n_Registers(hAD9106, DAC1AGAIN,4,uDACx_val);
	for(uint8_t i=0;i<4;i++){
		uDACx_val[i] = (uDACx_val[i] & DACx_GAIN_CAL)>>DACx_GAIN_CAL_POS;
	}
	AD9106_write_n_Registers(hAD9106, DAC1AGAIN,4,uDACx_val);

	//	10.Reset the CAL_MODE_EN bit and the calibration clock bit, CAL_CLK_EN, in Register 0x0D (CALCONFIG) to Logic 0 to disable the calibration clock.
	uCALCONFIG &=  ~(CAL_MODE_EN | CAL_CLK_EN);
	AD9106_writeRegister(hAD9106, CALCONFIG, uCALCONFIG);
	//	11.Set the CAL_MODE_EN bit in Register 0x0D (CALCONFIG) to Logic 0 to set the RSETx and gain control muxes to normal operationmode.
	AD9106_writeRegister(hAD9106, CALCONFIG, uCALCONFIG);
	//	12.Disable the calibration clock bit, CAL_CLK_EN, inRegister 0x0D (CALCONFIG).
	AD9106_writeRegister(hAD9106, CALCONFIG, uCALCONFIG);
	AD9106_RAM_Update(hAD9106);
}

void AD9106_RAM_Update(AD9106_HandleTypeDef *hAD9106){
	AD9106_writeRegister(hAD9106, RAMUPDATE, 1<<RAMPUPDATE_POS);
}

void AD9106_read_n_Registers(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data_size, uint16_t* reg_data)
{
	reg_addr |= (1<<15); //read indicator
	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hAD9106->hspi, (uint8_t*) &reg_addr, 1, AD9106_SPI_TIMEOUT);
	HAL_SPI_Receive(hAD9106->hspi, (uint8_t*) reg_data, reg_data_size, AD9106_SPI_TIMEOUT);
	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_SET);
}

void AD9106_write_n_Registers(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data_size, uint16_t* reg_data)
{
	reg_addr &= ~(1<<15); //write indicator
	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hAD9106->hspi, (uint8_t*) &reg_addr, 1, AD9106_SPI_TIMEOUT);
	HAL_SPI_Transmit(hAD9106->hspi, (uint8_t*) reg_data, reg_data_size, AD9106_SPI_TIMEOUT);
	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_SET);
}

uint16_t AD9106_readRegister(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr)
{
	uint16_t reg_data = 0;
	reg_addr |= (1<<15); //read indicator
	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hAD9106->hspi, (uint8_t*) &reg_addr, 1, AD9106_SPI_TIMEOUT);
	HAL_SPI_Receive(hAD9106->hspi, (uint8_t*) &reg_data, 1, AD9106_SPI_TIMEOUT);
	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_SET);
	return reg_data;
}

void AD9106_writeRegister(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data)
{
	uint16_t transmit_data[2];	//Zum Übertragen der Daten mit einem HAL Aufruf (HAL Aufruf kostet viel Zeit)
	reg_addr &= ~(1<<15); //write indicator
	transmit_data[0] = reg_addr;
	transmit_data[1] = reg_data;

	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hAD9106->hspi, (uint8_t*) transmit_data, 2, AD9106_SPI_TIMEOUT);
	HAL_GPIO_WritePin(hAD9106->CS_Port, hAD9106->CS_Pin, GPIO_PIN_SET);
}
