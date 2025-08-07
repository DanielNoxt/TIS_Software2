/*
 * DDS.c
 *
 *  Created on: May 4, 2024
 *      Author: D.Claassen
 */

#include "DDS.h"
#include "Math.h"


void DDS_Init(AD9106_HandleTypeDef *hAD9106){
	uint16_t uConfig_Reg[4];

	AD9106_Init(hAD9106);

	//DAC1 Analog Gain Register (DAC1AGAIN, Address 0x01)
	uConfig_Reg[0] = 0x4000;
	//DAC2 Analog Gain Register (DAC2AGAIN, Address 0x02)
	uConfig_Reg[1] = 0x4000;
	//DAC3 Analog Gain Register (DAC3AGAIN, Address 0x03)
	uConfig_Reg[2] = 0x4000;
	//DAC4 Analog Gain Register (DAC4AGAIN, Address 0x04)
	uConfig_Reg[3] = 0x4000;
	AD9106_write_n_Registers(hAD9106, DAC1AGAIN, 4, &uConfig_Reg[0]);

	//Command/Status Register (PAT_STATUS, Address 0x1E)
	AD9106_writeRegister(hAD9106, PAT_STATUS, 1<<RUN_POS);

	AD9106_set_waveform_sinus(hAD9106);

	int16_t iDACx_DGain[4] = {0,0,0,0};
	AD9106_set_DACx_digital_gain(hAD9106, &iDACx_DGain[0]);

	AD9106_set_DSS_Freq_in_Hz(hAD9106, 1e3);

	uConfig_Reg[0] = 0x0;
	uConfig_Reg[1] = 0x0;
	uConfig_Reg[2] = 0x0;
	uConfig_Reg[3] = 0x0;
	AD9106_set_DSS_Phase(hAD9106, &uConfig_Reg[0]);


	AD9106_RAM_Update(hAD9106);
}

float DDS_Calc_Freq(float fFreq){
	uint32_t DDS_TW = (uint32_t)roundf((fFreq/(float)AD9106_F_CLK)*(1<<24));
	float fFreq_ist = (float)AD9106_F_CLK * DDS_TW / (1<<24);
	return fFreq_ist;
}


float DDS_get_Freq(AD9106_SinusSettings_TypeDef *hSinusSettings){
	float fFreq_ist = (float)AD9106_F_CLK * hSinusSettings->u32_DDS_TW / (1<<24);
	return fFreq_ist;
}

float DDS_Preset_Calc(AD9106_SinusSettings_TypeDef *hSinusSettings, float fFreq){
	float fFreq_ist = -1;
	hSinusSettings->u32_DDS_TW = (uint32_t)roundf((fFreq/(float)AD9106_F_CLK)*(1<<24));
	fFreq_ist = DDS_get_Freq(hSinusSettings);

	return fFreq_ist;
}

void DDS_set_Preset(AD9106_HandleTypeDef *hAD9106, AD9106_SinusSettings_TypeDef *hSinusSettings){

	AD9106_set_DSS_Freq(hAD9106, hSinusSettings->u32_DDS_TW);
	AD9106_set_DACx_digital_gain(hAD9106, &hSinusSettings->iDACx_Dig_Gain[0]);
	AD9106_set_DSS_Phase(hAD9106, &hSinusSettings->uDDSx_Phase[0]);

	AD9106_RAM_Update(hAD9106);
}
