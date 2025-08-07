/*
 * DDS.h
 *
 *  Created on: May 4, 2024
 *      Author: D.Claassen
 */

#ifndef INC_DDS_H_
#define INC_DDS_H_

#include "AD9106.h"


void DDS_Init(AD9106_HandleTypeDef *hAD9106);
float DDS_Calc_Freq(float fFreq);
float DDS_get_Freq(AD9106_SinusSettings_TypeDef *hSinusSettings);
float DDS_Preset_Calc(AD9106_SinusSettings_TypeDef *hSinusSettings, float fFreq);
void DDS_set_Preset(AD9106_HandleTypeDef *hAD9106, AD9106_SinusSettings_TypeDef *hSinusSettings);

#endif /* INC_DDS_H_ */
