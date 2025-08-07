/*
 * LockIn.c
 *
 *  Created on: 11.03.2023
 *      Author: D.Claassen
 */


#include "LockIn.h"
#include "Math.h"

//AppError_t LockIn_Set_Freq(uint8_t Ch, float freq){
//	//Setze Sollfrequenz und berechne den bestmöglichen TimerPrescaler
//	TIS_Sensor.PWM_freq_soll[Ch] = freq;
//	LockIn_Preset[Ch].PWM_Period = SinGen_Calc_Prescaler(freq);
//	//Berechne tatsächliche Frequenz und generiere LUT
//	TIS_Sensor.PWM_freq_ist[Ch] = APB2_TIMER_FREQ/((LockIn_Preset[Ch].PWM_Period+1)*(TIM8->ARR+1));
//	LockIn_LUT_Gen(Ch);
//	return APP_ERROR_OK;
//}

AppError_t LockIn_LUT_Gen(uint8_t Preset){
	float t = 0;
	for(uint32_t i=0; i<ADC_C_DATA_DMA_MAX_LENGTH; i++){
		t = i * ADC_SAMPLE_TIME;
		LockIn_Preset[Preset].Vr[i] = (ADC_RANGE_MAX*LOCK_IN_AMP_REF) * cos(2*M_PI*t*TIS_Sensor.Preset[Preset].fFreq_ist);
		LockIn_Preset[Preset].Vr_90[i] = (ADC_RANGE_MAX*LOCK_IN_AMP_REF) * sin(2*M_PI*t*TIS_Sensor.Preset[Preset].fFreq_ist);
	}

	return APP_ERROR_OK;
}


AppError_t LockIn_CalcAdd(uint8_t Preset, LockIn_Results_TypeDef *hLockIn_Results){

	int32_t Sum_Y[3] = {0};
	int32_t Sum_Y_90[3] = {0};

	uint32_t k=0;
	for(uint32_t i=0; i<ADC_Sampling.Length; i++){
		for(uint8_t j= 0; j<3 ; j++){
			Sum_Y[j] = Sum_Y[j]+LockIn_Preset[Preset].Vr[i]*(ADC_Sampling.Data_DMA[k]-ADC_RANGE_MAX);
			Sum_Y_90[j] = Sum_Y_90[j]+LockIn_Preset[Preset].Vr_90[i]*(ADC_Sampling.Data_DMA[k]-ADC_RANGE_MAX);
			k++;
		}
	}

	float y_f[3] = {0};
	float y_90_f[3] = {0};
	for(uint8_t i= 0; i<3 ; i++){
		y_f[i] = Sum_Y[i] / (float)ADC_Sampling.Length;
		y_90_f[i] = Sum_Y_90[i] / (float)ADC_Sampling.Length;

		hLockIn_Results->Ampl[i] += 2*sqrtf((y_f[i] * y_f[i]+y_90_f[i] * y_90_f[i]))/(ADC_RANGE_MAX * LOCK_IN_AMP_REF);
		hLockIn_Results->Phase[i] += atan2f(y_f[i],y_90_f[i]);
//		hLockIn_Results->Ampl[i] += y_f[i];
//		hLockIn_Results->Phase[i] += y_90_f[i];
	}

	return 0;
}

void Lock_In_Results_Reset(LockIn_Results_TypeDef *hLockIn_Results){
	for(uint8_t i = 0; i < 3; i++){
		hLockIn_Results->Ampl[i] = 0;
		hLockIn_Results->Phase[i] = 0;
	}
}

void Lock_In_FillADC(uint8_t Preset, uint32_t length){
	ADC_Sampling.Length = length;
	uint32_t k=0;
	for(uint32_t i=0; i<length; i++){
		float t = i * ADC_SAMPLE_TIME;
		ADC_Sampling.Data_DMA[k] = ADC_RANGE_MAX + ADC_RANGE_MAX*cos(2*M_PI*t*TIS_Sensor.Preset[Preset].fFreq_ist+0);
		k++;
		ADC_Sampling.Data_DMA[k] = ADC_RANGE_MAX + ADC_RANGE_MAX*cos(2*M_PI*t*TIS_Sensor.Preset[Preset].fFreq_ist+M_PI_4);
		k++;
		ADC_Sampling.Data_DMA[k] = ADC_RANGE_MAX + ADC_RANGE_MAX*cos(2*M_PI*t*TIS_Sensor.Preset[Preset].fFreq_ist+M_PI_2);
		k++;
	}
}


