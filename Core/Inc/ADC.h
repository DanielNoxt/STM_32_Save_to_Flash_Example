/*
 * ADC.h
 *
 *  Created on: Jun 29, 2023
 *      Author: D.Claassen
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#include "AppErrorHandling.h"

float ADC_val_to_voltage(uint32_t uVal);

AppError_t ADC_ReadTemp(ADC_Temp_ResultsTypeDef *hTemp);
AppError_t ADC_Vbat(float *hVbat);

AppError_t ADC_Temp_Calc_Slope_Offset(ADC_Temp_ResultsTypeDef *hTemp);

AppError_t ADC_Sample_Elec_poll(void);
AppError_t ADC_C_DMAGetVal(uint8_t channel, uint32_t Number, uint16_t *val);

//Privat
void ADC_Elec_Init(uint32_t ADC1_Channel, uint32_t ADC2_Channel, uint32_t ADC3_Channel);

//Debugging
void ADC_Sort_Channels(void);


#endif /* INC_ADC_H_ */
