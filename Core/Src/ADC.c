/*
 * ADC.c
 *
 *  Created on: Jun 29, 2023
 *      Author: D.Claassen
 */


#include "ADC.h"

/**
 * @brief  Rechne ADC_val in Spannung um
 * @param  ADC-Rohwert als uint32_t
 * @retval Spannung in Volt
 */
float ADC_val_to_voltage(uint32_t uVal) {
	return uVal * ADC_VRef / (float) ((1 << ADC_RES) - 1);
}


/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hTemp pointer zur ADC_Temp_ResultsTypeDef structure, beinhaltet Offset, Slope und das Endergebniss
 * @retval APP_ERROR_OK
 */
AppError_t ADC_ReadTemp(ADC_Temp_ResultsTypeDef *hTemp) {
	ADC123_COMMON->CCR |= ADC_CCR_TSVREFE;
	ADC_HandleTypeDef hadcT;
	hadcT.Instance = ADC1;
	hadcT.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadcT.Init.Resolution = ADC_RESOLUTION_12B;
	hadcT.Init.ScanConvMode = DISABLE;
	hadcT.Init.ContinuousConvMode = DISABLE;
	hadcT.Init.DiscontinuousConvMode = DISABLE;
	hadcT.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadcT.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadcT.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadcT.Init.NbrOfConversion = 1;
	hadcT.Init.DMAContinuousRequests = DISABLE;
	hadcT.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadcT) != HAL_OK) {
		Error_Handler();
	}

	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;		// Samplingtime>Tmin --> n > Tmin*fADC
	if (HAL_ADC_ConfigChannel(&hadcT, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	HAL_ADC_Start(&hadcT);
	HAL_StatusTypeDef Error = HAL_ADC_PollForConversion(&hadcT, 10);	//Warte auf Wandlung

	uint32_t uADC = 0;
	if(Error == HAL_OK){
		uADC = HAL_ADC_GetValue(&hadcT);
	}
	//	  float Temp_ohneCalibrierung = ((ADC_val_to_voltage(uADC) - 0.76)/2.5E-3)+25;		//Average slope: 2.5 mV/°C, V25(1) Voltage at 25 °C: 0.76 V
	hTemp->Temp = hTemp->Slope * uADC + hTemp->Offset;
	return APP_ERROR_OK;
}

/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
AppError_t ADC_Vbat(float *hVbat){
	ADC_HandleTypeDef hadcT;
	hadcT.Instance = ADC1;
	hadcT.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadcT.Init.Resolution = ADC_RESOLUTION_12B;
	hadcT.Init.ScanConvMode = DISABLE;
	hadcT.Init.ContinuousConvMode = DISABLE;
	hadcT.Init.DiscontinuousConvMode = DISABLE;
	hadcT.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadcT.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadcT.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadcT.Init.NbrOfConversion = 1;
	hadcT.Init.DMAContinuousRequests = DISABLE;
	hadcT.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadcT) != HAL_OK) {
		Error_Handler();
	}

	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADCx_CHANNEL[V_BATT_FB];
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadcT, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	HAL_ADC_Start(&hadcT);
	HAL_StatusTypeDef Error = HAL_ADC_PollForConversion(&hadcT, 10);	//Warte auf Wandlung

	uint32_t uADC = 0;
	if(Error == HAL_OK){
		uADC = HAL_ADC_GetValue(&hadcT);
	}

	*hVbat = uADC * ADC_VRef * ((ADC_V_BATT_R1+ADC_V_BATT_R2)/ADC_V_BATT_R1)/ (float) ((1 << ADC_RES) - 1);
	return APP_ERROR_OK;
}

/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
AppError_t ADC_Temp_Calc_Slope_Offset(ADC_Temp_ResultsTypeDef *hTemp) {
	hTemp->Temp = 0;
	float TS_CAL1 = (int32_t) *TEMPSENSOR_CAL1_ADDR
			* (((float) TEMPSENSOR_CAL_VREFANALOG / 1000) / ADC_VRef );
	float TS_CAL2 = (int32_t) *TEMPSENSOR_CAL2_ADDR
			* (((float) TEMPSENSOR_CAL_VREFANALOG / 1000) / ADC_VRef );
	hTemp->Slope = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)
					/ (TS_CAL2 - TS_CAL1);
	hTemp->Offset = TEMPSENSOR_CAL1_TEMP - hTemp->Slope * TS_CAL1;

	return APP_ERROR_OK;
}



/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
AppError_t ADC_Sample_Elec_poll(void) {
	ADC_Elec_Init(ADCx_CHANNEL[ADC_C_Results.Elec[0]], ADCx_CHANNEL[ADC_C_Results.Elec[1]],
			ADCx_CHANNEL[ADC_C_Results.Elec[2]]);

	ADC_C_Results.readIndex.Channel = 0;
	ADC_C_Results.readIndex.Pos = 0;
	ADC_C_Results.ConCplet = 1;
	ADC_C_Results.TimerSync = 1;
	__HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
	while(ADC_C_Results.TimerSync != 0);			//Warte bis der Timer wieder bei 0 losläuft
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) &ADC_C_Results.Data_DMA[0],
			3*ADC_C_Results.Length);
	while (ADC_C_Results.ConCplet != 0);   //Warte auf ADC

	return APP_ERROR_OK;
}


/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
AppError_t ADC_C_DMAGetVal(uint8_t ADC_Nr, uint32_t Messung_Nr, uint16_t *val){
	if(ADC_Nr<3 && Messung_Nr < ADC_C_Results.Length){
		*val = ADC_C_Results.Data_DMA[ADC_Nr + 3*Messung_Nr];
		return APP_ERROR_OK;
	}
	return APP_ERROR_INVARG;
}


/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
void ADC_Elec_Init(uint32_t ADC1_Channel, uint32_t ADC2_Channel, uint32_t ADC3_Channel) {
	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/** ADC1                **/
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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
	multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}
	sConfig.Channel = ADC1_Channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** ADC2                **/
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
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	sConfig.Channel = ADC2_Channel;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** ADC3                **/
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
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	sConfig.Channel = ADC3_Channel;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc3);
}

void ADC_Sort_Channels(void){

	uint16_t ADC_D1[100];
	uint16_t ADC_D2[100];
	uint16_t ADC_D3[100];

//
//	for(uint16_t i=0;i<ADC_C_DATA_DMA_LENGTH && i<100 ;i++){
//		ADC_C_GetVal(0, i, &ADC_D1[i]);
//		ADC_C_GetVal(1, i, &ADC_D2[i]);
//		ADC_C_GetVal(1, i, &ADC_D3[i]);
//	}

	uint16_t j = 0;
	for(uint16_t i=0;i<ADC_C_DATA_DMA_MAX_LENGTH && i<100 ;i++){
		ADC_D1[i] = ADC_C_Results.Data_DMA[j];
		j++;
		ADC_D2[i] = ADC_C_Results.Data_DMA[j];
		j++;
		ADC_D3[i] = ADC_C_Results.Data_DMA[j];
		j++;
	}

}

