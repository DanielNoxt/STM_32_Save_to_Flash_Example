/*
 * CAN.c
 *
 *  Created on: Mar 3, 2023
 *      Author: D.Claassen
 */

#include "CAN.h"
#include "Bootloader.h"
#include "LockIn.h"

CAN_TxHeaderTypeDef CAN_Tx_Header;
CAN_TRx_Data_TypeDef CAN_Tx_Data;
uint32_t CAN_Tx_Mailbox;
CAN_RxHeaderTypeDef CAN_Rx_Header;
CAN_TRx_Data_TypeDef CAN_Rx_Data;

//Todo: CAN-BUS Rücksetzfunktion für Fehlerfall

AppError_t CAN_SendMessage(CAN_HandleTypeDef *hcan);

/**
 * @brief  Initialisiere den Tx-Header mit Standardwerten
 * @retval None
 */
void CAN_Init_Tx_Header(void) {
	CAN_Tx_Header.StdId = CAN_SENSOR_ADR;
	CAN_Tx_Header.DLC = 0;
	//Werden danach nicht mehr verändert!
	CAN_Tx_Header.ExtId = 0;
	CAN_Tx_Header.IDE = CAN_ID_STD;
	CAN_Tx_Header.RTR = CAN_RTR_DATA;
	CAN_Tx_Header.TransmitGlobalTime = DISABLE;
}


/**
 * @brief  Sende Aplitude und Phase vom LockIn aller drei ADCs
 * @param  hCAN pointer zur CAN_HandleTypeDef structure für den verwendeten CAN-Port
 * @param  uint8_t Preset: Auswahl, welcher der Frequenzen verwendet wurde
 * @retval APP_ERROR_OK
 */
AppError_t CAN_Send_LockInData(CAN_HandleTypeDef *hCAN, uint8_t Preset) {

	for (uint8_t i = 0; i < 3; i++) {
		while (HAL_CAN_GetTxMailboxesFreeLevel(hCAN) == 0);

		CAN_Tx_Data.f32[0] = LockIn[Preset].Ampl[i];
		CAN_Tx_Data.f32[1] = LockIn[Preset].Phase[i];

		CAN_Tx_Header.StdId = CAN_SENSOR_ADR + CAN_DATA_ID_LOCK_IN_OFFSET + CAN_DATA_ID_ANZAHL_ELEKTRODEN * Preset
				+ ADC_C_Results.Elec[i];
		CAN_Tx_Header.DLC = 8;
		CAN_SendMessage(hCAN);
	}
	return APP_ERROR_OK;
}


/**
 * @brief  Sende Aplitude und Phase vom LockIn aller drei ADCs
 * @param  hCAN pointer zur CAN_HandleTypeDef structure für den verwendeten CAN-Port
 * @param  uint8_t Preset: Auswahl, welcher der Frequenzen verwendet wurde
 * @retval APP_ERROR_OK
 */
AppError_t CAN_Send_ADC_RawData(CAN_HandleTypeDef *hCAN, uint8_t Preset){
	//Fülle CAN-Mailboxen bis voll:
	while (HAL_CAN_GetTxMailboxesFreeLevel(hCAN) > 0) {		//Fülle Mailbox, Falls eine Mailbox Frei
		uint8_t i = 0;
		while (i < 4 && ADC_C_Results.readIndex.Pos < ADC_C_Results.Length) { //Fülle die Mailbox, bis diese Voll ist oder keine Werte zum Senden verbleiben

			CAN_Tx_Data.u16[i] =
					ADC_C_Results.Data_DMA[ADC_C_Results.readIndex.Channel
											 + 3 * ADC_C_Results.readIndex.Pos];
			i++;

			ADC_C_Results.readIndex.Pos++;
		}

		CAN_Tx_Header.StdId = CAN_SENSOR_ADR + CAN_DATA_ID_ADC_VAL_OFFSET + CAN_DATA_ID_ANZAHL_ELEKTRODEN * Preset
				+ ADC_C_Results.Elec[ADC_C_Results.readIndex.Channel];
		CAN_Tx_Header.DLC = 2 * i;
		CAN_SendMessage(hCAN);

		if (ADC_C_Results.readIndex.Pos == ADC_C_Results.Length) {
			ADC_C_Results.readIndex.Channel++;
			ADC_C_Results.readIndex.Pos = 0;
		}
	}

	return APP_ERROR_OK;
}










//Privat
AppError_t CAN_SendMessage(CAN_HandleTypeDef *hCAN) {
	if (HAL_CAN_AddTxMessage(hCAN, &CAN_Tx_Header, CAN_Tx_Data.u8,
			&CAN_Tx_Mailbox) != HAL_OK) {
		Error_Handler();
		return APP_ERROR_CAN_TX;
	}
	return APP_ERROR_OK;
}

static AppError_t CAN_ReadADCParameter(void) {
	//Prüfe ob die ausgewählten ADCs passen (Nur ADC3 Kann nicht alle Channel)
	if (CAN_Rx_Data.u8[1] < ELEC_COUNT && CAN_Rx_Data.u8[2] < ELEC_COUNT
			&& (CAN_Rx_Data.u8[3] == EREF || CAN_Rx_Data.u8[3] == E3)
			&& (CAN_Rx_Data.u32[1] <= ADC_C_DATA_DMA_MAX_LENGTH)) {
		ADC_C_Results.Elec[0] = CAN_Rx_Data.u8[1];
		ADC_C_Results.Elec[1] = CAN_Rx_Data.u8[2];
		ADC_C_Results.Elec[2] = CAN_Rx_Data.u8[3];
		ADC_C_Results.Length = CAN_Rx_Data.u32[1];
		return APP_ERROR_OK;
	}
	return APP_ERROR_INVARG;
}



AppError_t CAN_ReadCMD(void) {
	if ((CAN_Rx_Header.StdId == CAN_SENSOR_ADR
			&& CAN_Rx_Data.u8[0] == CAN_CMD_START_BOOTLOADER) || CAN_Rx_Header.StdId == CAN_BOOTLOADER_ID) {
		JumpToBootloader();

		return APP_ERROR_OK;
	}

	if ((CAN_Rx_Header.StdId == CAN_BASE_ADR
			|| CAN_Rx_Header.StdId == CAN_SENSOR_ADR)) {
		switch (CAN_Rx_Data.u8[0]) {
		case CAN_CMD_IDLE:
			state_t_next = stateIdle;
			break;
		case CAN_CMD_RX_SAVE_DATA2FLASH:
			FlashWrite();
			break;
		case CAN_CMD_RX_PWM_FREQ:
			if ((CAN_Rx_Data.f32[1] < PWM_FREQ_MIN || CAN_Rx_Data.f32[1] > PWM_FREQ_MAX) || CAN_Rx_Data.u8[1]>=3){
				state_t_next = stateSendError;
				return APP_ERROR_INVARG;
			}
			LockIn_Set_Freq(CAN_Rx_Data.u8[1], CAN_Rx_Data.f32[1]);
			break;
		case CAN_CMD_RX_ADC_LENGTH:
			if (CAN_Rx_Data.u32[1] <= ADC_C_DATA_DMA_MIN_LENGTH || CAN_Rx_Data.u32[1] >= ADC_C_DATA_DMA_MAX_LENGTH){
				state_t_next = stateSendError;
				return APP_ERROR_INVARG;
			}
			ADC_C_Results.Length = CAN_Rx_Data.u32[1];
			break;
		case CAN_CMD_RX_CAN_PRESET:
			if (CAN_Rx_Data.u8[1] >= CAN_FREQ_Count){
				state_t_next = stateSendError;
				return APP_ERROR_INVARG;
			}
			TIS_Sensor.CAN_Freq_Preset = CAN_Rx_Data.u8[1];
			state_t_next = stateSetCanFreq;
			break;
		case CAN_CMD_RX_TIMING_LOCK_IN:
			TIS_Sensor.Timing_LockInData = CAN_Rx_Data.u32[1];
			break;
		case CAN_CMD_RX_TIMING_RAW_DATA:
			if(CAN_Rx_Data.u8[1]<=1){
				TIS_Sensor.SendRawData = CAN_Rx_Data.u8[1];
			}
			TIS_Sensor.Timing_AdditionalData = CAN_Rx_Data.u32[1];
			break;

		default:
			return APP_ERROR_INVARG;
			break;
		}
		return APP_ERROR_OK;
	}
	return APP_ERROR_INVARG;
}


AppError_t CAN_SendError(CAN_HandleTypeDef *hCAN, AppError_t Error) {
	CAN_Tx_Data.u8[0] = Error;
	CAN_Tx_Header.StdId = CAN_SENSOR_ADR + CAN_DATA_ID_ERROR;
	CAN_Tx_Header.DLC = 1;
	CAN_SendMessage(hCAN);
	return APP_ERROR_OK;
}

AppError_t CAN_SendId(CAN_HandleTypeDef *hCAN) {

	CAN_Tx_Data.u32[0] = HAL_GetUIDw0();
	CAN_Tx_Header.StdId = CAN_SENSOR_ADR + 0;
	CAN_Tx_Header.DLC = 4;
	CAN_SendMessage(hCAN);

	CAN_Tx_Data.u32[0] = HAL_GetUIDw1();
	CAN_Tx_Data.u32[1] = HAL_GetUIDw2();
	CAN_Tx_Header.DLC = 8;
	CAN_SendMessage(hCAN);
	return APP_ERROR_OK;
}

AppError_t CAN_SendTemp_Vbat(CAN_HandleTypeDef *hCAN, ADC_Temp_ResultsTypeDef *hTemp, float *hVbat){
	while (HAL_CAN_GetTxMailboxesFreeLevel(hCAN) == 0)
		;


	CAN_Tx_Data.f32[0] = hTemp->Temp;
	CAN_Tx_Data.f32[1] = *hVbat;
	CAN_Tx_Header.StdId = CAN_SENSOR_ADR + CAN_DATA_ID_TEMP_VBAT;
	CAN_Tx_Header.DLC = 8;
	CAN_SendMessage(hCAN);
	return APP_ERROR_OK;
}

AppError_t CAN_SendFreq(CAN_HandleTypeDef *hCAN){

	CAN_Tx_Header.DLC = 4;	//Floats übertragen

	for(uint8_t i = 0; i<3; i++){
		CAN_Tx_Header.StdId = CAN_SENSOR_ADR + CAN_DATA_ID_FREQ_OFFSET + i;
		CAN_Tx_Data.f32[0] = LockIn[i].PWM_freq_ist;
		while (HAL_CAN_GetTxMailboxesFreeLevel(hCAN) == 0);
		CAN_SendMessage(hCAN);
	}
	return APP_ERROR_OK;
}

AppError_t CAN_SendSensorData(CAN_HandleTypeDef *hCAN){
	CAN_Tx_Data.u16[0] = TIS_Sensor.SensorSN;
	//CAN_Tx_Data.f32[1] = TIS_Sensor.PWM_freq_ist;

//	CAN_Tx_Header.StdId = CAN_SENSOR_ADR + CAN_DATA_ID_SENSOR_DATA;
//	CAN_Tx_Header.DLC = 4;
//	CAN_SendMessage(hCAN);
	return APP_ERROR_OK;
}


//AppError_t CAN2_TRX_Set_Power(uint8_t Power) {
//	if (Power == CAN_TRX_SET_POWER_ON || Power == CAN_TRX_SET_POWER_LOW_POWER) {
//		HAL_GPIO_WritePin(CAN2_RS_GPIO_Port, CAN2_RS_Pin, Power);
//		return APP_ERROR_OK;
//	}
//	return APP_ERROR_INVARG;
//}

AppError_t CAN2_TRX_Set_Loopback(uint8_t AB) {
	if (AB == CAN_TRX_SET_LOOPBACK_OFF || AB == CAN_TRX_SET_LOOPBACK_ON) {
		HAL_GPIO_WritePin(CAN2_AB_GPIO_Port, CAN2_AB_Pin, AB);
		return APP_ERROR_OK;
	}
	return APP_ERROR_INVARG;
}

AppError_t CAN_Set_Mode(CAN_HandleTypeDef *hCAN, uint8_t CAN_Mode) {
	switch (CAN_Mode) {
	case CAN_SET_DISABLE:
		HAL_CAN_DeactivateNotification(hCAN, CAN_IT_ERROR);
		HAL_CAN_DeactivateNotification(hCAN, CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_AbortTxRequest(hCAN, CAN_TX_MAILBOX0);
		HAL_CAN_AbortTxRequest(hCAN, CAN_TX_MAILBOX1);
		HAL_CAN_AbortTxRequest(hCAN, CAN_TX_MAILBOX2);
		HAL_CAN_Stop(hCAN);
		return APP_ERROR_OK;
	case CAN_SET_ENABLE:
		HAL_CAN_ResetError(hCAN);
		HAL_CAN_Start(hCAN);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_ERROR);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_RX_FIFO0_MSG_PENDING);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_RX_FIFO0_FULL);
		HAL_CAN_ActivateNotification(hCAN, CAN_IT_RX_FIFO0_OVERRUN);
		return APP_ERROR_OK;
	case CAN_SET_SLEEP:
		return APP_ERROR_OK;
	default:
		return APP_ERROR_INVARG;
	}
}

AppError_t CAN_Set_Freq_Preset(CAN_HandleTypeDef *hCAN, CAN_FREQ_TypeDef Preset){
	uint32_t BuffTimingLockIn = TIS_Sensor.Timing_LockInData;
	uint32_t BuffTimingRawData = TIS_Sensor.Timing_AdditionalData;
	TIS_Sensor.Timing_LockInData = 0;
	TIS_Sensor.Timing_AdditionalData = 0;

	uint32_t CAN_Prescaler = 0;
	switch(Preset){
	case(CAN_FREQ_100KHZ):
			CAN_Prescaler = 30;
			break;
	case(CAN_FREQ_125KHZ):
			CAN_Prescaler = 24;
			break;
	case(CAN_FREQ_200KHZ):
			CAN_Prescaler = 15;
			break;
	case(CAN_FREQ_250KHZ):
			CAN_Prescaler = 12;
			break;
	case(CAN_FREQ_500KHZ):
			CAN_Prescaler = 6;
			break;
	case(CAN_FREQ_1000KHZ):
			CAN_Prescaler = 3;
			break;
	default:
		CAN_Prescaler = 30;
		break;
	}
	  hCAN->Init.Prescaler = CAN_Prescaler;
	  if (HAL_CAN_Init(hCAN) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  HAL_CAN_Start(hCAN);
	TIS_Sensor.CAN_Freq_Preset = Preset;

	TIS_Sensor.Timing_LockInData = BuffTimingLockIn;
		TIS_Sensor.Timing_AdditionalData = BuffTimingRawData;
	return APP_ERROR_OK;
}

//DeviceID uint32_t HAL_GetDEVID(void)
