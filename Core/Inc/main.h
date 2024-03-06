/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AppErrorHandling.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void FlashWrite(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Status_LED_Pin GPIO_PIN_3
#define Status_LED_GPIO_Port GPIOE
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define SinGen_PWM_Pin GPIO_PIN_6
#define SinGen_PWM_GPIO_Port GPIOC
#define CAN2_AB_Pin GPIO_PIN_8
#define CAN2_AB_GPIO_Port GPIOC
#define EN_5V_Pin GPIO_PIN_12
#define EN_5V_GPIO_Port GPIOC
#define CAN2_RX_Pin GPIO_PIN_5
#define CAN2_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef enum  {
	EREF,
	E1,
	E2,
	E3,
	V_SINGEN,
	V_BATT_FB,
	ELEC_COUNT
}A_IN;

typedef enum {
	stateIdle,
	stateSendLockIn,
	stateSendAdditionalData,
	stateSendError,
	stateSetCanFreq,
	stateCount
} state_t_TypeDef;

/*
 * The bytes are coded in memory in Little Endian format. The lowest numbered byte in a word
is considered the word’s least significant byte and the highest numbered byte the most
significant.
 */

#define SENSOR_SN 1002

//CAN-Adress:
#define CAN_BASE_ADR	1000		//Übertragung vom PC an alle Sensoren
#define CAN_SENSOR_ADR	(CAN_BASE_ADR+1+(SENSOR_SN-1001)*100)
#define CAN_BOOTLOADER_ID  0x79

//CAN Data-ID
#define CAN_DATA_ID_LOCK_IN_OFFSET 	0x01
#define CAN_DATA_ID_ADC_VAL_OFFSET	0x10

#define CAN_DATA_ID_TEMP_VBAT		0x32
#define CAN_DATA_ID_SENSOR_DATA		0x33
#define CAN_DATA_ID_UNIQUE_ID		0x34
#define CAN_DATA_ID_FREQ_OFFSET		0x35
#define CAN_DATA_ID_ERROR			0x63

#define CAN_DATA_ID_ANZAHL_ELEKTRODEN	0x05

//CAN-Befehlssätze für CAN_BASE_ADR und CAN_SENSOR_ADR
#define CAN_CMD_IDLE	 			0x00
#define CAN_CMD_RX_SAVE_DATA2FLASH	0x01
#define CAN_CMD_RX_PWM_FREQ			0x02
#define CAN_CMD_RX_ADC_LENGTH 		0x03
#define CAN_CMD_RX_CAN_PRESET 		0x04
#define CAN_CMD_RX_TIMING_LOCK_IN	0x05
#define CAN_CMD_RX_TIMING_RAW_DATA	0x06

//CAN-Befehlssätze für CAN_SENSOR_ADR
#define CAN_CMD_START_BOOTLOADER 	0xAA

typedef enum {
	CAN_FREQ_100KHZ,
	CAN_FREQ_125KHZ,
	CAN_FREQ_200KHZ,
	CAN_FREQ_250KHZ,
	CAN_FREQ_500KHZ,
	CAN_FREQ_1000KHZ,
	CAN_FREQ_Count
} CAN_FREQ_TypeDef;

//Clocks, Timings
#define PCLK2_FREQ 90E6
#define APB2_PERIPHERAL_FREQ PCLK2_FREQ
#define APB2_TIMER_FREQ PCLK2_FREQ*2
#define PWM_FREQ_BASE0 	80E3
#define PWM_FREQ_BASE1 	120E3
#define PWM_FREQ_BASE2 	200E3
#define PWM_FREQ_MIN	(float)50E3
#define PWM_FREQ_MAX 	(float)500E3
#define	CAN_FREQ_PRESET	CAN_FREQ_100KHZ

#define TIME_SEND_LOCK_IN_DATA 		(uint32_t)15		//Wartezeit in ms
#define TIME_SEND_ADDITIONAL_DATA 	(uint32_t)120000	//Wartezeit in ms

//ADCs
#define ADC_VRef (float) 2.5
#define ADC_FREQ PCLK2_FREQ/4
#define ADC_SYNC_DELAY_CYCLES	5
#define ADC_SAMPLING_CYCLES		3
#define ADC_CONV_CYCLES			12
#define ADC_RES					12
#define ADC_RANGE_MIN 			(int16_t)(-(1<<ADC_RES)/2)        // ADC Measrange
#define ADC_RANGE_MAX 			(int16_t)((1<<ADC_RES)/2-1)
#define LOCK_IN_AMP_REF 		(float)0.1
#define ADC_SAMPLE_FREQ 		(float)(ADC_FREQ/(float)(ADC_SAMPLING_CYCLES+ADC_CONV_CYCLES))
#define ADC_SAMPLE_TIME			(float)(1/ADC_SAMPLE_FREQ)
#define ADC_C_DATA_DMA_LENGTH	1000
#define ADC_C_DATA_DMA_MIN_LENGTH	50		//Minimale Anzahl an Werten für den LockIn, bevor dieser nur noch Quatsch rechnet.
#define ADC_C_DATA_DMA_MAX_LENGTH	5000
#define ADC_C_DATA_DMA_SIZE		3*ADC_C_DATA_DMA_MAX_LENGTH
#define ADC_CHANNEL_EREF		ADC_CHANNEL_3		//ADC123_IN3 	-Pin26
#define ADC_CHANNEL_E1			ADC_CHANNEL_9		//ADC12_IN9		-Pin36
#define ADC_CHANNEL_E2			ADC_CHANNEL_8		//ADC12_IN8		-Pin35
#define ADC_CHANNEL_E3			ADC_CHANNEL_1		//ADC123_IN1	-Pin24
#define ADC_CHANNEL_V_SINGEN	ADC_CHANNEL_15		//ADC12_IN15	-Pin34
#define ADC_CHANNEL_V_BATT_FB	ADC_CHANNEL_2		//ADC123_IN2	-Pin25
#define ADC_V_BATT_R1	(float)1e3
#define ADC_V_BATT_R2	(float)10e3


typedef struct {
	uint8_t Channel;
	uint32_t Pos;
}Data_DMA_Index_TypeDef;

typedef struct {
	volatile uint32_t Elec[3];
	volatile uint16_t Data_DMA[ADC_C_DATA_DMA_SIZE];
	volatile Data_DMA_Index_TypeDef readIndex;
	volatile uint8_t ConCplet;
	volatile uint8_t TimerSync;		//Statusbit, welches zum Synchronisieren auf den nächsten Timerinterrupt dient
	volatile uint32_t Length;
} ADC_C_ResultsTypeDef;

typedef struct {
	float Temp;
	float Slope;
	float Offset;
} ADC_Temp_ResultsTypeDef;

typedef struct {
	int16_t Vr[ADC_C_DATA_DMA_MAX_LENGTH];
	int16_t Vr_90[ADC_C_DATA_DMA_MAX_LENGTH];
	float Ampl[3];
	float Phase[3];
	volatile float PWM_freq_soll;
	float PWM_freq_ist;
	uint16_t PWM_Period;
} LockIn_TypeDef;

typedef struct {
	const uint16_t SensorSN;
	uint32_t Timing_LockInData;
	uint32_t Timing_AdditionalData;
	uint32_t SendRawData;
	uint32_t CAN_Freq_Preset;
}TIS_Sensor_TypeDef;

typedef struct
{
	float PWM_freq_soll[3];
	uint32_t Timing_LockInData;
	uint32_t Timing_AdditionalData;
	uint32_t SendRawData;
	CAN_FREQ_TypeDef CAN_Freq_Preset;
	uint32_t ADC_Length;
}Flash_Data_TypeDef;


//Globale Variablen, werden in der Main reserviert und überall anders als extern gehandhabt.
#ifdef __MAIN_C
Flash_Data_TypeDef *Userdata = (Flash_Data_TypeDef*)0x8060000;
TIS_Sensor_TypeDef TIS_Sensor = {SENSOR_SN};
ADC_C_ResultsTypeDef ADC_C_Results;
ADC_Temp_ResultsTypeDef ADC_uC_Temp_Results;
LockIn_TypeDef LockIn[3];
float VBat = 0;
	state_t_TypeDef state_t_cur = stateIdle;
	volatile state_t_TypeDef state_t_next = stateIdle;

	volatile AppError_t LastError = APP_ERROR_OK;

	const uint32_t ADCx_CHANNEL[ELEC_COUNT] = {ADC_CHANNEL_3,ADC_CHANNEL_9,ADC_CHANNEL_8,ADC_CHANNEL_1,ADC_CHANNEL_15,ADC_CHANNEL_2};

#else
	extern Flash_Data_TypeDef *Userdata;
	extern TIS_Sensor_TypeDef TIS_Sensor;
	extern ADC_C_ResultsTypeDef ADC_C_Results;
	extern ADC_Temp_ResultsTypeDef ADC_uC_Temp_Results;
	extern LockIn_TypeDef LockIn[3];

	extern state_t_TypeDef state_t_cur;
	extern volatile state_t_TypeDef state_t_next;

	extern const uint32_t ADCx_CHANNEL[ELEC_COUNT];

	extern ADC_HandleTypeDef hadc1;
	extern ADC_HandleTypeDef hadc2;
	extern ADC_HandleTypeDef hadc3;
	extern DMA_HandleTypeDef hdma_adc1;
	extern DMA_HandleTypeDef hdma_adc2;
	extern DMA_HandleTypeDef hdma_adc3;

	extern CAN_HandleTypeDef hcan2;

	extern CRC_HandleTypeDef hcrc;

	extern TIM_HandleTypeDef htim8;
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
