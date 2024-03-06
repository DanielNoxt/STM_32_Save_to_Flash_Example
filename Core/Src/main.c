/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

#define __MAIN_C
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "ADC.h"
#include "Bootloader.h"
#include "CAN.h"
#include "LockIn.h"
#include "SinGen.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
extern CAN_RxHeaderTypeDef CAN_Rx_Header;
extern CAN_TRx_Data_TypeDef CAN_Rx_Data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
AppError_t SystemCheck(void);

void InitGlobalVar(void);
void nextstate(state_t_TypeDef nstate_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//JumpToBootloader();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_CAN2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

	//Initialisieren aller Parameter
	InitGlobalVar();

	HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);

	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

	//CAN2_TRX_Set_Power(CAN_TRX_SET_POWER_ON);		//Deaktiviert
	CAN2_TRX_Set_Loopback(CAN_TRX_SET_LOOPBACK_OFF);
	CAN_Set_Mode(&hcan2, CAN_SET_ENABLE);
	__enable_irq();

	//  SystemCheck();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		switch (state_t_cur) {
		case stateIdle:
			//nextstate(stateIdle);
			break;

		case stateSendLockIn:
			//HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
			ADC_C_Results.Elec[0] = E1;
			ADC_C_Results.Elec[1] = E2;
			ADC_C_Results.Elec[2] = E3;
			for (uint8_t i = 0; i < 3; i++) {
				ADC_Sample_Elec_poll();
				if (i == 2) {
					SinGen_RefreshTimer(0);
				} else {
					SinGen_RefreshTimer(i + 1);
				}
				LockIn_Calc(i);	//Refresh vor dem Berechnen, damit Tiefpässe vorm erneuten Samplen auf neue Frequenz eingeschwungen sind
				CAN_Send_LockInData(&hcan2, i);
				//ADC_Sort_Channels();
			}
			nextstate(stateIdle);
			break;

		case stateSendAdditionalData:
			for (uint8_t i = 0; i < 3; i++) {	//Toggle alle 3 Frequenzen durch
				ADC_C_Results.Elec[0] = E1;
				ADC_C_Results.Elec[1] = E2;
				ADC_C_Results.Elec[2] = E3;
				ADC_Sample_Elec_poll();
				LockIn_Calc(i);
				CAN_Send_LockInData(&hcan2, i);
				if(TIS_Sensor.SendRawData){
					while (ADC_C_Results.readIndex.Channel < 3
							&& ADC_C_Results.readIndex.Pos < ADC_C_Results.Length) {
						CAN_Send_ADC_RawData(&hcan2, i);
					}
				}


				ADC_C_Results.Elec[0] = V_SINGEN;
				ADC_C_Results.Elec[1] = EREF;
				ADC_C_Results.Elec[2] = E3;
				ADC_Sample_Elec_poll();
				if (i == 2) {
					SinGen_RefreshTimer(0);
				} else {
					SinGen_RefreshTimer(i + 1);
				}
				LockIn_Calc(i);	//Timer Refresh vor dem Berechnen, damit Tiefpässe vorm erneuten Samplen auf neue Frequenz eingeschwungen sind
				CAN_Send_LockInData(&hcan2, i);
				//Sende Rohwerte
				if(TIS_Sensor.SendRawData){
					while (ADC_C_Results.readIndex.Channel < 3
							&& ADC_C_Results.readIndex.Pos < ADC_C_Results.Length) {
						CAN_Send_ADC_RawData(&hcan2, i);
					}
				}
				//ADC_Sort_Channels();
			}
			ADC_ReadTemp(&ADC_uC_Temp_Results);
			ADC_Vbat(&VBat);
			CAN_SendTemp_Vbat(&hcan2, &ADC_uC_Temp_Results, &VBat);
			CAN_SendFreq(&hcan2);
			nextstate(stateIdle);
			break;

		case stateSendError:
			CAN_SendError(&hcan2, LastError);
			nextstate(stateIdle);
			break;
		case stateSetCanFreq:
			CAN_Set_Freq_Preset(&hcan2, TIS_Sensor.CAN_Freq_Preset);
			nextstate(stateIdle);
			break;
		default:
			nextstate(stateIdle);
			break;
		}

		//HAL_Delay(5);
		state_t_cur = state_t_next;
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
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
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
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
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
	CAN_Init_Tx_Header();
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */
	//Bittimings: http://www.bittiming.can-wiki.info/
  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 24;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	//Filtersetup für allgemein eingehende Nachrichten auf CAN_BASE_ADR
	CAN_FilterTypeDef CAN_FILTER_CONFIG;
	CAN_FILTER_CONFIG.FilterActivation = CAN_FILTER_ENABLE;
	CAN_FILTER_CONFIG.SlaveStartFilterBank = 1;	//insgesamt 28 Filter, geteilt zwischen CAN1 (Master) CAN2(Slave) aktuell Can1: 0 Can2: 1-27
	CAN_FILTER_CONFIG.FilterBank = 1;		//Alles ab SlaveStartFilterBank
	CAN_FILTER_CONFIG.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_FILTER_CONFIG.FilterIdHigh = CAN_BASE_ADR << 5;	//Shift um 5, da EXID die ersten 5 Bits belegt. [RM0390 Rev 6 S.1057]
	CAN_FILTER_CONFIG.FilterIdLow = 0x0000;
	CAN_FILTER_CONFIG.FilterMaskIdHigh = 0x7FF << 5;
	CAN_FILTER_CONFIG.FilterMaskIdLow = 0x0000;
	CAN_FILTER_CONFIG.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FILTER_CONFIG.FilterScale = CAN_FILTERSCALE_32BIT;
	if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FILTER_CONFIG) != HAL_OK) {
		Error_Handler();
	}

	//Filtersetup für eingehende Nachrichten auf
	CAN_FILTER_CONFIG.FilterActivation = CAN_FILTER_ENABLE;
	CAN_FILTER_CONFIG.SlaveStartFilterBank = 1;	//insgesamt 28 Filter, geteilt zwischen CAN1 (Master) CAN2(Slave) aktuell Can1: 0 Can2: 1-27
	CAN_FILTER_CONFIG.FilterBank = 2;		//Alles ab SlaveStartFilterBank
	CAN_FILTER_CONFIG.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_FILTER_CONFIG.FilterIdHigh = CAN_SENSOR_ADR << 5;//Shift um 5, da EXID die ersten 5 Bits belegt. [RM0390 Rev 6 S.1057]
	CAN_FILTER_CONFIG.FilterIdLow = 0x0000;
	CAN_FILTER_CONFIG.FilterMaskIdHigh = 0x7FF << 5;
	CAN_FILTER_CONFIG.FilterMaskIdLow = 0x0000;
	CAN_FILTER_CONFIG.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FILTER_CONFIG.FilterScale = CAN_FILTERSCALE_32BIT;
	if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FILTER_CONFIG) != HAL_OK) {
		Error_Handler();
	}

	//	//Filtersetup für eingehende Nachrichten auf
	//	CAN_FILTER_CONFIG.FilterActivation = CAN_FILTER_ENABLE;
	//	CAN_FILTER_CONFIG.SlaveStartFilterBank = 1;	//insgesamt 28 Filter, geteilt zwischen CAN1 (Master) CAN2(Slave) aktuell Can1: 0 Can2: 1-27
	//	CAN_FILTER_CONFIG.FilterBank = 3;		//Alles ab SlaveStartFilterBank
	//	CAN_FILTER_CONFIG.FilterFIFOAssignment = CAN_RX_FIFO0;
	//	CAN_FILTER_CONFIG.FilterIdHigh = CAN_BOOTLOADER_ID << 5;//Shift um 5, da EXID die ersten 5 Bits belegt. [RM0390 Rev 6 S.1057]
	//	CAN_FILTER_CONFIG.FilterIdLow = 0x0000;
	//	CAN_FILTER_CONFIG.FilterMaskIdHigh = 0x7FF << 5;
	//	CAN_FILTER_CONFIG.FilterMaskIdLow = 0x0000;
	//	CAN_FILTER_CONFIG.FilterMode = CAN_FILTERMODE_IDMASK;
	//	CAN_FILTER_CONFIG.FilterScale = CAN_FILTERSCALE_32BIT;
	//	if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FILTER_CONFIG) != HAL_OK) {
	//		Error_Handler();
	//	}
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 179;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAN2_AB_Pin|EN_5V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Status_LED_Pin */
  GPIO_InitStruct.Pin = Status_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Status_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN2_AB_Pin EN_5V_Pin */
  GPIO_InitStruct.Pin = CAN2_AB_Pin|EN_5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ADC_C_Results.ConCplet = 0;
	//HAL_ADCEx_MultiModeStop_DMA(&hadc1);

}

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	ADC_C_Results.TimerSync = 0;
}

AppError_t SystemCheck(void) {
	// Spannungen Überprüfen, insbesondere Vref, und 5 V
	return APP_ERROR_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_Header, CAN_Rx_Data.u8);
	LastError = CAN_ReadCMD();
}

void nextstate(state_t_TypeDef nstate_t) {
	if (state_t_cur == state_t_next) {//Dann wurde dieser nicht vom Host geändert.
		state_t_next = nstate_t;
	}
}

void InitGlobalVar(void) {

	//überprüfe, ob der Flashspeicher gesetzt wurde
	uint8_t Flash_Unwritten = 1;
	for (uint32_t i = 0; i < sizeof(Flash_Data_TypeDef); i++) {
		if (*((uint8_t*) Userdata + i) != 0xFF) {
			Flash_Unwritten = 0;
			break;
		}
	}

	//Falls erster Start nach Reset: Beschreibe Flash
	if (Flash_Unwritten) {
		LockIn[0].PWM_freq_soll = PWM_FREQ_BASE0;
		LockIn[1].PWM_freq_soll = PWM_FREQ_BASE1;
		LockIn[2].PWM_freq_soll = PWM_FREQ_BASE2;
		TIS_Sensor.Timing_LockInData = TIME_SEND_LOCK_IN_DATA;
		TIS_Sensor.Timing_AdditionalData = TIME_SEND_ADDITIONAL_DATA;
		TIS_Sensor.SendRawData = 1;
		TIS_Sensor.CAN_Freq_Preset = CAN_FREQ_PRESET;
		ADC_C_Results.Length = ADC_C_DATA_DMA_LENGTH;
		FlashWrite();
	}


	ADC_Temp_Calc_Slope_Offset(&ADC_uC_Temp_Results);

	ADC_C_Results.Elec[0] = E1;
	ADC_C_Results.Elec[1] = E2;
	ADC_C_Results.Elec[2] = E3;


	LockIn_Set_Freq(0, Userdata->PWM_freq_soll[0]);
	LockIn_Set_Freq(1, Userdata->PWM_freq_soll[1]);
	LockIn_Set_Freq(2, Userdata->PWM_freq_soll[2]);

	CAN_Set_Freq_Preset(&hcan2, Userdata->CAN_Freq_Preset);

	ADC_C_Results.Length = Userdata->ADC_Length;
	TIS_Sensor.Timing_LockInData = Userdata->Timing_LockInData;
	TIS_Sensor.Timing_AdditionalData = Userdata->Timing_AdditionalData;
	TIS_Sensor.SendRawData = Userdata->SendRawData;
	state_t_next = stateIdle;
}

void FlashWrite(void) {
	FLASH_EraseInitTypeDef Erase;
	Erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	//Erase.Banks = FLASH_BANK_1;
	Erase.Sector = FLASH_SECTOR_7;
	Erase.NbSectors = 1;
	Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	uint32_t ERrrror = 0;
	HAL_FLASHEx_Erase(&Erase, &ERrrror);

	union {
		float fVal;
		uint32_t uVal;
	} Freq;

	for (uint8_t i = 0; i < 3; i++) {
		Freq.fVal = LockIn[i].PWM_freq_soll;
		HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) &Userdata->PWM_freq_soll[i], (uint32_t) Freq.uVal);
	}
	HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) &Userdata->CAN_Freq_Preset, (uint32_t) TIS_Sensor.CAN_Freq_Preset);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) &Userdata->ADC_Length, (uint32_t) ADC_C_Results.Length);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) &Userdata->Timing_AdditionalData, (uint32_t) TIS_Sensor.Timing_AdditionalData);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) &Userdata->Timing_LockInData, (uint32_t) TIS_Sensor.Timing_LockInData);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) &Userdata->SendRawData, (uint32_t) TIS_Sensor.SendRawData);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);
	HAL_NVIC_SystemReset();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
