/*
 * CAN_Bootloader.c
 *
 *  Created on: 13.05.2023
 *      Author: D.Claassen
 */


#include <Bootloader.h>




static void CAN_Filter_DeInit(void){
	CAN_FilterTypeDef CAN_FILTER_CONFIG;
		CAN_FILTER_CONFIG.FilterActivation = CAN_FILTER_DISABLE;
		CAN_FILTER_CONFIG.SlaveStartFilterBank = 0;	//insgesamt 28 Filter, geteilt zwischen CAN1 (Master) CAN2(Slave) aktuell Can1: 0 Can2: 1-27
		CAN_FILTER_CONFIG.FilterBank = 1;		//Alles ab SlaveStartFilterBank
		CAN_FILTER_CONFIG.FilterFIFOAssignment = CAN_RX_FIFO0;
		CAN_FILTER_CONFIG.FilterIdHigh = 0;	//Shift um 5, da EXID die ersten 5 Bits belegt. [RM0390 Rev 6 S.1057]
		CAN_FILTER_CONFIG.FilterIdLow = 0;
		CAN_FILTER_CONFIG.FilterMaskIdHigh = 0;
		CAN_FILTER_CONFIG.FilterMaskIdLow = 0;
		CAN_FILTER_CONFIG.FilterMode = CAN_FILTERMODE_IDMASK;
		CAN_FILTER_CONFIG.FilterScale = CAN_FILTERSCALE_16BIT;
		HAL_CAN_ConfigFilter(&hcan2, &CAN_FILTER_CONFIG);

		CAN_FILTER_CONFIG.FilterBank = 2;
		HAL_CAN_ConfigFilter(&hcan2, &CAN_FILTER_CONFIG);
}



//Once initialized the CAN2 configuration is: Baudrate 125 kbps, 11-bit identifier
/*Pins laut Datenblatt
**PB5 pin: CAN2 in reception mode. Used in
	alternate push-pull, pull-up mode.
**PB13 pin: CAN2 in transmission mode.
	Used in alternate push-pull, pull-up mode.


	Read protection (RDP) zum Schluss enablen
	0xBB
	*/
void JumpToBootloader(void){
	void (*SysMemBootJump)(void);

	 /**
	     * Step: Set system memory address.
	     *
	     *       For STM32F429, system memory is on 0x1FFF 0000
	     *       For other families, check AN2606 document table 110 with descriptions of memory addresses
	     */
	volatile uint32_t SystemMemoryAddr = 0x1FFF0000;


	//RCC zurücksetzen, Systick wird benötigt --> Interrupts erst später deaktivieren
	HAL_RCC_DeInit();

	//Deinit used Components
//	HAL_ADCEx_MultiModeStop_DMA(&hadc1);
//	HAL_DMA_DeInit(hadc1.DMA_Handle);
//	HAL_ADC_DeInit(&hadc1);
//	HAL_ADC_MspDeInit(&hadc1);
//
//	HAL_ADC_Stop(&hadc2);
//	HAL_DMA_DeInit(hadc2.DMA_Handle);
//	HAL_ADC_DeInit(&hadc2);
//	HAL_ADC_MspDeInit(&hadc2);
//
//	HAL_ADC_Stop(&hadc3);
//	HAL_DMA_DeInit(hadc3.DMA_Handle);
//	HAL_ADC_DeInit(&hadc3);
//	HAL_ADC_MspDeInit(&hadc3);
//
//	CAN_Filter_DeInit();
//	HAL_CAN_DeInit(&hcan2);
//	HAL_CAN_MspDeInit(&hcan2);
//
//	HAL_CRC_DeInit(&hcrc);
//	HAL_CRC_MspDeInit(&hcrc);
//
//	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_DeInit(&htim8);
//	HAL_TIM_Base_DeInit(&htim8);
//	HAL_TIM_Base_MspDeInit(&htim8);


	//de-Initializes common part of the HAL and stops the systick.
	HAL_DeInit();
	//Manueller Reset aller Systick-Register
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
	//Interrupts deaktivieren
	__disable_irq();
    /**
     * Step: Remap system memory to address 0x0000 0000 in address space
     *       For each family registers may be different.
     *       Check reference manual for each family.
     *
     *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
     *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
     *       For others, check family reference manual
     */

	//__HAL_RCC_SYSCFG_CLK_ENABLE();	//https://github.com/akospasztor/stm32-bootloader/blob/e6a489ee57b72e83ce877d271085464ce6ebd536/lib/stm32-bootloader/bootloader.c#L120
	//Setzte die jump memory location zu System Memory
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();


    /**
     * Step: Set jump memory location for system memory
     *       Use address with 4 bytes offset which specifies jump location where program starts
     *       "starts code execution from the boot memory starting from 0x0000 0004" (RM0390 S. 62)
     */

    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(SystemMemoryAddr + 4)));

    /**
        * Step: Set main stack pointer.
        *       This step must be done last otherwise local variables in this function
        *       don't have proper value since stack pointer is located on different position
        *
        *       Set direct address location which specifies stack pointer in SRAM location
        */
       __set_MSP(*(uint32_t *)SystemMemoryAddr);

       /**
        * Step: Actually call our function to jump to set location
        *       This will start system memory execution
        */
       SysMemBootJump();


}
