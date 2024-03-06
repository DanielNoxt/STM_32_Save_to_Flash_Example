/*
 * SinGen.c
 *
 *  Created on: 06.07.2023
 *      Author: D.Claassen
 */

#include "SinGen.h"
#include "math.h"

uint16_t SinGen_Calc_Prescaler(float freq){
	//Berechne Wert für Prescaler
	//Die Counterperiod (Gepeichert im Autoreloadregister (ARR)) wird dabei nicht verändert, dadurch bleibt das Pulsverhältnis automatisch konstant
	return (roundf(APB2_TIMER_FREQ/(freq*(TIM8->ARR+1)))-1);
}

void SinGen_RefreshTimer(uint8_t Preset){
	//Vorberechnete werde in den Timer Laden
	TIM8->PSC = LockIn[Preset].PWM_Period;
}
