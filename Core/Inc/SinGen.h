/*
 * SinGen.h
 *
 *  Created on: 06.07.2023
 *      Author: D.Claassen
 */

#ifndef INC_SINGEN_H_
#define INC_SINGEN_H_

#include "main.h"
#include "AppErrorHandling.h"

//Funktionen für die Sinusanregung mit einem festen DutyCyle
uint16_t SinGen_Calc_Prescaler(float freq);
void SinGen_RefreshTimer(uint8_t Preset);

#endif /* INC_SINGEN_H_ */
