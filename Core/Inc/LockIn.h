/*
 * LockIn.h
 *
 *  Created on: 11.03.2023
 *      Author: D.Claassen
 */

#ifndef INC_LOCKIN_H_
#define INC_LOCKIN_H_

#include "main.h"

AppError_t LockIn_Set_Freq(uint8_t Ch, float freq);

//Alte Funktionen
AppError_t LockIn_LUT_Gen(uint8_t Ch);
AppError_t LockIn_Calc(uint8_t Ch);

void Lock_In_FillADC(uint8_t Ch, uint32_t length);

#endif /* INC_LOCKIN_H_ */
