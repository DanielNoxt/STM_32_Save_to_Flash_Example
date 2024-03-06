/*
 * AppErrorHandling.h
 *
 *  Created on: 20.04.2023
 *      Author: D.Claassen
 */

#ifndef INC_APPERRORHANDLING_H_
#define INC_APPERRORHANDLING_H_


// Todo: Errno Abfrage
typedef enum AppError_e
{
    // No error
    APP_ERROR_OK = 0,

    // Invalid arguments (ex: NULL pointer where a valid pointer is required)
	APP_ERROR_INVARG,

    // Fehlerhafte Spannungsverorgung für Analogschaltungen
    APP_ERROR_ANALOG_PWR,

    //
    APP_ERROR_CAN_TX,

    //
    APP_ERROR_CAN_RX,

    // Total # of errors in this list (NOT AN ACTUAL ERROR CODE);
    APP_ERROR_COUNT,
} AppError_t;



const char* AppError_str(AppError_t err);


#endif /* INC_APPERRORHANDLING_H_ */
