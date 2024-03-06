/*
 * AppErrorHandling.c
 *
 *  Created on: 20.04.2023
 *      Author: D.Claassen
 */


#include "AppErrorHandling.h"
#include "stddef.h"


const char* const APP_ERROR_STRS[] =
{
    "APP_ERROR_OK",
    "APP_ERROR_INVARG",
    "APP_ERROR_ANALOG_PWR",
    "APP_ERROR_Count",
};


/**
  * @brief Funktion, in welche die analoge Spannungsversorgung aktiviert wird.
  * @param None
  * @retval AppError_t
  */
const char* AppError_str(AppError_t err){
	    const char* err_str = NULL;

	    // Ensure error codes are within the valid array index range
	    if (err >= APP_ERROR_COUNT)
	    {
	        goto done;
	    }

	    err_str = APP_ERROR_STRS[err];

	done:
	    return err_str;
	}
