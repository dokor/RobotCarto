#ifndef 	H_TEMPERATURE
#define		H_TEMPERATURE 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"

extern	int16_t Temperature_Exterieure ;


extern void Mesure_Temperature_IT (uint32_t Low_Val, uint32_t High_Val); 



#endif
