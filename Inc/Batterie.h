#ifndef 	H_BATTERIE
#define		H_BATTERIE 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"


extern	uint16_t		V_Batterie ;							/* Exprimée en mv */		

extern void 				Mesure_Batterie_IT ( uint32_t Val ); 

#endif
