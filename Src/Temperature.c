

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "typegen.h"
#include "temperature.h"


int16_t Temperature_Exterieure = 0 ; 		 // exprimée en 1/10 ° c



/* Interruption sur lecture analogique  
	 fonction appelée par call back de l'ADC3 HAL_ADC_ConvCpltCallback 
	 dans fichier stm32f4xx_it.c	
	 L'ADC3 fait une mesure toutes les PERIODE_ASS_VITESSE déclenchée par OC du Timer5
	 un filtrage par moyenne simple de NOMBRE_CUMUL_TEMP_EXTERIEUR échantillons donne une mesure de 
	 température toutes les PERIODE_ASS_VITESSE * NOMBRE_CUMUL_TEMP_EXTERIEUR  
	 La fonction a été construite de manière à ne pas utiliser de nombre float 
		=> pas de sauvegarde des registres du coprocesseur 	
		C'est une mesure différentielle */	
		
void Mesure_Temperature_IT (uint32_t Low_Val, uint32_t High_Val)
{
#define	NOMBRE_CUMUL_TEMP_EXTERIEUR 500 
	
static	uint32_t Temperature_Exterieure_Cumul = 0 ; 
static	uint16_t NbCumul = 0 ;	
	
	Temperature_Exterieure_Cumul += ( High_Val - Low_Val );	
	if ( ++NbCumul >= NOMBRE_CUMUL_TEMP_EXTERIEUR )
	{
		Temperature_Exterieure = (TENSION_ANALOGIQUE * Temperature_Exterieure_Cumul) / (4095 * NOMBRE_CUMUL_TEMP_EXTERIEUR)  ; // en 1/10 ° c
		NbCumul = 0 ;
		Temperature_Exterieure_Cumul = 0 ;
	}
}



