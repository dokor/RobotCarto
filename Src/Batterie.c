/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "typegen.h"
#include "Batterie.h"


uint16_t		V_Batterie ;							/* Exprimée en mv */		
																					
																					
/* Interruption sur lecture analogique  
	 fonction appelée par call back de l'ADC3 HAL_ADC_ConvCpltCallback 
	 dans fichier stm32f4xx_it.c	
	 L'ADC3 fait une mesure toutes les PERIODE_ASS_VITESSE déclenchée par OC du Timer5
	 un filtrage par moyenne simple de NOMBRE_CUMUL_BATTERIE échantillons donne une mesure de 
	 la tension toutes les PERIODE_ASS_VITESSE * NOMBRE_CUMUL_BATTERIE  
	 La fonction a été construite de manière à ne pas utiliser de nombre float 
		=> pas de sauvegarde des registres du coprocesseur 	*/																																										

void Mesure_Batterie_IT ( uint32_t Val )
{
#define COEF_CONVERSION_DIVISEUR  4660		// * 1000 et à ajuster expérimentalement 
#define	NOMBRE_CUMUL_BATTERIE 		100 

static	uint32_t Tension_Batterie_Cumul = 0 ; 
static	uint16_t  NbCumul = 0 ;	
	
	Tension_Batterie_Cumul += Val ;	
	
	if ( ++NbCumul >= NOMBRE_CUMUL_BATTERIE )
	{
		Tension_Batterie_Cumul = ((TENSION_ANALOGIQUE * Tension_Batterie_Cumul) / (4095 * NOMBRE_CUMUL_BATTERIE ) ) * COEF_CONVERSION_DIVISEUR  ; 
		V_Batterie = Tension_Batterie_Cumul / 1000 ;
		NbCumul = 0 ;
		Tension_Batterie_Cumul = 0 ;
	}	
}


