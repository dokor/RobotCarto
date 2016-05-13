/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "US_Capteur.h"
#include "typegen.h"

/* TIM2, pour gérer des temporisations avec forte priorité */
/***********************************************************/

/* TIM2, TIM_CHANNEL_1 exclusivement Réservé pour la gestion des capteurs US */ 
void Arme_Tempo_USCapteur (uint32_t  Duree)
{
	__HAL_TIM_SET_COMPARE  ( &htim2, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim2) + Duree );
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);	
}

/* TIM2, TIM_CHANNEL_1 exclusivement Réservé pour la gestion des capteurs US 
		l'utilisation de cette fonction implique que Arme_Tempo_USCapteur a été 
		préalablement appelé																														*/ 
void Relance_Tempo_USCapteur (uint32_t  Duree)
{
	__HAL_TIM_SET_COMPARE  ( &htim2, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim2) + Duree );
}

/* TIM2, TIM_CHANNEL_1 exclusivement Réservé pour la gestion des capteurs US 
	 inhibe le canal et désactive interruption */
void Desarme_Tempo_USCapteur  ( void )
{
	HAL_TIM_OC_Stop_IT (&htim2, TIM_CHANNEL_1);
}	




/* TIM5, pour gérer des temporisations avec faible priorité */
/************************************************************/

/* TIM5, TIM_CHANNEL_1 exclusivement Réservé pour la gestion des capteurs US */ 
void Arme_Tempo_LCD16x2 (uint32_t  Duree)
{
	__HAL_TIM_SET_COMPARE  ( &htim5, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim5) + Duree );
	HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);	
}

/* TIM5, TIM_CHANNEL_1 exclusivement Réservé pour la gestion de la carte LCD16x2 via I2C
		l'utilisation de cette fonction implique que Arme_Tempo_LCD16x2 a été 
		préalablement appelé																														*/ 
void Relance_Tempo_LCD16x2 (uint32_t  Duree)
{
	__HAL_TIM_SET_COMPARE  ( &htim5, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim5) + Duree );
}

/* TIM5, TIM_CHANNEL_1 exclusivement Réservé pour la gestion de la carte LCD16x2 via I2C
	 inhibe le canal et désactive interruption */
void Desarme_Tempo_LCD16x2  ( void )
{
	HAL_TIM_OC_Stop_IT (&htim5, TIM_CHANNEL_1);
}	


/* TIM5, TIM_CHANNEL_2 exclusivement Réservé pour la gestion de l'asservissement de vitesse */ 
void Arme_Tempo_ASS_Vitesse (uint32_t  Duree)
{
	__HAL_TIM_SET_COMPARE  ( &htim5, TIM_CHANNEL_2, __HAL_TIM_GET_COUNTER(&htim5) + Duree );
	HAL_TIM_OC_Start_IT		 ( &htim5, TIM_CHANNEL_2);	
}

/* TIM5, TIM_CHANNEL_2 exclusivement Réservé pour la gestion de l'asservissement de vitesse 
		l'utilisation de cette fonction implique que Arme_Tempo_ASS_Vitesse a été 
		préalablement appelé																														*/ 
void Relance_Tempo_ASS_Vitesse(uint32_t  Duree)
{
	__HAL_TIM_SET_COMPARE  ( &htim5, TIM_CHANNEL_2, __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2) + Duree );
}

/* TIM5, TIM_CHANNEL_2 exclusivement Réservé pour la gestion de l'asservissement de vitesse 
	 inhibe le canal et désactive interruption */
void Desarme_Tempo_ASS_Vitesse  ( void )
{
	HAL_TIM_OC_Stop_IT (&htim5, TIM_CHANNEL_2);
}	



