

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "typegen.h"
#include "Phares.h"


/* Renomme le timer de gestion des phares Avant */
#define		Timer_Phares  						htim13
#define		PWM_PHARES								TIM_CHANNEL_1

uint16_t Phare_Luminosite  = 0 ; 		 // exprimée de 0 à 1000

void Phares_Start ( void )
{
	HAL_TIM_PWM_Start    (&Timer_Phares,PWM_PHARES);	
}	
		

void Phares_Commande (void)
{
	 __HAL_TIM_SetCompare(&Timer_Phares, PWM_PHARES, Phare_Luminosite );			
}


