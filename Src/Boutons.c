/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"
#include "LCD16x2.h"
#include "WifiCom.h" 
#include "boutons.h"


T_Boolean	Bouton_Etat	(T_Bouton Bouton) 
{
#ifdef	LCD16X2 	
	uint16_t  masque_but = 0x0001 ;
#endif
	T_Boolean	etat =	FAUX ; 
	switch ( Bouton )
	{
		case	BOUTON_ON_BOARD :
			etat =  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) ; // on fera mieux plus tard 
		break ; 
#ifdef	LCD16X2 			
		case	BOUTON_BUT1 :
		case	BOUTON_BUT2 :
		case	BOUTON_BUT3 :		
		case	BOUTON_BUT4 :
			masque_but <<= ( Bouton - BOUTON_BUT1 ) ;
		  etat =  ( LireEtatBoutons_Du_LCD16x2 () &  masque_but ) ? VRAI : FAUX ;	
		break ; 
#endif		
		case BOUTON1_SUR_EXTENSION :
			etat =  HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14) ; // on fera mieux plus tard 
		break ; 
		case BOUTON2_SUR_EXTENSION :
			etat =  HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15) ; // on fera mieux plus tard 
		break ; 
		default :
		break ;  
	}
	return ( etat );
}



