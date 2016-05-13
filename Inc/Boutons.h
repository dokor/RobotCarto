
#ifndef	H_BOUTONS
#define H_BOUTONS


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"


typedef enum {
	BOUTON_ON_BOARD			= 0 ,
	BOUTON_BUT1, 							// BUT1 sur afficheur LCD16x2 si présent 
	BOUTON_BUT2, 							// BUT1 sur afficheur LCD16x2 si présent 
	BOUTON_BUT3, 							// BUT1 sur afficheur LCD16x2 si présent 
	BOUTON_BUT4, 							// BUT1 sur afficheur LCD16x2 si présent 	
	BOUTON1_SUR_EXTENSION,		// BOUTON 1 sur carte extension 
	BOUTON2_SUR_EXTENSION			// BOUTON 2 sur carte extension 
} E_Bouton ;

#define	T_Bouton int8_t 


extern T_Boolean	Bouton_Etat	(T_Bouton Bouton) ;


#endif
