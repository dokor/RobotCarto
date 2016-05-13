
#ifndef 	H_SAUVE_RESTAURE
#define		H_SAUVE_RESTAURE 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"

extern	T_OuiNonErreur Etat_Sauvegarde;


extern void Gestion_Sauvegarde ( void ) ;


extern	HAL_StatusTypeDef Restaure_Tout ( void  ) ;


extern	void Sauvegarde_Tout_Request (void) ;


extern  void Initialise_Sauvegarde ( void  ) ;


#endif

