
#ifndef 	H_TEMPO
#define		H_TEMPO 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"




extern 	void Arme_Tempo_USCapteur    (uint32_t  Duree) ;

extern	void Relance_Tempo_USCapteur (uint32_t  Duree) ;

extern  void Desarme_Tempo_USCapteur ( void ) ;


extern void Arme_Tempo_LCD16x2 (uint32_t  Duree) ;

extern void Relance_Tempo_LCD16x2 (uint32_t  Duree) ;

extern void Desarme_Tempo_LCD16x2  ( void ) ;


extern void Arme_Tempo_ASS_Vitesse (uint32_t  Duree) ;
																													
extern void Relance_Tempo_ASS_Vitesse(uint32_t  Duree);

extern void Desarme_Tempo_ASS_Vitesse  ( void );

#endif
