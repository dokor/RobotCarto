

#ifndef H_LCD16x2 
#define H_LCD16x2 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"

extern	uint8_t					EtatDesBoutonsLCD ; 

#define	LireEtatBoutons_Du_LCD16x2() (EtatDesBoutonsLCD)   



extern	void 						LCD16x2_Initialise ( void ) ;

extern	T_Boolean				LCD16x2_Disponible ( void ) ;

extern 	T_OuiNonErreur	LCD16x2_ClearScreen ( void ) ;

extern 	T_OuiNonErreur	LCD16x2_SetBackLight ( uint8_t Niveau ) ;



extern 	T_OuiNonErreur	LCD16x2_GetId ( void ) ; 

extern 	T_OuiNonErreur	LCD16x2_GetFirmware ( void ) ;

extern 	T_OuiNonErreur	LCD16x2_WriteData (char *Buf, uint8_t Taille, uint8_t PosX, uint8_t PosY ) ; 

extern 	T_OuiNonErreur	LCD16x2_GetButton ( void ) ;






extern void LCD16x2_Gestion_Tempo_IT ( void ) ;

extern void LCD16x2_Emission_IT ( void ) ;

extern void LCD16x2_Reception_IT ( void ) ;







#endif
