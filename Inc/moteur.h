/**
  ******************************************************************************
  * File Name          : MOTEUR.h
  * Date               : 13/07/2015 
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
 
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __moteur_H
#define __moteur_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

extern void MX_StartPWM_Moteurs ( void) ;

extern void MX_Vitesse_Moteur_Droit ( unsigned int vitesse );

extern void MX_Vitesse_Moteur_Gauche ( unsigned int vitesse );

extern void MX_StartPWM_Servos ( void) ;

extern void MX_Position_Servo1 ( int position );



#ifdef __cplusplus
}
#endif
#endif /*__ moteur_H */

/**
  * @}
  */

/**
  * @}
  */


