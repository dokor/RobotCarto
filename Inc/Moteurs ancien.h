
#ifndef 	H_MOTEUR
#define		H_MOTEUR 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"

/* Le type et les noms de référence des moteurs */ 
typedef	enum { 
	MOTEUR1	= 0,
	MOTEUR2,
	MAX_MOTEUR
}	E_Moteur ;

#define	T_Moteur	uint8_t 


/* Renommer les moteurs en fonction de leur position */
#define MOTEUR_DROIT 	MOTEUR1
#define MOTEUR_GAUCHE MOTEUR2


/* Le type et les noms des commandes pour la fonction Moteur_Consigne */ 
typedef	enum { 
	M_ARRET	= 0,
	M_ARRET_FREIN, 				
	M_AVANT, 
	M_ARRIERE,
	M_VITESSE, 
	M_MAX_COMMANDE			// à utiliser pour test borne 
}	E_CommandeMoteur ;

#define	T_CommandeMoteur uint8_t 


/* Le type et les noms de référence des servos  */ 
typedef	enum { 
	SERVO1	= 0,
	MAX_SERVO
}	E_Servo ;

#define	T_Servo	uint8_t 

/* Renommer les servo en fonction de leur utilisation */
#define	SERVO_DIRECTION			SERVO1  // ceci est un exemple 
extern  T_Boolean Mesure_Vitesse_Prete (T_Moteur Moteur)   ;
extern  float	Lire_VitesseMoteur (T_Moteur Moteur)  ;
extern void 			Moteurs_Start			 					(void); 

extern void 			Moteur_Consigne 						( T_Moteur Moteur, T_CommandeMoteur Commande, float Consigne ) ;

extern float  		Moteur_Vitesse 							( T_Moteur Moteur ) ;

extern void 			Robot_Commande ( T_CommandeMoteur Commande, uint16_t Vitesse, int16_t Angle ) ;

extern  void Moteur_Commande ( T_Moteur Moteur, T_CommandeMoteur Commande, uint16_t Vitesse ) ;

extern void 			Servos_Start_PWM 						(void) ;

extern void 			Servo_Position 							(T_Servo Servo, int16_t position) ;



extern void 		VitesseMoteur_Mesure_IT 			(TIM_HandleTypeDef *htim) ;

extern void 		VitesseMoteur_Debordement_IT 	(TIM_HandleTypeDef *htim) ;

extern void			ASServissement_VitesseMoteur 	(void);


#endif 


