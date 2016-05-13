
#ifndef 	H_US_CAPTEUR
#define		H_US_CAPTEUR 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"


/* Le type et les noms de référence des capteurs */ 
typedef	enum { 
	US_CAPTEUR1	= 0,
	US_CAPTEUR2,
	US_CAPTEUR3,
	US_CAPTEUR4,
	US_MAX_CAPTEUR
}	E_US_Capteur ;

#define	T_US_Capteur	uint8_t

/* couverture des capteurs */
#define	US_GAUCHE		US_CAPTEUR3 
#define	US_ARRIERE	US_CAPTEUR2 
#define	US_AVANT		US_CAPTEUR1 
#define	US_DROIT		US_CAPTEUR4 

/* Renommer les capteurs en fonction de leur position */
#define	US_CAPTEUR_AVANT	CAPTEUR1  // ceci est un exemple 

extern T_Boolean 	Us_Mesure_Distance_Prete 	(T_US_Capteur Capteur) ;  

extern uint16_t 	Us_LireDistance 					(T_US_Capteur	Capteur);

extern void  			Us_SelectInterrogation 		(T_US_Capteur	Capteur, T_MarcheArret MarcheArret ) ;

extern void				Us_MarcheArretGeneral			(T_MarcheArret MarcheArret ) ;



extern void 			Us_GestionMesure_IT 			(TIM_HandleTypeDef *htim) ;

extern void 			Us_GestionTrigger_IT 			(TIM_HandleTypeDef *htim) ;

extern	uint16_t 	Distance_Obstacle_Av ; 
extern	uint16_t 	Distance_Obstacle_Ar ; 	
extern T_Boolean 	TestCollisionArriere			(void);
extern T_Boolean 	TestCollisionAvant				(void);
#define			DISTANCE_SECURITE		30


#endif 

