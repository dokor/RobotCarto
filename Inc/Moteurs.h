
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


typedef enum {
	PILOTAGE_BOUCLE_OUVERTE = 1,
	PILOTAGE_MANUEL, 
	CALIBRATION_MAGNETOMETRE,
	PILOTAGE_AUTONOME,  
	PILOTAGE_MANUEL_ASSISTE 
} E_Mode_Commande ; 
#define T_Mode_Commande uint8_t 

extern T_Mode_Commande Mode_Commande ;



/* Le type et les noms des commandes pour la fonction Moteur_Consigne */ 
typedef	enum { 
	M_ARRET	= 1,
	M_ARRET_FREIN, 				
	M_AVANT, 
	M_ARRIERE,	
	M_MAX_COMMANDE			// à utiliser pour test borne 
}	E_CommandeMoteur ;

#define	T_CommandeMoteur uint16_t 


typedef struct {
	int16_t   				CoefP;							/* Proportionnel */
	int16_t  					CoefI;							/* Integrale */
	int16_t   				CoefD;							/* Dérivée */	
	int16_t						Commande_Moteur ;		/* Valeur appliquée au PWM 0 à */	
	int16_t						ErreurPrecedente ;	/* Erreur T-1 pour calcul dérivée */
	int16_t						ErreurSomme ;				/* Somme pour calcul Intégrale */	
	uint16_t 					Consigne ;		
	uint16_t					Encodeur_Delta ;	
	uint16_t					Encodeur_Precedent ;
	T_CommandeMoteur 	Commande ;	
} T_Etat_Moteur ; 

extern T_Etat_Moteur Etat_Moteur[MAX_MOTEUR]  ;



typedef struct {
	int16_t					CoefA;
	int16_t					CoefB;	
}T_CoefAB;

typedef struct {
	T_CoefAB							MarcheAvant;
	T_CoefAB							MarcheArriere;	
}T_Compensation_Ligne_Droite;

extern T_Compensation_Ligne_Droite Compensation_Ligne_Droite;

#define		PERIODE_ASS_VITESSE	    	10000		// 20 ms période d'asservissement vitesse des moteurs 
#define		PERIODE_ASS_VITESSE_S			( (float)PERIODE_ASS_VITESSE / 1000000.0f ) 
#define		DEMI_ENTRE_ROUE					 	107.0f		// Mesure entre axe de roue / 2 (mm) 		
#define		COEF_REDUCTEUR 				  	74.83f		// Rapport de réduction du moteur																					
#define		ENCODEUR_PAR_TOUR				 	3592.0f		  // pour une résolution de roue
#define		ENCODEUR_PAR_PERIODE			( ENCODEUR_PAR_TOUR	 * PERIODE_ASS_VITESSE_S ) 																				
#define		RAYON_DE_ROUE							 26.2f		// en mm  


/* Pour générer le code, ne conserver que la nature de vitesse désirée */
 //#define		VITESSE_TOURS_PAR_SECONDE
// #define		VITESSE_ANGULAIRE 						// Rad/s
	 #define		VITESSE_LINEAIRE 							// mm/s
// #define		VITESSE_ENCODEUR							// abs(Delatencodeur) 	/* utile pour lecture vitesse uniquement 

#define				CONSIGNE_MAX_VITESSE_LINEAIRE 				300 
#define				CONSIGNE_MAX_VITESSE_ANGULAIRE 				(float) ( CONSIGNE_MAX_VITESSE_LINEAIRE  / RAYON_DE_ROUE )
#define				CONSIGNE_MAX_TOURS_PAR_SECONDE 				(float) ( CONSIGNE_MAX_VITESSE_ANGULAIRE / ( 2.0 * 3.14159) ) 
#define				CONSIGNE_NULLE_VITESSE_LINAIRE				 20 		// en dessous considéré comme 0 

#define				CONSIGNE_ANGULAIRE_NULLE							 0.0f  	// en dessous considéré comme 0


typedef struct {
	float		Vitesse ;
	float 	ConsigneAngulaire ;
} T_ConsigneManuelle ;

extern T_ConsigneManuelle  ConsigneManuelle  ;

typedef struct {
	T_CommandeMoteur Commande ;
	uint16_t 				 Vitesse ;
} T_ConsigneBoucle_Ouverte ;

extern T_ConsigneBoucle_Ouverte ConsigneBoucle_Ouverte[MAX_MOTEUR] ; 		



extern	float VitesseMesure	[MAX_MOTEUR] ;

extern	 float ConsigneAngulaireNulle ;
/* Le type et les noms de référence des servos  */ 
typedef	enum { 
	SERVO1	= 0,
	SERVO2, 
	MAX_SERVO
}	E_Servo ;

#define	T_Servo	uint8_t 

/* Renommer les servo en fonction de leur utilisation */
#define	SERVO_SITE			SERVO1 
#define	SERVO_AZIMUT		SERVO2 


#define OFFSET_SERVO1   -3				
#define	OFFSET_SERVO2		6 			// 6°

extern uint16_t		V_Batterie ;

extern void 			Moteurs_Start			 					(void); 

extern float  		Moteur_Vitesse 							( T_Moteur Moteur ) ;

extern void 			Robot_Commande 							( float Consigne, float RayonDeCourbure ) ;

extern void 			Servos_Start_PWM 						(void) ;

extern void 			Servo_Position 							(T_Servo Servo, int16_t position) ;


extern void				ASServissement_VitesseMoteur 	(void);




#endif 


