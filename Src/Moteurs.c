

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "typegen.h"
#include "tempo.h"
#include "moteurs.h"
#include "stdlib.h"
#include "HMC5883L.h"
#include "US_Capteur.h"


/* Renomme les pins de commande des moteurs 
		toutes ces pins doivent appartenir au même GPIO			*/
#define		GPIOMOTEUR								GPIOF	

#define		MOTEUR1_BRAKE							GPIO_PIN_2
#define		MOTEUR1_DIRECTION					GPIO_PIN_14
#define   MOTEUR1_FAUTE							GPIO_PIN_12

#define		MOTEUR2_BRAKE							GPIO_PIN_5
#define		MOTEUR2_DIRECTION					GPIO_PIN_15
#define   MOTEUR2_FAUTE							GPIO_PIN_13


/* Renomme les timers et canaux utilisés */
#define		Timer_Moteur_1  					htim10
#define		PWM_MOTEUR_1							TIM_CHANNEL_1
#define		Timer_Moteur_2  					htim11 
#define		PWM_MOTEUR_2							TIM_CHANNEL_1

#define		Timer_Encodeur_1					htim4
#define		CAPTURE_VITESSEMOTEUR_1		TIM_CHANNEL_3 

#define		Timer_Encodeur_2					htim3
#define		CAPTURE_VITESSEMOTEUR_2		TIM_CHANNEL_3 

#define		Timer_Servo_1							htim9
#define		PWM_SERVO_1								TIM_CHANNEL_1 
#define		Timer_Servo_2							htim9
#define		PWM_SERVO_2								TIM_CHANNEL_2 



/* Structure de configuration/commande d'un moteur */
typedef struct  {
		uint16_t					Brake_Gpio ;
		uint16_t					Direction_Gpio ;
	  uint16_t					Faute_Gpio ;
		TIM_HandleTypeDef *htim ;	
		uint32_t 					Canal ;
}	T_Config_Moteur ; 

const	T_Config_Moteur Config_Moteur[MAX_MOTEUR]	= { { MOTEUR1_BRAKE, MOTEUR1_DIRECTION, MOTEUR1_FAUTE, &Timer_Moteur_1, PWM_MOTEUR_1 },
																										{ MOTEUR2_BRAKE, MOTEUR2_DIRECTION, MOTEUR2_FAUTE, &Timer_Moteur_2, PWM_MOTEUR_2 }	};																										
																										


T_Etat_Moteur Etat_Moteur[MAX_MOTEUR] = { { 6, 4, 0, 0, 0, 0, 0, 0, 0, M_ARRET },
																					{ 6, 4, 0, 0, 0, 0, 0, 0, 0, M_ARRET }}  ;
										

T_Compensation_Ligne_Droite Compensation_Ligne_Droite = { {2, 5},		//avant  Compensation ligne droite valeurs par défaut
														  {0, 5} }; 	// arriere
																					
	/* pour le mode de commande PILOTAGE_BOUCLE_OUVERTE */
T_ConsigneBoucle_Ouverte ConsigneBoucle_Ouverte[MAX_MOTEUR] = {  {M_ARRET, 0 },
																																 {M_ARRET, 0 } }; 																				
	

																					
	/* pour le mode de commande PILOTAGE_MANUEL */
T_ConsigneManuelle  ConsigneManuelle  = {0.0f, 0.0f};




#define			PERIODE_ASSERVISSEMENT_CAP  33


float ConsigneAngulaireNulle = 0.0 ; 
																					
float VitesseMesure	[MAX_MOTEUR] ;


T_Mode_Commande Mode_Commande = PILOTAGE_MANUEL ;




#ifdef DEBUG_PID
#define MAX_DBG_PID     4000		// soit 4s d'enregistrement 
uint16_t dbg_pid[MAX_MOTEUR][MAX_DBG_PID] ;
#endif																				

																					

																					
/* démarre les PWMs gestion des moteurs avec valeur par défaut (0) */
static void Moteurs_Start_PWM ( void) 
{
	HAL_TIM_PWM_Start    (&Timer_Moteur_1,PWM_MOTEUR_1);
	HAL_TIM_PWM_Start    (&Timer_Moteur_2,PWM_MOTEUR_2);
	
	HAL_TIM_Encoder_Start(&Timer_Encodeur_1, TIM_CHANNEL_ALL) ;
	HAL_TIM_IC_Start		 (&Timer_Encodeur_1, CAPTURE_VITESSEMOTEUR_1);	
	
	HAL_TIM_Encoder_Start(&Timer_Encodeur_2, TIM_CHANNEL_ALL) ;
	HAL_TIM_IC_Start		 (&Timer_Encodeur_2, CAPTURE_VITESSEMOTEUR_2);
}


/* 	Commande le moteur 
		Remarque : il faut préalablement avoir démarré les PWM  Vitesse de 0 à 1000	*/
static void Moteur_Commande ( T_Moteur Moteur, T_CommandeMoteur Commande, uint16_t Vitesse )
{
	if ( Moteur <	MAX_MOTEUR ) 
	{	/* SI le moteur existe */
		T_Config_Moteur const *config_moteur ;
		config_moteur = &Config_Moteur[Moteur] ;
		
		// ajouter gestion de Faute du moteur si disponible il faut tension alim > 11, 5 v 
				
		switch ( Commande )
		{
			case M_ARRET :				// Free running motor
				HAL_GPIO_WritePin(GPIOMOTEUR, config_moteur->Brake_Gpio,  GPIO_PIN_RESET );	
				Vitesse = 0; 				// force à 0
			break ; 
			case M_ARRET_FREIN :	// Fast motor stop 
				HAL_GPIO_WritePin(GPIOMOTEUR, config_moteur->Brake_Gpio,  GPIO_PIN_SET );						
				Vitesse = 0; 				// force à 0			
			break ;
			case M_AVANT :				// Marche Avant 		
				HAL_GPIO_WritePin(GPIOMOTEUR, config_moteur->Direction_Gpio,  GPIO_PIN_SET );					
				HAL_GPIO_WritePin(GPIOMOTEUR, config_moteur->Brake_Gpio,  GPIO_PIN_SET );			// slow decay 				
			break ;
			case M_ARRIERE :			// Marche Arriere 						
				HAL_GPIO_WritePin(GPIOMOTEUR, config_moteur->Direction_Gpio,  GPIO_PIN_RESET );					
				HAL_GPIO_WritePin(GPIOMOTEUR, config_moteur->Brake_Gpio,  GPIO_PIN_SET );			// slow decay 				
			break ; 
			default :
				return ;

		}
		if ( Vitesse > 600 )   // voir à limiter si V alim très > à V max moteur
			Vitesse = 600;
			
	 __HAL_TIM_SetCompare(config_moteur->htim, config_moteur->Canal, Vitesse);				
	}
} 



/* 	Démarrage des PWM, de la mesure de vitesse, et de la procédure d'asservissement vitesse */
void 	Moteurs_Start (void)
{
	Moteurs_Start_PWM ();
	Moteur_Commande ( MOTEUR_DROIT,  M_ARRET, 0 )	;	
	Moteur_Commande ( MOTEUR_GAUCHE, M_ARRET, 0 )	;		
	Arme_Tempo_ASS_Vitesse ( PERIODE_ASS_VITESSE ) ; 	
}


static void 	Moteur_Consigne ( T_Moteur Moteur, T_CommandeMoteur Commande, float Consigne )
{
	if ( Moteur <	MAX_MOTEUR ) 
	{	/* SI le moteur existe */
		T_Etat_Moteur *etat_moteur = &Etat_Moteur[Moteur] ;
		if ( Commande < M_MAX_COMMANDE )
		{
			float consigne = Consigne ;

			etat_moteur->Commande = Commande ;

			#ifdef VITESSE_TOURS_PAR_SECONDE 
				etat_moteur->Consigne = consigne * ( ENCODEUR_PAR_PERIODE ) ;
			#else
				#ifdef VITESSE_ANGULAIRE 
					etat_moteur->Consigne = consigne * ( ( ENCODEUR_PAR_PERIODE ) / ( 2.0f * 3.14159f ) );			
				#else 
					#ifdef VITESSE_LINEAIRE 
						etat_moteur->Consigne = consigne * ( ( ENCODEUR_PAR_PERIODE ) / ( 2.0f * 3.14159f * RAYON_DE_ROUE ) );
					#endif
				#endif
			#endif	
			if ( Moteur == MOTEUR_DROIT )
			{
				uint16_t coef_a = 0 ;
				uint16_t coef_b = 0 ;
				if ( Commande == M_AVANT ) 
				{
					coef_a = Compensation_Ligne_Droite.MarcheAvant.CoefA ; 
					coef_b = Compensation_Ligne_Droite.MarcheAvant.CoefB ; 
				}
				else
				{
					if ( Commande == M_ARRIERE ) 
					{
						coef_a = Compensation_Ligne_Droite.MarcheArriere.CoefA ; 
						coef_b = Compensation_Ligne_Droite.MarcheArriere.CoefB ; 
					}
				}
				etat_moteur->Consigne += (coef_a + ( (etat_moteur->Consigne * coef_b) / 100  ) ); 
			}		
		}
	}		
}


/* Pour remonter la vitesse au niveau de l'application */
float  Moteur_Vitesse ( T_Moteur Moteur )
{
	if ( Moteur <	MAX_MOTEUR ) 
	{	/* SI le moteur existe */		
		#ifdef VITESSE_ENCODEUR 
			return ( Etat_Moteur[Moteur].Encodeur_Delta ) ;
		#else
			#ifdef VITESSE_TOURS_PAR_SECONDE 
				return ( Etat_Moteur[Moteur].Encodeur_Delta / ( ENCODEUR_PAR_PERIODE  ) ) ;
			#else
				#ifdef VITESSE_ANGULAIRE 
				/* Calcul de la vitesse angulaire de la roue en rad/s */
					return (( Etat_Moteur[Moteur].Encodeur_Delta / ( ENCODEUR_PAR_PERIODE  ) ) * 2.0f * 3.14159f ) ;
				#else
					#ifdef VITESSE_LINEAIRE 
						/* Calcul de la vitesse linéaire de la roue en mm/s */
						return (( Etat_Moteur[Moteur].Encodeur_Delta / ( ENCODEUR_PAR_PERIODE  ) ) * 2.0f * 3.14159f * RAYON_DE_ROUE );
					#endif
				#endif
			#endif
		#endif
	}
	else 
		return 0.0f ;
}






/* Consigne de vitesse linéaire en mm/s ou rad/s ou tour/s  et ConsigneAngulaire en °/s */
void Robot_Commande ( float ConsigneVitesse, float ConsigneAngulaire )
{
/*	static float ErreurSomme = 0; 
	static float ErreurPrecedente = 0;*/
	float cons_max = CONSIGNE_MAX_VITESSE_LINEAIRE;
	/* il faut transformer la consigne de vitesse selon son unité en vitesse linaire */
	#ifdef VITESSE_TOURS_PAR_SECONDE 
		Consigne *= ( 2.0f * 3.14159f * RAYON_DE_ROUE ) ;
	#else
		#ifdef VITESSE_ANGULAIRE 
			Consigne *= RAYON_DE_ROUE ;
		#else 
			#ifdef VITESSE_LINEAIRE 
					// pas de conversion 
			#endif
		#endif
	#endif	
	
	if ( 			( abs ( ConsigneVitesse )   < CONSIGNE_NULLE_VITESSE_LINAIRE ) 
				&&  ( abs ( ConsigneAngulaire ) < CONSIGNE_ANGULAIRE_NULLE 			 )   )	
	{
		Moteur_Consigne ( MOTEUR_DROIT,   M_ARRET,   0.0f ) ;	
		Moteur_Consigne ( MOTEUR_GAUCHE,  M_ARRET,   0.0f ) ;	
 
	//	Suivi_De_Cap.CapInitial = Compas;  // Lecture de la valeur initiale du compas 
/*		ErreurSomme = 0; 
		ErreurPrecedente = 0; */

	}
	else
	{
		if ( abs (ConsigneAngulaire) < CONSIGNE_ANGULAIRE_NULLE )
		{
		
#ifdef	V_ANGULAIRE_AUTO			
			 if ( ConsigneVitesse >  0.0f )
				ConsigneAngulaire = 		Compensation_Ligne_Droite.MarcheAvant.CoefA * ConsigneVitesse 
														 +	Compensation_Ligne_Droite.MarcheAvant.CoefB ;
			else
				ConsigneAngulaire = 		Compensation_Ligne_Droite.MarcheArriere.CoefA * ConsigneVitesse 
														 +	Compensation_Ligne_Droite.MarcheArriere.CoefB ;
			ConsigneAngulaireNulle = ConsigneAngulaire  ;
#else
			ConsigneAngulaire = ConsigneAngulaireNulle  ;
#endif
		
/*			Suivi_De_Cap.CapCourant = Compas;
			Suivi_De_Cap.DeltaCap = Suivi_De_Cap.CapCourant - Suivi_De_Cap.CapInitial; // erreur 
			ErreurSomme += Suivi_De_Cap.DeltaCap;
			ErreurPrecedente = Suivi_De_Cap.DeltaCap ;
			
			Suivi_De_Cap.ConsigneAngulaire = Suivi_De_Cap.DeltaCap * Suivi_De_Cap.CoefP + Suivi_De_Cap.CoefI * ErreurSomme ;
			ConsigneAngulaire = Suivi_De_Cap.ConsigneAngulaire; */
		}
		
		/* il faut transformer la consigne Angulaire exprimée en °/s en rad/s */ 
		ConsigneAngulaire *= (3.14159f / 180.0f) ;
			
		float deltaC 		   = DEMI_ENTRE_ROUE * ConsigneAngulaire; 
		float cons_gauche  = ConsigneVitesse - deltaC ;
		float	cons_droite  = ConsigneVitesse + deltaC ;
		float plus_grande_Consigne = cons_droite ;
		
		/* il faut borner la consigne calculée par rapport à consigne max admissible */
		if ( abs (cons_droite) < abs ( cons_gauche ) )
		{
			plus_grande_Consigne = cons_gauche ;				
		}
		if ( abs (plus_grande_Consigne) > cons_max )
		{
			float cons_rapport = cons_max / plus_grande_Consigne ;
			cons_gauche *= cons_rapport ;
			cons_droite *= cons_rapport ;
		}			
		Moteur_Consigne ( MOTEUR_DROIT,  cons_droite > 0 ? M_AVANT : M_ARRIERE, abs(cons_droite) ) ;			
		Moteur_Consigne ( MOTEUR_GAUCHE, cons_gauche > 0 ? M_AVANT : M_ARRIERE, abs(cons_gauche) ) ;				 
	}	
}






/* fonction d'asservissement vitesse moteur appelée par interruption tempo périodique
   à réarmer à chaque appel (durée d'exécution environ 3 µs avec callback interruption ) */
void	ASServissement_VitesseMoteur 	(void)
{
	T_Moteur 				 moteur ;	 
	T_CommandeMoteur direction ; 

#ifdef DEBUG_PID	
	static uint16_t cpt_dbg_pid = 0 ; 
#endif	
	
	/* lire la vitesse des moteurs */		
	for ( moteur = MOTEUR1 ; moteur < MAX_MOTEUR ; moteur ++ )
	{	
		uint16_t encodeur ;
		T_Etat_Moteur *etat_moteur = &Etat_Moteur[moteur] ; 

		if ( moteur == MOTEUR1 )
		{
			encodeur  =  __HAL_TIM_GET_COMPARE( &Timer_Encodeur_1, CAPTURE_VITESSEMOTEUR_1 )  ;
			direction =  __HAL_TIM_IS_TIM_COUNTING_DOWN ( &Timer_Encodeur_1 )  ?  M_ARRIERE : M_AVANT ;			
    }			
		else
		{
			encodeur  =  __HAL_TIM_GET_COMPARE( &Timer_Encodeur_2, CAPTURE_VITESSEMOTEUR_2 ) ;
			direction =  __HAL_TIM_IS_TIM_COUNTING_DOWN ( &Timer_Encodeur_2 )  ?  M_ARRIERE : M_AVANT ;			
		}
		etat_moteur->Encodeur_Delta 		= encodeur - etat_moteur->Encodeur_Precedent ;
		etat_moteur->Encodeur_Precedent = encodeur ;

		if ( etat_moteur->Encodeur_Delta == 0 ) 
			direction = etat_moteur->Commande ; 	/* si moteur à l'arrêt pour pouvoir redémarrer + cf** */
		
		if ( direction == M_ARRIERE ) 
			etat_moteur->Encodeur_Delta = ( ~ etat_moteur->Encodeur_Delta ) + 1 ;  
		
	
		
		if  ( Mode_Commande != PILOTAGE_BOUCLE_OUVERTE )
		{	/* asservissement de la vitesse des moteurs . */		
			{
				int16_t erreur, delta_erreur ;
				if ( ( etat_moteur->Consigne != 0 ) && ( direction == etat_moteur->Commande ) )  // cf** pour parer aux changements brutaux de direction 
				{
					erreur				 					 			= etat_moteur->Consigne - etat_moteur->Encodeur_Delta ;
					delta_erreur  		 			 			= erreur - etat_moteur->ErreurPrecedente ;
					etat_moteur->ErreurSomme			+= erreur ;
					etat_moteur->ErreurPrecedente = erreur ;
#ifdef DEBUG_PID					
					if ( cpt_dbg_pid < MAX_DBG_PID ) 
						dbg_pid[moteur][cpt_dbg_pid ] = etat_moteur->Encodeur_Delta; 
#endif				 			
				}
				else
				{
					erreur				 					 			= 0 ;
					delta_erreur  		 			 			= 0 ;
					etat_moteur->ErreurSomme			= 0 ;
					etat_moteur->ErreurPrecedente = 0 ;				
				}

				etat_moteur->Commande_Moteur = 		 etat_moteur->CoefP *  erreur  
																				+	 etat_moteur->CoefI * etat_moteur->ErreurSomme
																				+	 etat_moteur->CoefD * delta_erreur ;
																			
				if ( etat_moteur->Commande_Moteur < 0 )
					etat_moteur->Commande_Moteur = 0 ; 
				// la borne max est testée dans Moteur_Commande */
			}		
			Moteur_Commande	(moteur, etat_moteur->Commande, etat_moteur->Commande_Moteur ) ; 	
		}
		else 
		{ /* Boucle ouverte */
			Moteur_Commande (moteur, ConsigneBoucle_Ouverte[moteur].Commande,  ConsigneBoucle_Ouverte[moteur].Vitesse ) ;					
		}
	}
#ifdef DEBUG_PID		
	cpt_dbg_pid ++ ;
#endif
}



/* démarre les PWMs gestion des servos avec valeur par défaut (1500 soit 0°) */
void Servos_Start_PWM (void) 
{
	HAL_TIM_PWM_Start(&Timer_Servo_1,PWM_SERVO_1);
	HAL_TIM_PWM_Start(&Timer_Servo_1,PWM_SERVO_2);
}

/* 	Commande le servo  
		Remarque : il faut préalablement avoir démarré les PWM 	*/ 
void Servo_Position ( T_Servo  Servo, int16_t Position )
{			
	int valtimer ;
	switch ( Servo )
	{
		case SERVO1 :		/* à  revoir */
		{	/* pilote servo 1 selon caractéristiques ci-dessous avec position de -90 à + 90° BOFF
											 900µs = -90°   1500µs = 0°    2100µs = +90° 
				 pour le PWM	 900						1500					 2100   			soit 10µs par ° 		*/
			Position = -Position ;  // On inverse le signe pour gérer angle de site dans lebon sens / à orientation du servo 
			/* borne la valeur */
			if ( Position < -90 ) 
				Position = -90 ;
			else
				if ( Position > 60 )  // limite mécanique du servo 
					Position = 60 ;

			/* on passe tout en positif donc position de 0 à 180° */
			Position += ( 90  + OFFSET_SERVO1 );  
			valtimer = ( Position * 10) + 600 ;
			__HAL_TIM_SetCompare(&Timer_Servo_1, PWM_SERVO_1, valtimer);
		}
		break ; 
		
		case SERVO2 :
		{	/* pilote servo 1 selon caractéristiques ci-dessous avec position de -60 à + 60° 
											 900µs = -60°   1500µs = 0°    2100µs = +60° 
				 pour le PWM	 900						1500					 2100   			soit 6.6µs par ° 		*/

			/* borne la valeur */
			if ( Position < -90) 
				Position = -90 ;
			else
				if ( Position > 90 ) 
					Position = 90 ;
			/* on passe tout en positif donc position de 0 à 120° */
			Position +=  ( 90  + OFFSET_SERVO2 ); 
			valtimer = ( Position * 10 ) + 600 ;
			__HAL_TIM_SetCompare(&Timer_Servo_2, PWM_SERVO_2, valtimer);
		}
		break ; 
		
		default :
		break ;
	}
} 
