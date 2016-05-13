
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"
#include "US_Capteur.h"
#include "Moteurs.h"
#include "Rplidar.h"
#include "Guidage.h"
#include "WifiCom.h"
#include "stdlib.h"
#include "math.h"



T_Suivi_De_Mur  Suivi_De_Mur = { 90.0f, 45.0f, 0.707f, 150.0f, 100.0f, 0.0f, 0.0f };

#define			DEBUG_GUIDAGE 


	/* Les états de l'automate de gestion suivi de mur */
enum {
			SUIVI_MUR_ARRET, 
			SUIVI_MUR,
			SUIVI_VIRAGE,
			SUIVI_DEBUT_VIRAGE,
			SUIVI_SORTIE_VIRAGE,

} E_Etat_Automate_Suivi_Mur ;
typedef uint8_t T_Etat_Automate_Suivi_Mur ; 


#ifdef	DEBUG_GUIDAGE 
T_Angle_Distance  angle_droit ;
T_Angle_Distance  angle_droit_plus ;
T_Angle_Distance  angle_droit_moins ;

				float	erreur  ;
static 	float	distance_reprise ; 
static 	float	consigne ; 	
				float deltat_T ;
#endif 

//static T_Etat_Automate_Suivi_Mur  Etat_Automate_Suivi_Mur = SUIVI_MUR ;
T_Etat_Automate_Suivi_Mur  Etat_Automate_Suivi_Mur = SUIVI_MUR ;


float Guidage_Suivi_Mur (void)
{
#ifndef	DEBUG_GUIDAGE 
	T_Angle_Distance  angle_droit ;
	T_Angle_Distance  angle_droit_plus ;
	T_Angle_Distance  angle_droit_moins ;
	
	T_Angle_Distance  angle_dp_plus_alph ;
	T_Angle_Distance  angle_dp_moins_alph ;

					float	erreur  ;
	static 	float	distance_reprise ; 
	static 	float	consigne ; 	
					float deltat_T ;
#endif 	
	static uint32_t date_precedente = 0 ;
 
	static float			erreur_somme ;				/* Somme pour calcul Intégrale */	
	static float 			erreur_precedente  ;		
	static float			delta_erreur  ;	
	
	T_Angle_Distance	*ptr_hypotenuse ;
	T_Angle_Distance	*ptr_adjacent ;	
	
	
  
	T_Boolean					asservissement_request = FAUX ; 
	T_OuiNonErreur		suivi_mur_droit  ; 			/* OUI pour suivi mur droit  NON pour suivi mur gauche ERREUR à voir ? */

	suivi_mur_droit = ( Suivi_De_Mur.Angle_Origine == 90.0f ) ? OUI : NON ; 
	
	if ( ConsigneManuelle.ConsigneAngulaire ==  0 )
	{
		if ( Rplidar_New_Scan () == VRAI)
		{
			uint32_t dt = HAL_GetTick() ; 		
			if ( date_precedente == 0 )
			{ /* Signifie que l'on rentre ou revient dans l'asservissement */
				deltat_T = 290 ; 	// deltat_T = 290 ms si premiere fois	
				/* Remise à zero du PID */
				erreur_somme 			= 0.0f;				
				erreur_precedente = 0.0f ;		 					
				delta_erreur 			= 0.0f ;		
				consigne 					= 0.0f ; 
			}
			else
			{
				deltat_T = ( float )( dt - date_precedente ); 
			}
			
			date_precedente = dt ; 

			angle_droit.Angle = Suivi_De_Mur.Angle_Origine ;
			
			if ( suivi_mur_droit == OUI )
			{	/* 90° suivi de mur à droite */
				angle_droit_moins.Angle = Suivi_De_Mur.Angle_Origine + Suivi_De_Mur.Delta_Angle ;
				angle_droit_plus.Angle  = Suivi_De_Mur.Angle_Origine - Suivi_De_Mur.Delta_Angle ;		
			}
			else
			{ /* 270° suivi de mur à gauche */
				angle_droit_moins.Angle = Suivi_De_Mur.Angle_Origine - Suivi_De_Mur.Delta_Angle ;
				angle_droit_plus.Angle  = Suivi_De_Mur.Angle_Origine + Suivi_De_Mur.Delta_Angle ;							
			}
				
			Rplidar_Get_Distance ( &angle_droit ) ;
			Rplidar_Get_Distance ( &angle_droit_moins ) ;			
			Rplidar_Get_Distance ( &angle_droit_plus ) ;

			Rplidar_Get_Distance ( &angle_dp_plus_alph ) ;			
			Rplidar_Get_Distance ( &angle_dp_moins_alph ) ;
			
#ifdef	DEBUG_GUIDAGE 				
			Suivi_De_Mur.NotUsed3 = angle_droit_plus.Distance ;					// pour visu sur PC dans un premier temps
			Suivi_De_Mur.NotUsed4 = angle_droit_plus.Angle ;
			Suivi_De_Mur.NotUsed5 = angle_droit_moins.Distance ;				// pour visu sur PC dans un premier temps
			Suivi_De_Mur.NotUsed6 = angle_droit_moins.Angle ;		
			Suivi_De_Mur.NotUsed7 = angle_droit.Distance ;		// pour visu sur PC dans un premier temps
			Suivi_De_Mur.NotUsed8 = angle_droit.Angle ;
			
			Suivi_De_Mur.NotUsed9 = angle_droit_plus.Distance ;					// pour visu sur PC dans un premier temps
			Suivi_De_Mur.NotUsed10 = angle_droit_plus.Angle ;
			
#endif					
			if  ( ConsigneManuelle.Vitesse == 0 )	 
			{	/* Arrêt par l'opérateur */
				date_precedente = 0  ;	
				consigne = 0 ;
				Etat_Automate_Suivi_Mur = SUIVI_MUR ; 				
			} 
			else 
			{				
				switch ( Etat_Automate_Suivi_Mur )
				{
					case SUIVI_MUR : 
						if ( angle_droit_plus.Distance > 3 * angle_droit_moins.Distance ) 
						{ /* un virage à droite se précise */ 
							/* déterminer la distance restant à parcourir avant virage sachant que cet échantillon est vieux de 214 ms  */
							/* Pour cela il faut chercher l'angle qui détecte encore le mur et calculer la distance entre cet angle 
							et l'angle à 90 puis estimer la distance parcourue depuis l'échantillon et jusqu'au prochain échantillon 
							qui sera présent au bout de 214 ms  soit une durée totale de 2* 214ms = 428 ms
							exemple à la vitesse de 300 ms / s cela fait une distance d'environ 13 cm */
							//float distance_a_parcourir = ConsigneManuelle.Vitesse * 0.50f ;
							//if ( suivi_mur_droit == OUI )
							//	angle_droit_plus.Angle = Suivi_De_Mur.Angle_Origine - ( atan ( distance_a_parcourir / angle_droit.Distance ) * 180.0f / 3.14159f );  /* 90° suivi de mur à droite */
							//else
							//	angle_droit_plus.Angle = Suivi_De_Mur.Angle_Origine + ( atan ( distance_a_parcourir / angle_droit.Distance ) * 180.0f / 3.14159f );	/* 270° suivi de mur à gauche */
							
							//Rplidar_Get_Distance ( &angle_droit_plus ) ;	
#ifdef	DEBUG_GUIDAGE 							
							//Suivi_De_Mur.NotUsed3 = angle_droit_plus.Distance ;				// pour visu sur PC dans un premier temps
							//Suivi_De_Mur.NotUsed4 = angle_droit_plus.Angle ;	
#endif							
							if ( angle_droit_plus.Distance > 3 * angle_droit_moins.Distance ) 
							{  /* alors la il faut tourner avec une vitesse angulaire calculée selon le rayon = distance_a_90 */
								//date_precedente = 0  ;	/* pour réinitialiser le PID dès la reprise */
								distance_reprise = angle_droit_moins.Distance ;
								//consigne = ((ConsigneManuelle.Vitesse / angle_droit.Distance) * 180.0f / 3.141519f) ; 
								consigne = 20.0f ; 
								if ( suivi_mur_droit == OUI )
								{
									consigne = - consigne ;
								}
								Etat_Automate_Suivi_Mur = SUIVI_DEBUT_VIRAGE ;
								break  ; 
							}		
						}
						asservissement_request = VRAI ; 					
						ptr_hypotenuse = &angle_droit_moins ;						
						ptr_adjacent	 = &angle_droit ;
					break ; 
					

					case SUIVI_DEBUT_VIRAGE :
						if( angle_droit_moins.Distance > 1.5f * distance_reprise)
						{
							Etat_Automate_Suivi_Mur = SUIVI_VIRAGE ; 
						}
					break ;
					
					case SUIVI_VIRAGE :
						if ( angle_droit_plus.Distance <  (1.1f * distance_reprise) ) 			// une petite marge ???
						{
							date_precedente = 0  ;	/* pour réinitialiser le PID dès la reprise */
							consigne = 0 ;						// on redresse 
							Etat_Automate_Suivi_Mur = SUIVI_SORTIE_VIRAGE ;  
						}
					break ;	

					case SUIVI_SORTIE_VIRAGE :
						// tant que distance arriere incorrecte, on asservit avec la distance avant 
						asservissement_request = VRAI ; 			
						ptr_hypotenuse = &angle_droit_plus ;						
						ptr_adjacent	 = &angle_droit ;					
					
						if ( angle_droit_moins.Distance < 1.1f * angle_droit_plus.Distance ) 
							Etat_Automate_Suivi_Mur = SUIVI_MUR ; 
					break ; 					
				}
						
			}
		}
		if ( asservissement_request == VRAI )
		{
			erreur =   cos ( (ptr_hypotenuse->Angle - ptr_adjacent->Angle) * 3.14159f /180.0f ) - ( ptr_adjacent->Distance / ptr_hypotenuse->Distance ) ;
			if ( suivi_mur_droit == NON )
			{				
				erreur = - erreur ;
			}	
			erreur_somme += erreur ; 
			delta_erreur = erreur - erreur_precedente ; 
			erreur_precedente = erreur ;

			consigne  = 		 Suivi_De_Mur.CoefP * erreur  
									+	 ( Suivi_De_Mur.CoefI * erreur_somme ) * ( deltat_T / 1000.0f ) 
									+	 ( Suivi_De_Mur.CoefD * delta_erreur ) / ( deltat_T / 1000.0f ) ;		
		}			
		return ( consigne ) ; 
	}
	else
	{	/* Reprise par le pilote de la commande de direction */
		/* tout à 0 */
		date_precedente = 0  ;
		Etat_Automate_Suivi_Mur = SUIVI_MUR ; 
#ifdef	DEBUG_GUIDAGE 
		if ( Rplidar_New_Scan () == VRAI)
		{
				angle_droit.Angle = Suivi_De_Mur.Angle_Origine ;
				if ( suivi_mur_droit == OUI )
				{	/* 90° suivi de mur à droite */
					angle_droit_moins.Angle = Suivi_De_Mur.Angle_Origine + Suivi_De_Mur.Delta_Angle ;
					angle_droit_plus.Angle   = Suivi_De_Mur.Angle_Origine - Suivi_De_Mur.Delta_Angle ;		
				}
				else
				{ /* 270° suivi de mur à gauche */
					angle_droit_moins.Angle = Suivi_De_Mur.Angle_Origine - Suivi_De_Mur.Delta_Angle ;
					angle_droit_plus.Angle   = Suivi_De_Mur.Angle_Origine + Suivi_De_Mur.Delta_Angle ;							
				}
				
				Rplidar_Get_Distance ( &angle_droit ) ;
				Rplidar_Get_Distance ( &angle_droit_moins) ;			
				Rplidar_Get_Distance ( &angle_droit_plus ) ;	
				
				Suivi_De_Mur.NotUsed3 = angle_droit_plus.Distance ;				// pour visu sur PC dans un premier temps
				Suivi_De_Mur.NotUsed4 = angle_droit_plus.Angle ;	
				Suivi_De_Mur.NotUsed5 = angle_droit_moins.Distance ;				// pour visu sur PC dans un premier temps
				Suivi_De_Mur.NotUsed6 = angle_droit_moins.Angle ;		
				Suivi_De_Mur.NotUsed7 = angle_droit.Distance ;				// pour visu sur PC dans un premier temps
				Suivi_De_Mur.NotUsed8 = angle_droit.Angle ;							
			}
#endif 		
		return ( ConsigneManuelle.ConsigneAngulaire ); 
	}	
}


