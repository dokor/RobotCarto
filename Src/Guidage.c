
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

float 	Distance_Mur = 300.0f ; 


/* remettre en local */
 	float 				distance_a_90 ;
	float 				distance_arriere ;
	float 				distance_avant ;
	float 				angle_a_90 ;
	float 				angle_arriere ;
	float 				angle_avant ;
	float					erreur = 0.0f ;
	float					consigne = 0.0f ; 	
float Guidage_Suivi_Mur (void)
{
	static uint32_t date_courante = 0 ;
				 float 		deltat_T ; 
	static float		erreur_somme = 0.0f;				/* Somme pour calcul Intégrale */	
	static float 		erreur_precedente = 0.0f ;		
	static float		delta_erreur = 0.0f ;	

	if ( ConsigneManuelle.ConsigneAngulaire ==  0 )
	{
		if ( Rplidar_New_Scan () == VRAI)
		{
			uint32_t dt = HAL_GetTick() ; 
			deltat_T = ( float )( dt - date_courante );
			date_courante = dt ; 
			if  ( ConsigneManuelle.Vitesse == 0 )	 
			{	
				erreur_somme = 0.0f;				
				erreur_precedente = 0.0f ;		 					
				delta_erreur = 0.0f ;		
				consigne = 0 ; 
			} 
			else 
			{
		//		if (distance_a_90 != 0 ||distance_arriere != 0 )
		//		{
					angle_a_90 = Suivi_De_Mur.Angle_Origine ;
					Rplidar_Get_Distance ( &distance_a_90, &angle_a_90 ) ;
				
					angle_arriere = Suivi_De_Mur.Angle_Origine + Suivi_De_Mur.Delta_Angle ;
					Rplidar_Get_Distance ( &distance_arriere, &angle_arriere ) ;			

					angle_avant = Suivi_De_Mur.Angle_Origine - Suivi_De_Mur.Delta_Angle ;
					Rplidar_Get_Distance ( &distance_avant, &angle_avant) ;					
					Suivi_De_Mur.NotUsed3 = distance_avant ;				// pour visu sur PC dans un premier temps
					
		//			if ( distance_avant > (distance_a_90 * 0.707f) + 15.0f )
		//			{
		//				if( distance_a_90 > (distance_arriere * 0.707f) + 15.0f) 
		//				{
							//return (consigne = 45) ; 
		//				}
		//			}
					erreur =  /*cos ( angle_arriere - angle_a_90 ) */ Suivi_De_Mur.CosDelta_Angle - ( distance_a_90 / distance_arriere ) ;
					erreur_somme += erreur ; 
					delta_erreur = erreur - erreur_precedente ; 
					erreur_precedente = erreur ;

					consigne  = 		 Suivi_De_Mur.CoefP * erreur  
											+	 ( Suivi_De_Mur.CoefI * erreur_somme ) * ( deltat_T / 1000 ) 
											+	 ( Suivi_De_Mur.CoefD * delta_erreur ) / ( deltat_T / 1000 ) ;					
			//	}
			}
		}
		return ( consigne ) ; 
	}
	else
	{	/* Reprise par le pilote de la commande de direction */
		/* tout à 0 */
		erreur_somme = 0.0f;				/* Somme pour calcul Intégrale */	
		erreur_precedente = 0.0f ;		 					
		delta_erreur = 0.0f ;		
		consigne = 0 ; 		
		return ( ConsigneManuelle.ConsigneAngulaire ); 
	}	
}


