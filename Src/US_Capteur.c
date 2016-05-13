/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "typegen.h"
#include "Tempo.h"
#include "US_Capteur.h"
#include "Moteurs.h"


#define			DELAI_INTER_MESURE				3000		//  3 ms 
#define			DUREE_TRIGGER								11 		// 11 µs 
#define			MAX_VALEUR_ABERRANTE				 2		// au dela de 2 mesures > MAX_TEMPS la mesure infinie est définitivement validée  
#define			MAX_TEMPS								 40600		// µs au dela mesure considérée infinie environ 7,00 m

/* Toutes les sorties de commande des triggers doivent être sur le même GPIO */
#define			GPIO_US						GPIOE


typedef struct {
	uint16_t   						Temps ;						/* exprimé en µs */
	int16_t	 							Distance ;				/* cm = Temps / 58 calculé dans Us_LireDistance */
	int16_t								OffsetDistance ;	/* cm  à compléter après identification */
	uint16_t 							GPIO_Pin ; 				/* uniquement de GPIO_US à définir ci-dessus */	
	uint8_t								ValeurAberrante ;	/* Compteur de valeurs > à MAX_TEMPS */
	T_MarcheArret					MarcheArret ;			/* interrogation autorisée pour ce capteur */
	volatile T_Boolean		MesurePrete ; 		/* Si VRAI alors on peut lire nouvelle Mesure */
/* Distance = Temps /58 + OffsetDistance ; avec OffsetDistance positif ou négatif 
		à mesurer pour chaque capteur lorsque ceux-ci seront fixés et à inscrire dans
		l'initialisation de UltraSons */
} T_UnCapteurUS ;
/* Remarque concernant le time-out du capteur US et du calcul de la distance:
		valeur mesurée à environ 190 ms ce qui correspond à 2 débordements du timer 1 en phase de mesure 
		la valeur résultante est de 190 000  -  2* 65535 = 58 930 soit  10 m 
		cette valeur est bien au delà de ce que peut mesurer le capteur 
		donc pour toute valeur de distance calculée > 700 cm  alors Distance = 10 m soit infini
		Conclusion : il est inutile de comptabiliser les débordement du timer 	

IMPORTANT : pour l'application, en cas de besoin de mesure plus rapide, il ne faut pas oublier 
			qu'un capteur qui mesure l'infini est pénalisant ( 190 ms ) de perdu par capteur				
			aussi mettre ce ou ces capteur à l'arrêt permettra d'obtenir les valeurs utiles 
			plus rapidement																																		*/

/* Structure d'état du module cpateur US */
typedef struct {
	T_UnCapteurUS 	UnCapteur[US_MAX_CAPTEUR] ;
	T_MarcheArret		MarcheArret ;								/* Marche Arrêt général  */
	T_US_Capteur		CapteurEnCours ;						/* US_CAPTEUR1 à MAX_CAPTEUR_US - 1 */
} T_CapteurUS ; 

T_CapteurUS  UltraSons = { 	{	{ 0, -1, 0, GPIO_PIN_7,  0, ARRET },
															{ 0, -1, 0, GPIO_PIN_8,  0, ARRET },
															{ 0, -1, 0, GPIO_PIN_10, 0, ARRET },
															{ 0, -1, 0, GPIO_PIN_12, 0, ARRET } },
															ARRET,
															US_MAX_CAPTEUR - 1		} ;	/* dernier pour commencer par le premier !!! */

											
uint16_t 	Distance_Obstacle_Av = 10000; 
uint16_t 	Distance_Obstacle_Ar = 10000; 															
															
															
/* Lire la distance en cm mesurée pour le capteur, retourne 10000 si infinie ou < environ 2 cm */
uint16_t Us_LireDistance ( T_US_Capteur Capteur )
{
	uint16_t distance = 10000 ; // par défaut si erreur sur paramètre Capteur
	
	if ( Capteur < US_MAX_CAPTEUR )
	{	 
		UltraSons.UnCapteur[Capteur].MesurePrete = FAUX ; 
		UltraSons.UnCapteur[Capteur].Distance = 	( UltraSons.UnCapteur[Capteur].Temps /58 )
																						+   UltraSons.UnCapteur[Capteur].OffsetDistance  ;
		distance = UltraSons.UnCapteur[Capteur].Distance ;			
	}
	return (distance);
}

/* Vérifie si une nouvelle mesure est disponible 
	 fonction à appeler avant Us_LireDistance 
   pour éviter de lire plusieurs fois la même chose */
T_Boolean Us_Mesure_Distance_Prete ( T_US_Capteur Capteur )  
{
	if ( Capteur < US_MAX_CAPTEUR )
	{	
		return ( UltraSons.UnCapteur[Capteur].MesurePrete ) ; 
	}
	else
		return (FAUX); 
}


/* Active ou désactive un capteur US 
	 Remarque :	dans l'état du logiciel, il est interdit d'activer un capteur non installé */
void  Us_SelectInterrogation ( T_US_Capteur Capteur, T_MarcheArret MarcheArret )
{
	if ( Capteur < US_MAX_CAPTEUR )
	{
		UltraSons.UnCapteur[Capteur].MarcheArret = MarcheArret ;
	}
}	



/* Active ou désactive la mesure US  */
void Us_MarcheArretGeneral (T_MarcheArret MarcheArret)
{
	if ( MarcheArret == MARCHE )
	{
		if ( UltraSons.MarcheArret == ARRET )
		{ /* Si pas déjà démarré */
			/* Force toutes les pins de trigger à 0 */
/*		uint16_t capteur ;	
			for ( capteur = US_CAPTEUR1 ; capteur < US_MAX_CAPTEUR ; capteur++ )
				HAL_GPIO_WritePin ( GPIO_US, UltraSons.UnCapteur[capteur].GPIO_Pin, GPIO_PIN_RESET ) ;	*/
			/* Chargement et démarrage du timer de gestion de l'impulsion */
			/* la premiere mesure aura lieu à l'échéance de TEMPO_INTER_MESURE 
					c'est une sécurité en cas de multiples interruptions dans les 2 lignes suivantes */
			Arme_Tempo_USCapteur ( DELAI_INTER_MESURE ) ;	
			/* Démarrage du timer de mesure */
			HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);				
		}
	}	
	UltraSons.MarcheArret = MarcheArret ;
}




															
													
/* Interruption sur capture du front descendant de l'écho 
	 fonction appelée par call back capture du timer 1			
	 dans fichier stm32f4xx_it.c														 */																
void Us_GestionMesure_IT (TIM_HandleTypeDef *htim)
{
	uint16_t temps  = __HAL_TIM_GET_COMPARE(htim, htim->Channel) ;
	
	if ( ( temps > MAX_TEMPS ) || ( temps < 250 ) ) // soit plus de 7m ou - de 4 cm 
	{
		if ( ++ UltraSons.UnCapteur[UltraSons.CapteurEnCours].ValeurAberrante > MAX_VALEUR_ABERRANTE )
		{ /* mesure finalement acceptée */
			UltraSons.UnCapteur[UltraSons.CapteurEnCours].Temps = MAX_TEMPS ;			
			UltraSons.UnCapteur[UltraSons.CapteurEnCours].ValeurAberrante = 0 ; 
			UltraSons.UnCapteur[UltraSons.CapteurEnCours].MesurePrete = VRAI; 
		}			
	}
	else 
	{
		UltraSons.UnCapteur[UltraSons.CapteurEnCours].Temps = temps ; 		
		UltraSons.UnCapteur[UltraSons.CapteurEnCours].ValeurAberrante = 0 ;  
		UltraSons.UnCapteur[UltraSons.CapteurEnCours].MesurePrete = VRAI; 		
	}
	/* Lance tempo inter-mesure et active le timer de gestion des tempos  */
	Arme_Tempo_USCapteur ( DELAI_INTER_MESURE ) ;
}


/* Interruption sur temporisation utilisée pour le délai inter mesure 
																					 pour l'envoi de l'impulsion de trigger
	 fonction appelée par call back compare du timer 2	TIM_CHANNEL_1													
	 dans fichier stm32f4xx_it.c														 */	
void Us_GestionTrigger_IT (TIM_HandleTypeDef *htim)
{
	if ( HAL_GPIO_ReadPin ( GPIO_US, UltraSons.UnCapteur[UltraSons.CapteurEnCours].GPIO_Pin ) )
	{  /* si elle est à 1 alors fin de pulse, remise à 0 */
		HAL_GPIO_WritePin ( GPIO_US, UltraSons.UnCapteur[UltraSons.CapteurEnCours].GPIO_Pin, GPIO_PIN_RESET ) ;	
		Desarme_Tempo_USCapteur () ; 
	}
	else
	{	/* tempo inter mesure écoulée détermine capteur suivant et lance la pulse de 10 µs */	
		if ( ++ UltraSons.CapteurEnCours >= US_MAX_CAPTEUR ) 
			UltraSons.CapteurEnCours = US_CAPTEUR1 ;
		if ( UltraSons.MarcheArret == MARCHE )
		{	/* si l'arrêt général n'est pas demandé */					
			if ( UltraSons.UnCapteur[UltraSons.CapteurEnCours].MarcheArret == MARCHE ) 
			{	/* l'interrogation est demandée, on active la pin et la tempo de 10 µs */
				HAL_GPIO_WritePin ( GPIO_US, UltraSons.UnCapteur[UltraSons.CapteurEnCours].GPIO_Pin, GPIO_PIN_SET ) ;	
			}	/* sinon seulement la tempo pour passer au capteur suivant */
			Relance_Tempo_USCapteur ( DUREE_TRIGGER ) ;					
		}
		else
		{	/* Si arrêt général demandé alors on cesse toute mesure */						
			Desarme_Tempo_USCapteur () ; 
			HAL_TIM_IC_Stop_IT (&htim1, TIM_CHANNEL_1);										
		}	
	}
}	


T_Boolean TestCollisionAvant(void)
{
	Distance_Obstacle_Av = Us_LireDistance ( US_AVANT ) ;
	if(Distance_Obstacle_Av < DISTANCE_SECURITE)
	{
		return VRAI;
	}		
	return FAUX;
}

T_Boolean TestCollisionArriere(void)
{
	Distance_Obstacle_Ar = Us_LireDistance ( US_ARRIERE ) ;
	if(Distance_Obstacle_Ar < DISTANCE_SECURITE)
	{
		return VRAI;
	}
	return FAUX;	
}

