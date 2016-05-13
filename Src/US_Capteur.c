/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "typegen.h"
#include "Tempo.h"
#include "US_Capteur.h"
#include "Moteurs.h"


#define			DELAI_INTER_MESURE				3000		//  3 ms 
#define			DUREE_TRIGGER								11 		// 11 �s 
#define			MAX_VALEUR_ABERRANTE				 2		// au dela de 2 mesures > MAX_TEMPS la mesure infinie est d�finitivement valid�e  
#define			MAX_TEMPS								 40600		// �s au dela mesure consid�r�e infinie environ 7,00 m

/* Toutes les sorties de commande des triggers doivent �tre sur le m�me GPIO */
#define			GPIO_US						GPIOE


typedef struct {
	uint16_t   						Temps ;						/* exprim� en �s */
	int16_t	 							Distance ;				/* cm = Temps / 58 calcul� dans Us_LireDistance */
	int16_t								OffsetDistance ;	/* cm  � compl�ter apr�s identification */
	uint16_t 							GPIO_Pin ; 				/* uniquement de GPIO_US � d�finir ci-dessus */	
	uint8_t								ValeurAberrante ;	/* Compteur de valeurs > � MAX_TEMPS */
	T_MarcheArret					MarcheArret ;			/* interrogation autoris�e pour ce capteur */
	volatile T_Boolean		MesurePrete ; 		/* Si VRAI alors on peut lire nouvelle Mesure */
/* Distance = Temps /58 + OffsetDistance ; avec OffsetDistance positif ou n�gatif 
		� mesurer pour chaque capteur lorsque ceux-ci seront fix�s et � inscrire dans
		l'initialisation de UltraSons */
} T_UnCapteurUS ;
/* Remarque concernant le time-out du capteur US et du calcul de la distance:
		valeur mesur�e � environ 190 ms ce qui correspond � 2 d�bordements du timer 1 en phase de mesure 
		la valeur r�sultante est de 190 000  -  2* 65535 = 58 930 soit  10 m 
		cette valeur est bien au del� de ce que peut mesurer le capteur 
		donc pour toute valeur de distance calcul�e > 700 cm  alors Distance = 10 m soit infini
		Conclusion : il est inutile de comptabiliser les d�bordement du timer 	

IMPORTANT : pour l'application, en cas de besoin de mesure plus rapide, il ne faut pas oublier 
			qu'un capteur qui mesure l'infini est p�nalisant ( 190 ms ) de perdu par capteur				
			aussi mettre ce ou ces capteur � l'arr�t permettra d'obtenir les valeurs utiles 
			plus rapidement																																		*/

/* Structure d'�tat du module cpateur US */
typedef struct {
	T_UnCapteurUS 	UnCapteur[US_MAX_CAPTEUR] ;
	T_MarcheArret		MarcheArret ;								/* Marche Arr�t g�n�ral  */
	T_US_Capteur		CapteurEnCours ;						/* US_CAPTEUR1 � MAX_CAPTEUR_US - 1 */
} T_CapteurUS ; 

T_CapteurUS  UltraSons = { 	{	{ 0, -1, 0, GPIO_PIN_7,  0, ARRET },
															{ 0, -1, 0, GPIO_PIN_8,  0, ARRET },
															{ 0, -1, 0, GPIO_PIN_10, 0, ARRET },
															{ 0, -1, 0, GPIO_PIN_12, 0, ARRET } },
															ARRET,
															US_MAX_CAPTEUR - 1		} ;	/* dernier pour commencer par le premier !!! */

											
uint16_t 	Distance_Obstacle_Av = 10000; 
uint16_t 	Distance_Obstacle_Ar = 10000; 															
															
															
/* Lire la distance en cm mesur�e pour le capteur, retourne 10000 si infinie ou < environ 2 cm */
uint16_t Us_LireDistance ( T_US_Capteur Capteur )
{
	uint16_t distance = 10000 ; // par d�faut si erreur sur param�tre Capteur
	
	if ( Capteur < US_MAX_CAPTEUR )
	{	 
		UltraSons.UnCapteur[Capteur].MesurePrete = FAUX ; 
		UltraSons.UnCapteur[Capteur].Distance = 	( UltraSons.UnCapteur[Capteur].Temps /58 )
																						+   UltraSons.UnCapteur[Capteur].OffsetDistance  ;
		distance = UltraSons.UnCapteur[Capteur].Distance ;			
	}
	return (distance);
}

/* V�rifie si une nouvelle mesure est disponible 
	 fonction � appeler avant Us_LireDistance 
   pour �viter de lire plusieurs fois la m�me chose */
T_Boolean Us_Mesure_Distance_Prete ( T_US_Capteur Capteur )  
{
	if ( Capteur < US_MAX_CAPTEUR )
	{	
		return ( UltraSons.UnCapteur[Capteur].MesurePrete ) ; 
	}
	else
		return (FAUX); 
}


/* Active ou d�sactive un capteur US 
	 Remarque :	dans l'�tat du logiciel, il est interdit d'activer un capteur non install� */
void  Us_SelectInterrogation ( T_US_Capteur Capteur, T_MarcheArret MarcheArret )
{
	if ( Capteur < US_MAX_CAPTEUR )
	{
		UltraSons.UnCapteur[Capteur].MarcheArret = MarcheArret ;
	}
}	



/* Active ou d�sactive la mesure US  */
void Us_MarcheArretGeneral (T_MarcheArret MarcheArret)
{
	if ( MarcheArret == MARCHE )
	{
		if ( UltraSons.MarcheArret == ARRET )
		{ /* Si pas d�j� d�marr� */
			/* Force toutes les pins de trigger � 0 */
/*		uint16_t capteur ;	
			for ( capteur = US_CAPTEUR1 ; capteur < US_MAX_CAPTEUR ; capteur++ )
				HAL_GPIO_WritePin ( GPIO_US, UltraSons.UnCapteur[capteur].GPIO_Pin, GPIO_PIN_RESET ) ;	*/
			/* Chargement et d�marrage du timer de gestion de l'impulsion */
			/* la premiere mesure aura lieu � l'�ch�ance de TEMPO_INTER_MESURE 
					c'est une s�curit� en cas de multiples interruptions dans les 2 lignes suivantes */
			Arme_Tempo_USCapteur ( DELAI_INTER_MESURE ) ;	
			/* D�marrage du timer de mesure */
			HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);				
		}
	}	
	UltraSons.MarcheArret = MarcheArret ;
}




															
													
/* Interruption sur capture du front descendant de l'�cho 
	 fonction appel�e par call back capture du timer 1			
	 dans fichier stm32f4xx_it.c														 */																
void Us_GestionMesure_IT (TIM_HandleTypeDef *htim)
{
	uint16_t temps  = __HAL_TIM_GET_COMPARE(htim, htim->Channel) ;
	
	if ( ( temps > MAX_TEMPS ) || ( temps < 250 ) ) // soit plus de 7m ou - de 4 cm 
	{
		if ( ++ UltraSons.UnCapteur[UltraSons.CapteurEnCours].ValeurAberrante > MAX_VALEUR_ABERRANTE )
		{ /* mesure finalement accept�e */
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


/* Interruption sur temporisation utilis�e pour le d�lai inter mesure 
																					 pour l'envoi de l'impulsion de trigger
	 fonction appel�e par call back compare du timer 2	TIM_CHANNEL_1													
	 dans fichier stm32f4xx_it.c														 */	
void Us_GestionTrigger_IT (TIM_HandleTypeDef *htim)
{
	if ( HAL_GPIO_ReadPin ( GPIO_US, UltraSons.UnCapteur[UltraSons.CapteurEnCours].GPIO_Pin ) )
	{  /* si elle est � 1 alors fin de pulse, remise � 0 */
		HAL_GPIO_WritePin ( GPIO_US, UltraSons.UnCapteur[UltraSons.CapteurEnCours].GPIO_Pin, GPIO_PIN_RESET ) ;	
		Desarme_Tempo_USCapteur () ; 
	}
	else
	{	/* tempo inter mesure �coul�e d�termine capteur suivant et lance la pulse de 10 �s */	
		if ( ++ UltraSons.CapteurEnCours >= US_MAX_CAPTEUR ) 
			UltraSons.CapteurEnCours = US_CAPTEUR1 ;
		if ( UltraSons.MarcheArret == MARCHE )
		{	/* si l'arr�t g�n�ral n'est pas demand� */					
			if ( UltraSons.UnCapteur[UltraSons.CapteurEnCours].MarcheArret == MARCHE ) 
			{	/* l'interrogation est demand�e, on active la pin et la tempo de 10 �s */
				HAL_GPIO_WritePin ( GPIO_US, UltraSons.UnCapteur[UltraSons.CapteurEnCours].GPIO_Pin, GPIO_PIN_SET ) ;	
			}	/* sinon seulement la tempo pour passer au capteur suivant */
			Relance_Tempo_USCapteur ( DUREE_TRIGGER ) ;					
		}
		else
		{	/* Si arr�t g�n�ral demand� alors on cesse toute mesure */						
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

