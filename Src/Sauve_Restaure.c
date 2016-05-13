


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "typegen.h"
#include "tempo.h"
#include "moteurs.h"
#include "Phares.h"
#include "Rplidar.h"
#include "HMC5883L.h"
#include "stdlib.h"
#include "spi_flash.h"
#include "Guidage.h"

/* Sauvegarde effectuée par écriture de page de 256 octets sur un sous-secteur de 4 Ko 
	soit un total de 16 blocs différents 
	Ces blocs sont tous écrits dans le même appel de la fonction du fait de la nécessité d'effacement du sous-secteur 
	avant écriture 	*/

#define ADRESSE_SUB_SECTEUR_CONFIG				0
#define	ADRESSE_SAUVEGARDE_CONFIG_BLOC1 	ADRESSE_SUB_SECTEUR_CONFIG 
#define ADRESSE_SAUVEGARDE_CONFIG_BLOC2		ADRESSE_SAUVEGARDE_CONFIG_BLOC1 + 256 
#define ADRESSE_SAUVEGARDE_CONFIG_BLOC3		ADRESSE_SAUVEGARDE_CONFIG_BLOC2 + 256 
/* etc ...*/

/* Construire la structure par blocs de données de taille maximale 255 octets pour écriture suivi d'un octet crc automatique généré par SPI */
typedef struct {
	int16_t   									MD_CoefP;							/* Moteur DROIT  Proportionnel */
	int16_t  										MD_CoefI;							/* Integrale */
	int16_t   									MD_CoefD;							/* Dérivée */	
	int16_t   									MG_CoefP;							/* Moteur GAUCHE Proportionnel */
	int16_t  										MG_CoefI;							/* Integrale */
	int16_t   									MG_CoefD;							/* Dérivée */
	T_Compensation_Ligne_Droite	Compensation_Ligne_Droite;
	uint16_t										Phare_Luminosite ;
	int16_t											Offset_Orientation_Lidar ;
	T_HMC5883L_Compensation 		HMC5883L_Compensation ;
	T_Suivi_De_Mur  						Suivi_De_Mur ;
} T_Valeurs_Sauvegardees_Bloc1 ; 

/* typedef struct {
	
} T_Valeurs_Sauvegardees_Bloc2 ;  */

	/* etc ... */


typedef union {
		T_Valeurs_Sauvegardees_Bloc1  Bloc1 ; 
//		T_Valeurs_Sauvegardees_Bloc2  Bloc2 ; 
	/* etc ... */
} T_Valeurs_Sauvegardees_TousLesBlocs ; 

enum {
			REPOS,
			ECRITURE_BLOC1, 
			ECRITURE_BLOC2, 
			ECRITURE_BLOC3, 
			ECRITURE_BLOC4, 
			ECRITURE_BLOC5, 
			ECRITURE_BLOC6, 
			ECRITURE_BLOC7, 
			ECRITURE_BLOC8, 
			ECRITURE_BLOC9, 
			ECRITURE_BLOC10, 
			ECRITURE_BLOC11, 
			ECRITURE_BLOC12, 
			ECRITURE_BLOC13, 
			ECRITURE_BLOC14, 
			ECRITURE_BLOC15, 
			ECRITURE_BLOC16,			
			REQUETE_SAUVEGARDE,
			EFFACEMENT_EN_COURS, 
			ECRITURE_EN_COURS,	
} E_Etat_Automate_Sauvegarde ;

typedef uint8_t T_Etat_Automate_Sauvegarde ;

T_Etat_Automate_Sauvegarde	Etat_Automate_Sauvegarde = REPOS ;

T_OuiNonErreur Etat_Sauvegarde = OUI;


HAL_StatusTypeDef Restaure_Tout ( void  )
{
	T_Valeurs_Sauvegardees_TousLesBlocs  valeurs_sauvegardees  ;
	HAL_StatusTypeDef status ;
	if ( ( status = SPI_FLASH_BufferRead( (uint8_t*) &valeurs_sauvegardees.Bloc1, ADRESSE_SAUVEGARDE_CONFIG_BLOC1 , sizeof(T_Valeurs_Sauvegardees_Bloc1) ) ) == HAL_OK ) 
	{	/* copie des données sources dans le buffer */
		Etat_Moteur[MOTEUR_DROIT].CoefP		=	valeurs_sauvegardees.Bloc1.MD_CoefP ; 
		Etat_Moteur[MOTEUR_DROIT].CoefI		=	valeurs_sauvegardees.Bloc1.MD_CoefI ;
		Etat_Moteur[MOTEUR_DROIT].CoefD		= valeurs_sauvegardees.Bloc1.MD_CoefD ;	
		Etat_Moteur[MOTEUR_GAUCHE].CoefP	= valeurs_sauvegardees.Bloc1.MG_CoefP ; 
		Etat_Moteur[MOTEUR_GAUCHE].CoefI	=	valeurs_sauvegardees.Bloc1.MG_CoefI ;
		Etat_Moteur[MOTEUR_GAUCHE].CoefD	=	valeurs_sauvegardees.Bloc1.MG_CoefD ;		
		Compensation_Ligne_Droite					= valeurs_sauvegardees.Bloc1.Compensation_Ligne_Droite;		
		Phare_Luminosite									= valeurs_sauvegardees.Bloc1.Phare_Luminosite ;
		Offset_Orientation_Lidar					= valeurs_sauvegardees.Bloc1.Offset_Orientation_Lidar ;
		HMC5883L_Compensation							= valeurs_sauvegardees.Bloc1.HMC5883L_Compensation ;
		Suivi_De_Mur											= valeurs_sauvegardees.Bloc1.Suivi_De_Mur;
		/* bloc suivant */
	}	
	return ( status ) ;
}	

void Sauvegarde_Tout_Request (void) 
{
	if ( Etat_Automate_Sauvegarde  == REPOS )
		Etat_Automate_Sauvegarde = REQUETE_SAUVEGARDE ;	
}


/* fonction à appeler périodiquement pour répondre aux demandes de sauvegarde issues du Wifi */
T_OuiNonErreur Gestion_Sauvegarde ( void ) 
{
	static 	T_Etat_Automate_Sauvegarde	Etat_Suivant ;
	T_Valeurs_Sauvegardees_TousLesBlocs  valeurs_sauvegardees  ;
	T_OuiNonErreur status ;
	switch ( Etat_Automate_Sauvegarde )
	{
		case REPOS :
			status = Etat_Sauvegarde ;	/* pour conserver l'état précédent d'accès 	*/
		break ; 
		
		case REQUETE_SAUVEGARDE :	/* Positionné par Sauvegarde_Tout_Request ()  */
			/* Lancer l'effacement du sous secteur */
			if  ( SPI_FLASH_SubSectorErase(ADRESSE_SUB_SECTEUR_CONFIG, NON) == HAL_OK )
			{
				Etat_Automate_Sauvegarde = EFFACEMENT_EN_COURS ; 
				Etat_Suivant = ECRITURE_BLOC1 ;
				status = NON ; 
			}
			else
			{
				Etat_Automate_Sauvegarde = REPOS ; 	
				status = ERREUR ; 				
			}
		break ;
			
		
		case EFFACEMENT_EN_COURS :
		case ECRITURE_EN_COURS :
			switch ( ( status = SPI_FLASH_TestForWriteEnd () ) ) 
			{
				case OUI :
					Etat_Automate_Sauvegarde = Etat_Suivant ; 
					status = NON ;
				break ;
				case NON :
				break ; 
				case ERREUR : 
					Etat_Automate_Sauvegarde = REPOS ; 	
				break ;			
			}
		break ;
		
		case ECRITURE_BLOC1 :
			/* copie des données sources dans le buffer */
			valeurs_sauvegardees.Bloc1.MD_CoefP 									= Etat_Moteur[MOTEUR_DROIT].CoefP ; 
			valeurs_sauvegardees.Bloc1.MD_CoefI 									= Etat_Moteur[MOTEUR_DROIT].CoefI ;
			valeurs_sauvegardees.Bloc1.MD_CoefD 									= Etat_Moteur[MOTEUR_DROIT].CoefD ;	
			valeurs_sauvegardees.Bloc1.MG_CoefP 									= Etat_Moteur[MOTEUR_GAUCHE].CoefP ; 
			valeurs_sauvegardees.Bloc1.MG_CoefI 									= Etat_Moteur[MOTEUR_GAUCHE].CoefI ;
			valeurs_sauvegardees.Bloc1.MG_CoefD 							  	= Etat_Moteur[MOTEUR_GAUCHE].CoefD ;
			valeurs_sauvegardees.Bloc1.Compensation_Ligne_Droite 	= Compensation_Ligne_Droite;		
			valeurs_sauvegardees.Bloc1.Phare_Luminosite				  	= Phare_Luminosite ;
			valeurs_sauvegardees.Bloc1.Offset_Orientation_Lidar 	= Offset_Orientation_Lidar ;
			valeurs_sauvegardees.Bloc1.HMC5883L_Compensation			= HMC5883L_Compensation ;
			valeurs_sauvegardees.Bloc1.Suivi_De_Mur							   = Suivi_De_Mur;
			/* envoi du buffer dans la mémoire flash */
			if ( ( status = SPI_FLASH_PageWrite( (uint8_t*) &valeurs_sauvegardees.Bloc1,  ADRESSE_SAUVEGARDE_CONFIG_BLOC1 , sizeof(T_Valeurs_Sauvegardees_Bloc1), NON ) ) == HAL_OK )
			{
				Etat_Automate_Sauvegarde = ECRITURE_EN_COURS ; 
				Etat_Suivant = ECRITURE_BLOC2 ;		
				status = NON ; 
			}	
			else	
			{
				Etat_Automate_Sauvegarde = REPOS ; 	
				status = ERREUR ; 
			}
		break ; 
		
		case ECRITURE_BLOC2 :
			/* faire sur modéle ECRITURE_BLOC1 si besoin. Extension possible jusqu'à BLOC16 */	

			/* Si bloc2 est utilisé mettre ces 2 lignes dans ecriture bloc suivant */
			/* terminé sans erreur */		
			Etat_Automate_Sauvegarde = REPOS ; 
			status = OUI ;					
		break ; 	

		default	:
				Etat_Automate_Sauvegarde = REPOS ; 	
				status = ERREUR ; 			
		break ;
		
	}
	Etat_Sauvegarde  = status ; 
	return status ;
}


void Initialise_Sauvegarde ( void  )
{	
	SPI_FLASH_Init () ;

	if ( SPI_FLASH_WaitForWriteEnd() == HAL_OK ) 
	{
		if ( SPI_FLASH_ReadID () == 0x00207115 )
		{
			if ( Restaure_Tout () != HAL_OK )
			{	/* défaut de lecture alors on sauvegarde de nouveau les valeurs usine */
				Sauvegarde_Tout_Request () ; 
				while ( ( Etat_Sauvegarde = Gestion_Sauvegarde () ) == NON ) { } ;	
				if ( Etat_Sauvegarde == OUI )
				{
					if ( Restaure_Tout () != HAL_OK )
						Etat_Sauvegarde = ERREUR ;						
				}
			}	
			else
				Etat_Sauvegarde = OUI ;							
		}
		else
			Etat_Sauvegarde = ERREUR ;				
	}
	else
		Etat_Sauvegarde = ERREUR ;		
}


