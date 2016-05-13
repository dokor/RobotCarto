
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"
#include "usart.h"
#include "Tempo.h"
#include "Rplidar.h"
#include "WifiCom.h"
#include "stdlib.h"


/* Renomme la pin de commande du moteur Lidar  */
#define		GPIOMOTEURLIDAR						GPIOG	

#define		MOTEUR_LIDAR							GPIO_PIN_7

T_MarcheArret						Status_Moteur_Lidar ;

#define		START_FLAG_REQUEST		0xA5

	/* Le Start_Flag du message de réponse */
#define		START_FLAG_RESPONSE1	0xA5				
#define		START_FLAG_RESPONSE2	0x5A

	/* Les Data_Type du message de réponse */ 
#define		DATA_SCAN							0x81
#define		DATA_INFO							0x04
#define		DATA_HEALTH						0x06

#define		DATA_DUMMY						0xFF 					// créé localement  

#define		START_FLAG_MASK				0x03 
#define		NEW_360								0x01					// à utiliser avec START_FLAG_MASK
#define		NEXT_360							0x02					// à utiliser avec START_FLAG_MASK
#define		CONTROL_ANGLE_MASK		0x01 
#define		CONTROL_ANGLE					0x01					// à utiliser avec CONTROL_ANGLE_MASK



	/* Les états de l'automate de gestion Lidar */
enum {
			SCAN_DEFAUT,
			SCAN_REPOS,
			SCAN_REQUEST_HEALTH ,	
			SCAN_REQUEST_SCAN,						
			SCAN_START_SCAN,
			SCAN_SUPERVISE_SCAN,	
			SCAN_WAIT_SENDING_REQUEST,
			SCAN_WAIT_STOP,
			SCAN_WAIT_RESET, 	
			SCAN_WAIT_HEALTH,							
			SCAN_WAIT_INFO, 		

} E_Etat_Automate_Rplidar ;
typedef volatile uint8_t T_Etat_Automate_Rplidar ;
volatile uint32_t	TimeoutVerifReponse = 0 ; 

T_Etat_Automate_Rplidar	volatile Etat_Automate_Rplidar 	= SCAN_REPOS ;	




int16_t							Offset_Orientation_Lidar = 	-92	;			// Valeur par défaut si sauvegarde HS		

	/* Pour recevoir la réponse d'acquittement de la requête */
#define							TAILLE_REPONSE  7 
uint8_t							Buf_Response[TAILLE_REPONSE] ;

	/* Pour recevoir un SCAN */
#define							TAILLE_SCAN			5
	
	/* Pour le status du Rplidar */
#define							TAILLE_HEALTH		3
uint8_t							Buf_Health[TAILLE_HEALTH] ;
uint8_t							Status_Rplidar ;
uint16_t						ErrorCode_Rplidar ;


	/* pour les infos du Rplidar */
#define							TAILLE_INFO			20
uint8_t							Buf_Info [TAILLE_INFO] ;

	/* pour l'envoi d'une requête */
#define							TAILLE_REQUETE	2
uint8_t							Buf_Request[2] = { START_FLAG_REQUEST, 0 } ;	

	/* 360° de scan pour envoi sur le WIFI et gestion locale */
T_BufMessageLong 		BufMessageScan ;	
/* Indique à la couche application qu'un scan est disponible dans BufMessageScan  */
volatile T_Boolean New_Scan_Disponible = FAUX ; 

/* Contient les indices des 4 angles droits */
uint16_t						Huit_Angles [8] ;

	/* Pour réception des données issues du Lidar */
#define							TAILLE_BUF_REC		3020					// Au maximum il y a moins de 300 scans (293) à la vitesse de rotation du moteur soit 1500 octets * 2 + 20
																										// pour contenir 2 Scans 360 	
uint8_t							Buf_Rec[TAILLE_BUF_REC]  ;

uint16_t						Indice_Debut_Lecture  ;
#define  Get_Indice_Courant_DMA()	( TAILLE_BUF_REC - __HAL_DMA_GET_COUNTER(*(&Rplidar_Uart.hdmarx)) ) 

uint8_t    					Transmission_Rate_Request ; 
uint8_t    					Transmission_Rate ; 
T_Boolean 					Premiere_Reception ;


	/* On et Off du moteur */
static void	Rplidar_Moteur (T_MarcheArret Etat)
{
	Status_Moteur_Lidar = Etat ;
	HAL_GPIO_WritePin(GPIOMOTEURLIDAR, MOTEUR_LIDAR,  Etat == MARCHE ? GPIO_PIN_SET : GPIO_PIN_RESET);	
}


static uint16_t Rplidar_Calcule_ProchainDepart ( uint16_t Depart )
{
	uint16_t reste ;
	reste =  (TAILLE_BUF_REC - Depart) % TAILLE_SCAN ;
	if ( reste != 0 )
		return ( TAILLE_SCAN - reste );
	else
		return ( 0 ) ; 
}


static T_Boolean Rplidar_Cherche_Debut_360 ( uint16_t *A_partir_De, uint16_t JusquaMax )
{
	T_Boolean found = FAUX ;	
	do 
	{
		if  ( (( Buf_Rec[*A_partir_De] & START_FLAG_MASK ) == NEW_360 ) && (( Buf_Rec[*A_partir_De+1] & CONTROL_ANGLE_MASK) == CONTROL_ANGLE ) )
			found = VRAI ;
		else
		{
			*A_partir_De += TAILLE_SCAN ;
			if ( *A_partir_De >= JusquaMax )
				return( FAUX ) ;						// Pas trouvé le début du scan 360 il y a problème de synchro 
		}
	}
	while ( ! found ) ;
	return ( found ) ;
}


/* Avec suppression du superflux et compensation de l'angle 
	retourne 	ERREUR si qualité = 0 
						OUI si Angle_Recherche est trouvé
						NON si pas trouvé 			*/
static 	T_OuiNonErreur	Charge_Data_Scan ( T_Scan *Scan, uint8_t *Data , uint16_t Angle_Recherche) 
{
	T_OuiNonErreur retour = ERREUR ;   // par défaut 
	uint16_t  quality = Data[0] >> 2 ;
	if ( quality != 0 )
	{
		Scan->Quality 	=  quality ;
		Scan->Angle   	= (Data[1]) | (Data[2] << 8) ;
		Scan->Angle    >>= 1 ;
		if ( Scan->Angle > ( abs(Offset_Orientation_Lidar) << 6 ) )
			Scan->Angle += ( Offset_Orientation_Lidar << 6 ) ;
		else
			Scan->Angle += ( (360 + Offset_Orientation_Lidar ) << 6 ) ;
		Scan->Distance	=  Data[3] | (Data[4] << 8) ;
		if ( Angle_Recherche == 0 )
		{			
			retour = Scan->Angle > ( 2 << 6 ) ? NON : OUI ;		// pour trouver le 0
		}
		else
		{
			retour =  Scan->Angle >= Angle_Recherche ? OUI : NON ;	
		}
	}
	return ( retour ) ;
}


	

static uint16_t Rplidar_Charge_360_Scan ( uint16_t Idc_Buf_Rec, T_Scan *Scan )
{
  uint16_t 	angle_recherche = 6 ;   // soit 270°
	uint16_t	nombre_de_scan = 0 ; 
	T_Boolean fini = FAUX ;	
	T_Scan 		*ScanPtr = Scan ;
	
	// Charge les scans jusqu'au prochain NEW_360 ou buffer Scan plein 
	// Idc_Buf_Rec = Indice du premier octet du premier SCAN NEW_360 
	do
	{
		switch (  Charge_Data_Scan ( ScanPtr , &Buf_Rec[Idc_Buf_Rec], (angle_recherche * (45 << 6 )) ) )
		{
			case OUI :
				Huit_Angles[angle_recherche++] = nombre_de_scan; 
				angle_recherche &= 0x0007 ; 				
			case NON : 
				++ScanPtr ; 
				++ nombre_de_scan ;			
			break ; 
				
			default :
			case ERREUR :				
			break ;
		}		
		Idc_Buf_Rec += TAILLE_SCAN ;	
		
		if ( (( Buf_Rec[Idc_Buf_Rec] & START_FLAG_MASK ) == NEXT_360 ) && (( Buf_Rec[Idc_Buf_Rec+1] & CONTROL_ANGLE_MASK) == CONTROL_ANGLE ) )
		{
			if ( nombre_de_scan >= MAX_SCAN_DANS_BUFFER )
				fini = VRAI ; // buffer Scan plein 
		}
		else
		{	// Trouvé le prochain NEW_360 ou erreur !!
			fini = VRAI; 	
		}
	}
	while (!fini) ;
	
	return ( nombre_de_scan * sizeof (T_Scan) ) ; 
	
}

	/* Envoi d'une requête au Lidar */
static void Rplidar_Request ( uint8_t Command, T_Etat_Automate_Rplidar *Etat_Automate  )
{
	Buf_Request[1] =  Command ;	
	if ( HAL_UART_Transmit_IT(&Rplidar_Uart, (uint8_t *) &Buf_Request, TAILLE_REQUETE )  == HAL_OK )	
		*Etat_Automate = SCAN_WAIT_SENDING_REQUEST ;
}

 
static uint16_t Rplidar_LireData (uint8_t *Data, uint16_t TailleAttendue)
{
	uint16_t 	indice_courant = Get_Indice_Courant_DMA() ;
	uint16_t	taille_recue  ; 
	uint16_t	nb ;
	
	/* combien d'octets ont-ils été reçus */
	taille_recue 	= indice_courant - Indice_Debut_Lecture ;			
	if ( indice_courant < Indice_Debut_Lecture )	
	{ /* il y a eu Rool-over du buffer */
		taille_recue += TAILLE_BUF_REC ;
	}	
	if ( taille_recue < TailleAttendue )		
		return ( 0 ) ; 			/* rien reçu */	

	for ( nb = 0 ; nb < TailleAttendue ; nb++ )
	{
		Data[nb] = Buf_Rec[Indice_Debut_Lecture] ;
		if ( ++Indice_Debut_Lecture >= TAILLE_BUF_REC )
			Indice_Debut_Lecture = 0 ; 
	}
	return (TailleAttendue) ; 
}


static T_Boolean Rplidar_Verif_Response ( uint8_t TypeData ) 
{
	/* la réponse à la requête */
	if ( Rplidar_LireData ( Buf_Response, TAILLE_REPONSE ) ) 
	{
		if ( ( Buf_Response[0]  == START_FLAG_RESPONSE1 ) && ( Buf_Response[1]  == START_FLAG_RESPONSE2 ) )		
		{
			if ( Buf_Response[6] == TypeData )
				return ( VRAI ); 
		}
	}
	return ( FAUX ); 
}
			

/* fonction d'it réception */
void Rplidar_Receive_It (void)
{
	if ( Buf_Request[1] == COMMAND_SCAN )
	{
		if ( Premiere_Reception == VRAI ) 
		{
			Premiere_Reception = FAUX ; 
			if (  Rplidar_Verif_Response ( DATA_SCAN ) )
			{			
				Indice_Debut_Lecture = Rplidar_Calcule_ProchainDepart ( Indice_Debut_Lecture ) ; 
			}
			else
			{ // on n'a pas trouvé le début du scan360 
				//il faut envoyer un stop car liaison désynchronisée */
				Status_Rplidar = RPLIDAR_DEFAUT_SYNCHRO_SCAN ;
				Rplidar_Request ( COMMAND_STOP, &Etat_Automate_Rplidar ) ;				
			}
		}
		else
		{	
			uint16_t indice_debut 	= Indice_Debut_Lecture ;
			TimeoutVerifReponse 	 	= HAL_GetTick () + 500;	
			Indice_Debut_Lecture 		= Rplidar_Calcule_ProchainDepart ( Indice_Debut_Lecture ) ; 			
			
			if ( Rplidar_Cherche_Debut_360 ( &indice_debut , TAILLE_BUF_REC ) == VRAI )
			{
				New_Scan_Disponible = FAUX ;
				if ( ( BufMessageScan.Total_Octet = Rplidar_Charge_360_Scan ( indice_debut, &BufMessageScan.Data.Scan[0] ) ) != 0 )
				{
					New_Scan_Disponible = VRAI ; 
					BufMessageScan.Type = TYPE_MSG_SCAN ; 
					if ( -- Transmission_Rate == 0 )
					{
						Transmission_Rate = Transmission_Rate_Request ;							
						WifiCom_Transmit_Transport (&BufMessageScan) ; 		
					}	
				}
				else
				{	// on n'a pas trouvé la fin du scan360 
					//il faut envoyer un stop car liaison désynchronisée */
					Status_Rplidar = RPLIDAR_DEFAUT_SYNCHRO_SCAN ;
					Rplidar_Request ( COMMAND_STOP, &Etat_Automate_Rplidar ) ;
				}						
			}
			else
			{	// on n'a pas trouvé le début du scan360 
				//il faut envoyer un stop car liaison désynchronisée */
				Status_Rplidar = RPLIDAR_DEFAUT_SYNCHRO_SCAN ;
				Rplidar_Request ( COMMAND_STOP, &Etat_Automate_Rplidar ) ;			
			}				
		}		 
	}
}



/* Fonction d'it émission */
void Rplidar_Transmit_It (void)
{
	uint32_t	delay  ; 			// ms 
	switch ( Buf_Request[1] )
	{	
		case 	COMMAND_STOP :	
			delay = 3 ; 
			Etat_Automate_Rplidar = SCAN_WAIT_STOP ;			
		break ;			
		case	COMMAND_RESET	:
			delay = 15 ;
			Etat_Automate_Rplidar = SCAN_WAIT_RESET ;
		break ; 
		case	COMMAND_SCAN :
			Transmission_Rate = Transmission_Rate_Request  ;  
			Premiere_Reception = VRAI ; 											//  pour passer un tour	
			delay = 2000 ;																		// 2s initialement pour permettre initialisation du Lidar	
			Etat_Automate_Rplidar = SCAN_SUPERVISE_SCAN ;	
			Indice_Debut_Lecture = Get_Indice_Courant_DMA();
		break ; 			
		case	COMMAND_GET_INFO :
			delay = 5 ; 			
			Etat_Automate_Rplidar = SCAN_WAIT_INFO ;	
			Indice_Debut_Lecture = Get_Indice_Courant_DMA();			
		break ; 		
		case	COMMAND_GET_HEALTH :	
			delay = 3 ; 
			Etat_Automate_Rplidar = SCAN_WAIT_HEALTH ;
			Indice_Debut_Lecture = Get_Indice_Courant_DMA();		
		break ; 
		default :
			delay = 0 ;
		break ; 
	}	
	TimeoutVerifReponse  =  HAL_GetTick () + delay;		
}









/*	Requête au Rplidar 
				Command 		: COMMAND_..  défini dans Rplidar.h
				Repeat_Time	: utile uniquement avec Command = COMMAND_SCAN 
											delay en ms entre les scans 360° 500 ms à 1000ms au minimum à vérifier selon encombrement Wifi
		Une requête est acceptée lorsque l'automate est à l'état REPOS soit lorsque la précédente
		requête à été exécutée 
																	*/
void  Lazer_Scan_Request (uint8_t	Command, uint8_t Transmission_Rate ) 
{
	switch ( Command )
	{
		case 	COMMAND_STOP :
			Rplidar_Request ( COMMAND_STOP, &Etat_Automate_Rplidar ) ; 
			Rplidar_Moteur (ARRET) ;
		break ;
		case  COMMAND_RESET :
			Rplidar_Request ( COMMAND_RESET, &Etat_Automate_Rplidar ) ; 
		break ;
		case  COMMAND_SCAN :		
			Transmission_Rate_Request = Transmission_Rate ;
			Etat_Automate_Rplidar = SCAN_REQUEST_SCAN ;
			Rplidar_Moteur (MARCHE) ;			
		break ;
		case COMMAND_GET_INFO :
			Rplidar_Request ( COMMAND_GET_INFO, &Etat_Automate_Rplidar ) ;		
		break ;
		case COMMAND_GET_HEALTH :
			Etat_Automate_Rplidar = SCAN_REQUEST_HEALTH ;
		break ;	
		default :
		break ; 
	}
}


T_Boolean Rplidar_New_Scan ( void) 
{
	T_Boolean resu  = New_Scan_Disponible ;
	if ( New_Scan_Disponible == VRAI  )
	{
		New_Scan_Disponible = FAUX ; 			
	}
	return (resu) ;	 
}

/* paramètre = *Angle correspondant à l'angle recherché 0° à 359° attention pas de décimale !!!!! 
		retourne distance et angle immédiatement supérieur ou inférieur selon écart / à *Angle */
void Rplidar_Get_Distance ( T_Angle_Distance *AngleDistance ) 
{
	uint16_t idc, indice_debut, indice_precedent_debut ;
	uint16_t distance ; 
	uint16_t angle, angle_precedent; 
	uint16_t nb_scan = BufMessageScan.Total_Octet/ sizeof (T_Scan) ;	
	uint16_t angle_recherche = AngleDistance->Angle ; 
	T_Boolean found = FAUX ; 
	
	/* on se positionne dans le buffer selon la valeur recherchée */
	idc = angle_recherche / 45 ;
	indice_debut 	= Huit_Angles[idc] ;
	indice_precedent_debut =	indice_debut != 0 ? indice_debut - 1 : nb_scan - 1 ;
	angle = BufMessageScan.Data.Scan[indice_precedent_debut].Angle ;
	angle_recherche <<= 6 ;	/* formate l'angle selon format d'un scan */

	/* angle donne l'angle exact ou immédiatement supérieur , indice_debut donne sa position 
		et angle_precedent donne l'angle exact ou immédiatement inférieur, indice_precedent_debut donne sa position*/
	do {
			angle_precedent = angle ;
			angle = BufMessageScan.Data.Scan[indice_debut].Angle ;		
			if ( angle < angle_recherche )
			{
				indice_precedent_debut = indice_debut ;
				if ( ++indice_debut >= nb_scan )
					indice_debut = 0 ; 	
			}	
			else
			{
				found = VRAI ; 
			}
	}
	while ( ! found ) ;
	
	/* déterminer l'angle le plus proche de celui demandé */
	uint16_t delta_angle_precedent = angle_recherche - angle_precedent ; 	
	uint16_t delta_angle 					 = angle - angle_recherche ; 	
	
	if ( delta_angle_precedent < delta_angle  )
	{	/* l'angle inférieur est plus proche aussi on le choisit */
		indice_debut = indice_precedent_debut ;
	}
	
	angle 	 = BufMessageScan.Data.Scan[indice_debut].Angle ;	

	if ( BufMessageScan.Data.Scan[indice_debut].Quality != 0 )	
		distance = BufMessageScan.Data.Scan[indice_debut].Distance ;		 
	else
		distance = 10000 << 2 ;		/* qualité = 0 convention distance = infinie soit 10 m */	

	AngleDistance->Angle    = (float)(angle) 		/ 64.0f ;
	AngleDistance->Distance = (float)(distance) / 4.0f  ;		
}



/* Cette fonction doit être appelée périodiquement */
void Rplidar_Gestion (void) 
{
	static 	uint8_t		nb_defaut = 0 ;
	static	T_Etat_Automate_Rplidar	Etat_Automate_Rplidar_Suivant = SCAN_REPOS ;
	
	switch ( Etat_Automate_Rplidar )
	{
		case SCAN_DEFAUT :
		break ; 
		
		case SCAN_REPOS :
		break ;
				
		case SCAN_REQUEST_HEALTH :
			Etat_Automate_Rplidar_Suivant = SCAN_REPOS ;								// uniquement si pas d'erreur voir Cf 1 			
			Rplidar_Request ( COMMAND_GET_HEALTH, &Etat_Automate_Rplidar ) ;
		break ; 
											
		/* Demande scan sur 360° */
		case SCAN_REQUEST_SCAN :
			Etat_Automate_Rplidar_Suivant = SCAN_START_SCAN ;						// uniquement si pas d'erreur voir Cf 1				
			/* Vérification préalable du status du Lidar */
			Rplidar_Request ( COMMAND_GET_HEALTH, &Etat_Automate_Rplidar ) ;		
		break ; 
				
			/* Démarre le scan sur 360 ° */
		case SCAN_START_SCAN :
		  Rplidar_Request (COMMAND_SCAN, &Etat_Automate_Rplidar ) ;	
		break ;
	
			/* Attendre fin d'envoi de la requête */
		case	SCAN_WAIT_SENDING_REQUEST :
		break ;
		

		case SCAN_WAIT_STOP :
			if ( TimeoutVerifReponse < HAL_GetTick() )
			{	
				Etat_Automate_Rplidar = SCAN_REPOS;					
			}		
		break ;
	
		case SCAN_WAIT_RESET :
			if ( TimeoutVerifReponse < HAL_GetTick() )
			{	
				Etat_Automate_Rplidar = SCAN_REQUEST_HEALTH ;					
			}		
		break ;
								
		case SCAN_WAIT_HEALTH :	
			if ( TimeoutVerifReponse < HAL_GetTick() )
			{	
				if (  Rplidar_Verif_Response ( DATA_HEALTH ) )
				{
					if ( Rplidar_LireData ( Buf_Info, TAILLE_HEALTH ) )
					{
						Status_Rplidar 			= Buf_Health[0] ;
						ErrorCode_Rplidar   = Buf_Health[1] | (Buf_Health[2] << 8) ;;			
						if ( Status_Rplidar != RPLIDAR_ERROR )
						{	/* Rplidar Ok, enchaine la commande */
							Etat_Automate_Rplidar = Etat_Automate_Rplidar_Suivant ; 	// Cf 1  uniquement si pas d'erreur 
							nb_defaut = 0 ;	
						}
						else
						{	/* Rplidar en défaut */
							if ( ++ nb_defaut > 5 )
							{
								Etat_Automate_Rplidar = SCAN_DEFAUT ;			// terminé pour lui au bout de 5 essais consécutifs
							}
							else
							{
								Rplidar_Request ( COMMAND_RESET, &Etat_Automate_Rplidar );		// tentative suivante	
							}
						}													
					}
					else
					{
						Status_Rplidar 				= RPLIDAR_TIMEOUT ; 	
						Etat_Automate_Rplidar = SCAN_REPOS ;						
					}								
				}
				else
				{
					Status_Rplidar 				= RPLIDAR_TIMEOUT ; 		
					Etat_Automate_Rplidar = SCAN_REPOS ;				
				}
			}					
		break ; 
		
		case SCAN_WAIT_INFO : 
			if ( TimeoutVerifReponse < HAL_GetTick() )
			{	
				if (  Rplidar_Verif_Response ( DATA_INFO ) )
				{
					if ( Rplidar_LireData ( Buf_Info, TAILLE_INFO ) )
					{
						Etat_Automate_Rplidar = SCAN_REPOS;								
					}
					else
					{
						Status_Rplidar 				= RPLIDAR_TIMEOUT ; 		
						Etat_Automate_Rplidar = SCAN_REPOS ;							
					}								
				}
				else
				{
					Status_Rplidar 				= RPLIDAR_TIMEOUT ; 		
					Etat_Automate_Rplidar = SCAN_REPOS ;						
				}
			}				
		break ;	
		
	
		case SCAN_SUPERVISE_SCAN :
			if ( TimeoutVerifReponse < HAL_GetTick() )
			{	
				Status_Rplidar 			= RPLIDAR_TIMEOUT ; 					
				Rplidar_Request ( COMMAND_STOP, &Etat_Automate_Rplidar ) ;	
			}				
		break ; 
			
		default :
			Etat_Automate_Rplidar = SCAN_REPOS ;	
		break ; 
			
	}
}


void Rplidar_Start ( void ) 
{
	Rplidar_Moteur (ARRET)  ;	
	HAL_UART_Receive_DMA(&Rplidar_Uart, Buf_Rec, TAILLE_BUF_REC) ;
}


