/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "typegen.h"
#include "Tempo.h"
#include "Moteurs.h"
#include "US_Capteur.h"
#include "usart.h"
#include "Sauve_Restaure.h"
#include "Rplidar.h"
#include "WifiCom.h"
#include "HMC5883L.h"
#include "string.h"




#define	PERIODE_ENVOI_WIFI 			80				// ms 



T_Requete	 Requete_Exterieure 			= REQUETE_MSG_REGLAGE ; 


 

volatile uint8_t			Idc_Buf_Rec_Ecr  ;				// +1 mod MAX_BUF_REC par fonction d'interruption reception série
																									// on considére pour ce protocole qu'il n'est pas possible de recevoir plus de 2 messages
																									// consécutifs sans en avoir extrait un d'ou MAX_BUF_REC = 2 
volatile uint8_t			Idc_Buf_Rec_Lec  ;				// +1 mod MAX_BUF_REC par fonction Wifi_Gestion 

#define			MAX_BUF_REC				2 									// !!!!!!!  Valeur maximale = 4 
 
volatile 		T_Boolean 	Etat_Buf_Rec_Wifi[MAX_BUF_REC] ; 
 
#define			Si_Recu_De_Wifi()								Etat_Buf_Rec_Wifi[Idc_Buf_Rec_Lec] == VRAI  

#define			Mise_A_Jour_Idc_Buf_Lec_Ecr()		Etat_Buf_Rec_Wifi[Idc_Buf_Rec_Lec] = FAUX ;   	\
																						if ( ++Idc_Buf_Rec_Lec >= MAX_BUF_REC )										\
																							Idc_Buf_Rec_Lec = 0 ;	
																						
#define			Mise_A_Jour_Idc_Buf_Rec_Ecr()		Etat_Buf_Rec_Wifi[Idc_Buf_Rec_Ecr] = VRAI ;   	\
																						if ( ++Idc_Buf_Rec_Ecr >= MAX_BUF_REC )										\
																							Idc_Buf_Rec_Ecr = 0 ;	
	
T_Buf_Wifi  Wifi_Buf_Rec[MAX_BUF_REC] ;  
T_Buf_Wifi  Wifi_Buf_Emi ;
uint8_t			Max_Repetition ;	// max répétition sur reception NAck lors de l'envoi d'un message normal 



#define	MAX_REPETITION_TRANSPORT  2 
#define	MAX_REPETITION					  2 



typedef struct {
	T_BufMessageLong *Buf ;
	uint8_t						Max_Repetition ;
	T_Etat_Transport	Etat_Transport ;	
	T_Etat_Transport	Etat_Transport_Precedent ;
	uint8_t						Reserve ; 
	uint16_t					Position_Premier_Octet ;				// Position du premier octet envoyé Data[0] si fractionnement 
	T_Buf_Wifi  			Wifi_Buf_Emi ;	
} T_Transport_Emis ; 

T_Transport_Emis Transport_Emis = { NULL, MAX_REPETITION_TRANSPORT , DISPO, DONE, 0 } ; 

T_Presence_Host Presence_Host = HOST_ABSENT ;


T_Presence_Host Host_Present ( void )
{
	return ( Presence_Host ) ;
}

	
	uint32_t 	Calcule_Checksum ( uint32_t pBuffer[], uint32_t BufferLength )
{
	  uint32_t index = 0;
	  uint32_t checksum = 0 ;
	  /* Enter Data to the CRC calculator */
	  for(index = 0; index < BufferLength; index++)
	  {
	    checksum += pBuffer[index] ;
	  }
	  return ( (~checksum ) + 1 ) ;
}

int8_t WifiCom_Transmit_NextPacket (void)
{
	int8_t status ; 
	
	if ( Transport_Emis.Position_Premier_Octet < Transport_Emis.Buf->Total_Octet )
	{ 
		Transport_Emis.Wifi_Buf_Emi.Transport.Header1									 =  HEADER1 ;
		Transport_Emis.Wifi_Buf_Emi.Transport.Header2									 =  HEADER2 ;		
		Transport_Emis.Wifi_Buf_Emi.Transport.Type 		 					 			 =  Transport_Emis.Buf->Type ; 
		Transport_Emis.Wifi_Buf_Emi.Transport.Total_Octet			 				 =  Transport_Emis.Buf->Total_Octet ; 
		Transport_Emis.Wifi_Buf_Emi.Transport.Position_Premier_Octet	 =  Transport_Emis.Position_Premier_Octet; 	
		Transport_Emis.Wifi_Buf_Emi.Transport.Nombre_Doctets		 			 =   ( Transport_Emis.Buf->Total_Octet - Transport_Emis.Position_Premier_Octet ) >= TAILLE_BUF_TRANSPORT 
																											 ?	TAILLE_BUF_TRANSPORT  
																											 : ( Transport_Emis.Buf->Total_Octet - Transport_Emis.Position_Premier_Octet ) ; // sinon le reste des octets à envoyer
		
		memcpy ( Transport_Emis.Wifi_Buf_Emi.Buf_Transport.Data, &Transport_Emis.Buf->Data.Byte[Transport_Emis.Wifi_Buf_Emi.Transport.Position_Premier_Octet ], Transport_Emis.Wifi_Buf_Emi.Transport.Nombre_Doctets ) ;
	//	Transport_Emis.Wifi_Buf_Emi.Buf_Transport.Crc				= HAL_CRC_Calculate(&hcrc, Transport_Emis.Wifi_Buf_Emi.Word, TAILLE_MESSAGE_TRANSPORT /4) ; 
		Transport_Emis.Wifi_Buf_Emi.Buf_Transport.Crc	= Calcule_Checksum ( Transport_Emis.Wifi_Buf_Emi.Word, TAILLE_MESSAGE_TRANSPORT /4 ) ; 
		status =  HAL_UART_Transmit_DMA(&Wifi_Com_Uart, Transport_Emis.Wifi_Buf_Emi.Octet, TAILLE_MESSAGE) ;		
		if ( status == HAL_OK )
		{	// mise à jour pour prochain packet
			Transport_Emis.Position_Premier_Octet	+= Transport_Emis.Wifi_Buf_Emi.Transport.Nombre_Doctets	 ; 
	  }
	}
	else
	{ /* envoi terminé */
		status = -1 ;  // indique plus rien à transmettre 
	}
	return ( status ) ;
}





T_Boolean WifiCom_Transmit_Transport (T_BufMessageLong *BufMessageLong)
{	
	if ( Transport_Emis.Etat_Transport == DISPO  )
	{
		Transport_Emis.Position_Premier_Octet = 0;  // pour premier paquet 
		Transport_Emis.Buf 						= BufMessageLong ;

		Transport_Emis.Max_Repetition = MAX_REPETITION_TRANSPORT ; 
		Transport_Emis.Etat_Transport = DEMANDE ; 
		Transport_Emis.Buf->Etat_Transport	= DEMANDE ;
		return (VRAI) ; 
	}
	return (FAUX) ;
}

extern uint8_t  Etat_Automate_Suivi_Mur ;

HAL_StatusTypeDef WifiCom_Transmit_Reglage ( void )
{
	uint32_t etat_uart = HAL_UART_GetState ( &Wifi_Com_Uart ) ;
	
	if ( ( etat_uart != HAL_UART_STATE_BUSY_TX ) && ( etat_uart != HAL_UART_STATE_BUSY_TX_RX ) )
	{	/* Charge le buffer émission */
		Wifi_Buf_Emi.Transport.Header1														=  HEADER1 ;
		Wifi_Buf_Emi.Transport.Header2														=  HEADER2 ;			
		Wifi_Buf_Emi.Transport.Type 		 					   							=  TYPE_MSG_REGLAGE ; 
		Wifi_Buf_Emi.Transport.Total_Octet			 		 							=  0 ; 
		Wifi_Buf_Emi.Transport.Position_Premier_Octet							=  0 ; 
		Wifi_Buf_Emi.Transport.Nombre_Doctets				 							=  TAILLE_MESSAGE_REGLAGE ;
		
		Wifi_Buf_Emi.Reglage.Etat_Moteur_Droit 										=  Etat_Moteur[MOTEUR_DROIT] ; 		
		Wifi_Buf_Emi.Reglage.Etat_Moteur_Gauche										=  Etat_Moteur[MOTEUR_GAUCHE] ; 
		Wifi_Buf_Emi.Reglage.ConsigneManuelle.Vitesse							=  ConsigneManuelle.Vitesse ;
		Wifi_Buf_Emi.Reglage.ConsigneManuelle.ConsigneAngulaire		=  ConsigneManuelle.ConsigneAngulaire ;		
		Wifi_Buf_Emi.Reglage.VitesseMesure[MOTEUR_DROIT] 					=  VitesseMesure[MOTEUR_DROIT] ;
		Wifi_Buf_Emi.Reglage.VitesseMesure[MOTEUR_GAUCHE] 				=  VitesseMesure[MOTEUR_GAUCHE] ;
		Wifi_Buf_Emi.Reglage.Phare_Luminosite 										=  Phare_Luminosite ;
		Wifi_Buf_Emi.Reglage.Temperature_Exterieure 							=	 Etat_Automate_Suivi_Mur ;//=  Temperature_Exterieure ;
		Wifi_Buf_Emi.Reglage.V_Batterie 													=  V_Batterie ;		
		Wifi_Buf_Emi.Reglage.Etat_Sauvegarde 											=  Etat_Sauvegarde;								
		Wifi_Buf_Emi.Reglage.Mode_Commande	 											=	 Mode_Commande ;											
		Wifi_Buf_Emi.Reglage.ConsigneBoucle_Ouverte_Moteur_Droit  =  ConsigneBoucle_Ouverte[MOTEUR_DROIT]; 
		Wifi_Buf_Emi.Reglage.ConsigneBoucle_Ouverte_Moteur_Gauche =  ConsigneBoucle_Ouverte[MOTEUR_GAUCHE] ;
		Wifi_Buf_Emi.Reglage.Status_Lidar 												=  Status_Rplidar ;												
		Wifi_Buf_Emi.Reglage.Status_Moteur_Lidar									=  Status_Moteur_Lidar;										
		Wifi_Buf_Emi.Reglage.Erreur_Code_Lidar 										=  ErrorCode_Rplidar;
		Wifi_Buf_Emi.Reglage.Offset_Orientation_Lidar 						=  Offset_Orientation_Lidar ;
		Wifi_Buf_Emi.Reglage.NotUsed1 														=  Offset_Orientation_Lidar ;    // dispo apour autre 
		Wifi_Buf_Emi.Reglage.Compas																=  Compas ;
		Wifi_Buf_Emi.Reglage.ConsigneAngulaireNulle 							=	 ConsigneAngulaireNulle ;
		Wifi_Buf_Emi.Reglage.Compensation_Ligne_Droite						=  Compensation_Ligne_Droite;
		Wifi_Buf_Emi.Reglage.Distance_Obstacle_Av 								=  Distance_Obstacle_Av ;
		Wifi_Buf_Emi.Reglage.Distance_Obstacle_Ar 								=  Distance_Obstacle_Ar ;		
		Wifi_Buf_Emi.Reglage.Suivi_De_Mur													=	 Suivi_De_Mur;
		
		Wifi_Buf_Emi.Reglage.Crc	= Calcule_Checksum ( Wifi_Buf_Emi.Word, TAILLE_MESSAGE_REGLAGE /4);
		Max_Repetition = MAX_REPETITION ;
		return ( HAL_UART_Transmit_DMA(&Wifi_Com_Uart, Wifi_Buf_Emi.Octet, TAILLE_MESSAGE) );
	}
	else
		return ( HAL_BUSY ); 
}


HAL_StatusTypeDef WifiCom_Transmit_Service ( T_Requete Requete,	T_Ack_Nack Ack_Nack, T_Presence_Host Etat_Host )
{
	uint32_t etat_uart = HAL_UART_GetState ( &Wifi_Com_Uart ) ;
	
	if ( ( etat_uart != HAL_UART_STATE_BUSY_TX ) && ( etat_uart != HAL_UART_STATE_BUSY_TX_RX ) )
	{	/* Charge le buffer émission */
		Wifi_Buf_Emi.Transport.Header1											=  HEADER1 ;
		Wifi_Buf_Emi.Transport.Header2											=  HEADER2 ;					
		Wifi_Buf_Emi.Transport.Type 		 					   				=  TYPE_MSG_DE_SERVICE ; 
		Wifi_Buf_Emi.Transport.Total_Octet			 		 				=  0 ; 
		Wifi_Buf_Emi.Transport.Position_Premier_Octet				=  0 ; 	
		Wifi_Buf_Emi.Transport.Nombre_Doctets				 				=  TAILLE_MESSAGE_DE_SERVICE ;
		Wifi_Buf_Emi.De_Service.Type.Service.Requete 		 		= 	Requete ; 		
		Wifi_Buf_Emi.De_Service.Type.Service.Ack_Nack 		 	= 	Ack_Nack ;
		Wifi_Buf_Emi.De_Service.Type.Service.Presence_Host	= 	Etat_Host ;		
		Wifi_Buf_Emi.De_Service.Crc									 				= Calcule_Checksum ( Wifi_Buf_Emi.Word, TAILLE_MESSAGE_DE_SERVICE /4);
		Max_Repetition = 0 ;
		return ( HAL_UART_Transmit_DMA(&Wifi_Com_Uart, Wifi_Buf_Emi.Octet, TAILLE_MESSAGE) );
	}
	else
		return ( HAL_BUSY ); 		
}


static HAL_StatusTypeDef WifiCom_Reception (uint8_t Buf_Idc)
{
	return ( HAL_UART_Receive_DMA(&Wifi_Com_Uart, Wifi_Buf_Rec[Buf_Idc].Octet, TAILLE_MESSAGE )	);
}

HAL_StatusTypeDef WifiCom_Communication_Start (void)
{
	Idc_Buf_Rec_Ecr = 0 ;			
	Idc_Buf_Rec_Lec = 0 ;			
	for ( int i = 0 ; i < MAX_BUF_REC ; i++ )  
		Etat_Buf_Rec_Wifi[i] = FAUX ; 
	return ( WifiCom_Reception (0) ); 
}

#ifdef DEBUG_WIFI 
	uint32_t  ERREUR_CHECKSUM = 0 ; 
	uint32_t	Msg_Reglage_Rx = 0 ; 
	uint32_t	Msg_Requete_RxAck = 0 ; 
	uint32_t	Msg_Requete_RxNack = 0 ; 
	uint32_t	Msg_Requete_service = 0 ; 
	uint32_t 	Mesg_reglage_Tx = 0 ; 
	uint32_t 	Mesg_transport_Tx = 0 ; 
	uint32_t	Msg_Requete_TxAck = 0 ; 
	uint32_t	Msg_Requete_TxNack = 0 ; 
	uint32_t 	Mesg_transport_Abort = 0 ;
	uint32_t 	Mesg_defaut_Synchro = 0 ;
	uint32_t	CPT_time_out_wifi = 0 ; 
	uint32_t	CPT_Demande_Initialisation = 0 ; 
#endif

static T_Ack_Nack WifiCom_Lire_Data ( T_Buf_Wifi *Buf )
{
  T_Ack_Nack qualite ;
	uint16_t 	taille = 0 ; 					// par défaut 
	uint32_t  crc_recu ;

	
	if ( 			( Buf->Transport.Header1 ==  HEADER1 )
				&&	( Buf->Transport.Header2 ==  HEADER2 )	)
	{	/* Header correct */	
		switch ( Buf->Transport.Type	)
		{	/* vérifie le type de message reçu */
			case TYPE_MSG_REGLAGE :
				taille = Buf->Transport.Nombre_Doctets;
				crc_recu = Buf->Reglage.Crc ;		
			break ;
			
			case TYPE_MSG_DE_SERVICE :
				taille = Buf->Transport.Nombre_Doctets ;
				crc_recu = Buf->De_Service.Crc ;
			break ;

/*			case TYPE_MSG_SCAN :
				taille = TAILLE_MESSAGE_TRANSPORT ;
				crc_recu = Buf->Buf_Transport.Crc ;
			break ;  */
			
			default :
			/* reçu un message de type inconnu */			
			break ; 
		}
	}
		
	if ( taille != 0 )
	{
		if ( 	crc_recu == Calcule_Checksum (Buf->Word, taille/4)  ) 			
		{	/* le checksum est bon alors récupére les données */
			switch ( Buf->Transport.Type	)
			{
				case TYPE_MSG_REGLAGE :		
					Etat_Moteur[MOTEUR_DROIT].CoefP  			= Buf->Reglage.Etat_Moteur_Droit.CoefP ;
					Etat_Moteur[MOTEUR_DROIT].CoefI  			= Buf->Reglage.Etat_Moteur_Droit.CoefI ;
					Etat_Moteur[MOTEUR_DROIT].CoefD  			= Buf->Reglage.Etat_Moteur_Droit.CoefD ;
					Etat_Moteur[MOTEUR_GAUCHE].CoefP 			= Buf->Reglage.Etat_Moteur_Gauche.CoefP ;
					Etat_Moteur[MOTEUR_GAUCHE].CoefI 			= Buf->Reglage.Etat_Moteur_Gauche.CoefI ;
					Etat_Moteur[MOTEUR_GAUCHE].CoefD 			= Buf->Reglage.Etat_Moteur_Gauche.CoefD ;
					ConsigneManuelle.Vitesse			   			= Buf->Reglage.ConsigneManuelle.Vitesse;
					ConsigneManuelle.ConsigneAngulaire		= Buf->Reglage.ConsigneManuelle.ConsigneAngulaire ;	
					Phare_Luminosite 											=	Buf->Reglage.Phare_Luminosite  ;								
					Mode_Commande	 												=	Buf->Reglage.Mode_Commande ;											
					ConsigneBoucle_Ouverte[MOTEUR_DROIT]	= Buf->Reglage.ConsigneBoucle_Ouverte_Moteur_Droit  ; 
					ConsigneBoucle_Ouverte[MOTEUR_GAUCHE]	= Buf->Reglage.ConsigneBoucle_Ouverte_Moteur_Gauche ;
					Offset_Orientation_Lidar							= Buf->Reglage.Offset_Orientation_Lidar ;
					ConsigneAngulaireNulle								= Buf->Reglage.ConsigneAngulaireNulle ;
					Compensation_Ligne_Droite							= Buf->Reglage.Compensation_Ligne_Droite;
					Suivi_De_Mur													= Buf->Reglage.Suivi_De_Mur	;
			
					qualite = SANS_ACK_NACK ; 
					
			#ifdef DEBUG_WIFI 
					++ Msg_Reglage_Rx ; 
			#endif
				break ;
					
/*				case TYPE_MSG_SCAN :   transfert du contenu au bon endroit et vérifie si scan complet 
					LazerScanRec.Transport.Type 		 							=  Buf->Transport.Type	; 
					LazerScanRec.Transport.Total_Octet 		 				=  Buf->Transport.Total_Octet	; 
					LazerScanRec.Transport.Position_Premier_Octet	=  Buf->Transport.Position_Premier_Octet ; 	
					if ( Buf->Transport.Position_Premier_Octet == 0 ) 
						LazerScanRec.Transport.Nombre_Doctets = 0 ; 
					memcpy ( &LazerScanRec.Data[Buf->Transport.Position_Premier_Octet], Buf->Buf_Transport.Data, Buf->Transport.Nombre_Doctets ) ;					
					LazerScanRec.Transport.Nombre_Doctets		+=  Buf->Transport.Nombre_Doctets ;
				  if ( LazerScanRec.Transport.Nombre_Doctets == LazerScanRec.Transport.Total_Octet )
					{	 le scan a été reçu entierement 
						Lazer_Scan_Received () ;					
					}			
					qualite = ACK_MSG ;
				break ; 
*/				
				case TYPE_MSG_DE_SERVICE :					
					switch ( Buf->De_Service.Type.Service.Requete ) 
					{
						case REQUETE_ACK_NACK :			// réception du nack du message envoyé précédemment 
																		// ou de ack / nack du message TYPE_MSG_SCAN envoyé précédemment	
							Presence_Host = Buf->De_Service.Type.Service.Presence_Host ;
						
							if ( Transport_Emis.Etat_Transport == EN_COUR )
							{	/* si envoi d'un message long en cour */
								if ( Buf->De_Service.Type.Service.Ack_Nack == ACK_MSG )
								{	/* envoi du paquet suivant s'il en reste */
									Transport_Emis.Max_Repetition = MAX_REPETITION_TRANSPORT ;
									if  ( WifiCom_Transmit_NextPacket() == -1 )
									{ /* Envoi du message terminé, recu ACK du dernier paquet envoyé*/	
										Transport_Emis.Etat_Transport = DONE ; 
									}
				#ifdef DEBUG_WIFI								
									++ Msg_Requete_RxAck ;
				#endif
								}
								else
								{	/* renvoi du message */						
									if ( Transport_Emis.Max_Repetition-- != 0 )
									{
									  HAL_UART_Transmit_DMA(&Wifi_Com_Uart, Transport_Emis.Wifi_Buf_Emi.Octet, TAILLE_MESSAGE) ;
									}
									else
									{ /* Envoi du message aborté*/
										Transport_Emis.Etat_Transport = ABORT ;																					
									}	
				#ifdef DEBUG_WIFI									
									++Msg_Requete_RxNack	 ;
				#endif
								}	
							}
							else
							{
								if ( Buf->De_Service.Type.Service.Ack_Nack == NACK_MSG )
								{	/* renvoi du message */
									if ( Max_Repetition-- != 0 )
									{
										HAL_UART_Transmit_DMA(&Wifi_Com_Uart, Wifi_Buf_Emi.Octet, TAILLE_MESSAGE);
									}
									else
									{ /* Envoi du message aborté*/	
										Max_Repetition = 0 ; 										
									}
				#ifdef DEBUG_WIFI								
									++Msg_Requete_RxNack	 ;	
				#endif					
								}		
				#ifdef DEBUG_WIFI						
								else
										++Msg_Requete_RxAck	 ;	
				#endif				
							}					
						break ; 
							
						case REQUETE_SERVICE :
							/* message envoyé par défaut par la carte wifi lorsque pas de message du Host */
							Presence_Host = Buf->De_Service.Type.Service.Presence_Host ;
				#ifdef DEBUG_WIFI							
							++ Msg_Requete_service	;	
				#endif
						break ; 
											
						case REQUETE_MSG_REGLAGE :
							Requete_Exterieure = REQUETE_MSG_REGLAGE ; 
						break ; 
						
						case REQUETE_ENREGISTRE_PARAM1 :
							Sauvegarde_Tout_Request () ; 
						break ; 
						
						case REQUETE_ENREGISTRE_PARAM2 :
						break ; 
						
						case REQUETE_MSG_SCAN :
							Lazer_Scan_Request (Buf->De_Service.Type.Demande.Arg1, Buf->De_Service.Type.Demande.Arg2) ;
						break ;												

#ifdef HMC5883L_VALIDATION 							
						case REQUETE_MSG_COMPAS_XY :
							HMC5883L_Request (Buf->De_Service.Type.Demande.Arg1 ) ;
						break ;
#endif
						default :
								/* reçu une requete inconnue */						
						break ; 
					}	
					qualite = SANS_ACK_NACK ;	 /* on n'acquitte pas les messages de requete */
				break ;
				
				default :
					/* reçu un message de type inconnu On ne peut normalement jamais arriver ici */		
					qualite = SANS_ACK_NACK ;					
				break ; 
			}				
		}
		else
		{ /* le message reçu présente une erreur de CRC */
	#ifdef DEBUG_WIFI				
			ERREUR_CHECKSUM ++ ; 		
	#endif
			if ( Transport_Emis.Etat_Transport == EN_COUR )
			{ /* Envoi du message terminé car on ne se comprend plus c'est une erreur sur reception du msg d'ack/nack */
				Transport_Emis.Etat_Transport = ABORT ;		
				qualite = SANS_ACK_NACK ;					
			}	
			else
			{				
				qualite = NACK_MSG ;
			}
		} 
	}
	else
	{ /* reçu un message de type inconnu ou header non conforme */	
		if ( Transport_Emis.Etat_Transport == EN_COUR )
		{ /* Envoi du message terminé car on ne se comprend plus c'est une erreur sur reception du msg d'ack/nack */
			Transport_Emis.Etat_Transport = ABORT ;		
		}			
		qualite = ERREUR_SYNCHRO ;	
	#ifdef DEBUG_WIFI				
		++ Mesg_defaut_Synchro  ;
	#endif		
	}
	return ( qualite ) ;
}



/* return :	FAUX  si time-out réception */

T_Boolean  WifiCom_Gestion ( void ) 
{
	static uint32_t	time_out_wifi = 2000 ;
	static uint32_t	time_out_transport = 0 ;	
	static uint32_t	Date_Envoi = 500 ; 
	static T_Boolean Demande_Initialisation = FAUX ; 	
	T_Boolean etat_com_wifi = VRAI ; 

	
	
	
	
	if ( Demande_Initialisation == VRAI )
	{
		if ( time_out_wifi <= HAL_GetTick () )		
		{
			Demande_Initialisation = FAUX ;
	#ifdef DEBUG_WIFI					
			++CPT_Demande_Initialisation ; 
	#endif
			WifiCom_Communication_Start () ; 
			time_out_wifi = HAL_GetTick () + 1000 ;
		}
	}
	else
	{	
		if ( Si_Recu_De_Wifi() )
		{			
			T_Ack_Nack qualite = WifiCom_Lire_Data ( &Wifi_Buf_Rec [Idc_Buf_Rec_Lec] ) ;		
			Mise_A_Jour_Idc_Buf_Lec_Ecr() ;	
			switch ( qualite )
			{			
				case  ACK_MSG 	:
		#ifdef DEBUG_WIFI							
					++Msg_Requete_TxAck	 ;	
		#endif		
					// on a reçu un message transport alors envoyer un message de service avec ack ou nack, ou un message normal en erreur envoyer NACK
					WifiCom_Transmit_Service ( REQUETE_ACK_NACK, qualite, 0 ) ; 
				break ;			
				case  NACK_MSG  :
		#ifdef DEBUG_WIFI							
					++Msg_Requete_TxNack	 ;	
		#endif		
					// on a reçu un message transport alors envoyer un message de service avec ack ou nack, ou un message normal en erreur envoyer NACK
					WifiCom_Transmit_Service ( REQUETE_ACK_NACK, qualite, 0 ) ; 
				break ; 			
	
				case ERREUR_SYNCHRO :
					Demande_Initialisation = VRAI ;						
					HAL_UART_DMAStop ( &Wifi_Com_Uart) ;
					/* Lance une petite tempo pour permettre à l'it Rx générée par AbortDMA de passer 
						 avant la réinitialisation */
					time_out_wifi = HAL_GetTick () + 5 ;				
					return ( VRAI ) ; 
			
				default : 
				break ; 	
			}
			time_out_wifi = HAL_GetTick () + (10 * PERIODE_ENVOI_WIFI)  ; 	//  ms 			
		}
		else
		{
			if ( time_out_wifi <= HAL_GetTick () )
			{
			#ifdef DEBUG_WIFI					
					++CPT_time_out_wifi ;
			#endif	
					__HAL_UART_CLEAR_PEFLAG(&Wifi_Com_Uart);
				
//					Demande_Initialisation = VRAI ;						
//					HAL_UART_DMAStop ( &Wifi_Com_Uart) ;
					/* Lance une petite tempo pour permettre à l'it Rx générée par AbortDMA de passer 
						 avant la réinitialisation */
	//				time_out_wifi = HAL_GetTick () + 5 ;
					time_out_wifi = HAL_GetTick () + (10 * PERIODE_ENVOI_WIFI)  ;
					return ( FAUX ) ; 
			}
		}	
	

		int8_t status = HAL_ERROR ;
		switch ( Transport_Emis.Etat_Transport )
		{		
			case DISPO : 
				/* Selon la position de l'automate demandé par le distant , envoi du message */
				
				if ( Presence_Host == HOST_DEFAUT )
				{
						WifiCom_Transmit_Service ( REQUETE_RESET_WIFI, 0, 0 ) ; 
						Presence_Host = HOST_ABSENT ;
				}
				else
				{
					switch ( Requete_Exterieure ) 
					{
						case REQUETE_MSG_REGLAGE :
							if ( Date_Envoi <= HAL_GetTick () )
							{
								if ( (status = WifiCom_Transmit_Reglage()) == HAL_OK )
								{	
					#ifdef DEBUG_WIFI											
									++Mesg_reglage_Tx ;
					#endif 
								}
							}
						break ;
							
						default : 
						break ; 
					}
				}
			break ;
				
			case DEMANDE :
				if ( Date_Envoi <= HAL_GetTick () )
				{
					if ( (status = WifiCom_Transmit_NextPacket ()) == HAL_OK )
					{
						Transport_Emis.Etat_Transport 		 = EN_COUR ; 
						Transport_Emis.Buf->Etat_Transport = EN_COUR ; 
						/* arme time-out réception */ 
						time_out_transport = HAL_GetTick () + 10 ; 	//  < à 6,5 ms max pour transfert d'un message de 2000 octets					
					}
				}
			break ;					
			
			case DONE :
		#ifdef DEBUG_WIFI		
				++ Mesg_transport_Tx ;
		#endif	
			  Transport_Emis.Etat_Transport_Precedent = DONE ; 			
				Transport_Emis.Etat_Transport 					= DISPO ; 
				Transport_Emis.Buf->Etat_Transport 			= DISPO ; 			
			break ;
			
			case ABORT :
		#ifdef DEBUG_WIFI						
				++ Mesg_transport_Abort ;
		#endif	
			  Transport_Emis.Etat_Transport_Precedent = ABORT ; 
				Transport_Emis.Etat_Transport 					= DISPO ; 
				Transport_Emis.Buf->Etat_Transport 			= DISPO ; 	
			break ;
			
			case EN_COUR :
				if ( time_out_transport <= HAL_GetTick () )
					Transport_Emis.Etat_Transport = ABORT ;		// abort transport car pas de réponse de la carte WIFI 			
			break ;
			default : 
			break ; 	

		}
		if ( status == HAL_OK )
		{	/* un envoi a eu lieu calcule prochaine date d'envoi */ 
			Date_Envoi = HAL_GetTick () + PERIODE_ENVOI_WIFI ; 
		}		

	}
	return ( etat_com_wifi ) ;
}





void WifiCom_Receive_It (void)
{
	Mise_A_Jour_Idc_Buf_Rec_Ecr() ;
	WifiCom_Reception (Idc_Buf_Rec_Ecr) ; 
}







