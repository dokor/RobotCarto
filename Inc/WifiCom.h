
#ifndef 	H_WIFICOM
#define		H_WIFICOM 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"
#include "Moteurs.h"
#include "US_Capteur.h"
#include "Batterie.h"
#include "Phares.h"
#include "Temperature.h"
#include "Sauve_Restaure.h"
#include "Rplidar.h"
#include "HMC5883L.h"
#include "Guidage.h"



#define Wifi_Com_Uart			huart6   // il faudra également modifier l'affection dans stm32f4xx_it.c


#define	TAILLE_CRC				 			 4   	   // sur 4 octets 
#define	TAILLE_BUF_TRANSPORT	 200			// doit être multiple de 4 octets et 5 pour contenir un les échantillons de scan lazer


#define	HEADER1			0x55AA55AA 
#define	HEADER2			0x65BA65BA

typedef enum {
		TYPE_MSG_INTERDIT,
		TYPE_MSG_DE_SERVICE = 1, 
		TYPE_MSG_REGLAGE, 									//  
		TYPE_MSG_LONG = 40,									// à partir de 40 tous les messages de taille > TAILLE_BUF_TRANSPORT
		TYPE_MSG_SCAN = 41, 	
		TYPE_MSG_COMPAS_XY,
		TYPE_MSG_MAX_MESSAGE_LONG = 50
} E_TypeMessage ;	
typedef 	uint16_t		T_TypeMessage ;	 



typedef struct {
	uint32_t						Header1 ;												// = HEADER1 
	uint32_t						Header2 ;												// = HEADER2 
	T_TypeMessage 			Type ;
	uint16_t						Total_Octet ;										// Nombre total d'octets à envoyer avec fractionnement 
	uint16_t						Position_Premier_Octet ;				// Position du premier octet envoyé Data[0] si fractionnement 
	uint16_t						Nombre_Doctets ;								// Nombre d'octets dans le message 
} T_Transport ;																				// taille = 16 octets 



typedef enum {
	REQUETE_ACK_NACK,											// pour les acquittement des messages 
	REQUETE_SERVICE = 1, 									// pour entretenir liaison entre robot et carte wifi
	REQUETE_RESET_WIFI = 2,										// pour demander le reset de la carte Wifi 
	REQUETE_MSG_REGLAGE,									// le host demande l'envoi de messages réglage
	REQUETE_ENREGISTRE_PARAM1,						// le host demande l'enregistrement en EEPROM du bloc de paramètres
	REQUETE_ENREGISTRE_PARAM2,						// le host demande l'enregistrement en EEPROM du bloc de paramètres
																				// à partir de 40 tous les messages de taille > TAILLE_BUF_TRANSPORT
	REQUETE_MSG_SCAN = 41,								// le host demande un scan 	
	REQUETE_MSG_COMPAS_XY
} E_Requete ;
typedef		uint16_t	T_Requete ; 

typedef enum {
	ACK_MSG  = 0x55AA,
	NACK_MSG = 0xAA55,
	SANS_ACK_NACK,
	ERREUR_SYNCHRO = 0xFF00 							
} E_Ack_Nack ;
typedef	uint16_t	T_Ack_Nack ; 

typedef enum {
	HOST_PRESENT  = 0,
	HOST_DEFAUT	  = 0x01020304,
	HOST_ABSENT   = 0x01121314,
} E_Presence_Host ;
typedef uint32_t	T_Presence_Host ; 

typedef struct {
	T_Requete				Requete ;
	T_Ack_Nack			Ack_Nack ;
	T_Presence_Host	Presence_Host ; 
} T_Service ; 	

typedef struct {
	T_Requete				Requete ;
	uint16_t				Arg1 ;
	uint32_t				Arg2 ; 
} T_Demande ; 

typedef struct {												
  T_Transport									Transport ;
		union	{
		T_Service										Service ;
		T_Demande										Demande ;
		} Type ;
	uint32_t										Crc ;		
} T_Buf_De_Service ;
#define		TAILLE_MESSAGE_DE_SERVICE 			( sizeof ( T_Buf_De_Service ) - TAILLE_CRC ) 
	

typedef struct {												
  T_Transport									Transport ;															// 16 octets
	T_Etat_Moteur								Etat_Moteur_Droit ;											// 20 
	T_Etat_Moteur								Etat_Moteur_Gauche ;										// 20 
	T_ConsigneManuelle					ConsigneManuelle ;											// 8 
	float 											VitesseMesure	[MAX_MOTEUR] ;						// 8 
	uint16_t										Phare_Luminosite ;											// 2
	int16_t 										Temperature_Exterieure ;								// 2
	uint16_t										V_Batterie ;														// 2 
	T_OuiNonErreur 							Etat_Sauvegarde;												// 1 
	T_Mode_Commande 						Mode_Commande ;													// 1
	T_ConsigneBoucle_Ouverte 		ConsigneBoucle_Ouverte_Moteur_Droit ; 	// 4 
	T_ConsigneBoucle_Ouverte 		ConsigneBoucle_Ouverte_Moteur_Gauche ; 	// 4 			
	uint8_t											Status_Lidar ;													// 1 
	T_MarcheArret								Status_Moteur_Lidar	;										// 1
	uint16_t										Erreur_Code_Lidar ;											// 2
	int16_t											Offset_Orientation_Lidar;								// 2	
	int16_t											NotUsed1 ;															// 2		
	float												Compas ;																// 4		
	float												ConsigneAngulaireNulle ;								// 4	 
	T_Compensation_Ligne_Droite Compensation_Ligne_Droite;							// 8 
	uint16_t 										Distance_Obstacle_Av ;									// 2 
	uint16_t 										Distance_Obstacle_Ar ;									// 2
	T_Suivi_De_Mur  						Suivi_De_Mur				;										// 48	// total 164 octets utiles
	uint32_t										Crc ;																							
} T_Buf_Reglage ;
#define		TAILLE_MESSAGE_REGLAGE					( sizeof ( T_Buf_Reglage ) - TAILLE_CRC ) 


typedef struct {
  T_Transport					Transport ;
	uint8_t							Data[ TAILLE_BUF_TRANSPORT ] ; 	// par blocs 
 	uint32_t						Crc ;
} T_Buf_Transport ;
#define	TAILLE_MESSAGE_TRANSPORT					(	sizeof ( T_Buf_Transport ) - TAILLE_CRC )



#define	TAILLE_MESSAGE										( sizeof ( T_Buf_Transport )  )	 // c'est normalement le plus grand message utile envoyé


typedef union {
		uint8_t 										Octet [ TAILLE_MESSAGE ] ;
	  uint32_t 										Word  [ TAILLE_MESSAGE / 4 ] ; 
		T_Transport									Transport ;
		T_Buf_Reglage	  						Reglage ;
		T_Buf_De_Service						De_Service ;
		T_Buf_Transport							Buf_Transport ;
} T_Buf_Wifi ;


typedef enum {
	DISPO,
	DEMANDE, 
	EN_COUR,
	DONE, 
	ABORT,
} E_Etat_Transport ; 
typedef uint16_t T_Etat_Transport ;


typedef struct {
	T_TypeMessage 			Type ;													
	uint16_t						Total_Octet ;										// Nombre total d'octets à envoyer 
	T_Etat_Transport		Etat_Transport ;
	union	{
			uint8_t								Byte[10 * TAILLE_BUF_TRANSPORT] ;
			T_Scan								Scan[MAX_SCAN_DANS_BUFFER] ;
			T_HMC5883L_CompasXY		CompasXY[MAX_COMPAS_XY_DANS_BUFFER] ; 
	} Data ;
} T_BufMessageLong ;




/* Cette macro est réservée au module de gestion des boutons (voir boutons.h) */
#define	LireEtatBoutons_Du_Wifi() 				(Recu_Du_Wifi->Boutons) 

/* Ces macros permettent de lire les données reçues du Wifi */
#define	LireConsigneVitesse_Du_Wifi()			(Recu_Du_Wifi->ConsigneVitesse) 
#define	LireConsigneRayonCourbe_Du_Wifi()	(Recu_Du_Wifi->ConsigneRayonCourbe) 


extern T_Presence_Host Host_Present ( void ) ;

extern HAL_StatusTypeDef WifiCom_Communication_Start (void) ;

extern T_Boolean WifiCom_Transmit_Transport (T_BufMessageLong *BufMessageLong) ;

extern T_Boolean  WifiCom_Gestion ( void )  ;

extern	void WifiCom_Receive_It (void);

extern	void WifiCom_Transmit_It (void);



#endif 

