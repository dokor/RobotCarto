/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "i2c.h"
#include "typegen.h"
#include "Tempo.h"
#include "LCD16x2.h"



/* Renomme le bus I2C utilis� */
#define	LCD16x2_I2c				hi2c2
	/* en cas de modification, modifier �galement le fichier stm32f4xx_it.c */


#define	TEMPO_PREMIER_CARACTERE			 2000 // �S C'est malin quand on ne sait pas programmer 
#define TEMPO_INTER_CARACTERE 			 1000 // * (Positionx) C'est vraiment malin quand on ne sait pas programmer 
																				  // soit une tempo de 2000 + 15*1000= 17 ms pour le dernier caract�re de la ligne
																				
#define	TEMPO_APRES_CLEAR_SCREEN		50000	// 50ms 

/* Les commandes � envoyer � la carte LCD */
typedef enum {
	NOP							= 0, 
	GET_BUTTON			= 0x05, 
	GET_ID					= 0x20, 
	GET_FIRMWARE		= 0x21,
	CLEAR_SCREEN		= 0x60, 
	WRITE_DATA			= 0x61, 
	SET_BACK_LIGHT	= 0x62
} E_LCD16x2_Commande ;

#define	T_LCD16x2_Commande	uint8_t 

typedef	struct {
	volatile T_LCD16x2_Commande 		Commande 	; // Si = NOP alors LCD disponible 
	volatile T_LCD16x2_Commande 		CommandePrecedente ; 	
	uint8_t						 		I2CTx_Buf[4];		// buffer d'un message I2C au plus 4 octets
	uint8_t								Tx_Buf[16];			// 16 caract�res max par ligne 
	uint8_t								Tx_IdcBuf	;			// indice d'acc�s au Tx_Buf[]
	uint8_t								Tx_NbChar ;			// Nombre d'octets � envoyer de Tx_Buf en commencant par Pox_X et PosY
	uint8_t						 		I2CRx_Buf ;			// ici on re�oit au plus 1 octet	
	uint8_t								Identif		;			// 	
	uint8_t								Firmware	;			// 	
	T_Boolean							Request_GetBouton ;
} T_LCD16x2_Gestion ;
T_LCD16x2_Gestion LCD16x2_Gestion ; 

uint8_t			EtatDesBoutonsLCD ; 



/* Fonction � appeler imp�rativementune seule fois au d�but de l'application */
void LCD16x2_Initialise ( void ) 
{
	LCD16x2_Gestion.Commande = NOP ; 
	Arme_Tempo_LCD16x2 (0xFFFFFFFE) ;		// arm�e en permanence 
}


/* V�rifier si la pr�cdente commande est termin�e */
T_Boolean LCD16x2_Disponible ( void )
{
	return ( LCD16x2_Gestion.Commande == NOP ) ;
}


/* Si disponible, efface l'�cran */
T_OuiNonErreur	LCD16x2_ClearScreen ( void ) 
{
	if ( LCD16x2_Gestion.Commande == NOP )
	{
		LCD16x2_Gestion.Commande 		 = CLEAR_SCREEN ;
		LCD16x2_Gestion.I2CTx_Buf[0] = LCD16x2_Gestion.Commande ;
		
		if ( HAL_I2C_Master_Transmit_IT(&LCD16x2_I2c, 0x60, LCD16x2_Gestion.I2CTx_Buf, 1) != HAL_OK )
		{
			LCD16x2_Gestion.Commande = NOP ;
			return ( ERREUR );		
		}
		return (OUI) ; 
	}
	else 
		return (NON) ;
}	

/* Si disponible, r�gle  niveau de luminosit� */
T_OuiNonErreur	LCD16x2_SetBackLight ( uint8_t Niveau ) 
{
	if ( LCD16x2_Gestion.Commande == NOP )
	{
		LCD16x2_Gestion.Commande 		 = SET_BACK_LIGHT ;
		LCD16x2_Gestion.I2CTx_Buf[0] = LCD16x2_Gestion.Commande ;
		LCD16x2_Gestion.I2CTx_Buf[1] = Niveau ;
		if ( HAL_I2C_Master_Transmit_IT(&LCD16x2_I2c, 0x60, LCD16x2_Gestion.I2CTx_Buf, 2) != HAL_OK )
		{
			LCD16x2_Gestion.Commande = NOP ;
			return ( ERREUR );		
		}
		return ( OUI) ; 
	}
	else 
		return ( NON) ;
}	


/* Si disponible, lire Identification de la carte  */
T_OuiNonErreur	LCD16x2_GetId ( void ) 
{
	if ( LCD16x2_Gestion.Commande == NOP )
	{
		LCD16x2_Gestion.Commande 		 = GET_ID ;
		LCD16x2_Gestion.I2CTx_Buf[0] = LCD16x2_Gestion.Commande ;
		
		if ( HAL_I2C_Master_Transmit_IT(&LCD16x2_I2c, 0x60, LCD16x2_Gestion.I2CTx_Buf, 1) != HAL_OK )
		{
			LCD16x2_Gestion.Commande = NOP ;
			return ( ERREUR );		
		}
		return ( OUI) ; 
	}
	else 
		return ( NON) ;
}	

/* Si disponible, lire Identification du logiciel � bord de la carte  */
T_OuiNonErreur	LCD16x2_GetFirmware ( void ) 
{
	if ( LCD16x2_Gestion.Commande == NOP )
	{
		LCD16x2_Gestion.Commande 		 = GET_FIRMWARE ;
		LCD16x2_Gestion.I2CTx_Buf[0] = LCD16x2_Gestion.Commande ;
		
		if ( HAL_I2C_Master_Transmit_IT(&LCD16x2_I2c, 0x60, LCD16x2_Gestion.I2CTx_Buf, 1) != HAL_OK )
		{ 
			LCD16x2_Gestion.Commande = NOP ;
			return ( ERREUR );		
		}
		return ( OUI) ; 
	}
	else 
		return ( NON) ;
}	


/* Si disponible affiche du texte de Buf � l'�cran */
T_OuiNonErreur	LCD16x2_WriteData (char *Buf, uint8_t Taille, uint8_t PosX, uint8_t PosY )
{
	if ( LCD16x2_Gestion.Commande == NOP )
	{
		LCD16x2_Gestion.Commande 		 = WRITE_DATA ;
		LCD16x2_Gestion.Tx_IdcBuf		 = 0 ; 
		LCD16x2_Gestion.Tx_NbChar		 = Taille ;
		/* Copie des caract�res � envoyer */
		while ( Taille-- )
		{
			LCD16x2_Gestion.Tx_Buf[Taille] = Buf[Taille] ;
		}
		/* envoi du premier caract�re */
		LCD16x2_Gestion.I2CTx_Buf[0] = LCD16x2_Gestion.Commande ;
		LCD16x2_Gestion.I2CTx_Buf[1] = PosY ;		// PosY initial 
		LCD16x2_Gestion.I2CTx_Buf[2] = PosX ;		// PosX initial 
		LCD16x2_Gestion.I2CTx_Buf[3] = LCD16x2_Gestion.Tx_Buf[0]		; 
		if ( HAL_I2C_Master_Transmit_IT(&LCD16x2_I2c, 0x60, LCD16x2_Gestion.I2CTx_Buf, 4) != HAL_OK )
		{
			LCD16x2_Gestion.Commande = NOP ;
			return ( ERREUR );		
		}
		return ( OUI) ; 		
	}
	else 
		return ( NON) ;
}	



/* 	Interrogation de la carte LCD pour obtenir l'�tat des boutons poussoirs
		l'interrogation est lanc�e imm�diatement si disponible sinon marqu�e pour 
		�tre effectu�e � la fin de la commande en cours. 
		Fonction � appeler p�riodiquement dans la boucle principale du projet et 
		l'interrogation est autoris�e toutes les 100ms 									*/
T_OuiNonErreur	LCD16x2_GetButton ( void ) 
{
	static uint32_t	date_interro_bouton = 0 ;
		
	if ( date_interro_bouton <= HAL_GetTick () )
	{	/* tentative d'interrogation des Boutons toutes les 100 ms */		
		if ( LCD16x2_Gestion.Commande == NOP )
		{
			LCD16x2_Gestion.Commande 		 = GET_BUTTON ;
			LCD16x2_Gestion.I2CTx_Buf[0] = LCD16x2_Gestion.Commande ;
		
			if ( HAL_I2C_Master_Transmit_IT(&LCD16x2_I2c, 0x60, LCD16x2_Gestion.I2CTx_Buf, 1) != HAL_OK )
			{
				LCD16x2_Gestion.Commande = NOP ;
				return ( ERREUR );		
			}				
			LCD16x2_Gestion.Request_GetBouton = FAUX ;	
			date_interro_bouton = HAL_GetTick () + 100 ;			
			return ( OUI) ; 
		}
		else 
		{	/* il faut le diff�rer, sera fait � l'issue de la commande en cours */
			LCD16x2_Gestion.Request_GetBouton = VRAI ; 
			return ( NON) ;		
		}
	}
	return ( NON) ;
}


/* Interruption sur temporisation utilis�e pour le d�lai inter envoi 
	 fonction appel�e par call back compare du timer 5	TIM_CHANNEL_1													
	 dans fichier stm32f4xx_it.c														 */	
void LCD16x2_Gestion_Tempo_IT ( void ) 
{
	switch ( LCD16x2_Gestion.Commande )
	{
		case NOP :
		case GET_BUTTON :
		case GET_ID :
		case GET_FIRMWARE :
		case CLEAR_SCREEN :
		case SET_BACK_LIGHT :
			LCD16x2_Gestion.CommandePrecedente = LCD16x2_Gestion.Commande ; // pour debug 
			LCD16x2_Gestion.Commande = NOP ; 
		break ; 

		case WRITE_DATA :
			if ( --LCD16x2_Gestion.Tx_NbChar )
			{
				++ LCD16x2_Gestion.Tx_IdcBuf ;
				++ LCD16x2_Gestion.I2CTx_Buf[2] ; // ++ PosX Courant 
				LCD16x2_Gestion.I2CTx_Buf[3] = LCD16x2_Gestion.Tx_Buf[LCD16x2_Gestion.Tx_IdcBuf] ;
				if ( HAL_I2C_Master_Transmit_IT(&LCD16x2_I2c, 0x60, LCD16x2_Gestion.I2CTx_Buf, 4) != HAL_OK )
				{
					/* Annule emission en cours  */
					LCD16x2_Gestion.Tx_NbChar = 1 ;
					Relance_Tempo_LCD16x2 (TEMPO_PREMIER_CARACTERE) ;	  // a v�rifier !!!!!!! 
				}
			}
			else
			{		
				LCD16x2_Gestion.CommandePrecedente = LCD16x2_Gestion.Commande ; // pour debug 				
				LCD16x2_Gestion.Commande = NOP ; 			
			}
		break ;
						
		default :	/* accident */
			LCD16x2_Gestion.CommandePrecedente = LCD16x2_Gestion.Commande ; // pour debug 			
			LCD16x2_Gestion.Commande = NOP ; 				
		break ;		
  }	
	
	if ( ( LCD16x2_Gestion.Commande == NOP ) && (LCD16x2_Gestion.Request_GetBouton == VRAI))
	{
		switch ( LCD16x2_GetButton () )
		{
			case OUI :
				LCD16x2_Gestion.Request_GetBouton = FAUX ;
			break ;
			case NON :
			break ; 
			case ERREUR :
			break ; 
		}			
	}
}


/* Interruption �mission I2C 
	 fonction appel�e par call back HAL_I2C_MasterTxCpltCallback dans fichier stm32f4xx_it.c	*/		
void LCD16x2_Emission_IT ( void )
{
	switch ( LCD16x2_Gestion.Commande )
	{
		case CLEAR_SCREEN :
			// il faut armer une tempo avant de rendre le LCD dispo */
			Relance_Tempo_LCD16x2 (TEMPO_APRES_CLEAR_SCREEN) ;
		break ; 
	
		case GET_BUTTON :
		case GET_ID	:
		case GET_FIRMWARE :
			// Lance la r�ception d'un octet
			if  ( HAL_I2C_Master_Receive_IT(&LCD16x2_I2c, 0x60, &LCD16x2_Gestion.I2CRx_Buf, 1) != HAL_OK )
			{	// Erreur 
				// il faut armer une tempo avant de rendre le LCD dispo */			
				Relance_Tempo_LCD16x2 (TEMPO_PREMIER_CARACTERE) ;		
			}
		break ; 
			
		case SET_BACK_LIGHT :
			// il faut armer une tempo avant de rendre le LCD dispo */			
			Relance_Tempo_LCD16x2 (TEMPO_PREMIER_CARACTERE) ;
		break ;	
		
		case WRITE_DATA :
			// lance tempo en fonction de PosX courant avant d'envoyer caract�re suivant ou pour rendre Dispo si c'�tait le dernier cararact�re
			Relance_Tempo_LCD16x2 (TEMPO_PREMIER_CARACTERE + ( TEMPO_INTER_CARACTERE * LCD16x2_Gestion.I2CTx_Buf[2])) ;				
		break ;
			

		default :	/* accident */
			// le default d'ici ira dans le default de LCD16x2_Gestion_Tempo_IT */				
			Relance_Tempo_LCD16x2 (TEMPO_PREMIER_CARACTERE) ;	
		break ;		
  }		
}


/* Interruption r�ception I2C 
	 fonction appel�e par call back HAL_I2C_MasterRxCpltCallback dans fichier stm32f4xx_it.c	*/	
void LCD16x2_Reception_IT ( void )
{
	switch ( LCD16x2_Gestion.Commande )
	{	
		case GET_BUTTON :
			EtatDesBoutonsLCD				 = ~LCD16x2_Gestion.I2CRx_Buf ; // compl�ment pour obtenir 1 = appuy� 
		break ; 
		case GET_ID	:
			LCD16x2_Gestion.Identif  = LCD16x2_Gestion.I2CRx_Buf ;   			
		break ; 
		case GET_FIRMWARE :
			LCD16x2_Gestion.Firmware  = LCD16x2_Gestion.I2CRx_Buf ; 	
		break ; 			
		default :	/* accident */			
		break ;		
  }		
	// il faut armer une tempo avant de rendre le LCD dispo */			
	Relance_Tempo_LCD16x2 (TEMPO_PREMIER_CARACTERE) ;
}



