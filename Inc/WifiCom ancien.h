
#ifndef 	H_WIFICOM
#define		H_WIFICOM 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"

extern	uint16_t					EtatDesBoutonsWifi ; 

#define	LireEtatBoutons_Du_Wifi() (EtatDesBoutonsWifi)  


extern T_Boolean 	WifiCom_Si_Nouveau_Message ( void );


extern uint8_t WifiCom_Lire_Taille_Nouveau_Message ( void );


extern uint8_t *  WifiCom_Lire_Nouveau_Message ( void );



extern HAL_StatusTypeDef WifiCom_Transmit(uint8_t *pData, uint16_t Size);

extern HAL_StatusTypeDef WifiCom_Receive_Start ( uint8_t *pData) ;


extern	void WifiCom_UART_IRQHandler(UART_HandleTypeDef *huart);





#endif 

