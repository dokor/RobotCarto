
#ifndef 	H_RPLIDAR
#define		H_RPLIDAR

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"


#define Rplidar_Uart			huart2   // il faudra également modifier l'affection dans stm32f4xx_it.c


	/* Les Commandes du message de requete */
#define		COMMAND_STOP					0x25
#define		COMMAND_RESET					0x40
#define		COMMAND_SCAN					0x20			// Avec Vitesse de transmission ci-dessous
#define		COMMAND_FORCE_SCAN		0x21			// Ne pas utiliser 
#define		COMMAND_GET_INFO			0x50
#define		COMMAND_GET_HEALTH		0x52

		/* Vitesse de transmission des scans 360 */
#define		RPLIDAR_HIGH_SPEED		0x01
#define		RPLIDAR_MEDIUM_SPEED	0x02
#define		RPLIDAR_LOW_SPEED			0x03


typedef  struct {
	__packed  uint16_t		Quality ;
	__packed 	uint16_t		Angle ;
	__packed  uint16_t		Distance ;
} T_Scan ;
#define		MAX_SCAN_DANS_BUFFER	333
	

		/* Définition d'un point utilisé par Guidage.c et les recherches dans Rplidar.c */
typedef struct {
	float		Angle ;
	float		Distance ;
} T_Angle_Distance ;



	/* Les codes du Status_Moteur_Lidar */
#define		RPLIDAR_NO_ERROR 									0x00
#define		RPLIDAR_WARNING										0x01
#define		RPLIDAR_ERROR											0x02
#define		RPLIDAR_TIMEOUT										0xFA					// erreurs détectée localement
#define		RPLIDAR_TIMEOUT_REQUEST_RESPONSE	0xFB
#define		RPLIDAR_DEFAUT_SYNCHRO_SCAN				0xFF 

extern	T_MarcheArret		Status_Moteur_Lidar ;
extern	uint8_t					Status_Rplidar ;
extern	uint16_t				ErrorCode_Rplidar ;

extern  int16_t					Offset_Orientation_Lidar ;




extern T_Boolean Rplidar_New_Scan ( void) ;
extern void Rplidar_Get_Distance ( T_Angle_Distance *AngleDistance ) ;


extern void Rplidar_Gestion (void) ;
extern void Rplidar_Start (void) ;
extern void Lazer_Scan_Request (uint8_t	Command, uint8_t Transmission_Rate ) ;


extern void Rplidar_Receive_It  (void) ; 
extern void Rplidar_Transmit_It (void) ;
extern void Rplidar_Error_It 		(void) ;


#endif 

