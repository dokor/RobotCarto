//MPU6050 I2C library for ARM STM32F103xx Microcontrollers - Main source file
//Has bit, byte and buffer I2C R/W functions
// 23/05/2012 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2012-05-23 - initial release. Thanks to Jeff Rowberg <jeff@rowberg.net> for his AVR/Arduino
//                  based MPU6050 development which inspired me & taken as reference to develop this.
/* ============================================================================================
 MPU6050 device I2C library code for ARM STM32F103xx is placed under the MIT license
 Copyright (c) 2012 Harinadha Reddy Chintalapalli

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ================================================================================================
 */

/* Includes */
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "typegen.h"
#include "MPU6050.h"
#include "Moteurs.h"
#include "math.h"
#include "stdlib.h"
#include "Kalman.h"


/** @defgroup MPU6050_Library
 * @{
 */
 



#define PI						(float) 3.14159
#define RAD_TO_DEG	 	(float) ( (float) 180 / PI ) 
#define	PERIODE_ASSERV_POSITION_S			0.002f 
 
#define	MAX_ECHANTILLON_CALIBRATION  50 
#define NON_BLOCKING_MODE  0 
#define BLOCKING_MODE  		 1 

typedef enum {
	INITIAL						= 0, 
	STANDBY,
	CALIBRATION,
	ASSERVISSEMENT
} E_MPU6050_Mode_Fonctionnement ;
typedef uint8_t  	T_MPU6050_Mode_Fonctionnement; 

typedef enum {
	NOP							= 0, 
	WRITE_ONLY,
	WRITE_READ
} E_MPU6050_Commande ;
typedef uint8_t 	T_MPU6050_Commande ;


typedef	struct {
	volatile T_MPU6050_Mode_Fonctionnement		Mode_Fonctionnement; 
	volatile T_MPU6050_Commande 		Commande 	; 					// Si = NOP alors MPU6050 disponible 
	uint8_t						 							I2CTx_Buf[16];				// buffer émission d'un message I2C 
	uint8_t													Rx_NbChar ;						// Nombre d'octets à recevoir lors d'une commande WRITE_READ
	uint8_t						 							*I2CRx_Buf ;				  // Pointeur sur buffer de réception
	uint8_t													Nb_Echantillon_Calibration ;
} T_MPU6050_Gestion ;
T_MPU6050_Gestion MPU6050_Gestion ; 

uint8_t I2C_Buf_Rec[14] ; 

typedef struct {
	int16_t			X;
	int16_t			Y;
	int16_t			Z;
} T_Acceleration ; 

T_Acceleration Acc ;
T_Acceleration AccZero = { 0 , 0,  0 } ; 

typedef struct {
	int16_t			X;
	int16_t			Y;
	int16_t			Z;
} T_Gyroscope ; 

T_Gyroscope		 Gyro ;
T_Gyroscope		 GyroZero ;

float Acc_Angle  ; 
float Gyro_Angle ;
float Tanguage ;


/* Interruption émission I2C 
	 fonction appelée par call back HAL_I2C_MasterTxCpltCallback dans fichier stm32f4xx_it.c	*/		
void MPU6050_Emission_IT ( void )
{
	switch ( MPU6050_Gestion.Commande )
	{
		case WRITE_ONLY :			
			MPU6050_Gestion.Commande = NOP ;				/* Commande terminée */
		break ; 
	
		case WRITE_READ :
			// Lance la réception 
			if  ( HAL_I2C_Master_Receive_IT(&MPU6050_I2c, MPU6050_DEFAULT_ADDRESS, MPU6050_Gestion.I2CRx_Buf, MPU6050_Gestion.Rx_NbChar) != HAL_OK )
			{	// Erreur 
			}
		break ; 

		default :	/* accident */
		break ;		
  }		
}


void Mise_a_Jour_Acc_Gyro ( uint8_t *data )
{	/* voir à ne conserver que ce qu'il faut  Attention XYZ différe du à position du capteur */
	Acc.X =		(int16_t ) ( data[0]  << 8 | data[1])  ; 
	Acc.Y =   (int16_t ) ( data[2]  << 8 | data[3])  ; 	// Gravitée faire -
	Acc.Z =   (int16_t ) ( data[4]  << 8 | data[5])  ; 
		// Température non utilisée 		
	Gyro.X =  (int16_t ) ( data[8]  << 8 | data[9])  ; 	 // Tangage (Pitch)
	Gyro.Y =  (int16_t ) ( data[10]  << 8 | data[11])  ; // Lacet		(Yaw)
	Gyro.Z =  (int16_t ) ( data[12]  << 8 | data[13])  ; // Roulis (Roll)
}


static T_Boolean  TestMinMax(int16_t *array, uint8_t length, int16_t maxDifference) 
{ // Used to check that the robot is laying still while calibrating
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i < length; i++) {
    if (array[i] < min)
      min = array[i];
    else if (array[i] > max)
      max = array[i];
  }
  return(max - min < maxDifference);
}

 


/* Interruption réception I2C 
	 fonction appelée par call back HAL_I2C_MasterRxCpltCallback dans fichier stm32f4xx_it.c	*/	
void MPU6050_Reception_IT ( void )
{

	switch ( MPU6050_Gestion.Mode_Fonctionnement )
	{	
		case INITIAL  :
		case STANDBY	:
			MPU6050_Gestion.Commande = NOP ;				/* Commande terminée */
		break ; 
		
		case CALIBRATION :
		{	
			static int32_t gyro_somme ; 
			static int16_t gyro_echantillon[MAX_ECHANTILLON_CALIBRATION] ;
			
			MPU6050_Gestion.Commande = NOP ;				/* Commande terminée */			
			Mise_a_Jour_Acc_Gyro ( MPU6050_Gestion.I2CRx_Buf ) ; 
		  if ( MPU6050_Gestion.Nb_Echantillon_Calibration ==  0 )
			{ // démarrage de la calibration du Gyro */
				gyro_somme = 0 ; 
			}
			if ( MPU6050_Gestion.Nb_Echantillon_Calibration < MAX_ECHANTILLON_CALIBRATION )   													
			{	
				gyro_echantillon [MPU6050_Gestion.Nb_Echantillon_Calibration ++] = Gyro.X ; 
				gyro_somme += Gyro.X ;
			}
			else
			{
				if ( TestMinMax( gyro_echantillon, MAX_ECHANTILLON_CALIBRATION, 1000 ) == VRAI )
				{	// atan2 outputs the value of -Ï€ to Ï€ (radians) - see http://en.wikipedia.org/wiki/Atan2
					// We then convert it to 0 to 2Ï€ and then from radians to degrees
					Acc_Angle = (atan2((float)Acc.Y - AccZero.Y , (float)Acc.Z - AccZero.Z) + PI) * RAD_TO_DEG;
					Tanguage 			=  Acc_Angle ;
					Gyro_Angle 		=  Acc_Angle ; 
					Kalman_Set_Angle ( Acc_Angle ) ;  
					GyroZero.X  = gyro_somme / MAX_ECHANTILLON_CALIBRATION ; 				
					MPU6050_Gestion.Mode_Fonctionnement = STANDBY ; 			// terminé 
				}
				else
				{ // on recommence, le robot a bougé 
					MPU6050_Gestion.Nb_Echantillon_Calibration = 0 ; 
				}
			}	
		}
		break ; 
		
		case ASSERVISSEMENT :
		{
			float delta_t = PERIODE_ASSERV_POSITION_S ;
			
			Mise_a_Jour_Acc_Gyro ( MPU6050_Gestion.I2CRx_Buf ) ; 	
			Acc_Angle = (atan2((float)Acc.Y - AccZero.Y , (float)Acc.Z - AccZero.Z) + PI) * RAD_TO_DEG;
	
		  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
			if ((Acc_Angle < 90 && Tanguage > 270) || (Acc_Angle > 270 && Tanguage < 90)) 
			{
				Tanguage 			=  Acc_Angle ;
				Gyro_Angle 		=  Acc_Angle ; 
				Kalman_Set_Angle ( Acc_Angle ) ;  
			} 
			else 
			{
				float gyroRate = ((float)Gyro.X - GyroZero.X) / 131.0f; // Convert to deg/s
				Gyro_Angle += gyroRate * delta_t ; // Gyro angle is only used for debugging
				if ( Gyro_Angle < 0 || Gyro_Angle > 360)
						Gyro_Angle = Tanguage; // Reset the gyro angle when it has drifted too much
				Tanguage = Kalman_Get_Angle (Acc_Angle, gyroRate, delta_t); // Calculate the angle using a Kalman filter
			}		
	//		Asservissement_PositionMoteur  ( Tanguage, delta_t ) ; 
			MPU6050_Gestion.Commande = NOP ;				/* Commande terminée */
		}			
		break ; 
		
		default :	/* accident */			
		break ;		
  }		
}


 



T_OuiNonErreur	MPU6050_Write ( uint8_t regAddr, uint8_t *data, uint8_t length, uint8_t mode)
{
	if ( MPU6050_Gestion.Commande == NOP )
	{
		MPU6050_Gestion.Commande 		 = WRITE_ONLY ;
		MPU6050_Gestion.I2CTx_Buf[0] = regAddr ;
		for ( uint8_t idc = 0 ; idc < length ; ++idc ) 
		{
			MPU6050_Gestion.I2CTx_Buf[1+idc] = data[idc] ; 
		}
		
		if ( HAL_I2C_Master_Transmit_IT(&MPU6050_I2c, MPU6050_DEFAULT_ADDRESS, MPU6050_Gestion.I2CTx_Buf, 1+length) != HAL_OK )
		{ 
			MPU6050_Gestion.Commande = NOP ;
			return ( ERREUR );		
		}
		if ( mode == NON_BLOCKING_MODE ) 
			return ( OUI) ; 
		else
		{ /* Attendre la fin de la lecture */
			while ( MPU6050_Gestion.Commande  != NOP ) {} ;
			return ( OUI ) ; 
		} 
	}
	else 
		return ( NON) ;
}	


T_OuiNonErreur	MPU6050_Read ( uint8_t regAddr, uint8_t *data, uint8_t length, uint8_t mode )
{
	if ( MPU6050_Gestion.Commande == NOP )
	{
		MPU6050_Gestion.Commande 		 = WRITE_READ ;
		MPU6050_Gestion.I2CTx_Buf[0] = regAddr ;
		MPU6050_Gestion.I2CRx_Buf		 = data ; 
		MPU6050_Gestion.Rx_NbChar		 = length  ; 
		if ( HAL_I2C_Master_Transmit_IT(&MPU6050_I2c, MPU6050_DEFAULT_ADDRESS, MPU6050_Gestion.I2CTx_Buf, 1) != HAL_OK )
		{ 
			MPU6050_Gestion.Commande = NOP ;
			return ( ERREUR );		
		}
		if ( mode == NON_BLOCKING_MODE ) 
			return ( OUI) ; 
		else
		{ /* Attendre la fin de la lecture */
			while ( MPU6050_Gestion.Commande  != NOP ) {} ;
			return ( OUI ) ; 
		}
	}
	else 
		return ( NON) ;
}	

void  MPU6050_Calibration_Start ( void ) 
{
		MPU6050_Gestion.Nb_Echantillon_Calibration = 0  ; 
		MPU6050_Gestion.Mode_Fonctionnement = CALIBRATION ; 
	  while ( MPU6050_Gestion.Mode_Fonctionnement != STANDBY ) {} ; 
}

void  MPU6050_Asservissement_Start ( void ) 
{
		MPU6050_Gestion.Mode_Fonctionnement = ASSERVISSEMENT ; 
}


void MPU6050_Event_It 			( void )
{
	if ( MPU6050_Read ( MPU6050_RA_ACCEL_YOUT_H, I2C_Buf_Rec , 14, NON_BLOCKING_MODE ) != OUI )
	{
		
	}
}

T_OuiNonErreur MPU6050_Initialize ( void ) 
{
	uint8_t data[10] ; 
	MPU6050_Gestion.Mode_Fonctionnement = INITIAL ; 
	Kalman_Initialise () ; 
	if ( MPU6050_Read ( MPU6050_RA_WHO_AM_I, data , 1, BLOCKING_MODE ) == OUI )
	{
		if ( data[0] == MPU6050_ADDRESS_AD0_LOW )
		{
			data[0] = (1 << MPU6050_PWR1_DEVICE_RESET_BIT) ; 									// Reset device, this resets all internal registers to their default values
			MPU6050_Write ( MPU6050_RA_PWR_MGMT_1, data, 1, BLOCKING_MODE ) ;
			do 
			{
				 MPU6050_Read ( MPU6050_RA_PWR_MGMT_1, data , 1, BLOCKING_MODE ) ;
			}
			while ( data[0] & (1 << MPU6050_PWR1_DEVICE_RESET_BIT) ) ; // attendre que le reset se fasse 
			HAL_Delay ( 200 ) ;
			data[0] = (1 << 3) | (1 << 0) ; 							// Disable sleep mode, disable temperature sensor and use PLL as clock reference
			MPU6050_Write ( MPU6050_RA_PWR_MGMT_1, data, 1, BLOCKING_MODE ) ;
	//		data[0] = 0; // Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
	//		data[1] = 0x03; // Disable FSYNC and set 41 Hz Gyro filtering, 1 KHz sampling
	    data[0] = 1; // Set the sample rate to 500Hz - 1kHz/(1+1) = 500Hz
      data[1] = 0x03; // Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling	
	//		data[0] = 15; // Set the sample rate to 500Hz - 8kHz/(15+1) = 500Hz
	//		data[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling  			
			data[2] = 0X00; // Set Gyro Full Scale Range to +-250deg/s
			data[3] = 0x00; // Set Accelerometer Full Scale Range to +-2g
	//		data[4] = 0x03; // 41 Hz Acc filtering
			MPU6050_Write ( MPU6050_RA_SMPLRT_DIV, data, 4, BLOCKING_MODE ) ;
 		
			HAL_Delay ( 200 ) ;
	    data[0] = (1 << 5) | (1 << 4) | (1 << 1);    // Enable LATCH_INT_EN, INT_ANYRD_2CLEAR and BYPASS_EN
                                                   // When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
                                                   // When this bit is equal to 1, interrupt status is cleared if any read operation is performed
                                                   // When asserted, the I2C_MASTER interface pins (ES_CL and ES_DA) will go into 'bypass mode' when the I2C master interface is disabled
 //     data[1] = (1 << 0);                          // Enable RAW_RDY_EN - When set to 1, Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin
			MPU6050_Write ( MPU6050_RA_INT_PIN_CFG , data, 2, BLOCKING_MODE ) ;   

			MPU6050_Gestion.Mode_Fonctionnement = STANDBY ; 
		
			
			return ( OUI ); 
		}
		else
			return ( ERREUR ); 			
	}
	else
	  return ( ERREUR ); 
}	

/**
 * @}
 *//* end of group MPU6050_Library */
