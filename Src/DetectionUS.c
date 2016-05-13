/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"

#include "Tempo.h"
//#include "DetectionUS.h"
#include "US_Capteur.h"
#include "typegen.h"
#include "Moteur.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "typegen.h"
#include "stdio.h"
#include "moteurs.h"
#include "US_Capteur.h"
#include "Tempo.h"
#include "LCD16x2.h"
#include "Boutons.h"
#include "Temperature.h"
#include "Wificom.h"
#include "HMC5883L.h"
#include "Phares.h"
#include "Sauve_Restaure.h"
#include "Rplidar.h"
#include "MPU6050.h"
#include "HMC5883L.h"

uint16_t 	Distance_Obstacle_Av ;
uint16_t 	Distance_Obstacle_Ar ;
 
void VerifObstacleUS()
{
	if(Distance_Obstacle_Av < 10)
	{
		while(Distance_Obstacle_Av < 10 )
		{
			Robot_Commande(-100, 0); //Recule
		}
	}
	else if(Distance_Obstacle_Av > 10 && Distance_Obstacle_Av < 15)
	{
	  Robot_Commande(0, 0); //Stop
		//Scan
			Servo_Position(US_AVANT, 0); //Scan tout a gauche
		  Servo_Position(US_AVANT, 48); //Scan millieu
			Servo_Position(US_AVANT, 60); //Scan tout a droite
		//Décisions
			//
			//
	}		
}
