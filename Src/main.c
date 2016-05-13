/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
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
#include "Guidage.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Le buffer de lecture des entrées analogiques de ADC3 */
extern uint32_t	Buf_Adc[ 5 ] ;				/* 5 canaux à convertir */ 


T_Boolean	Defaut_Sauvegarde ;



int16_t 	Distance_Obstacle_Avant ; 
int16_t 	Distance_Obstacle_Arriere ; 
int16_t   Position_Servo = 45 ; 




		
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */

	/* Restauration des paramètres */
	Initialise_Sauvegarde ();
	
#ifdef RPLIDAR		
	Rplidar_Start() ;
#endif		

#ifdef SERVO
	Servos_Start_PWM ();	
#endif
	Phares_Start () ; 
	Moteurs_Start () ; 
	
#ifdef WIFI
  WifiCom_Communication_Start ()  ; 
#endif

	/* démarre CAN mesure température , courant moteur 1 et 2  et tension batterie (triggé par out compare timer 5 soit 
		 en même temps que l'asservissement vitesse					*/
	HAL_ADC_Start_DMA (&hadc3, Buf_Adc, 5 ); 

#ifdef	GYRO
	if ( MPU6050_Initialize () == OUI )
	{		
		HAL_Delay ( 1000 ) ;
		MPU6050_Calibration_Start () ; 
		MPU6050_Asservissement_Start (); 
	} 
#endif
	
#ifdef COMPAS
	HMC5883L_Start () ;
#endif	

#ifdef US_CAPTEUR
// Us_SelectInterrogation ( US_GAUCHE, 	MARCHE );
	Us_SelectInterrogation ( US_ARRIERE, 	MARCHE );
	Us_SelectInterrogation ( US_AVANT, 		MARCHE );
//	Us_SelectInterrogation ( US_DROIT, 		MARCHE );
	Us_MarcheArretGeneral  ( MARCHE ) ;
#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if ((TestCollisionAvant() == VRAI) && (ConsigneManuelle.Vitesse > CONSIGNE_NULLE_VITESSE_LINAIRE))
		{
			ConsigneManuelle.Vitesse = 0;
		}
		if ((TestCollisionArriere() == VRAI) && (ConsigneManuelle.Vitesse <  - CONSIGNE_NULLE_VITESSE_LINAIRE))
		{
			ConsigneManuelle.Vitesse = 0;
		} 
			
		switch ( Mode_Commande )
		{
			case 	PILOTAGE_BOUCLE_OUVERTE :
					/* Les valeurs sont directement prise en compte par it ASServissement_VitesseMoteur */
			break ;
		
			case 	PILOTAGE_MANUEL_ASSISTE :
				Robot_Commande ( ConsigneManuelle.Vitesse, Guidage_Suivi_Mur () ) ; 			
			break ; 
			
			case 	PILOTAGE_MANUEL :		
				Robot_Commande ( ConsigneManuelle.Vitesse, ConsigneManuelle.ConsigneAngulaire ) ;	
			break ;
			
			case 	CALIBRATION_MAGNETOMETRE :			
#ifdef COMPAS					
				if ( HMC5883L_Gestion(HMC5883L_CALIBRATION) == HMC5883L_END_CALIBRATION )
#endif
					Mode_Commande = PILOTAGE_MANUEL ; 
			break ;
			case 	PILOTAGE_AUTONOME :	
			break ;
		}

				
		
#ifdef WIFI
		if (  WifiCom_Gestion() == FAUX )
		{
			if ( Mode_Commande == PILOTAGE_MANUEL )
				Robot_Commande ( 0.0, 0.0 ) ; 
		}
#endif				
		
#ifdef COMPAS	
		if ( Mode_Commande != CALIBRATION_MAGNETOMETRE )
			HMC5883L_Gestion(HMC5883L_MESURE) ;	
#endif		
	
#ifdef RPLIDAR		
		Rplidar_Gestion () ; 
//		Guidage_Suivi_Mur ( 90 ) ; 
	/*	if (  Rplidar_HowMany_Scan () )
		{
			++ recu ; 
		}		*/
		
#endif	
				

#ifdef SERVO
		Servo_Position ( SERVO1, Position_Servo );
#endif
		

		/* mesure de vitesse pour info pilote */
		VitesseMesure[MOTEUR_DROIT]  = Moteur_Vitesse (MOTEUR_DROIT) ;	  
		VitesseMesure[MOTEUR_GAUCHE] = Moteur_Vitesse (MOTEUR_GAUCHE) ;	


		Gestion_Sauvegarde() ;

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 224;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
