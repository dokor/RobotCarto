/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "typegen.h"
#include "moteurs.h"
#include "tempo.h"
#include "US_Capteur.h"
#include "LCD16x2.h"
#include "Temperature.h"
#include "Batterie.h"
#include "Phares.h"
#include "WifiCom.h"
#include "Rplidar.h"
#include "HMC5883L.h"
#include "MPU6050.h"


/* Le buffer de lecture des entrées analogiques de ADC3 */
extern uint32_t	Buf_Adc[ 5 ] ;				/* 5 canaux à convertir */ 

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI Line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles DMA1 Stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
* @brief This function handles TIM1 Capture Compare interrupt.
*/
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
* @brief This function handles I2C1 error interrupt.
*/
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
* @brief This function handles I2C2 event interrupt.
*/
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
* @brief This function handles I2C2 error interrupt.
*/
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */

  /* USER CODE END I2C2_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
* @brief This function handles DMA2 Stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA2 Stream1 global interrupt.
*/
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
* @brief This function handles DMA2 Stream6 global interrupt.
*/
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
* @brief This function handles USART6 global interrupt.
*/
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* Callback des timers en débordement */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	switch ( (long) htim->Instance )
	{	/* détermine le timer pour ce Callback */
		
		default :
		break ;		
	}									
}

/* Callback des entrées de capture des timers  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  switch ( (long) htim->Instance )
	{	/* détermine le timer pour ce Callback */
		case (long) TIM1 :
		{	/* Timer employé pour la mesure Ultrasons */
			/******************************************/
			switch ( htim->Channel )
			{ /* détermine le canal du timer pour ce Callback */
				case HAL_TIM_ACTIVE_CHANNEL_1 :
				{
		#ifdef US_CAPTEUR						
					Us_GestionMesure_IT ( htim ) ;	
		#endif
				}
				break ;							
				default :
				break ;
			}
		}
		break ;
	
		default :
		break ;		
	}
}


/* Callback des comparateurs des timers   */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  switch ( (long) htim->Instance )
	{	/* détermine le timer pour ce Callback */
		case (long) TIM2 :
		{	/* timer 32 bits employé pour gérer des temporisations prioritaires non bloquantes jusqu'à environ 71 mn  1µs par unité */
			/************************************************************************************************************************/
			switch ( htim->Channel )			
			{ /* détermine le canal du timer pour ce Callback */
				case HAL_TIM_ACTIVE_CHANNEL_1 :	/* strictement réservé à gestion US */
				{
	#ifdef US_CAPTEUR				
					Us_GestionTrigger_IT (htim) ;
	#endif
				}
				break ;
				case HAL_TIM_ACTIVE_CHANNEL_2 :
				{		
				} 
				break ;		
				case HAL_TIM_ACTIVE_CHANNEL_3 :
				{
				}
				break ;	
				case HAL_TIM_ACTIVE_CHANNEL_4 :
				{
				}
				break ;					
				default :
				break ;
			}
		}
		break ;

		case (long) TIM5 :
		{	/* timer 32 bits employé pour gérer des temporisations non prioritaires non bloquantes jusqu'à environ 71 mn  1µs par unité */
			/****************************************************************************************************************************/
			switch ( htim->Channel )			
			{ /* détermine le canal du timer pour ce Callback */
				case HAL_TIM_ACTIVE_CHANNEL_1 :	/* strictement réservé à LCD16x2 */
				{
#ifdef	LCD16X2 						
					LCD16x2_Gestion_Tempo_IT () ;
#endif
				}
				break ;
				case HAL_TIM_ACTIVE_CHANNEL_2 :
				{	
			//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,  GPIO_PIN_SET );
					{ /* cette partie de code est spécifique au channel employé 
							 elle a pour objet de remettre à 0 immédiatement la sortie de comparaison 
						   et réinitialisation en comparaison 
						   c'est le trigger des timers 3 et 4 channel 3 */
						uint32_t tmpccmrx ; 
						uint32_t tmpccmrx_org ; 
					
						/* Get the TIMx CCMR1 register value */
						tmpccmrx_org  = tmpccmrx = htim->Instance->CCMR1;
    
						/* Reset the Output Compare mode and Capture/Compare selection Bits */
						tmpccmrx &= ~TIM_CCMR1_OC2M;
						/* Select the Output Compare Mode */
						tmpccmrx |= (TIM_OCMODE_FORCED_INACTIVE << 8);
					  /* Write to TIMx CCMR1 */
						htim->Instance->CCMR1 = tmpccmrx;				/* sortie forcée à 0 */
						
						/* reconfiguration en comparateur */
						htim->Instance->CCMR1 = tmpccmrx_org;
					}
					Relance_Tempo_ASS_Vitesse(PERIODE_ASS_VITESSE) ; 
					ASServissement_VitesseMoteur ();
	//						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,  GPIO_PIN_RESET );	
				} 
				break ;			
				case HAL_TIM_ACTIVE_CHANNEL_3 :
				{
				}
				break ;	
				case HAL_TIM_ACTIVE_CHANNEL_4 :
				{
				}
				break ;					
				default :
				break ;
			}
		}
		break ;		
		default :
		break ;
		
	}					
} 

/* Callback Reception I2C    */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	switch ( (long) hi2c->Instance )
	{
		case (long) I2C1 :
		{
#ifdef COMPAS
			HMC5883L_Reception_IT () ; 
#endif
#ifdef	GYRO			
			MPU6050_Reception_IT () ;	
#endif			
		}
		break ;
		case (long) I2C2 :
		{
#ifdef	LCD16X2 
			LCD16x2_Reception_IT () ;	
#endif			
		}
		break ;		
		default :
		break ;
	}
}	

/* Callback Emission I2C    */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	switch ( (long) hi2c->Instance )
	{
		case (long) I2C1 :
		{		
#ifdef	COMPAS
			HMC5883L_Emission_IT () ;
#endif
#ifdef GYRO
			MPU6050_Emission_IT () ;	
#endif			
		}
		break ;
		case (long) I2C2 :
		{
#ifdef	LCD16X2 			
			LCD16x2_Emission_IT () ;
#endif			
		}
		break ;		
		default :
		break ;
	}
}	


/* Callback Erreur I2C    */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	switch ( (long) hi2c->Instance )
	{
		case (long) I2C1 :
		{
#ifdef	COMPAS
#endif
#ifdef GYRO
#endif				
		}
		break ;		
		case (long) I2C2 :
		{
#ifdef	LCD16X2 		
#endif			
		}
		break ;				
		default :
		break ;
	}	
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch ( GPIO_Pin )
	{
		case GPIO_PIN_4 :
#ifdef	COMPAS
			HMC5883L_Event_IT () ;
#endif
#ifdef GYRO			
			MPU6050_Event_It () ; 
#endif
		break ;
		default :
		break ;
	}
}


/* Callback Fin de conversion Analogique Numérique    */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	switch ( (long) hadc->Instance )
	{
		case (long) ADC3 :			/* entrée mesure de température, courant moteur et tension batterie */
		{	
			Mesure_Temperature_IT (Buf_Adc[ 2 ], Buf_Adc[ 3 ]); 
			Mesure_Batterie_IT (Buf_Adc[ 0 ]) ; 
			Phares_Commande () ; 		/* Il est ainsi rafraichi périodiquement */
		}
		break ;
		default :
		break ;		
	}
}

/* Callback de la communication série via DMA */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	switch ( (long) huart->Instance )
	{
		case (long) USART2 : // avec Lidar
	#ifdef RPLIDAR		
//			Rplidar_Receive_It () ; 
	#endif	
		break ; 		
		
		case (long) USART6 : // avec carte WIFI 
		{
	#ifdef WIFI	
	#endif					
		}
		break ; 
		default :
		break ;		
			
	}		
}
  
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch ( (long) huart->Instance )
	{
		case (long) USART2 : // avec Lidar
	#ifdef RPLIDAR		
			Rplidar_Receive_It () ; 
	#endif	
		break ; 
		
		case (long) USART6 : // avec carte WIFI 
	#ifdef WIFI		
			if ( __HAL_DMA_GET_COUNTER (huart->hdmarx ) == 0 )
			{
				WifiCom_Receive_It () ; 
			}
	#endif
		break ; 
		default :
		break ;		
			
	}		
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	switch ( (long) huart->Instance )
	{
		case (long) USART2 : // avec Lidar
	#ifdef RPLIDAR	
	#endif				
		break ; 
		
		case (long) USART6 : // avec carte WIFI 
	#ifdef WIFI	
	#endif		
		break ; 
		
		default :
		break ;		
			
	}		
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)  
{
	switch ( (long) huart->Instance )
	{
		case (long) USART2 : // avec Lidar
	#ifdef RPLIDAR		
			Rplidar_Transmit_It () ; 
	#endif				
		break ; 
		
		case (long) USART6 : // avec carte WIFI 
	#ifdef WIFI	
	#endif		
		break ; 
		
		default :
		break ;		
			
	}		
}
	
	

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
