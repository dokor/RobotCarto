/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : spi_flash.h
* Author             : MCD Application Team
* Date First Issued  : 02/05/2007
* Description        : Header for spi_flash.c file.
********************************************************************************
* History:
* 04/02/2007: V0.2
* 02/05/2007: V0.1
* 23/03/2016: V1.0  A6R 
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void SPI_FLASH_Init(void);
HAL_StatusTypeDef SPI_FLASH_SubSectorErase(uint32_t SectorAddr, T_OuiNonErreur Attente );
HAL_StatusTypeDef SPI_FLASH_SectorErase(uint32_t SectorAddr, T_OuiNonErreur Attente );
HAL_StatusTypeDef SPI_FLASH_BulkErase( T_OuiNonErreur Attente );
HAL_StatusTypeDef	SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite, T_OuiNonErreur Attente );
HAL_StatusTypeDef	SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
HAL_StatusTypeDef SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
HAL_StatusTypeDef SPI_FLASH_WaitForWriteEnd(void);
T_OuiNonErreur  	SPI_FLASH_TestForWriteEnd(void); 
HAL_StatusTypeDef SPI_FLASH_WriteEnable(void);
uint32_t 					SPI_FLASH_ReadID(void);

#endif /* __SPI_FLASH_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
