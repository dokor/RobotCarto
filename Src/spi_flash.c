/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : spi_flash.c
* Author             : MCD Application Team
* Date First Issued  : 02/05/2007
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and SPI M25P64 FLASH.
********************************************************************************
* History:
* 04/02/2007: V0.2
* 02/05/2007: V0.1
* 23/03/2016: V1.0  A6R 
*										hormis l'initialisation, ce fichier n'utilise pas les 
*										fonctions d'écriture/lecture du HAL pour des raisons de 
*										réduction significative du code généré.
*										Tous les accès sont bloquants (pas d'it, pas de DMA)
*										testé sur M25PX16 de MICRON
********************************************************************************

*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "spi_flash.h"



/* Le spi employé */
#define   FlashSpi		hspi2

/* le port pour la commande du CS de la mémoire FLASH */
#define 	SPI_FLASH_ChipSelect(Etat)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, Etat );	


/* Private typedef -----------------------------------------------------------*/
#define SPI_FLASH_PAGE_SIZE 256

#define PAGE_PROG  0x02  /* Write to Memory instruction */
#define WRSR       0x01  /* Write Status Register instruction */ 
#define WREN       0x06  /* Write enable instruction */

#define READ       0x03  /* Read from Memory instruction */
#define RDSR       0x05  /* Read Status Register instruction  */
#define RDID       0x9F  /* Read identification */
#define SE         0xD8  /* Sector Erase instruction */
#define SSE        0x20  /* Sub-Sector Erase instruction */
#define BE         0xC7  /* Bulk Erase instruction */

/* bits du registre Status */
#define WIP_FLAG   0x01  /* Write In Progress (WIP) flag */
#define	WEL_FLAG	 0x02  /* Write Enable (WEL) flag */

#define DUMMY_BYTE 0xA5


#define	TIMEOUT_SPI		2		/* sécurité d'accès au SPI 2 ms */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_Init(void)
{
  /* le SPI ainsi que le Gpio pour le CS sont initialisés dans main via le HAL */
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_ChipSelect(GPIO_PIN_SET);
}

/**
  * @brief This function handles SPI Communication Timeout.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for SPI module.
  * @param  Flag: SPI flag to check
  * @param  Status: Flag status to check: RESET or set
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout)  
{
  uint32_t tickstart = 0;

  /* Get tick */ 
  tickstart = HAL_GetTick();

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) == RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          {
            SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) != RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          {
            SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Transmit and Receive an amount of data in blocking mode 
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for SPI module.
  * @param  pTxData: pointer to transmission data buffer
  * @param  pRxData: pointer to reception data buffer to be
  * @param  Size: amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef SPI_FLASH_Receive(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
  __IO uint16_t tmpreg;
  uint32_t tmpstate = 0, tmp = 0;
  
  tmpstate = hspi->State; 
  if((tmpstate == HAL_SPI_STATE_READY) || (tmpstate == HAL_SPI_STATE_BUSY_RX))
  {
    if((pRxData == NULL ) || (Size == 0))
    {
      return  HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);
 
    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if(hspi->State == HAL_SPI_STATE_READY)
    {
      hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

     /* Configure communication */   
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

    hspi->pRxBuffPtr  = pRxData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;  
    
    hspi->TxXferSize  = Size; 
    hspi->TxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->RxISR = 0;
    hspi->TxISR = 0;

    /* Reset CRC Calculation */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      SPI_RESET_CRC(hspi);
    }

    /* Check if the SPI is already enabled */ 
    if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    {
      /* Enable SPI peripheral */
      __HAL_SPI_ENABLE(hspi);
    }

    /* Transmit and Receive data in 8 Bit mode */
    if(hspi->TxXferCount == 0x01)
    {
      hspi->Instance->DR = DUMMY_BYTE ;
      hspi->TxXferCount--;
    }
    if(hspi->TxXferCount == 0)
    {
      /* Enable CRC Transmission */
      if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
      {
        hspi->Instance->CR1 |= SPI_CR1_CRCNEXT;
      }

      /* Wait until RXNE flag is set */
      if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }

      (*hspi->pRxBuffPtr) = hspi->Instance->DR;
      hspi->RxXferCount--;
    }
    else
    {
      while(hspi->TxXferCount > 0)
      {
        /* Wait until TXE flag is set to send data */
        if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }
        hspi->Instance->DR = DUMMY_BYTE ;
        hspi->TxXferCount--;
        /* Enable CRC Transmission */
        if((hspi->TxXferCount == 0) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
        {
          hspi->Instance->CR1 |= SPI_CR1_CRCNEXT;
        }
        /* Wait until RXNE flag is set */
        if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }
            
        (*hspi->pRxBuffPtr++) = hspi->Instance->DR;
        hspi->RxXferCount--;
      }
    }
    /* Read CRC from DR to close CRC calculation process */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait until RXNE flag is set */
      if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
      {
        hspi->ErrorCode |= HAL_SPI_ERROR_CRC;
        return HAL_TIMEOUT;
      }
      /* Read CRC */
      tmpreg = hspi->Instance->DR;
      UNUSED(tmpreg);
    }

    /* Wait until Busy flag is reset before disabling SPI */
    if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }
    
    hspi->State = HAL_SPI_STATE_READY;

    tmp = __HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR);
    /* Check if CRC error occurred */
    if((hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) && (tmp != RESET))
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_CRC;

      /* Reset CRC Calculation */
      if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
      {
        SPI_RESET_CRC(hspi);
      }

      /* Process Unlocked */
      __HAL_UNLOCK(hspi);
      
      return HAL_ERROR; 
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


/**
  * @brief  Transmit an amount of data in blocking mode
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for SPI module.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef SPI_FLASH_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{

  if(hspi->State == HAL_SPI_STATE_READY)
  {
    if((pData == NULL ) || (Size == 0)) 
    {
      return  HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Configure communication */
    hspi->State = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

    hspi->pTxBuffPtr = pData;
    hspi->TxXferSize = Size;
    hspi->TxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->TxISR = 0;
    hspi->RxISR = 0;
    hspi->RxXferSize   = 0;
    hspi->RxXferCount  = 0;

    /* Reset CRC Calculation */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      SPI_RESET_CRC(hspi);
    }

    /* Check if the SPI is already enabled */ 
    if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    {
      /* Enable SPI peripheral */
      __HAL_SPI_ENABLE(hspi);
    }

    /* Transmit data in 8 Bit mode */
    while(hspi->TxXferCount > 0)
    {
      /* Wait until TXE flag is set to send data */
      if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
      { 
        return HAL_TIMEOUT;
			}
      hspi->Instance->DR = (*hspi->pTxBuffPtr++);
      hspi->TxXferCount--;
    }
		/* Enable CRC Transmission */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) 
    {
      hspi->Instance->CR1 |= SPI_CR1_CRCNEXT;
    }
    /* Wait until TXE flag is set to send data */
    if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }

    /* Wait until Busy flag is reset before disabling SPI */
    if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
    { 
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }
 
    /* Clear OVERRUN flag in 2 Lines communication mode because received is not read */
    if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : force SPI_CRCCALCULATION_DISABLE 
* Return         : None
*******************************************************************************/
HAL_StatusTypeDef SPI_FLASH_WriteEnable(void)  
{
	uint8_t send_data = WREN ;
	HAL_StatusTypeDef status ; 
	
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_ChipSelect(GPIO_PIN_RESET);	
	FlashSpi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE ; 
	
  /* Send "Write Enable" instruction */
  status = SPI_FLASH_Transmit(&FlashSpi, &send_data, 1, TIMEOUT_SPI) ;

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_ChipSelect(GPIO_PIN_SET);	
	return (status) ;
}


/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the  
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
HAL_StatusTypeDef  SPI_FLASH_WaitForWriteEnd(void) 
{
	uint8_t send_receive_data = RDSR ;
	HAL_StatusTypeDef status ; 
  
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_ChipSelect(GPIO_PIN_RESET);	
	FlashSpi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE ;  
  /* Send "Read Status Register" instruction */	
  if  ( ( status = SPI_FLASH_Transmit(&FlashSpi, &send_receive_data, 1, TIMEOUT_SPI)) == HAL_OK )
  {
		/* Loop as long as the memory is busy with a write cycle */ 		
		do 
		{ 	 
			status = SPI_FLASH_Receive(&FlashSpi, &send_receive_data, 1, TIMEOUT_SPI) ;
		}
		while ( (send_receive_data & WIP_FLAG) != 0 ) ; /* Write in progress */ 
	}

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_ChipSelect(GPIO_PIN_SET);	 
	return (status) ;	
}


/*******************************************************************************
* Function Name  : SPI_FLASH_TestForWriteEnd
* Description    : Read the status of the Write In Progress (WIP) flag in the  
*                  FLASH's status  register 
* Input          : None
* Output         : None
* Return         : OUI = Terminé ; NON = En cours  ; Erreur si pb SPI
*******************************************************************************/
T_OuiNonErreur  SPI_FLASH_TestForWriteEnd(void) 
{
	uint8_t send_receive_data = RDSR ;
	HAL_StatusTypeDef status ; 
  
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_ChipSelect(GPIO_PIN_RESET);	
	FlashSpi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE ;  
  /* Send "Read Status Register" instruction */	
  if  ( ( status = SPI_FLASH_Transmit(&FlashSpi, &send_receive_data, 1, TIMEOUT_SPI)) == HAL_OK )
  {
			status = SPI_FLASH_Receive(&FlashSpi, &send_receive_data, 1, TIMEOUT_SPI) ;
	}
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_ChipSelect(GPIO_PIN_SET);	 
	if ( status == HAL_OK )
		return ( (send_receive_data & WIP_FLAG) != 0  ? NON : OUI  );
	else
		return ( ERREUR ) ;
}



/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification ou 0 si erreur
*******************************************************************************/
uint32_t SPI_FLASH_ReadID(void)
{   	
	uint8_t send_receive_data[3] ;
	HAL_StatusTypeDef status ; 
	
	send_receive_data[0] = RDID ;
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_ChipSelect(GPIO_PIN_RESET);	   
	FlashSpi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE ;  
	
  /* Send "RDID " instruction & read 3 bytes */
  if  ( ( status = SPI_FLASH_Transmit(&FlashSpi, send_receive_data, 1, TIMEOUT_SPI)) == HAL_OK )
	{
		/* Read 3 bytes from the FLASH */			
		status = SPI_FLASH_Receive(&FlashSpi, send_receive_data, 3, TIMEOUT_SPI) ;
	}
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_ChipSelect(GPIO_PIN_SET);
  if ( status == HAL_OK )
		return ( (send_receive_data[0] << 16) | (send_receive_data[1] << 8) | send_receive_data[2] );			
	else 
		return 0 ;
}



/*******************************************************************************
* Function Name  : SPI_FLASH_SubSectorErase			4K par sous-secteur
* Description    : Erases the specified FLASH sector.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
* Durée d'execution de la fonction  : de 70ms à 150ms
*******************************************************************************/
HAL_StatusTypeDef SPI_FLASH_SubSectorErase(uint32_t SectorAddr, T_OuiNonErreur Attente )
{	
	uint8_t send_data[4] ;
	HAL_StatusTypeDef status ;
 	
  /* SubSector Erase */ 
	send_data[0] = SSE ;
	send_data[1] = (SectorAddr & 0xFF0000) >> 16 ;
	send_data[2] = (SectorAddr & 0xFF00) >> 8 ;
	send_data[3] = (SectorAddr & 0xFF ) ;	

  /* Send write enable instruction */
  if ( (status = SPI_FLASH_WriteEnable() ) == HAL_OK )
	{		
		/* Select the FLASH: Chip Select low */
		SPI_FLASH_ChipSelect(GPIO_PIN_RESET);		
		/* Send instruction  */
		status = SPI_FLASH_Transmit(&FlashSpi, send_data, 4, TIMEOUT_SPI) ;
		/* Deselect the FLASH: Chip Select high */
		SPI_FLASH_ChipSelect(GPIO_PIN_SET);	
		
		if ( status == HAL_OK )
		{
			if ( Attente == OUI )
			{	/* Wait the end of Flash writing */
				status = SPI_FLASH_WaitForWriteEnd();
			}				
		}
	}
	return (status) ;
	
}



/*******************************************************************************
* Function Name  : SPI_FLASH_SectorErase			64K par secteur
* Description    : Erases the specified FLASH sector.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
* Durée d'execution de la fonction  : de 0.6s à 3s
*******************************************************************************/
HAL_StatusTypeDef SPI_FLASH_SectorErase(uint32_t SectorAddr, T_OuiNonErreur Attente )
{
	uint8_t send_data[4] ;
	HAL_StatusTypeDef status ;   
	/* Sector Erase */ 
	send_data[0] = SE ;
	send_data[1] = (SectorAddr & 0xFF0000) >> 16 ;
	send_data[2] = (SectorAddr & 0xFF00) >> 8 ;
	send_data[3] = (SectorAddr & 0xFF ) ;
	
  /* Send write enable instruction */
  if ( (status = SPI_FLASH_WriteEnable() ) == HAL_OK )
	{		
		/* Select the FLASH: Chip Select low */
		SPI_FLASH_ChipSelect(GPIO_PIN_RESET);		
		/* Send instruction  */
		status = SPI_FLASH_Transmit(&FlashSpi, send_data, 4, TIMEOUT_SPI) ;
		/* Deselect the FLASH: Chip Select high */
		SPI_FLASH_ChipSelect(GPIO_PIN_SET);	

		if ( status == HAL_OK )
		{
			if ( Attente == OUI )
			{	/* Wait the end of Flash writing */
				status = SPI_FLASH_WaitForWriteEnd();
			}				
		}
	}
	return (status) ;
}





/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
* Durée d'execution de la fonction  : de 15s à 80s
*******************************************************************************/
HAL_StatusTypeDef SPI_FLASH_BulkErase( T_OuiNonErreur Attente )
{ 			
	uint8_t send_data = BE ;
	HAL_StatusTypeDef status ; 
	
  /* Send write enable instruction */
  if ( (status = SPI_FLASH_WriteEnable() ) == HAL_OK )
	{		
		/* Select the FLASH: Chip Select low */
		SPI_FLASH_ChipSelect(GPIO_PIN_RESET);		
		/* Send instruction  */
		status = SPI_FLASH_Transmit(&FlashSpi, &send_data, 1, TIMEOUT_SPI) ;
		/* Deselect the FLASH: Chip Select high */
		SPI_FLASH_ChipSelect(GPIO_PIN_SET);	

		if ( status == HAL_OK )
		{
			if ( Attente == OUI )
			{	/* Wait the end of Flash writing */
				status = SPI_FLASH_WaitForWriteEnd();
			}				
		}	
	}
	return (status) ;
}



/*******************************************************************************
* Function Name  : SPI_FLASH_BufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read 
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : status de communication et vérifier si Erreur si Crc erreur 
*******************************************************************************/
HAL_StatusTypeDef SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
	uint8_t send_data[4] ;
	HAL_StatusTypeDef status ;
	
	send_data[0] =  READ ;	
	send_data[1] = (ReadAddr & 0xFF0000) >> 16 ;  /* Send ReadAddr high nibble address byte to read from */
	send_data[2] = (ReadAddr & 0xFF00) >> 8 ;			/* Send ReadAddr medium nibble address byte to read from */
	send_data[3] = (ReadAddr & 0xFF ) ;						/* Send ReadAddr low nibble address byte to read from */

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_ChipSelect(GPIO_PIN_RESET);		
	FlashSpi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE ;	
	
  /* Send "Read from Memory " instruction */	
	if ( ( status = SPI_FLASH_Transmit(&FlashSpi, send_data, 4, TIMEOUT_SPI) ) == HAL_OK)
	{	/* Lecture des données */
		FlashSpi.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE ;  
		status = SPI_FLASH_Receive(&FlashSpi, pBuffer, NumByteToRead, TIMEOUT_SPI) ;
  }
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_ChipSelect(GPIO_PIN_SET);		
	return (status) ;
}


/*******************************************************************************
* Function Name  : SPI_FLASH_PageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value. 
* Output         : None
* Return         : None
*******************************************************************************/
HAL_StatusTypeDef SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite,  T_OuiNonErreur Attente )
{
	uint8_t send_data[4] ;	
	HAL_StatusTypeDef status ;
	
	send_data[0] =  PAGE_PROG ;	
	send_data[1] = (WriteAddr & 0xFF0000) >> 16 ;  	/* Send WriteAddr high nibble address byte to read from */
	send_data[2] = (WriteAddr & 0xFF00) >> 8 ;			/* Send WriteAddr medium nibble address byte to read from */
	send_data[3] = (WriteAddr & 0xFF ) ;						/* Send WriteAddr low nibble address byte to read from */ 
  
	/* Enable the write access to the FLASH */
  if ( (status = SPI_FLASH_WriteEnable() ) == HAL_OK )
	{		
		/* Select the FLASH: Chip Select low */
		SPI_FLASH_ChipSelect(GPIO_PIN_RESET);		 
		/* Send "Write to Memory " instruction */    
		if ( ( status = SPI_FLASH_Transmit(&FlashSpi, send_data, 4, TIMEOUT_SPI) ) == HAL_OK )            
		{	/* écriture des données*/
			FlashSpi.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE ; 
			status = SPI_FLASH_Transmit(&FlashSpi, pBuffer, NumByteToWrite, TIMEOUT_SPI) ;
		}
		/* Deselect the FLASH: Chip Select high */
		SPI_FLASH_ChipSelect(GPIO_PIN_SET);
		if ( status == HAL_OK )
		{
			if ( Attente == OUI )
			{	/* Wait the end of Flash writing */
				status = SPI_FLASH_WaitForWriteEnd();
			}				
		}
	}
	return ( status) ;
}


/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
HAL_StatusTypeDef SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	HAL_StatusTypeDef status ;	
  uint8_t NumOfPage, NumOfSingle, Addr, count, temp;

  Addr 				= WriteAddr % SPI_FLASH_PAGE_SIZE;
  count 			= SPI_FLASH_PAGE_SIZE - Addr;
  NumOfPage 	=  NumByteToWrite / SPI_FLASH_PAGE_SIZE;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PAGE_SIZE;
  
  if(Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
  {
    if(NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite, OUI) ;
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */ 
    {
      while(NumOfPage--)
      {
        if ( (status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PAGE_SIZE, OUI)) != HAL_OK )
					return ( status ) ;
        WriteAddr +=  SPI_FLASH_PAGE_SIZE;
        pBuffer += SPI_FLASH_PAGE_SIZE;  
      }        
      status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle, OUI);      
   }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if(NumOfPage== 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if(NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
      {
      	temp = NumOfSingle - count;
      	
      	if ( (status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, count, OUI) ) != HAL_OK )
					return (status) ;
        WriteAddr +=  count;
        pBuffer += count; 
        
        status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp, OUI);       	
      }
      else
      {
      	status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite, OUI);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PAGE_SIZE;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PAGE_SIZE;	
      
      if ( (status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, count, OUI) ) != HAL_OK )
				return (status) ;			
      WriteAddr +=  count;
      pBuffer += count;  
     
      while(NumOfPage--)
      {
        if ( (status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PAGE_SIZE, OUI) ) != HAL_OK ) 
					return (status) ;					
        WriteAddr +=  SPI_FLASH_PAGE_SIZE;
        pBuffer += SPI_FLASH_PAGE_SIZE;  
      }
      
      if(NumOfSingle != 0)
      {
        status = SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle, OUI);
      }
    }
  }  
	return (status ) ;	
}




/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
