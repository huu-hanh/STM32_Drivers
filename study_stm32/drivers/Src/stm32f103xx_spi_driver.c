/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: May 26, 2023
 *      Author: NHHanh
 */

#include <stm32f103xx_spi_driver.h>

static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVRerr_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/***************************************************************************
 * @fn					- SPI_PeriClockControl
 *
 * @brief				- Enables or disables clock for a given SPI
 *
 * @param[in]			- SPIx base address
 * @param[in]			- "ENABLE" or "DISABLE" command for the SPIx clock
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi)
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if (pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if (pSPIx == SPI3)
			SPI3_PCLK_EN();
	}
	else
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if (pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if (pSPIx == SPI3)
			SPI3_PCLK_DI();
	}
}

/***************************************************************************
 * @fn					- SPI_Init
 *
 * @brief				- Configures SPI peripheral
 *
 * @param[in]			- SPI structure that holds configuration and SPI address
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable SPI clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0;

	tempreg |= (1 << SPI_CR1_SSI);

	// 1- Configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// 2- Configure DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// 3- Configure CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// 4- Configure CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 5- Configure DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 6- Configure SSM
	if (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN)
	{
		tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
	}
	else
	{
		tempreg &= ~(pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
		SPI_SSOEConfig(SPI2, ENABLE);
	}

	// 7- Configure DFF
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg &= ~(1 << SPI_CR1_RX_ONLY);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared RXONLY should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RX_ONLY);
	}

	pSPIHandle->pSPIx->CR1 |= tempreg;

	// pSPIHandle->pSPIx->CRCPR = pSPIHandle->SPIConfig.SPI_CRCPolynomial;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx);

/***************************************************************************
 * @fn					- SPI_GetFlagStatus
 *
 * @brief				- Fetches SPI_SR register flags
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Requested flag
 *
 * @return				- Flag status
 *
 * @Note				- none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***************************************************************************
 * @fn					- SPI_PeripheralControl
 *
 * @brief				- Enables or disables a given SPI peripheral
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Enable or disable command
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == 1)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/***************************************************************************
 * @fn					- SPI_SSIConfig
 *
 * @brief				- Enables or disables internal slave select when
 * 						  software slave management is enabled
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Enable or disable command
 *
 * @return				- Flag status
 *
 * @Note				- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == 1)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/***************************************************************************
 * @fn					- SPI_SSOEConfig
 *
 * @brief				- Slave select output control function
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Enable or disable command
 *
 *
 * @return				- none
 *
 * @Note				- This function will block until Len is 0
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == 1)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/***************************************************************************
 * @fn					- SPI_SendData
 *
 * @brief				- Sends the desired data through selected SPI interface
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Buffer for transmission data
 * @param[in]			- Data length
 *
 * @return				- none
 *
 * @Note				- This function will block until Len is 0
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		/*1- Wait until TXE is set (Tx buffer is empty)*/
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET)
			;

		/*2- Check DFF bit*/
		if (((pSPIx->CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BIT)
		{
			// 16-bit DFF
			// 3- Load data in to the Data Register
			pSPIx->DR = *((uint16_t *)pTxBuffer); // Write to data register
			(uint16_t *)pTxBuffer++;			  // Increase Tx buffer address by 2 bytes for the next package
			len -= 2;							  // Decrease length of the data to be sent by 2 bytes
		}
		else
		{
			// 8-bit DFF
			// 3- Load data in to the Data Register
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
		}
		len--;
	}
	while (SPI_GetFlagStatus(SPI2, SPI_FLAG_BUSY))
		;
}

/***************************************************************************
 * @fn					- SPI_ReceiveData
 *
 * @brief				- Receives data from specified SPI peripheral
 *
 * @param[in]			- Register addresses of a given SPI
 * @param[in]			- Buffer for reception data
 * @param[in]			- Expected data length for reception
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		/*1- Wait until RXNE is set (Rx buffer is not empty)*/
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET)
			;

		/*2- Check DFF bit*/
		if (((pSPIx->CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BIT)
		{
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			len -= 2;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
		}
		len--;
	}
}

/***************************************************************************
 * @fn					- SPI_SendDataIT
 *
 * @brief				- Saves buffer address and length information and
 * 						  enables SPI interrupt using TXEIE
 *
 * @param[in]			- Structure that holds configuration and register information
 * @param[in]			- Buffer for transmission data
 * @param[in]			- Data length
 *
 * @return				- State that shows if the peripheral is busy in transmission
 *
 * @Note				- This function will block until Len is 0
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_STATE_BUSY_IN_TX)
	{
		// 1. Save Tx buffer address and length information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over same peripheral
		pSPIHandle->TxState = SPI_STATE_BUSY_IN_TX;

		// 3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data transmission will be handled by the ISR code
	}
	return state;
}

/***************************************************************************
 * @fn					- SPI_ReceiveDataIT
 *
 * @brief				-
 *
 * @param[in]			- Structure that holds configuration and register information
 * @param[in]			- Buffer for reception data
 * @param[in]			- Expected data length
 *
 * @return				- State that shows if the peripheral is busy in reception
 *
 * @Note				- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_STATE_BUSY_IN_RX)
	{
		// 1. Save Rx buffer address and length information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over same peripheral
		pSPIHandle->RxState = SPI_STATE_BUSY_IN_RX;

		// 3. Enable RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/***************************************************************************
 * @fn					- SPI_IRQConfig
 *
 * @brief				- Enables or disables the relevant interrupt
 *
 * @param[in]			- IRQ number for the interrupt
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	uint8_t regx = IRQNumber / 32;
	if (EnorDi == ENABLE)
		NVIC_ISER->REG_NUM[regx] |= (1 << (IRQNumber % 32));
	else
		NVIC_ICER->REG_NUM[regx] |= (1 << (IRQNumber % 32));
}

/***************************************************************************
 * @fn					- SPI_IRQHandling
 *
 * @brief				-
 *
 * @param[in]			- Structure that holds configuration and register information
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	// Check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		// Handle TXE
		SPI_TXE_Interrupt_Handle(pSPIHandle);
	}

	// Check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		SPI_RXNE_Interrupt_Handle(pSPIHandle);
	}

	// Check for ovr flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		// Handle ovr error
		SPI_OVRerr_Interrupt_Handle(pSPIHandle);
	}
}

/***************************************************************************
 * @fn					- SPI_TXE_Interrupt_Handle
 *
 * @brief				- SPI transmission interrupt handler
 *
 * @param[in]			- Structure that holds configuration and register information
 *
 * @return				- none
 *
 * @Note				- none
 */
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	// Check DFF bit
	if (((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BIT)
	{
		// 16-bit DFF
		// Load data in to the Data Register
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer++);
		pSPIHandle->TxLen--;
	}
	else
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer++);

	pSPIHandle->TxLen--;

	if (!pSPIHandle->TxLen)
	{
		// TxLen is zero , so close the spi transmission and inform the application that
		// TX is over.
		// This prevents interrupts from setting up of TXE flag
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_STATE_READY;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/***************************************************************************
 * @fn					- SPI_RXNE_Interrupt_Handle
 *
 * @brief				- SPI reception interrupt handler
 *
 * @param[in]			- Structure that holds configuration and register information
 *
 * @return				- none
 *
 * @Note				- none
 */
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	if (((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BIT)
	{
		*((uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else
	{
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer++;
	}

	pSPIHandle->RxLen--;

	if (!pSPIHandle->RxLen)
	{
		// Reception is complete
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_STATE_READY;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/***************************************************************************
 * @fn					- SPI_OVRerr_Interrupt_Handle
 *
 * @brief				- SPI overrun error interrupt handler
 *
 * @param[in]			- Structure that holds configuration and register information
 *
 * @return				- none
 *
 * @Note				- none
 */
static void SPI_OVRerr_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
}
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
}
