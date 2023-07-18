/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: May 26, 2023
 *      Author: NHHanh
 */

#include "stm32f103xx_i2c_driver.h"
#include <stdint.h>

/**
 * Helper functions
 */
// TO DO: Create new files for helper functions
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/***************************************************************************
 * @fn					- I2C_PeriClockControl
 *
 * @brief				- Enables or disables clock for a given I2C
 *
 * @param[in]			- I2Cx base address
 * @param[in]			- "ENABLE" or "DISABLE" command for the I2Cx clock
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi)
	{
		if (pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_EN();
	}
	else
	{
		if (pI2Cx == I2C1)
			I2C1_PCLK_DI();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_DI();
	}
}

/***************************************************************************
 * @fn					- AckControl
 *
 * @brief				- Sets or clears ack bit
 *
 * @param[in]			- I2Cx base address
 * @param[in]			- "ENABLE" or "DISABLE" command for ack control
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/***************************************************************************
 * @fn					- I2C_GetFlagStatus
 *
 * @brief				- Fetches I2C_SR1 register flags
 *
 * @param[in]			- Register addresses of a given I2C
 * @param[in]			- Requested flag
 *
 * @return				- Flag status
 *
 * @Note				- none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t Flagname)
{
	if (pI2Cx->SR1 & Flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***************************************************************************
 * @fn					- I2C_Init
 *
 * @brief				- Configures I2C peripheral register given the
 * 						configuration
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//	// ACK control bit
	//	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	//	pI2CHandle ->pI2Cx ->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = tempreg & 0x3F;

	// program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR calculate
	tempreg = 0;
	uint16_t ccr_value;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SPEED_SM)
	{
		// Standard mode
		ccr_value = RCC_GetPCLK1Value() / (pI2CHandle->I2C_Config.I2C_SCLSpeed * 2);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (pI2CHandle->I2C_Config.I2C_SCLSpeed * 3);
			tempreg |= (ccr_value & 0xFFF);
		}
		else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			ccr_value = RCC_GetPCLK1Value() / (pI2CHandle->I2C_Config.I2C_SCLSpeed * 25);
			tempreg |= (ccr_value & 0xFFF);
		}
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE Configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SPEED_SM)
	{
		// Standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// Fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/***************************************************************************
 * @fn					- I2C_DeInit
 *
 * @brief				- Resets registers of a given I2C
 *
 * @param[in]			- Register addresses of a given I2C
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
}

/***************************************************************************
 * @fn					- I2C_PeripheralControl
 *
 * @brief				- Enables or disables a given I2C peripheral
 *
 * @param[in]			- Register addresses of a given I2C
 * @param[in]			- Enable or disable command
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		I2C_AckControl(pI2Cx, ENABLE); // Enable ack, this can not be set before PE = 1
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/***************************************************************************
 * @fn					- I2C_MasterSendData
 *
 * @brief				- Sending data over a given I2C peripheral
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 * @param[in]			- Buffer that holds the data to be sent
 * @param[in]			- Total byte count that will be sent
 * @param[in]			- Address of the device that master will convey it's message to
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t SlaveAddr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
	//	 Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;

	// 3. send the address of the slave with r/nw bit set to w(0)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. confirm that address phase is completed be checking the ADDR flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;

	// 5. clear the ADDR flag according to its software sequence
	//	 Note: Until ADDR is cleared SCL will be stretched
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. send the data until Len becomes 0
	while (len > 0)
	{
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
			;
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	// 7. when Len becomes 0 wait for TXE=1 and BTF=1 before generating the STOP condition
	//	 Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//	 when BTF=1 SCL will be stretched
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
		;

	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
		;

	// 8. Generate STOP condition and master need not to wait for completion of stop condition
	//	 Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/***************************************************************************
 * @fn					- I2C_MasterReceiveData
 *
 * @brief				- Receiving data over a given I2C peripheral, polling
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 * @param[in]			- Buffer that will record the received message
 * @param[in]			- Total byte count that will be received
 * @param[in]			- Address of the device that master will receive it's message from
 *
 * @return				- none
 *
 * @Note				- none
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t len, uint8_t SlaveAddr)
{
	// 1.Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2.Confirm start generation is completed by checking I2C_SR1 SB
	// Until SB is cleared SCL will be stretched
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;

	// 3.Send the address of the slave with r/w bit set to r(1) (total of 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4.Wait until address phase is completed by checking the I2C_SR1 ADDR
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;

	// Read only 1 byte from slave
	if (len == 1)
	{
		// 1.Disable Ack (signals end of reception)
		I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

		// 2.Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// 3.Wait until RxNE = 1 so data register is not empty (wait for reception to be complete)
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
			;

		// 4.Generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// 5.Read data in to buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}
	else
	{
		// 1.Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// 2.Start reception
		for (uint32_t i = len; i > 0; i--)
		{
			// Wait until RxNE is 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
				;

			if (i == 2)
			{ // When there is 2 bytes remaining close I2C reception
				// Disable ack before receiving last byte
				I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

				// Generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// 3.Read the data from data register
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			// 4.Increment buffer address
			pRxbuffer++;
		}
	}

	// Enable Ack
	if (pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
	{
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}
}

/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
}

void I2C_IRQPriConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
}

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t pI2CHandle, uint8_t AppEv)
{
}

/***************************************************************************
 * @fn					- I2C_GenerateStartCondition
 *
 * @brief				- Helper function to initiate I2C communications
 *
 * @param[in]			- Register addresses of a given I2C
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/***************************************************************************
 * @fn					- I2C_AddressPhaseWrite
 *
 * @brief				- Helper function to initiate address phase of I2C
 * 						communications when in transmission
 *
 * @param[in]			- Register addresses of a given I2C
 * @param[in]			- Address of the slave
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

/***************************************************************************
 * @fn					- I2C_AddressPhaseRead
 *
 * @brief				- Helper function to initiate address phase of I2C
 * 						communications when in reception
 *
 * @param[in]			- Register addresses of a given I2C
 * @param[in]			- Address of the slave
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // This byte contains slave address and r/w bit
	SlaveAddr |= 1;				// Set r/w bit to 'read' aka. set this bit
	pI2Cx->DR = SlaveAddr;
}

/***************************************************************************
 * @fn					- I2C_ClearADDR
 *
 * @brief				- Helper function to clear ADDR flag
 *
 * @param[in]			- I2C structure that holds configuration and I2C address
 *
 * @return				- none
 *
 * @Note				- ADDR flag is cleared when SR1 is read followed
 * 						by SR2 is read operations or by hardware when PE
 * 						flag is 0 (see reference manual: RM0090)
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

/***************************************************************************
 * @fn					- I2C_GenerateStop
 *
 * @brief				- Helper function to produce stop condition
 *
 * @param[in]			- Register addresses of a given I2C
 *
 * @return				- none
 *
 * @Note				- none
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
