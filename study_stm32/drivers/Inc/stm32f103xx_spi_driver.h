/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: May 25, 2023
 *      Author: NHHanh
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_


#include "stm32f103xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_SSM;
	uint8_t	SPI_SclkSpeed;
	uint8_t SPI_CRCPolynomial;

}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_Config_t	SPIConfig;
	SPI_RegDef_t	*pSPIx;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint8_t			TxLen;
	uint8_t			RxLen;
	uint8_t			TxState;
	uint8_t			RxState;

}SPI_Handle_t;

/*
 * Possible SPI application states
 */

#define SPI_STATE_READY				0
#define SPI_STATE_BUSY_IN_RX		1
#define SPI_STATE_BUSY_IN_TX		2

#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3

/*
 * @SPI_device MODE
 */
#define SPI_DEVICE_MODE_SLAVE	0
#define SPI_DEVICE_MODE_MASTER	1

/*
 * @SPI BUS CONFIGURATON
 */

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI DFF
 */

#define SPI_DFF_8BIT	0
#define SPI_DFF_16BIT	1

/*
 * @SPI CPHA CLOCK
 */
#define SPI_CPHA_LOW	0
#define SPI_CPHA_HIGH	1

/*
 * @SPI CPOL CLOCK
 */
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1

/*
 * @SPI SSM (SELECTED)
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/*
 * @SPI Speed configuration
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/* SPI Flag */
#define SPI_FLAG_TXE			(1 << SPI_SR_TXE)
#define SPI_FLAG_RXNE			(1 << SPI_SR_RXNE)
#define SPI_FLAG_BUSY			(1 << SPI_SR_BSY)

/************************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
 * Data Send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer,uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,uint32_t len);
/*
 * IQR configure and ISR Handle
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

__attribute__((weak))void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t  EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t  EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t  EnorDi);

#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */
