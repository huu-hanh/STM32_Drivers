/*
 * stm32f103xx_usart_driver.h
 *
 *  Created on: May 25, 2023
 *      Author: NHHanh
 */

#ifndef INC_STM32F103XX_USART_DRIVER_H_
#define INC_STM32F103XX_USART_DRIVER_H_


#include <stm32f103xx.h>

/*
 * This is a configuration structure for a USARTx peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*
 * This is a Handle structure for a I2Cx peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t	*pRxBuffer;
	uint8_t	TxLen;
	uint8_t	RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;

/*
 * @USART_Mode
 */
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

/*
 * @USART_ParityControl
 */
#define USART_PARITY_EN_EVEN	0
#define USART_PARITY_EN_ODD		1
#define USART_PARITY_DISABLE	2

/*
 * @USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 * @USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 * @USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 *@USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART Flag
 */
#define USART_FLAG_PE		(1<<USART_SR_PE)
#define USART_FLAG_FE		(1<<USART_SR_FE)
#define USART_FLAG_NE		(1<<USART_SR_NE)
#define USART_FLAG_ORE		(1<<USART_SR_ORE)
#define USART_FLAG_IDLE		(1<<USART_SR_IDLE)
#define USART_FLAG_RXNE		(1<<USART_SR_RXNE)
#define USART_FLAG_TC		(1<<USART_SR_TC)
#define USART_FLAG_TXE		(1<<USART_SR_TXE)
#define USART_FLAG_LBD		(1<<USART_SR_LBD)
#define USART_FLAG_CTS		(1<<USART_SR_CTS)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 	1
#define USART_BUSY_IN_TX 	2
#define USART_READY 		0

#define USART_EVENT_TX_CMPLT   0
#define	USART_EVENT_RX_CMPLT   1
#define	USART_EVENT_IDLE       2
#define	USART_EVENT_CTS  	   3
#define	USART_EVENT_PE    	   4
#define	USART_ERR_FE     	   5
#define	USART_ERR_NE    	   6
#define	USART_ERR_ORE    	   7


void USART_PeriClockControl(USART_RegDef_t *pUSARTx,uint8_t  EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARThandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);
/*
 * Data Send and receive
 */

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle ,uint8_t *pTxBuffer,uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle ,uint8_t *pRxBuffer,uint32_t len);

/*
 * IQR configure and ISR Handle
 */
void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
/*
 *
 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx,uint8_t  EnorDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

__attribute__((weak))void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv);

#endif /* INC_STM32F103XX_USART_DRIVER_H_ */
