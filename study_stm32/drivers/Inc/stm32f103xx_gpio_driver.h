/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: May 25, 2023
 *      Author: NHHanh
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"
#include <stdbool.h>


/*
 *Chỗ này là cấu hình cho GPOI - PIN
  */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t	GPIO_PinModeAndSpeed;
	uint8_t GPIO_PinPuPdControl	;
	uint8_t GPIO_CRF;
	uint8_t GPIO_PinAltFunMode;
	uint8_t GPIO_PinIRQ_Trigger;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t	*pGPIOx;	//This pointer holds the base address of the GPIO port
	GPIO_PinConfig_t	GPIO_PinConfig;	//This pointer holds the GPIO pin configuration settings
}GPIO_Handle_t;

/**
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 * */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/**
 * @GPIO_PIN_MODES
 * */
#define GPIO_MODE_IN 				0b00

#define GPIO_MODE_OUT_SPEED_10MHZ	0b01
#define GPIO_MODE_OUT_SPEED_2MHZ	0b10
#define GPIO_MODE_OUT_SPEED_50MHZ	0b11


/**
 * @GPIO_PIN_CNF
 * */
#define GPIO_CNF_GEOUT_PP			0b00
#define GPIO_CNF_GEOUT_OD			0b01
#define GPIO_CNF_ALOUT_PP			0b10
#define GPIO_CNF_ALOUT_OD			0b11


#define GPIO_CNF_IN_ANALOG			0b00
#define GPIO_CNF_IN_FLOAT			0b01
#define GPIO_CNF_IN_PUPD			0b10
#define GPIO_CNF_IN_RESERVED 		0b11

#define GPIO_PU						1
#define GPIO_PD						0


/**
 * @GPIO_PIN_IRQ
 * */
#define GPIO_IRQ_FT                 0b100 // Interrupt trigger falling edge
#define GPIO_IRQ_RT                 0b010 // Interrupt trigger rising edge
#define GPIO_IRQ_RFT                0b110 // Interrupt trigger falling and rising edge


/*******************************************************************************************************
 * 									APIs supported by this driver
 * 		For more information about the APIs to check the function definitions
*******************************************************************************************************/
/*
 * 	Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,bool EnorDi);


/*
 * 	Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * 	Read and Write data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * 	IRQ configuration and IRQ Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQSetPriority(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
