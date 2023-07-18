/*
 * stm32f103xx.h
 *
 *  Created on: May 25, 2023
 *      Author: NHHanh
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>



#define SYSTICK_ADDRESS					((volatile uint32_t *)0xE000E010)

/**
 * ARM Cortex-Mx Processor NVIC ISERx register addresses
 * */
#define NVIC_ISER_0                 ((volatile uint32_t *)0xE000E100)
#define NVIC_ISER_1                 ((volatile uint32_t *)0xE000E104)
#define NVIC_ISER_2                 ((volatile uint32_t *)0xE000E108)
#define NVIC_ISER_3                 ((volatile uint32_t *)0xE000E10C)
#define NVIC_ISER_4                 ((volatile uint32_t *)0xE000E110)
#define NVIC_ISER_5                 ((volatile uint32_t *)0xE000E114)
#define NVIC_ISER_6                 ((volatile uint32_t *)0xE000E118)
#define NVIC_ISER_7                 ((volatile uint32_t *)0xE000E11C)

/**
 * ARM Cortex-Mx Processor NVIC ICERx register addresses
 * */
#define NVIC_ICER_0                 ((volatile uint32_t *) 0XE000E180)
#define NVIC_ICER_1                 ((volatile uint32_t *) 0XE000E184)
#define NVIC_ICER_2                 ((volatile uint32_t *) 0XE000E188)
#define NVIC_ICER_3                 ((volatile uint32_t *) 0XE000E18C)
#define NVIC_ICER_4                 ((volatile uint32_t *) 0XE000E190)
#define NVIC_ICER_5                 ((volatile uint32_t *) 0XE000E194)
#define NVIC_ICER_6                 ((volatile uint32_t *) 0XE000E198)
#define NVIC_ICER_7                 ((volatile uint32_t *) 0XE000E19C)

/**
 * ARM Cortex-Mx Processor NVIC PR register addresses
 * */
#define NVIC_IPR_BASE_ADDR          ((volatile uint32_t *) 0xE000E400)

/**
 * Arm Cortex-Mx Processor number of priority bits implemented in Interrupt Priority Register
 * */
#define NO_IPR_BITS_IMPLEMENTED     4

/**
 * register definition structure for the NVIC ISER
 * */
typedef struct {
    volatile uint32_t REG_NUM[8];
} NVIC_ISER_RegDef_t;

/**
 * register definition structure for the NVIC ICER
 * */
typedef struct {
    volatile uint32_t REG_NUM[8];
} NVIC_ICER_RegDef_t;

/**
 * register definition for the NVIC ISER
 * */
#define NVIC_ISER  ((NVIC_ISER_RegDef_t *) NVIC_ISER_0)

/**
 * register definition for the NVIC ICER
 * */
#define NVIC_ICER  ((NVIC_ICER_RegDef_t *) NVIC_ICER_0)
/* ---------------------------------------------------------------- */

/**
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASE_ADDR             0x08000000UL
#define SRAM1_BASE_ADDR             0x20000000UL
#define ROM_BASE_ADDR               0x1FFFF000UL
#define SRAM                        SRAM1_BASE_ADDR

/**
 * AHBx and APBx Bus Peripheral base addresses
 * */

#define PERIPHERAL_BASE_ADDR        0x40000000UL
#define APB1_BASE_ADDR              PERIPHERAL_BASE_ADDR
#define APB2_BASE_ADDR              0x40010000UL
#define AHB_BASE_ADDR               0x40018000UL
#define AHB1_BASE_ADDR              AHB_BASE_ADDR

#define AHB_ADD_OFFSET(OFFSET)      (AHB_BASE_ADDR + (OFFSET))
#define APB1_ADD_OFFSET(OFFSET)     (APB1_BASE_ADDR + (OFFSET))
#define APB2_ADD_OFFSET(OFFSET)     (APB2_BASE_ADDR + (OFFSET))

/**
 * Base addresses of peripherals which are hanging on AHB bus
 * */
#define SDIO_BASE_ADDR                       AHB_ADD_OFFSET(0x0000UL)
#define DMA1_BASE_ADDR                       AHB_ADD_OFFSET(0x8000UL)
#define DMA2_BASE_ADDR                       AHB_ADD_OFFSET(0x8400UL)
#define RCC_BASE_ADDR                        AHB_ADD_OFFSET(0x9000UL)
#define FLASH_MEMORY_INTERFACE_BASE_ADDR     AHB_ADD_OFFSET(0xA000UL)
#define CRC_BASE_ADDR                        AHB_ADD_OFFSET(0xB000UL)
#define ETHERNET_BASE_ADDR                   AHB_ADD_OFFSET(0x10000UL)
#define USB_OTG_FS_BASE_ADDR                 AHB_ADD_OFFSET(0xFFE8000UL)
#define FSMC_BASE_ADDR                       AHB_ADD_OFFSET(0x5FFE8000UL)


/**
 * Base addresses of peripherals which are hanging on APB1 bus
 * */

#define I2C1_BASE_ADDR                       APB1_ADD_OFFSET(0x5400UL)
#define I2C2_BASE_ADDR                       APB1_ADD_OFFSET(0x5800UL)

#define SPI2_BASE_ADDR                       APB1_ADD_OFFSET(0x3800UL)
#define SPI3_BASE_ADDR                       APB1_ADD_OFFSET(0x3C00UL)

#define USART2_BASE_ADDR                     APB1_ADD_OFFSET(0x4400UL)
#define USART3_BASE_ADDR                     APB1_ADD_OFFSET(0x4800UL)

//#define UART4_BASE_ADDR                    APB1_ADD_OFFSET(0x4C00UL)
//#define UART5_BASE_ADDR                    APB1_ADD_OFFSET(0x5000UL)

#define TIM2_BASE_ADDR                       APB1_ADD_OFFSET(0x0000UL)
#define TIM3_BASE_ADDR                       APB1_ADD_OFFSET(0x0400UL)
#define TIM4_BASE_ADDR                       APB1_ADD_OFFSET(0x0800UL)
//#define TIM5_BASE_ADDR                     APB1_ADD_OFFSET(0x0C00UL)
//#define TIM6_BASE_ADDR                     APB1_ADD_OFFSET(0x1000UL)
//#define TIM7_BASE_ADDR                     APB1_ADD_OFFSET(0x1400UL)
//#define TIM12_BASE_ADDR                    APB1_ADD_OFFSET(0x1800UL)
//#define TIM13_BASE_ADDR                    APB1_ADD_OFFSET(0x1C00UL)
//#define TIM14_BASE_ADDR                    APB1_ADD_OFFSET(0x2000UL)

#define RTC_BASE_ADDR                        APB1_ADD_OFFSET(0x2800UL)

#define WWDG_BASE_ADDR                       APB1_ADD_OFFSET(0x2C00UL)
#define IWDG_BASE_ADDR                       APB1_ADD_OFFSET(0x3000UL)

#define PWR_BASE_ADDR                        APB1_ADD_OFFSET(0x7000UL)
#define DAC_BASE_ADDR                        APB1_ADD_OFFSET(0x7400UL)

/**
 * Base addresses of peripherals which are hanging on APB2 bus
 * */

#define GPIOA_BASE_ADDR                      APB2_ADD_OFFSET(0x0800UL)
#define GPIOB_BASE_ADDR                      APB2_ADD_OFFSET(0x0C00UL)
#define GPIOC_BASE_ADDR                      APB2_ADD_OFFSET(0x1000UL)
#define GPIOD_BASE_ADDR                      APB2_ADD_OFFSET(0x1400UL)
#define GPIOE_BASE_ADDR                      APB2_ADD_OFFSET(0x1800UL)
//#define GPIOF_BASE_ADDR                    APB2_ADD_OFFSET(0x1C00UL)
//#define GPIOG_BASE_ADDR                    APB2_ADD_OFFSET(0x2000UL)

#define SPI1_BASE_ADDR                       APB2_ADD_OFFSET(0x3000UL)

#define USART1_BASE_ADDR                     APB2_ADD_OFFSET(0x3800UL)

#define ADC1_BASE_ADDR                       APB2_ADD_OFFSET(0x2400UL)
#define ADC2_BASE_ADDR                       APB2_ADD_OFFSET(0x2800UL)
#define ADC3_BASE_ADDR                       APB2_ADD_OFFSET(0x3C00UL)

#define TIM1_BASE_ADDR                       APB2_ADD_OFFSET(0x2C00UL)
//#define TIM8_BASE_ADDR                     APB2_ADD_OFFSET(0x3400UL)
//#define TIM9_BASE_ADDR                     APB2_ADD_OFFSET(0x4C00UL)
//#define TIM10_BASE_ADDR                    APB2_ADD_OFFSET(0x5000UL)
//#define TIM11_BASE_ADDR                    APB2_ADD_OFFSET(0x5400UL)

#define EXTI_BASE_ADDR                       APB2_ADD_OFFSET(0x0400UL)

#define AFIO_BASE_ADDR                       APB2_ADD_OFFSET(0x0000UL)

/*
 * ----------------------------------------------------------------
 * Peripheral register definition structures
 * ----------------------------------------------------------------
 * */

/**
 * peripheral register definition structure for the GPIO
 * */
typedef struct {
    volatile uint32_t CR[2];     // Port configuration register         CR[0]: CRL (Address offset 0x00), CR[1]: CRH (Address offset 0x04),
    volatile uint32_t IDR;       // Port input data register            Address offset 0x08
    volatile uint32_t ODR;       // Port output data register           Address offset 0x0C
    volatile uint32_t BSRR;      // Port bit set/reset register         Address offset 0x10
    volatile uint32_t BRR;       // Port bit reset register             Address offset 0x14
    volatile uint32_t LCKR;      // Port bit set/reset register         Address offset 0x18
} GPIO_RegDef_t;

/**
 * peripheral register definition structure for the RCC
 * */
typedef struct {
    volatile uint32_t CR;       // Clock control register                   Address offset 0x00
    volatile uint32_t CFGR;     // Clock configuration register             Address offset 0x04
    volatile uint32_t CIR;      // Clock interrupt register                 Address offset 0x08
    volatile uint32_t APB2RSTR; // APB2 peripheral reset register           Address offset 0x0C
    volatile uint32_t APB1RSTR; // APB1 peripheral reset register           Address offset 0x10
    volatile uint32_t AHBENR;   // AHB Peripheral Clock enable register     Address offset 0x14
    volatile uint32_t APB2ENR;  // APB2 peripheral clock enable register    Address offset 0x18
    volatile uint32_t APB1ENR;  // APB1 peripheral clock enable register    Address offset 0x1C
    volatile uint32_t BDCR;     // Backup domain control register           Address offset 0x20
    volatile uint32_t CSR;      // Control/status register                  Address offset 0x24
    volatile uint32_t AHBSTR;   // AHB peripheral clock reset register      Address offset 0x28
    volatile uint32_t CFGR2;    // Clock configuration register2            Address offset 0x2C
} RCC_RegDef_t;

/**
 * peripheral register definition structure for the EXTI
 * */
typedef struct {
    volatile uint32_t IMR;       // Interrupt mask register                 Address offset 0x00
    volatile uint32_t EMR;       // Event mask register                     Address offset 0x04
    volatile uint32_t RTSR;      // Rising trigger selection register       Address offset 0x08
    volatile uint32_t FTSR;      // Falling trigger selection register      Address offset 0x0C
    volatile uint32_t SWIER;     // Software interrupt event register       Address offset 0x10
    volatile uint32_t PR;        // Pending register                        Address offset 0x14
} EXTI_RegDef_t;

/**
 * peripheral register definition structure for the SPI
 * */
typedef struct {
    volatile uint32_t CR1;          // SPI control register 1           Address offset 0x00
    volatile uint32_t CR2;          // SPI control register 2           Address offset 0x04
    volatile uint32_t SR;           // SPI status register              Address offset 0x08
    volatile uint32_t DR;           // SPI data register                Address offset 0x0C
    volatile uint32_t CRCPR;        // SPI CRC polynomial register      Address offset 0x10
    volatile uint32_t RXCRCR;       // SPI RX CRC register              Address offset 0x14
    volatile uint32_t TXCRCR;       // SPI TX CRC register              Address offset 0x18
    volatile uint32_t I2SCFGR;      // SPI_I2S configuration register   Address offset 0x1C
    volatile uint32_t I2SPR;        // SPI_I2S prescaler register       Address offset 0x20
} SPI_RegDef_t;

typedef struct
{
	volatile uint32_t	SR;
	volatile uint32_t	DR;
	volatile uint32_t	BRR;
	volatile uint32_t	CR1;
	volatile uint32_t	CR2;
	volatile uint32_t	CR3;
	volatile uint32_t	GTPR;

}USART_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;				// I2C Control register	1																	Address offset:0x00
	volatile uint32_t CR2;				// I2C Control register 2 																	Address offset:0x04
	volatile uint32_t OAR1;				// I2C Own address register 1																Address offset:0x08
	volatile uint32_t OAR2;				// I2C Own address register 2																Address offset:0x0C
	volatile uint32_t DR;				// I2C Data register																		Address offset:0x10
	volatile uint32_t SR1;				// I2C Status register 1																	Address offset:0x14
	volatile uint32_t SR2;				// I2C Status register 2																	Address offset:0x18
	volatile uint32_t CCR;				// I2C Clock control register																Address offset:0x1C
	volatile uint32_t TRISE;				// I2C TRISE register																		Address offset:0x20
}I2C_RegDef_t;


/**
 * peripheral register definition structure for the AFIO
 * */
typedef struct {
    volatile uint32_t EVCR;      // Event control register                              Address offset 0x00
    volatile uint32_t MAPR;      // AF remap and debug I/O configuration register       Address offset 0x04
    volatile uint32_t EXTICR[4]; // External interrupt configuration register 1-4       Address offset 0x08 - 0x14
    volatile uint32_t MAPR2;     // AF remap and debug I/O configuration register2      Address offset 0x04
} AFIO_RegDef_t;

/*------------------------------------------------------------------------------------------------------------
 * 					Register definition structures inside CORTEX M3
 -------------------------------------------------------------------------------------------------------------*/

typedef struct
{
	volatile uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
	volatile uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
	volatile uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
	volatile  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SYSTICK_RegDef_t;


/*
 * Peripheral definitions
 * */

#define GPIOA ((GPIO_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t *) GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t *) GPIOE_BASE_ADDR)
//#define GPIOF ((GPIO_RegDef_t *) GPIOF_BASE_ADDR)
//#define GPIOG ((GPIO_RegDef_t *) GPIOG_BASE_ADDR)

#define RCC   ((RCC_RegDef_t *) RCC_BASE_ADDR)
#define EXTI  ((EXTI_RegDef_t *) EXTI_BASE_ADDR)
#define AFIO  ((AFIO_RegDef_t *) AFIO_BASE_ADDR)

#define SYSTICK		((SYSTICK_RegDef_t *)SYSTICK_ADDRESS)

#define SPI1  ((SPI_RegDef_t *) SPI1_BASE_ADDR)
#define SPI2  ((SPI_RegDef_t *) SPI2_BASE_ADDR)
#define SPI3  ((SPI_RegDef_t *) SPI3_BASE_ADDR)

#define I2C1			((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASE_ADDR)

#define USART1			(USART_RegDef_t*)USART1_BASE_ADDR
#define USART2			(USART_RegDef_t*)USART2_BASE_ADDR
#define USART3			(USART_RegDef_t*)USART3_BASE_ADDR


/*
 * Clock enable/disable macros for AFIO peripheral
 * */
#define AFIO_PCLK_EN()       (RCC->APB2ENR |= (1 << 0))

#define AFIO_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 0))


/*
 * Clock enable/disable macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_EN()       (RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()       (RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()       (RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()       (RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()       (RCC->APB2ENR |= (1 << 6))

#define GPIOA_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 6))


/*
 * GPIOx peripheral reset macros
 * */
#define GPIOA_REG_RST()       do{(RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2));} while(false)
#define GPIOB_REG_RST()       do{(RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3));} while(false)
#define GPIOC_REG_RST()       do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));} while(false)
#define GPIOD_REG_RST()       do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));} while(false)
#define GPIOE_REG_RST()       do{(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));} while(false)

/**
 * SPIx peripheral reset macros
 * */
#define SPI1_REG_RST()       do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while(false)
#define SPI2_REG_RST()       do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));} while(false)
#define SPI3_REG_RST()       do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));} while(false)


/**
 * returns gpio port code for the given GPIO base address
 * */
#define GPIO_BASEADDR_TO_PORTCODE(x)   ((x == GPIOA) ? 0b0000 : \
                                        (x == GPIOB) ? 0b0001 : \
                                        (x == GPIOC) ? 0b0010 : \
                                        (x == GPIOD) ? 0b0011 : \
                                        (x == GPIOE) ? 0b0100 : 0)

/**
 * @IRQ_NUMBERS
 * IRQ(Interrupt Request) numbers
 * */
#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40

/**
 * @IRQ_PRIORITES
 * IRQ(Interrupt Request) priorities
 * */
#define NVIC_IRQ_PRIORITY_0     0
#define NVIC_IRQ_PRIORITY_1     1
#define NVIC_IRQ_PRIORITY_2     2
#define NVIC_IRQ_PRIORITY_3     3
#define NVIC_IRQ_PRIORITY_4     4
#define NVIC_IRQ_PRIORITY_5     5
#define NVIC_IRQ_PRIORITY_6     6
#define NVIC_IRQ_PRIORITY_7     7
#define NVIC_IRQ_PRIORITY_8     8
#define NVIC_IRQ_PRIORITY_9     9
#define NVIC_IRQ_PRIORITY_10    10
#define NVIC_IRQ_PRIORITY_11    11
#define NVIC_IRQ_PRIORITY_12    12
#define NVIC_IRQ_PRIORITY_13    13
#define NVIC_IRQ_PRIORITY_14    14
#define NVIC_IRQ_PRIORITY_15    15

/*
 * Clock enable/disable macros for I2Cx peripherals
 * */
#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock enable/disable macros for SPIx peripherals
 * */
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << 15))

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock enable/disable macros for USARTx peripherals
 * */
#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()        (RCC->APB1ENR |= (1 << 18))

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 18))

// some generic macros
#define ENABLE                      1
#define DISABLE                     0
#define SET                         ENABLE
#define RESET                       DISABLE
#define GPIO_PIN_SET                SET
#define GPIO_PIN_RESET              RESET
#define FLAG_RESET					RESET
#define FLAG_SET					SET

#define GPIO_OUT_INVERTED           1  // 1 => GPIO_PIN_SET: LOW, GPIO_PIN_RESET: HIGH ; 0 => GPIO_PIN_SET: HIGH, GPIO_PIN_RESET: LOW
#define GPIO_INVERT_IF_NEEDED(x)    (x^GPIO_OUT_INVERTED) // make sure GPIO_PIN_SET is HIGH, GPIO_PIN_RESET is LOW

/*******************************************
 * Bit position definitions of USART peripheral
 *******************************************/
/* USART SR */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NE			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

/* USART CR1 */
#define	USART_CR1_SBK		0
#define	USART_CR1_RWU		1
#define	USART_CR1_RE		2
#define	USART_CR1_TE		3
#define	USART_CR1_IDLEIE	4
#define	USART_CR1_RXNEIE	5
#define	USART_CR1_TCIE		6
#define	USART_CR1_TXEIE		7
#define	USART_CR1_PEIE		8
#define	USART_CR1_PS		9
#define	USART_CR1_PCE		10
#define	USART_CR1_WAKE		11
#define	USART_CR1_M			12
#define	USART_CR1_UE		13


/* USART CR2 */
#define	USART_CR2_ADD		0
#define	USART_CR2_LBDL		5
#define	USART_CR2_LBDIE		6
#define	USART_CR2_LBCL		8
#define	USART_CR2_CPHA		9
#define	USART_CR2_CPOL		10
#define	USART_CR2_CLKEN		11
#define	USART_CR2_STOP		12
#define	USART_CR2_LINEN		14


/* USART CR3 */
#define	USART_CR3_EIE		0
#define	USART_CR3_IREN		1
#define	USART_CR3_IRLP		2
#define	USART_CR3_HDSE		3
#define	USART_CR3_NACK		4
#define	USART_CR3_SCEN		5
#define	USART_CR3_DMAR		6
#define	USART_CR3_DMAT		7
#define	USART_CR3_RTSE		8
#define	USART_CR3_CTSE		9
#define	USART_CR3_CTSIE		10


/*******************************************
 * Bit position definitions of SPI peripheral
 *******************************************/
/* SPI CR1 */
#define SPI_CR1_CPHA            0
#define SPI_CR1_CPOL            1
#define SPI_CR1_MSTR            2
#define SPI_CR1_BR              3
#define SPI_CR1_SPE             6
#define SPI_CR1_LSBFIRST        7
#define SPI_CR1_SSI             8
#define SPI_CR1_SSM             9
#define SPI_CR1_RX_ONLY         10
#define SPI_CR1_DFF             11
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_CRCEN           13
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_BIDIMODE        15

/* SPI CR2 */
#define SPI_CR2_RXDMAEN         0
#define SPI_CR2_TXDMAEN         1
#define SPI_CR2_SSOE            2
#define SPI_CR2_ERRIE           5
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_TXEIE           7

/* SPI SR */
#define SPI_SR_RXNE             0
#define SPI_SR_TXE              1
#define SPI_SR_CHSIDE           2
#define SPI_SR_UDR              3
#define SPI_SR_CRCERR           4
#define SPI_SR_MODF             5
#define SPI_SR_OVR              6
#define SPI_SR_BSY              7



/*******************************************
 * Bit position definitions of I2C peripheral
 *******************************************/
/*
 * I2C CR1
 */
#define I2C_CR1_PE				0
#define I2C_CR1_NONSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_SWRST			15

/*
 * I2C CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10

/*
 * I2C OAR1
 */
#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD71			1
#define I2C_OAR1_ADD98			8
#define I2C_OAR1_ADDMODE		15

/*
 * I2C SR1
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_TIMEOUT			14

/*
 * I2C SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_DUALF			7

/*
 * I2C CCR
 */
#define I2C_SR2_CCR				0
#define I2C_SR2_DUTY			14
#define I2C_SR2_FS				15

/*
 * Configure the marcos for Systemtick
 */

#define SYSTICK_CSR_ENABLE			0
#define SYSTICK_CSR_TICKINT			1
#define SYSTICK_CSR_CLKSOURCE		2
#define SYSTICK_CSR_COUNTFLAG		16

//#include "stm32f103xx_gpio_driver.h"
//#include "stm32f103xx_spi_driver.h"
//#include "stm32f103xx_i2c_driver.h"
//#include "stm32f103xx_usart_driver.h"
#include "stm32f103xx_rcc.h"

#endif /* INC_STM32F103XX_H_ */
