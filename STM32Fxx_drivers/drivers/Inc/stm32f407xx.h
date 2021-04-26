/*
 * stm32f407xx.h
 *
 *  Created on: Mar 22, 2021
 *      Author: jeanm
 */

#include<stddef.h>
#include<stdint.h>


#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __vo volatile
#define __weak __attribute__((weak))

/******************Specific Register Details**********************/
//NVIC ISERx Register Address
#define NVIC_ISER0					((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t *)0xE000E10C)

//NVIC ICERx Register Address
#define NVIC_ICER0					((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t *)0xE000E18C)

#define NVIC_PR_BASE_ADDR			((__vo uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4 //Specific to the STM32F407

// Base Adresses of Flash and RAM
#define FLASH_BASE				0x08000000U
#define SRAM1_BASE				0x20000000U
#define SRAM2_BASE				0x2001C000U
#define ROM_BASE				0x1FFF0000U
#define SRAM					SRAM1_BASE

// AHB(1,2) and APB(1,2) Bus Peripheral Base Addresses
#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U

// AHB1 Bus Peripheral Base Addresses
#define GPIOA_BASE				(AHB1PERIPH_BASE + 0x000U)
#define GPIOB_BASE				(AHB1PERIPH_BASE + 0x400U)
#define GPIOC_BASE				(AHB1PERIPH_BASE + 0x800U)
#define GPIOD_BASE				(AHB1PERIPH_BASE + 0xC00U)
#define GPIOE_BASE				(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASE				(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASE				(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASE				(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASE				(AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASE				(AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASE				(AHB1PERIPH_BASE + 0x2800U)
#define RCC_BASE				(AHB1PERIPH_BASE + 0x3800U)

// APB1 Bus Peripheral Base Addresses
#define I2C1_BASE				(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASE				(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASE				(APB1PERIPH_BASE + 0x5C00U)

#define USART2_BASE				(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASE				(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASE				(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASE				(APB1PERIPH_BASE + 0x5000U)

#define SPI2_BASE				(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASE				(APB1PERIPH_BASE + 0x3C00U)

// APB2 Bus Peripheral Base Addresses
#define SPI1_BASE				(APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASE				(APB2PERIPH_BASE + 0x3400U)
#define USART1_BASE				(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASE				(APB2PERIPH_BASE + 0x1400U)
#define EXTI_BASE				(APB2PERIPH_BASE + 0x3C00U)
#define SYSCFG_BASE				(APB2PERIPH_BASE + 0x3800U)


/*********************Structures for peripheral registers******************/

// GPIO(A-K)

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASE)
#define GPIOJ ((GPIO_RegDef_t*)GPIOJ_BASE)
#define GPIOK ((GPIO_RegDef_t*)GPIOK_BASE)

// SPI(1-3)
#define SPI1 	((SPI_RegDef_t*)SPI1_BASE)
#define SPI2  	((SPI_RegDef_t*)SPI2_BASE)
#define SPI3 	((SPI_RegDef_t*)SPI3_BASE)
#define SPI4 	((SPI_RegDef_t*)SPI4_BASE)

#define RCC ((RCC_RegDef_t*)RCC_BASE)

#define EXTI ((EXTI_RegDef_t*)EXTI_BASE)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASE)


/*****************GPIO structure used by all peripheral: GPIO,SPI,I²C...******************************/
typedef struct
{
	__vo  uint32_t 	MODER; 		//GPIO port mode register
	__vo  uint32_t 	OTYPER;		//GPIO port output type register
	__vo  uint32_t 	OSPEEDR;	//GPIO port output speed register
	__vo  uint32_t 	PUPDR;		//GPIO port pull-up/pull-down register
	__vo  uint32_t 	IDR;		//GPIO port input data register
	__vo  uint32_t 	ODR;		//GPIO port output data register
	__vo  uint32_t 	BSRR;		//GPIO port bit set/reset register
	__vo  uint32_t 	LCKR;		//GPIO port configuration lock register
	__vo  uint32_t	AFR[2]; 	//GPIO alternate function low register -> ARF[0]=ARFL (LSB) and ARF[1]=ARFH (MSB).

} GPIO_RegDef_t;

/*****************SPI_structure***************************************/
typedef struct
{
		uint32_t 	CR1; 			//SPI_CR1 control register 1
		uint32_t 	CR2;			//SPI_CR2 port output type register
		uint32_t 	SR;	 			//SPI_SR status register
		uint32_t 	DR;				//SPI_DR data register
		uint32_t 	CRCPR;			//SPI_CRCPR CRC polynomial register
		uint32_t 	RXCRCR;			//SPI TX CRC register
		uint32_t 	TXCRCR;			//SPI TX CRC register
		uint32_t 	I2SCFGR;		//SPI_I2S configuration register
		uint32_t 	I2SPR;			//SPI_I2S prescaler register
} SPI_RegDef_t;

typedef struct
{
	__vo uint32_t 	CR; 			//RCC clock control register
	__vo uint32_t 	PLLCFGR;		//RCC PLL configuration register
	__vo uint32_t 	CFGR;			//RCC clock configuration register
	__vo uint32_t 	CIR;			//RCC clock interrupt register
	__vo uint32_t 	AHB1RSTR;		//RCC AHB1 peripheral reset register
	__vo uint32_t 	AHB2RSTR;		//RCC AHB2 peripheral reset register
	__vo uint32_t 	AHB3RSTR;		//RCC AHB3 peripheral reset register
		 uint32_t 	RESERVED0;		//Reserved 0x1C
	__vo uint32_t 	APB1RSTR;		//RCC APB1 peripheral reset register
	__vo uint32_t	APB2RSTR; 		//RCC APB2 peripheral reset register
	 	 uint32_t 	RESERVED1[2];	//Reserved 0x28-0x2C
	__vo uint32_t	AHB1ENR;		//RCC AHB1 peripheral clock enable register
	__vo uint32_t	AHB2ENR;		//RCC AHB2 peripheral clock enable register
	__vo uint32_t	AHB3ENR;		//RCC AHB3 peripheral clock enable register
		 uint32_t 	RESERVED2;		//Reserved 0x3C
	__vo uint32_t	APB1ENR;		//RCC APB1 peripheral clock enable register
	__vo uint32_t	APB2ENR;		//RCC APB2 peripheral clock enable register
		uint32_t 	RESERVED3[2];	//Reserved 0x48-0x4C
	__vo uint32_t	AHB1LPENR;		//RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t	AHB2LPENR;		//RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t	AHB3LPENR;		//RCC AHB3 peripheral clock enable in low power mode register
		 uint32_t 	RESERVED4;		//Reserved 0x5C
	__vo uint32_t	APB1LPENR;		//RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t	APB2LPENR;		//RCC APB2 peripheral clock enabled in low power mode register
		 uint32_t 	RESERVED5[2];	//Reserved 0x68-0x6C
	__vo uint32_t	BDCR;			//RCC Backup domain control register
	__vo uint32_t	CSR;			//RCC clock control & status register
		 uint32_t 	RESERVED6[2];	//Reserved 0x78-0x7C
	__vo uint32_t	SSCGR;			//RCC spread spectrum clock generation register
	__vo uint32_t	PLLI2SCFGR;		//RCC PLLI2S configuration register
	__vo uint32_t	PLLSAICFGR;		//RCC PLLSAICFGR configuration register
	__vo uint32_t	DCKCFGR;		//RCC DCKCFGR configuration register
} RCC_RegDef_t;


// EXTI structure
typedef struct
{
	__vo  uint32_t 	IMR; 		//EXTI_IMR  Interrupt mask register
	__vo  uint32_t 	EMR;		//EXTI_EMR Event mask register
	__vo  uint32_t 	RTSR;		//EXTI_RTSR Rising trigger selection register
	__vo  uint32_t 	FTSR;		//EXTI_FTSR Falling trigger selection register
	__vo  uint32_t 	SWIER;		//EXTI_SWIER Software interrupt event register
	__vo  uint32_t 	PR;			//EXTI_PR Pending register
} EXTI_RegDef_t;


// SYSCFG structure
typedef struct
{
	__vo  uint32_t 	MEMRMP; 		//SYSCFG_MEMRMP  memory remap register
	__vo  uint32_t 	PMC;			//SYSCFG_PMC peripheral mode configuration register
	__vo  uint32_t 	EXTICR[4];		//SYSCFG_EXTICR1 external interrupt configuration register 1 to 4
		  uint32_t 	RESERVED1[2];	//Not used
	__vo  uint32_t 	CMPCR;			//SYSCFG_CMPCR Compensation cell control register
		  uint32_t 	RESERVED2[2];	//Not used
	__vo  uint32_t 	CFGR;			//SYSCFG_CFGR ??? Not in Reference Manual???
} SYSCFG_RegDef_t;

/*******************Clock Enable for GPIO(A-K)******************************/
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |=(1<<8))
#define GPIOJ_PCLK_EN() (RCC->AHB1ENR |=(1<<9))
#define GPIOK_PCLK_EN() (RCC->AHB1ENR |=(1<<10))


/******************IRQ Number************************************************/
#define IRQ_NO_WWDG				0
#define IRQ_NO_PVD				1
#define IRQ_NO_TAMP_STAMP		2
#define IRQ_NO_RTC_WKUP			3
#define IRQ_NO_FLASH			4
#define IRQ_NO_RCC				5

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define IRQ_NO_DMA1_Stream0		11
#define IRQ_NO_DMA1_Stream1		12
#define IRQ_NO_DMA1_Stream2		13
#define IRQ_NO_DMA1_Stream3		14
#define IRQ_NO_DMA1_Stream4		15
#define IRQ_NO_DMA1_Stream5		16
#define IRQ_NO_DMA1_Stream6		17

#define IRQ_NO_ADC				18

#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73

#define IRQ_NO_SPI1 			35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

#define IRQ_NO_USART1 			37
#define IRQ_NO_USART2 			38
#define IRQ_NO_USART3 			39

#define IRQ_NO_UART4 			52
#define IRQ_NO_UART5 			53

/******************IRQ Priority************************************************/

#define NVIC_IRQ_PIO0			0
#define NVIC_IRQ_PIO1			1
#define NVIC_IRQ_PIO2			2
#define NVIC_IRQ_PIO3			3
#define NVIC_IRQ_PIO4			4
#define NVIC_IRQ_PIO5			5
#define NVIC_IRQ_PIO6			6
#define NVIC_IRQ_PIO7			7
#define NVIC_IRQ_PIO8			8
#define NVIC_IRQ_PIO9			9
#define NVIC_IRQ_PIO10			10
#define NVIC_IRQ_PIO11			11
#define NVIC_IRQ_PIO12			12
#define NVIC_IRQ_PIO13			13
#define NVIC_IRQ_PIO14			14
#define NVIC_IRQ_PIO15			15


/*******************External clock Enable,Internal Diasble*******************/
#define EXT_CLK_EN() 	do {(RCC->AHB1RSTR  |=(1<<16)); (RCC->CR  &=~(1<<0)); }while(0)

/*******************HPRE Clock Prescaler*******************/
#define CLOCK_DIV_2() 	(RCC->CFGR = 0x00000080)

/*******************Clock Disable for GPIO(A-I)******************************/
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &=~(1<<7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &=~(1<<8))
#define GPIOJ_PCLK_DI() (RCC->AHB1ENR &=~(1<<9))
#define GPIOK_PCLK_DI() (RCC->AHB1ENR &=~(1<<10))

/****************** GPIO(A-I) port reset******************************/
//Reset then Release GPIO ports
#define GPIOA_RST() 		do {(RCC->AHB1RSTR  |=(1<<0)); (RCC->AHB1RSTR  &=~(1<<0)); }while(0)  //trick to execute 2 instructions in 1 marco
#define GPIOB_RST() 		do {(RCC->AHB1RSTR  |=(1<<1)); (RCC->AHB1RSTR  &=~(1<<1)); }while(0)
#define GPIOC_RST() 		do {(RCC->AHB1RSTR  |=(1<<2)); (RCC->AHB1RSTR  &=~(1<<2)); }while(0)
#define GPIOD_RST() 		do {(RCC->AHB1RSTR  |=(1<<3)); (RCC->AHB1RSTR  &=~(1<<3)); }while(0)
#define GPIOE_RST() 		do {(RCC->AHB1RSTR  |=(1<<4)); (RCC->AHB1RSTR  &=~(1<<4)); }while(0)
#define GPIOF_RST() 		do {(RCC->AHB1RSTR  |=(1<<5)); (RCC->AHB1RSTR  &=~(1<<5)); }while(0)
#define GPIOG_RST() 		do {(RCC->AHB1RSTR  |=(1<<6)); (RCC->AHB1RSTR  &=~(1<<6)); }while(0)
#define GPIOH_RST() 		do {(RCC->AHB1RSTR  |=(1<<7)); (RCC->AHB1RSTR  &=~(1<<7)); }while(0)
#define GPIOI_RST() 		do {(RCC->AHB1RSTR  |=(1<<8)); (RCC->AHB1RSTR  &=~(1<<8)); }while(0)


# define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)?0:\
									 (x == GPIOB)?1:\
									 (x == GPIOC)?2:\
									 (x == GPIOD)?3:\
									 (x == GPIOE)?4:\
									 (x == GPIOF)?5:\
									 (x == GPIOG)?6:\
									 (x == GPIOH)?7:\
									 (x == GPIOH)?8:0)


/*******************Clock Enable for I2Cx (1-3)******************************/
#define I2C1_PCLK_EN() (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |=(1<<23))

/*******************Clock Disable for I2Cx (1-3)******************************/
#define I2C1_PCLK_DI() (RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &=~(1<<23))

/*******************Clock Enable for UART (2,3) and USART(4,5)***************/
#define USART2_PCLK_EN() (RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN() (RCC->APB1ENR  |=(1<<19))
#define UART5_PCLK_EN() (RCC->APB1ENR  |=(1<<20))

/*******************Clock Disable for UART (2,3) and USART(4,5)***************/
#define USART2_PCLK_DI() (RCC->APB1ENR &=~(1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &=~(1<<18))
#define UART4_PCLK_DI() (RCC->APB1ENR  &=~(1<<19))
#define UART5_PCLK_DI() (RCC->APB1ENR  &=~(1<<20))

/*******************Clock Enable for SPI (1-4)***************/
#define SPI1_PCLK_EN() (RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |=(1<<15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |=(1<<13))

/*******************Clock Disable for SPI (1-4)***************/
#define SPI1_PCLK_DI() (RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &=~(1<<15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &=~(1<<13))

/*******************Clock Enable for SYSCFG***************/
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |=(1<<14))

/*******************Clock Disable for SYSCFG***************/
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &=~(1<<14))

/*******************Some Generic Macros********************/
#define ENABLE 			1
#define DISABLE 		0

#define SET 			ENABLE
#define RESET 			DISABLE

#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

#define FLAG_SET		SET
#define FLAG_RESET		RESET

/*******************Bit position of SPI Peripheral*************************/
//CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

//CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

//SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

#include "stm32f407xx_gpio_drivers.h"
#include "stm32f407xx_spi_drivers.h"


#endif /* INC_STM32F407XX_H_ */
