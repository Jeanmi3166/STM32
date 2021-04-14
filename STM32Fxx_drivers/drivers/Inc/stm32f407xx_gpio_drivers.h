/*
 * stm32f407xx_gpio_drivers.h
 *
 *  Created on: 23 mars 2021
 *      Author: jeanm
 */

#ifndef SRC_STM32F407XX_GPIO_DRIVERS_H_
#define SRC_STM32F407XX_GPIO_DRIVERS_H_

#include "stm32f407xx.h"



typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			// See details at "GPIO available modes" below
	uint8_t GPIO_PinSpeed;			// See details at "GPIO Speed available modes" below
	uint8_t GPIO_PinPuPdControl;	// See details at "GPIO Out Push-Pull or Open Drain modes" below
	uint8_t GPIO_PinOPType;			// GPIO Out Push-Pull or Open Drain modes below
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;				//Pointer to hold the base address of the GPIO Peripheral
	GPIO_PinConfig_t GPIO_PinConfig;	//Variable to hold GPIO pin configuration settings
}GPIO_Handle_t;

//GPIO Pin number
# define GPIO_PIN_NO_0			0
# define GPIO_PIN_NO_1			1
# define GPIO_PIN_NO_2			2
# define GPIO_PIN_NO_3			3
# define GPIO_PIN_NO_4			4
# define GPIO_PIN_NO_5			5
# define GPIO_PIN_NO_6			6
# define GPIO_PIN_NO_7			7
# define GPIO_PIN_NO_8			8
# define GPIO_PIN_NO_9			9
# define GPIO_PIN_NO_10			10
# define GPIO_PIN_NO_11			11
# define GPIO_PIN_NO_12			12
# define GPIO_PIN_NO_13			13
# define GPIO_PIN_NO_14			14
# define GPIO_PIN_NO_15			15

//GPIO available modes
# define GPIO_MODE_IN 			0
# define GPIO_MODE_OUT 			1
# define GPIO_MODE_ALTFN 		2
# define GPIO_MODE_ANALOG 		3

//GPIO IRQ available modes
# define GPIO_MODE_IT_FT 		4
# define GPIO_MODE_IT_RT 		5
# define GPIO_MODE_IT_RFT 		6

//GPIO Out Push-Pull or Open Drain modes
# define GPIO_OP_TYPE_PP		0
# define GPIO_OP_TYPE_OD		1

//GPIO Speed available modes
# define GPIO_SPEED_LOW			0
# define GPIO_SPEED_MEDIUM		1
# define GPIO_SPEED_FAST		2
# define GPIO_SPEED_HIGH		3

//GPIO Pull Up en Pull Down modes
# define GPIO_NO_PUPD			0
# define GPIO_PIN_PU			1
# define GPIO_PIN_PD			2


/******************************API prototype Definition****************************/
//Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) ;
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Read and Write on GPIO
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value );
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Handling
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

#endif /* SRC_STM32F407XX_GPIO_DRIVERS_H_ */
