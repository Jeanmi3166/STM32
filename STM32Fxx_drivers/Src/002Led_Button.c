/*
 * 002Led_Button.c
 *
 *  Created on: 7 avr. 2021
 *      Author: jeanm
 */

#include "stm32f407xx.h"
#define FILTER_ADJ		10

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2; i ++);
}



int main(void)
{
	EXT_CLK_EN(); //External clock Enable

	/************Peripheral Clock Config*******************/
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Handle_t GpioLed, GpioBtn;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;

	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);  // PD12 out Led config


	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn); // PA0 in Button config


	while (1)

		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))

		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

	return 0;
}

