/*
 * 015_uart_tx.c
 *
 *  Created on: 6 mai 2021
 *      Author: jeanm
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart2_handle;

void debouncing(void)  //~200ms delay when clock is 16Mhz
{
		uint8_t count=0;
	do
	{

		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			count++;
		}
		else if (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)&&(count>0))
		{
			count--;
		}

	}
	while (count<100);

}



void USART2_Init(void)
{

	memset (&usart2_handle, 0, sizeof(usart2_handle));
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_2;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;
	memset (&usart_gpios, 0, sizeof(usart_gpios));
	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);


}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;
	memset (&GPIOBtn, 0, sizeof(GPIOBtn));

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
	memset (&GpioLed, 0, sizeof(GpioLed));
	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}


int main(void)
{

	GPIO_ButtonInit();

	USART2_GPIOInit();

    USART2_Init();

    USART_PeripheralControl(USART2,ENABLE);

    while(1)
    {
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		debouncing();

		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));
		while(  GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );
    }

	return 0;
}
