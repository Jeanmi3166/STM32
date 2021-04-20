/*
 * 008spi_cmd_handling.c
 *
 *  Created on: 15 avr. 2021
 *      Author: jeanm
 */

#include "stm32f407xx.h"
#include <string.h>

/*
 * PB14=MISO
 * PB15=MOSI
 * PB13=SCLK
 * PB12=NSS
 * ALT=5
 */

//commandes codes For ARDUINO
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

//ARDUINO analog pins
#define ANALOG_PIN0 			0
#define ANALOG_PIN1 			1
#define ANALOG_PIN2 			2
#define ANALOG_PIN3 			3
#define ANALOG_PIN4 			4

#define LED_PIN		 			9


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
//	memset(&SPIPins,0,sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode= SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled for NSS pin
	SPI_Init(&SPI2handle);
}


void GPIO_ButtonInit(void) //Button configuration
{
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn); // PA0 in Button config
}

uint8_t SPI_verifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		return 1;
	}
	return 0;
}

void delay(void)  //~200ms delay when clock is 16Mhz
{
//	for(uint32_t i = 0 ; i < 500000/2; i ++);
		uint8_t count=0;
	do
	{

		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			count++;
		}
		else if (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)&(count>0))
		{
			count--;
		}

	}
	while (count<100);

}



int main(void)
{
	uint8_t dummy_write = 0x55;
	uint8_t dummy_read = 0xAA;
	GPIO_ButtonInit();

	// Function to configure the GPIO used as SPI2 bus
	SPI2_GPIOInits();

	//Function to configure the SPI2 peripheral
	SPI2_Inits();

	/*making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by Hardware like this:
	 * When SPE=1, NSS keep to low ->SPI bus busy
	 * When SPE=0, NSS keep to high->SPI bus idle
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1)
	{
		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1.CMD_LEC_CTRL		<pin no(1)>			<value(1)>
		uint8_t comndcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;

		//Send command
		SPI_SendData(SPI2, &comndcode, 1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some dummy bits(1bute) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_verifyResponse(ackbyte))
		{
			//send arguments
			uint8_t args[2] ={0,0};
			args[0]=LED_PIN;
			args[1]=LED_ON;
			SPI_SendData(SPI2, args, 2);
		}

		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Desable SPI peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}

