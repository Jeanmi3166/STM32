/*
 * 008.1_spi_cmd_handling.c
 *
 *  Created on: 21 avr. 2021
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

#define READ		 			1
#define WRITE		 			0

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
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //2Mhz Clock
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



void SPI_ENorDE(uint8_t EnOrDe)
{
	if(EnOrDe == ENABLE)
	{
		//Enable SPI peripheral
		SPI_PeripheralControl(SPI2, ENABLE);
	}else
	{
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		//Desable SPI peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}
}





uint8_t SPI_verifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		return 1;
	}
	return 0;
}

void debouncing(void)  //~200ms delay when clock is 16Mhz
{
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

void delay(uint32_t del)  //~200ms delay when clock is 16Mhz
{
	for(uint32_t i = 0 ; i < del; i ++);
}


void Send_SPI_Commands(uint8_t* args, uint8_t arg_number, uint8_t read1_write0, uint8_t* ReadBuff, uint8_t read_nb, uint32_t del )
{
	uint8_t dummy_write = 0xAA;
	uint8_t dummy_read = 0;
	uint8_t ackbyte;


			//Send command
			SPI_SendData(SPI2, (args+0), 1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//Send some dummy bits(1bute) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			//Read the ack byte received
			SPI_ReceiveData(SPI2, &ackbyte, 1);

			if (SPI_verifyResponse(ackbyte))
			{
				//send arguments
				SPI_SendData(SPI2, (args+1), (arg_number-1)); // "+1" and "-1" command already sent

			}
		if (read1_write0 )
		{
				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2, &dummy_read, 1);

				delay(del);

				//Send some dummy bits(1bute) to fetch the response from the slave
				SPI_SendData(SPI2, &dummy_write, 1);

				//Read the byte received
				SPI_ReceiveData(SPI2, ReadBuff, read_nb);
		}
}


int main(void)
{

	uint8_t args[100];

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

				//1.CMD_LEC_CTRL		<pin no(1)>			<value(1)>
		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		debouncing();
		SPI_ENorDE(ENABLE); //enable the SPI2 peripheral
		*(args+0) = COMMAND_LED_CTRL;
		*(args+1) = LED_PIN;
		*(args+2) = LED_ON;
		Send_SPI_Commands(args, 3, WRITE, NULL, 0, 0);
		SPI_ENorDE(DISABLE); //disable the SPI2 peripheral

		//2.LED_STATE		<pin no(1)>
		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		debouncing();
		SPI_ENorDE(ENABLE);//enable the SPI2 peripheral
		*(args+0) = COMMAND_LED_READ;
		*(args+1) = LED_PIN ;
		Send_SPI_Commands(args, 2, READ, NULL, 0, 100);
		SPI_ENorDE(DISABLE);//disable the SPI2 peripheral

		//3.ANALOG_PIN_READING		<pin no(1)>
		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		debouncing();
		SPI_ENorDE(ENABLE);//enable the SPI2 peripheral
		*(args+0) = COMMAND_SENSOR_READ;
		*(args+1) = ANALOG_PIN0 ;
		Send_SPI_Commands(args, 2, READ, NULL, 1, 500);
		SPI_ENorDE(DISABLE);//disable the SPI2 peripheral

		//4.PRINT		<length> <message>
  		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		debouncing();
		SPI_ENorDE(ENABLE);//enable the SPI2 peripheral
		char strings[] = "Hello, I am Jean-Michel and I am glad to meet you!";
		uint8_t length = strlen (strings);

		*(args+0) = COMMAND_PRINT;
		*(args+1) = length ;
		for(uint8_t i=0; i<length; i++)
		{
			*(args+i+2) =*(strings+i);
		}
		Send_SPI_Commands((uint8_t*)args, (length+2), WRITE, NULL, 0, 0);//+2 to send command + frame length
		SPI_ENorDE(DISABLE);//disable the SPI2 peripheral

/*		//5.READ	<message>   Does not work!!!!!!!!!!!!
  		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		debouncing();
		SPI_ENorDE(ENABLE);//enable the SPI2 peripheral
		uint8_t  ReadBuff[10];
		*(args+0) = COMMAND_ID_READ;
		Send_SPI_Commands(args, 1, READ, ReadBuff, 10, 100);
		SPI_ENorDE(DISABLE);//disable the SPI2 peripheral
*/
	}
	return 0;
}



