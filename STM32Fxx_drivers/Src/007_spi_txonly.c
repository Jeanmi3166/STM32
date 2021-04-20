/*
 * 007_spi_txonly.c
 *
 *  Created on: 14 avr. 2021
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



void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
//	memset(&SPIPins,0,sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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
	SPI2handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode= SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
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
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn); // PA0 in Button config
}


void delay(void)  //~200ms delay when clock is 16Mhz
{
	for(uint32_t i = 0 ; i < 500000/2; i ++);
}



int main(void)
{

	char user_data[]="This Program is used to test the SPI communication between STM32F407 and ARDUINO UNO at 4Mhz";

	GPIO_ButtonInit();

	// Function to configure the GPIO used as SPI2 bus
	SPI2_GPIOInits();

	//Function to configure the SPI2 peripheral
	SPI2_Inits();

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	/*making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by Hardware like this:
	 * When SPE=1, NSS keep to low ->SPI bus busy
	 * When SPE=0, NSS keep to high->SPI bus idle
	 */
	SPI_SSOEConfig(SPI2, ENABLE);


	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			{
				delay();

				SPI_PeripheralControl(SPI2, ENABLE);

				//First send length frame
				uint8_t data_len = strlen(user_data);
				SPI_SendData(SPI2, &data_len, 1);

				//Function to send datas in user_data
				SPI_SendData(SPI2, (uint8_t*)user_data, data_len);

				//lets confirm the SPI is not busy
				while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

				//Desable SPI peripheral
				SPI_PeripheralControl(SPI2, DISABLE);
			}
	}


	return 0;
}



