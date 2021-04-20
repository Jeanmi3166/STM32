/*
 * 006spi_TX.c
 *
 *  Created on: 13 avr. 2021
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
	memset(&SPIPins,0,sizeof(SPIPins));

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
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	//NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);

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
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN; // Software slave management enabled for NSS pin
	SPI_Init(&SPI2handle);
}

int main(void)
{


	char user_data[]="Hello world";

	// Function to configure the GPIO used as SPI2 bus
	SPI2_GPIOInits();

	//Function to configure the SPI2 peripheral
	SPI2_Inits();

	//SSI config to 1 (pin pulled to 1 to indicate bus idle when multiple masters) when SSM is 1 (SW control of SSM)
	SPI_SSIConfig(SPI2,ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//Function to send datas in user_data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//lets confirm the SPI is not busy
	while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Desable SPI peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
	return 0;
}

