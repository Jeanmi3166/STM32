/*
 * stm32f407xx_spi_drivers.h
 *
 *  Created on: 12 avr. 2021
 *      Author: jeanm
 */

#ifndef INC_STM32F407XX_SPI_DRIVERS_H_
#define INC_STM32F407XX_SPI_DRIVERS_H_
#include "stm32f407xx.h"


typedef struct
{
			uint16_t 	SPI_DeviceMode; // Master/Slaves see @ SPI_DEVICE_MODE
			uint8_t 	SPI_BusConfig;	// Full Duplex, Simplex... see @ SPI_BUS_CONFIG
			uint8_t 	SPI_SclkSpeed;	// See @ SPI_CLK_SPEED
			uint8_t 	SPI_DFF;		//8 bits (default) or 16 bit format frame see @SPI_DFF
			uint8_t 	SPI_CPOL;		//CPOL=0(default) or CPOL=1  see @SPI_CPOL
			uint8_t 	SPI_CPHA;		//CPHA=0(default) or CPHA=1  see @SPI_CPHA
			uint8_t 	SPI_SSM;		//HW or SW management of SS pin see @SPI_SSM

} SPI_Config_t;



typedef struct
{
	SPI_RegDef_t *pSPIx;				//Pointer to hold the base address of the SPI Peripheral
	SPI_Config_t SPI_PinConfig;			//Variable to hold SPI pin configuration settings
}SPI_Handle_t;

//@ SPI_Device mode
# define SPI_DEVICE_MODE_MASTER				1
# define SPI_DEVICE_MODE_SLAVE				0

//@ SPI_BUS_CONFIG
#define SPI_BUS_CONFIG_FD               	1
#define SPI_BUS_CONFIG_HD               	2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   	3

//@ SPI_CLK_SPEED
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7

//@SPI_DFF
#define SPI_DFF_8BITS 						0
#define SPI_DFF_16BITS  					1

// @CPOL
#define SPI_CPOL_HIGH 						1
#define SPI_CPOL_LOW 						0

//@CPHA
#define SPI_CPHA_HIGH 						1
#define SPI_CPHA_LOW 						0

//@SPI_SSM
#define SPI_SSM_EN     						1
#define SPI_SSM_DI    						0
//SPI position of flags
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

/*********************************************************************************/
//								APIs Supported by SPI driver
/*********************************************************************************/

//Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//Init and De-Init
void SPI_Init(SPI_Handle_t *pSPIHandle) ;
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//SSI config to 1 (pin pulled to 1 to indicate bus idle when multiple masters) when SSM is 1 (SW control of SSM)
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//SSOE Configuration to have Hardware management of SSI pin
void SPI_SSOEConfig(SPI_RegDef_t * pSPIx, uint8_t EnOrDi);

//Data Sent and received
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len );
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer,uint32_t Len );

//IRQ Handling
void SPI_IRQHandling(SPI_Handle_t * pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

//Peripheral Enable
void SPI_PeripheralControl(SPI_RegDef_t * pSPIx, uint8_t EnOrDi);

//Peripheral Get status
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);


#endif /* INC_STM32F407XX_SPI_DRIVERS_H_ */
