/*
 * stm32f407xx_gpio_drivers.c
 *
 *  Created on: 23 mars 2021
 *      Author: jeanm
 */


#include "stm32f407xx_gpio_drivers.h"


/***************Peripheral Clock Setup**************************

 fn:		GPIO_PeriClockControl
 brief:		This function enables or disables peripheral  clock for the given GPIO port
 param[in]:	Base address of the GPIO Peripheral
 param[in]:	Enable or DISABLE macros
 return:	none
 note:		none
  */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}
		else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_DI();
		}
	}
}


/***************Peripheral Init and De-init**************************

 fn:		GPIO_Init & GPIO_DeInit
 brief:		This function enables or disables peripheral GPIO function (SET & RESET the function)
 param[in]:	Base address of the GPIO Peripheral
 param[in]:	none
 return:	none
 note:		none
  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
uint32_t temp = 0;

//Enable the peripheral clock
GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

//1. configure the mode of GPIO pin
if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG)
{
	// Non Interrupt mode
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->MODER &= ~(0x03<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing before OR below
	pGPIOHandle->pGPIOx->MODER |= temp;
}
else // Interrupt mode
{
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) //1.0 configure the Falling edge Trigger Interruption
	{
		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Clear the corresponding RTSR bit
		EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) //1.1 configure the Rising edge Trigger Interruption
	{
		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Clear the corresponding FTSR bit
		EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) //1.2 configure the Rising/Falling edge Trigger Interruption
	{
		//Enable the corresponding FTSR bit
		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Enable the corresponding RTSR bit
		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2.0 Configure the GPIO port selection in SYSCFG_EXTICR
	uint8_t temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
	uint8_t temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	SYSCFG-> EXTICR[temp1]= portcode << (temp2 *4 );


	//3.0 Enable the EXIT interupt delivrey using IMR
	EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
}

//2. configure the speed
temp=0;
temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing before OR below
pGPIOHandle->pGPIOx->OSPEEDR |=  temp;

//3. configure the Pu and Pd
temp=0;
temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
pGPIOHandle->pGPIOx->PUPDR &= ~(0x03<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing before OR below
pGPIOHandle->pGPIOx->PUPDR |=  temp;

//4. configure the Optype
temp=0;
temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
pGPIOHandle->pGPIOx->OTYPER &= ~(0x01<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing before OR below
pGPIOHandle->pGPIOx->OTYPER |=  temp;

//5. configure the Alt functions
uint8_t temp1, temp2 = 0;

if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
{
	temp1 =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
	temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8;
	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4 * temp2)); //Clearing before OR below
	pGPIOHandle->pGPIOx->AFR[temp1]|= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4 * temp2);
}



}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) // Allows to reset GPIO port then release it
{

			if (pGPIOx == GPIOA)
			{
				GPIOA_RST();
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_RST();
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_RST();
			}
			else if (pGPIOx == GPIOD)
			{
				GPIOD_RST();
			}
			else if (pGPIOx == GPIOE)
			{
				GPIOE_RST();
			}
			else if (pGPIOx == GPIOF)
			{
				GPIOF_RST();
			}
			else if (pGPIOx == GPIOG)
			{
				GPIOG_RST();
			}
			else if (pGPIOx == GPIOH)
			{
				GPIOH_RST();
			}
			else if (pGPIOx == GPIOI)
			{
				GPIOI_RST();
			}
}


/***************Read on GPIO Port and Pin**************************

 fn:		GPIO_ReadFromInputPin & GPIO_ReadFromInputPort
 brief:		This function Read Pin & Port level SET or RESET
 param[in]:	Base address of the GPIO Peripheral
 param[in]:	none
 return:	uint8_t value for Pin
 return:	uint16_t value for Port
 note:		none
  */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >>  PinNumber) & 0x00000001);  //Extract and masq Pin value
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/***************Write, Toggle on GPIO Port and Pin**************************

 fn:		GPIO_WriteToOutputPin & GPIO_WriteToOutputPort & GPIO_ToggleOutputPin
 brief:		This function Write Pin or Port level SET or RESET or Toggle Pin
 param[in]:	Base address of the GPIO Peripheral
 param[in]:	none
 return:	none
 note:		none
  */


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if ( Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |=(1<< PinNumber);
	}
	else
	{
		pGPIOx->ODR &=~(1<< PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value )
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<< PinNumber);
}


/***************IRQ Handling**************************

 fn:		GPIO_IRQConfig & GPIO_IRQHandling
 brief:		This function handle IRQ on GPIO Ports
 param[in]:	Base address of the GPIO Peripheral
 param[in]:	none
 return:	none
 note:		none
  */


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (IRQNumber <=31) //up to 31
		{
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}

		else if(IRQNumber > 31 && IRQNumber <64)//32 to 63
		{
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}
		else if (IRQNumber >= 64 && IRQNumber <96) //64 to 95
		{
			// Program ISER2 register
			*NVIC_ISER2|= (1 << IRQNumber%64);
		}

	}
	else
	{
		if (IRQNumber <=31) //up to 31
		{
			// Program ICER0 register
			*NVIC_ICER0|= (1 << IRQNumber);
		}

		else if(IRQNumber > 31 && IRQNumber <64)//32 to 63
		{
			// Program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber%32);
		}
		else if (IRQNumber >= 64 && IRQNumber <96) //64 to 95
		{
			// Program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber%64);
		}
	}
}



void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	// Find out the IPR register
	uint8_t iprx = IRQNumber /4;
	uint8_t iprx_section = IRQNumber %4;
	uint8_t shift_amount = (8 * iprx_section) +(8-NO_PR_BITS_IMPLEMENTED);
	//*(NVIC_PR_BASE_ADDR + iprx *4) |= (IRQPriority << (8 * shift_amount));
	*(NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}


void GPIO_IRQHandling(uint8_t PinNumber)
{
		//Clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1<<PinNumber))
	{
		EXTI->PR |= (1<< PinNumber); //Clear by one
	}
}
