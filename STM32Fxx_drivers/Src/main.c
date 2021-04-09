/*
 * main.c
 *
 *  Created on: 9 avr. 2021
 *      Author: jeanm
 */

#include "stm32f407xx.h"

int main (void)
{
	return 0;
}
void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
}
