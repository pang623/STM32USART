/*
 * Usart.c
 *
 *  Created on: 13 Jul 2021
 *      Author: Pang
 */

#include "Usart.h"
#include "stm32f4xx_hal.h"
#include <math.h>

uint32_t getUsartPeripheralBusFreq(UsartReg *usart) {
	if(usart == usart2)
		return HAL_RCC_GetPCLK1Freq();
	else
		return HAL_RCC_GetPCLK2Freq();
}

void usartSetBaudRate(UsartReg *usart, int baudrate) {
	uint32_t apbFreq = getUsartPeripheralBusFreq(usart);
	int over8 = (usart->CR1 & 0x8000) >> 15;
	float usart_div = apbFreq/(float)(8*(2-over8)*baudrate);
	double decimal, fractional;
	fractional = modf(usart_div, &decimal);
	usart->BRR = ((int)(decimal) << 4 | (int)(fractional*8*(2-over8)));
}

void usartConfigure(UsartReg *usart, UsartConfig configuration) {
	usart->CR1 |= configuration & 0xBFFF;
	usart->CR2 |= (configuration & 0x7F7F0000) >> 16;
	usart->CR3 |= (configuration & 0xFFF00000000) >> 32;
}

void usartSendChar(UsartReg *usart, uint8_t c) {

	usart->DR = c;
	while(!(usart->SR & USART_TXE));
	//while(!(usart->SR & USART_TC));
}

int usartSendString(UsartReg *usart, char *str, int len) {
	for(int i = 0; i < len; i++)
		usartSendChar(usart, str[i]);
	return len;
}
