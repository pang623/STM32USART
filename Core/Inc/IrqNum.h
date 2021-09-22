/*
 * IrqNum.h
 *
 *  Created on: 10 Aug 2021
 *      Author: Pang
 */

#ifndef INC_IRQNUM_H_
#define INC_IRQNUM_H_

typedef enum {
	TIM2_IRQ = 28,
	TIM4_IRQ = 30,
	ADC1_IRQ = 18,
	USART1_IRQ = 37,
	USART2_IRQ = 38,
	USART6_IRQ = 71,
} IrqNumber;

#endif /* INC_IRQNUM_H_ */
