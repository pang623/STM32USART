/*
 * Rcc.h
 *
 *  Created on: Jul 6, 2021
 *      Author: Pang
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "stdint.h"
#include "IO.h"

//RCC base address
#define RCC_BASE_ADDRESS 	0x40023800

//RCC pointer
#define rcc			 		((RccReg*)RCC_BASE_ADDRESS)

//RCC registers
typedef struct RccReg_t RccReg;
struct RccReg_t {
	_IO_ uint32_t CR;
	_IO_ uint32_t PLLCFGR;
	_IO_ uint32_t CFGR;
	_IO_ uint32_t CIR;
	_IO_ uint32_t AHB1RSTR;
	_IO_ uint32_t AHB2RSTR;
	_IO_ uint32_t Reserved0[2];
	_IO_ uint32_t APB1RSTR;
	_IO_ uint32_t APB2RSTR;
	_IO_ uint32_t Reserved1[2];
	_IO_ uint32_t AHB1ENR;
	_IO_ uint32_t AHB2ENR;
	_IO_ uint32_t Reserved2[2];
	_IO_ uint32_t APB1ENR;
	_IO_ uint32_t APB2ENR;
	_IO_ uint32_t Reserved3[2];
	_IO_ uint32_t AHB1LPENR;
	_IO_ uint32_t AHB2LPENR;
	_IO_ uint32_t Reserved4[2];
	_IO_ uint32_t APB1LPENR;
	_IO_ uint32_t APB2LPENR;
	_IO_ uint32_t Reserved5[2];
	_IO_ uint32_t BDCR;
	_IO_ uint32_t CSR;
	_IO_ uint32_t Reserved6[2];
	_IO_ uint32_t SSCGR;
	_IO_ uint32_t PLLI2SCFGR;
	_IO_ uint32_t Reserved7;
	_IO_ uint32_t DCKCFGR;
};

typedef enum {
	RCC_AHB1 = 0x10,
	RCC_AHB2 = 0x14,
	RCC_APB1 = 0x20,
	RCC_APB2 = 0x24,
} RccAdvancedBusOffset;

typedef enum {
	RCC_GPIOA = 0 | (RCC_AHB1 << 16),
	RCC_GPIOB = 1 | (RCC_AHB1 << 16),
	RCC_GPIOC = 2 | (RCC_AHB1 << 16),
	RCC_GPIOD = 3 | (RCC_AHB1 << 16),
	RCC_GPIOE = 4 | (RCC_AHB1 << 16),
	RCC_GPIOH = 7 | (RCC_AHB1 << 16),

	RCC_USART1 = 4 | (RCC_APB2 << 16),
	RCC_USART2 = 17 | (RCC_APB1 << 16),
	RCC_USART6 = 5 | (RCC_APB2 << 16),

	RCC_TIMER1 = 0 | (RCC_APB2 << 16),
	RCC_TIMER2 = 0 | (RCC_APB1 << 16),
	RCC_TIMER3 = 1 | (RCC_APB1 << 16),
	RCC_TIMER4 = 2 | (RCC_APB1 << 16),
	RCC_TIMER5 = 3 | (RCC_APB1 << 16),
	RCC_TIMER9 = 16 | (RCC_APB2 << 16),
	RCC_TIMER10 = 17 | (RCC_APB2 << 16),
	RCC_TIMER11 = 18 | (RCC_APB2 << 16),

	RCC_I2C1 = 21 | (RCC_APB1 << 16),
	RCC_I2C2 = 22 | (RCC_APB1 << 16),
	RCC_I2C3 = 23 | (RCC_APB1 << 16),

	RCC_ADC1 = 8 | (RCC_APB2 << 16),
} RccDevice;

#define	rccUnresetAndEnableDevice(rccDev)	\
		_rccUnresetAndEnableDevice(rccDev, 1)

void _rccUnresetAndEnableDevice(RccDevice rccDev, int reset);

#endif /* INC_RCC_H_ */
