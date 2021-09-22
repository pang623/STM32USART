/*
 * Rcc.c
 *
 *  Created on: 13 Jul 2021
 *      Author: Pang
 */

#include "Rcc.h"

void _rccUnresetAndEnableDevice(RccDevice rccDev, int reset) {
	if(reset)
		*((_IO_ uint32_t *)(RCC_BASE_ADDRESS + (rccDev >> 16))) |= (1 << (rccDev & 0xFFFF));
	*((_IO_ uint32_t *)(RCC_BASE_ADDRESS + (rccDev >> 16))) &= ~(1 << (rccDev & 0xFFFF));
	*((_IO_ uint32_t *)(RCC_BASE_ADDRESS + ((rccDev >> 16) + 0x20))) &= ~(1 << (rccDev & 0xFFFF));
	*((_IO_ uint32_t *)(RCC_BASE_ADDRESS + ((rccDev >> 16) + 0x20))) |= (1 << (rccDev & 0xFFFF));
}
/*
void rccResetUnresetDevice(_IO_ uint32_t *io, int bitNum) {
	*io &= ~(1 << bitNum);
	*io |= 1 << bitNum;			//Reset
	*io &= ~(1 << bitNum);		//Unreset
}

void rccEnableClock(_IO_ uint32_t *io, int bitNum) {
	*io &= ~(1 << bitNum);		//Clear bit
	*io |= 1 << bitNum;			//Enable clock
}

void rccUnresetAndEnableGpio(RccGpio rccGpio) {
	rccResetUnresetDevice(ahb1ResetReg, rccGpio);		//Reset Unreset
	rccEnableClock(ahb1EnClkReg, rccGpio);				//Enable clock
}

void rccUnresetAndEnableUsart(RccUsart rccUsart) {
	if(rccUsart == RCC_USART2) {
		rccResetUnresetDevice(apb1ResetReg, rccUsart);	//Reset Unreset
		rccEnableClock(apb1EnClkReg, rccUsart);			//Enable clock
	} else {
		rccResetUnresetDevice(apb2ResetReg, rccUsart);	//Reset Unreset
		rccEnableClock(apb2EnClkReg, rccUsart);			//Enable clock
	}
}
*/
