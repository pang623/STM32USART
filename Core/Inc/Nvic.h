/*
 * Nvic.h
 *
 *  Created on: 10 Aug 2021
 *      Author: Pang
 */

#ifndef INC_NVIC_H_
#define INC_NVIC_H_

#include "stdint.h"
#include "IO.h"
#include "IrqNum.h"

//NVIC base address
#define NVIC_BASE_ADDRESS 		0xE000E100

//NVIC pointer
#define nvic			 		((NvicReg*)NVIC_BASE_ADDRESS)

//NVIC registers
typedef struct NvicReg_t NvicReg;
struct NvicReg_t {
	_IO_ uint32_t ISER[8];
	_IO_ uint32_t RESERVED0[24];
	_IO_ uint32_t ICER[8];
	_IO_ uint32_t RESERVED1[24];
	_IO_ uint32_t ISPR[8];
	_IO_ uint32_t RESERVED2[24];
	_IO_ uint32_t ICPR[8];
	_IO_ uint32_t RESERVED3[24];
	_IO_ uint32_t IABR[8];
	_IO_ uint32_t RESERVED4[56];
	_IO_ uint32_t IPR[60];
	_IO_ uint32_t RESERVED5[644];
	_IO_ uint32_t STIR;
};

#define	nvicEnableIrq(irqNum)		\
	nvic->ISER[(irqNum) >> 5] |= 1 << ((irqNum) & 0x1F)

#define	nvicDisableIrq(irqNum)		\
	nvic->ICER[(irqNum) >> 5] = 1 << ((irqNum) & 0x1F)

#define	nvicSetPendingIrq(irqNum)	\
	nvic->ISPR[(irqNum) >> 5] = 1 << ((irqNum) & 0x1F)

#define	nvicClearPendingIrq(irqNum)	\
	nvic->ICPR[(irqNum) >> 5] = 1 << ((irqNum) & 0x1F)

#define	nvicSetIrqPriority(irqNum, priority)	\
	nvic->IPR[(irqNum) >> 2] |= ((priority << 4) << (((irqNum & 0x3) * 8)))

#define	nvicTriggerIrq(irqNum)		\
	nvic->STIR = irqNum

#endif /* INC_NVIC_H_ */
