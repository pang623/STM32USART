/*
 * Usart.h
 *
 *  Created on: 13 Jul 2021
 *      Author: Pang
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "stdint.h"
#include "IO.h"

//All USART base addresses
#define USART1_BASE_ADDRESS 	0x40011000
#define USART2_BASE_ADDRESS 	0x40004400
#define USART6_BASE_ADDRESS 	0x40011400

//All USART pointers
#define usart1			 		((UsartReg*)USART1_BASE_ADDRESS)
#define usart2			 		((UsartReg*)USART2_BASE_ADDRESS)
#define usart6			 		((UsartReg*)USART6_BASE_ADDRESS)

//USART registers
typedef struct UsartReg_t UsartReg;
struct UsartReg_t {
	_IO_ uint32_t SR;
	_IO_ uint32_t DR;
	_IO_ uint32_t BRR;
	_IO_ uint32_t CR1;
	_IO_ uint32_t CR2;
	_IO_ uint32_t CR3;
	_IO_ uint32_t GTPR;
};

typedef enum {
	//CR1
	USART_SEND_BREAK,
	USART_RCV_WAKEUP = 1 << 1,
	USART_RX_EN = 1 << 2,	USART_TX_EN = 1 << 3,
	USART_IDLE_IE = 1 << 4, USART_RXNE_IE = 1 << 5,
	USART_TC_IE = 1 << 6, USART_TXE_IE = 1 << 7,
	USART_PE_IE = 1 << 8,		//Parity error interrupt
	USART_DISABLE_PARITY = 0x0 << 9,
	USART_EVEN_PARITY = 0x2 << 9, USART_ODD_PARITY = 0x3 << 9,
	USART_WAKE_ON_IDLE = 0 << 11, USART_WAKE_ON_ADDR = 1 << 11,
	USART_8_DATA_BIT = 0 << 12, USART_9_DATA_BIT = 1 << 12,
	USART_EN = 1 << 13, USART_OVER8 = 1 << 15, USART_OVER16 = 0 << 15,
	//CR2
	USART_LBDL_10 = 0 << 21, USART_LBDL_11 = 1 << 21,
	USART_LBD_IE = 1 << 22,
	USART_LBCL_XOUT = 0 << 24, USART_LBCL_OUT = 1 << 24,
	USART_CPHA_FIRST = 0 << 25, USART_CPHA_SECOND = 1 << 25,
	USART_CPOL_LOW = 0 << 26, USART_LBCL_HIGH = 1 << 26,
	USART_CLK_EN = 1 << 27,
	USART_STOPB_1 = 0 << 28, USART_STOPB_0_5 = 1 << 28, USART_STOPB_2 = 2 << 28, USART_STOPB_1_5 = 3 << 28,
	USART_LIN_EN = 1 << 30,
	//CR3
	USART_E_IE = (uint64_t)1 << 32, USART_IR_EN = (uint64_t)1 << 33, USART_IR_LP = (uint64_t)1 << 34, USART_HALF_DUPLEX = (uint64_t)1 << 35,
	USART_SCARD_NACK = (uint64_t)1 << 36, USART_SCARD_EN = (uint64_t)1 << 37, USART_DMA_RX_EN = (uint64_t)1 << 38, USART_DMA_TX_EN = (uint64_t)1 << 39,
	USART_RTS_EN = (uint64_t)1 << 40, USART_CTS_EN = (uint64_t)1 << 41, USART_CTS_IE = (uint64_t)1 << 42,
	USART_THREEBIT = (uint64_t)0 << 43, USART_ONEBIT = (uint64_t)1 << 43,
} UsartConfig;

typedef enum {
	USART_PARITY_ERROR = 1, USART_FRAMING_ERROR = 1 << 1, USART_NOISE_DETECTED = 1 << 2,
	USART_OVERRUN_ERROR = 1 << 3, USART_IDLE_DETECTED = 1 << 4, USART_RXNE = 1 << 5,
	USART_TC = 1 << 6, USART_TXE = 1 << 7, USART_LIN_DETECTED = 1 << 8, USART_CTS = 1 << 9,
} UsartStatusFlags;

uint32_t getUsartPeripheralBusFreq(UsartReg *usart);
void usartSetBaudRate(UsartReg *usart, int baudrate);
void usartConfigure(UsartReg *usart, UsartConfig configuration);
void usartSendChar(UsartReg *usart, uint8_t c);
int usartSendString(UsartReg *usart, char *str, int len);

#endif /* INC_USART_H_ */
