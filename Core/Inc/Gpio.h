/*
 * Gpio.h
 *
 *  Created on: Jun 29, 2021
 *      Author: Pang
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stdint.h"
#include "IO.h"

//Base addresses of GPIOs
#define GPIOA_BASE_ADDRESS		0x40020000
#define GPIOB_BASE_ADDRESS		0x40020400
#define GPIOC_BASE_ADDRESS		0x40020800
#define GPIOD_BASE_ADDRESS		0x40020C00
#define GPIOE_BASE_ADDRESS		0x40021000

//All GPIO pointers
#define gpioA					((GpioReg*)GPIOA_BASE_ADDRESS)
#define gpioB					((GpioReg*)GPIOB_BASE_ADDRESS)
#define gpioC					((GpioReg*)GPIOC_BASE_ADDRESS)
#define gpioD					((GpioReg*)GPIOD_BASE_ADDRESS)
#define gpioE					((GpioReg*)GPIOE_BASE_ADDRESS)

//Define GPIO Pins
#define		PIN0				0
#define		PIN1				1
#define		PIN2				2
#define		PIN3				3
#define		PIN4				4
#define		PIN5				5
#define		PIN6				6
#define		PIN7				7
#define		PIN8				8
#define		PIN9				9
#define		PIN10				10
#define		PIN11				11
#define		PIN12				12
#define		PIN13				13
#define		PIN14				14
#define		PIN15				15

#define		GPIO_MODE_MASK(pin)			~(3 << (2 * pin))
#define		GPIO_SPEED_MASK(pin)		~(3 << (2 * pin))
#define		GPIO_PULL_MASK(pin)			~(3 << (2 * pin))
#define		GPIO_DRVR_MASK(pin)			~(1 << (pin))
#define		GPIO_AFR_MASK(pin)			~(15 << (4 * pin))

//GPIO registers
typedef struct GpioReg_t GpioReg;
struct GpioReg_t {
	_IO_ uint32_t MODER;
	_IO_ uint32_t OTYPER;
	_IO_ uint32_t OSPEEDR;
	_IO_ uint32_t PUPDR;
	_IO_ uint32_t IDR;
	_IO_ uint32_t ODR;
	_IO_ uint32_t BSRR;
	_IO_ uint32_t LCKR;
	_IO_ uint32_t AFR[2];
};

//GPIO configurations
typedef enum {
	//GPIO mode
	GPIO_INPUT, GPIO_OUTPUT, GPIO_ALT_FUNC, GPIO_ANALOG_IN,
	//GPIO output driver type
	GPIO_PUSH_PULL  = 0 << 4,  GPIO_OPEN_DRAIN = 1 << 4,
	//GPIO speed
	GPIO_LOW_SPEED  = 0 << 8,  GPIO_MED_SPEED  = 1 << 8,
	GPIO_FAST_SPEED = 2 << 8,  GPIO_HIGH_SPEED = 3 << 8,
	//GPIO Pull type
	GPIO_NO_PULL    = 0 << 12, GPIO_PULL_UP = 1 << 12,
	GPIO_PULL_DOWN  = 2 << 12,
	//GPIO Alternate function
	AF_0  =  0 << 16,  AF_1  =  1 << 16,  AF_2  =  2 << 16,  AF_3  =  3 << 16,
	AF_4  =  4 << 16,  AF_5  =  5 << 16,  AF_6  =  6 << 16,  AF_7  =  7 << 16,
	AF_8  =  8 << 16,  AF_9  =  9 << 16,  AF_10 = 10 << 16,  AF_11 = 11 << 16,
	AF_12 = 12 << 16,  AF_13 = 13 << 16,  AF_14 = 14 << 16,  AF_15 = 15 << 16,
} GpioConfig;

//Function prototypes
void gpioConfigurePin(GpioReg *port, int pin, GpioConfig configuration);
void gpioWritePin(GpioReg *gpio, int pin, int state);
int gpioReadPin(GpioReg *gpio, int pin);
void gpioLockPin(GpioReg *gpio, int pin);

#endif /* INC_GPIO_H_ */
