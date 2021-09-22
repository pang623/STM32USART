/*
 * Gpio.c
 *
 *  Created on: Jun 29, 2021
 *      Author: Pang
 */

#include "Gpio.h"

void gpioConfigurePin(GpioReg *port, int pin, GpioConfig configuration) {
	//configure GPIO mode
	port->MODER &= GPIO_MODE_MASK(pin);
	port->MODER |= (configuration & 0xF) << (2 * pin);
	//configure GPIO AFRs when AF mode is selected
	if(pin <= PIN7) {
		port->AFR[0] &= GPIO_AFR_MASK(pin);
		port->AFR[0] |= ((configuration & 0xF0000) >> 16) << (4 * pin);
	}else {
		port->AFR[1] &= GPIO_AFR_MASK(pin);
		port->AFR[1] |= ((configuration & 0xF0000) >> 16) << (4 * (pin - 8));
	}
	//configure GPIO output driver type
	port->OTYPER &= GPIO_DRVR_MASK(pin);
	port->OTYPER |= ((configuration & 0xF0) >> 4) << pin;
	//configure GPIO speed
	port->OSPEEDR &= GPIO_SPEED_MASK(pin);
	port->OSPEEDR |= ((configuration & 0xF00) >> 8) << (2 * pin);
	//configure GPIO pull type
	port->PUPDR &= GPIO_PULL_MASK(pin);
	port->PUPDR |= ((configuration & 0xF000) >> 12) << (2 * pin);
}

void gpioWritePin(GpioReg *gpio, int pin, int state) {
	if(state)
		gpio->BSRR = 1 << pin;
	else
		gpio->BSRR = 1 << (16 + pin);
}

int gpioReadPin(GpioReg *gpio, int pin) {
	return (((gpio->IDR) >> pin) & 1);
}

void gpioLockPin(GpioReg *gpio, int pin) {
	gpio->LCKR = (1 << 16) | 1 << pin;
	gpio->LCKR = 1 << pin;
	gpio->LCKR = (1 << 16) | 1 << pin;
	gpio->LCKR = gpio->LCKR;
}
