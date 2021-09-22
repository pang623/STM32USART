/*
 * Adc.c
 *
 *  Created on: Sep 6, 2021
 *      Author: Pang
 */

#include "Adc.h"

void adcSetSampleTime(AdcReg *adc, int chnNum, AdcSampleTime time) {
	adc->SMPR[!(chnNum/10)] &= ~(0x7 << ((chnNum % 10) * 3));
	adc->SMPR[!(chnNum/10)] |= time << ((chnNum % 10) * 3);
}

void configureADC(AdcReg *adc, AdcConfig cfg) {
	adc->CR1 &= ~(cfg & 0xFFFFFFFF);
	adc->CR1 |= cfg & 0xFFFFFFFF;
	adc->CR2 &= ~(cfg >> 32);
	adc->CR2 |= cfg >> 32;
}

void adcSetChannelSequence(AdcReg *adc, int *channels, int seqLen) {
	for(int i = 0; i < seqLen; i++) {
		adc->SQR[2 - (i / 6)] &= ~(0x1F << ((i % 6) * 5));
		adc->SQR[2 - (i / 6)] |= channels[i] << ((i % 6) * 5);
	}
}

void adcCommonConfigure(AdcCommonConfig cfg) {
	ADC_COMM->CCR &= cfg;
	ADC_COMM->CCR |= cfg;
}

void adcConfigureRegularChannel(AdcReg *adc, AdcRegularChn cfg) {
	adc->CR2 &= ~(0x7F << 24);
	adc->CR2 |= cfg;
}
