/*
 * Adc.h
 *
 *  Created on: 31 Aug 2021
 *      Author: Pang
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "IO.H"
#include "stdint.h"

//All ADC base addresses
#define ADC1_BASE_ADDRESS 		0x40012000

#define	ADC_COMM				((AdcCommon*)(ADC1_BASE_ADDRESS + 0x300))

//All ADC pointers
#define adc1			 		((AdcReg*)ADC1_BASE_ADDRESS)

//ADC registers
typedef struct ADC_t AdcReg;
struct ADC_t {
	_IO_ uint32_t SR;
	_IO_ uint32_t CR1;
	_IO_ uint32_t CR2;
	_IO_ uint32_t SMPR[2];
	_IO_ uint32_t JOFR[4];
	_IO_ uint32_t HTR;
	_IO_ uint32_t LTR;
	_IO_ uint32_t SQR[3];
	_IO_ uint32_t JSQR;
	_IO_ uint32_t JDR[4];
	_IO_ uint32_t DR;
};

typedef struct ADC_Common_t AdcCommon;
struct ADC_Common_t{
	_IO_ uint32_t CCR;
};

typedef enum {
	//CR1 (LEFT -> DISCNUM, AWDCH)
	ADC_EOC_INT_EN = 1 << 5,
	ADC_ANALOG_WATCHDOG_INT_EN = 1 << 6,
	ADC_INJECTED_INT_EN = 1 << 7,
	ADC_SCAN_MODE = 1 << 8,
	ADC_SINGLE_CHANNEL_WATCHDOG_EN = 1 << 9,
	ADC_AUTO_INJECTED_CONVERSION_EN = 1 << 10,
	ADC_DISCONT_REGULAR_CHN_EN = 1 << 11,
	ADC_DISCONT_INJECTED_CHN_EN = 1 << 12,
	ADC_INJECTED_CHN_WATCHDOG_EN = 1 << 22,
	ADC_REGULAR_CHN_WATCHDOG_EN = 1 << 23,
	ADC_RESOLUTION_12 = 0 << 24, ADC_RESOLUTION_10 = 1 << 24,
	ADC_RESOLUTION_8 = 2 << 24, ADC_RESOLUTION_6 = 3 << 24,
	ADC_OVERRUN_INT_EN = 1 << 26,
	//CR2 (LEFT -> BIT 30:16)
	ADC_EN = 1LL << (0 + 32),
	ADC_SINGLE_MODE = 0LL << (1 + 32), ADC_CONTINUOUS_MODE = 1LL << (1 + 32),
	ADC_DMA_EN = 1LL << (8 + 32),
	ADC_ISSUE_DMA = 1LL << (9 + 32),
	ADC_EOC_OVERRUN_CHECK_ON_DMA_SET = 0LL << (10 + 32), ADC_EOC_OVERRUN_CHECK = 1LL << (10 + 32),
	ADC_RIGHT_ALIGN = 0LL << (11 + 32), ADC_LEFT_ALIGN = 1LL << (11 + 32),
}AdcConfig;

typedef enum {
	SAMPLE_3_CYCLES, SAMPLE_15_CYCLES, SAMPLE_28_CYCLES, SAMPLE_56_CYCLES,
	SAMPLE_84_CYCLES, SAMPLE_112_CYCLES, SAMPLE_144_CYCLES, SAMPLE_480_CYCLES,
}AdcSampleTime;

typedef enum {
	ADC_PSC_2 = 0 << 16, ADC_PSC_4 = 1 << 16, ADC_PSC_6 = 2 << 16, ADC_PSC_8 = 3 << 16,
	ADC_VBAT_EN = 1 << 22, ADC_TEMP_SENSOR_EN = 1 << 23,
}AdcCommonConfig;

typedef enum {
	ADC_TIM4_CH4_TRG = 9 << 24, ADC_TRG_RISING_EDGE = 1 << 28, ADC_REG_CHN_START = 1 << 30,
}AdcRegularChn;

typedef enum {
	ADC_WATCHDOG_OCCURED = 1,
	ADC_END_OF_CONVERSION = 1 << 1,
	ADC_INJECTED_EOC = 1 << 2,
	ADC_INJECTED_CONVERSION_STARTED = 1 << 3,
	ADC_REGULAR_CONVERSION_STARTED = 1 << 4,
	ADC_OVERRUN_OCCURED = 1 << 5,
}AdcStatusFlags;

void adcSetSampleTime(AdcReg *adc, int chnNum, AdcSampleTime time);
void adcSetChannelSequence(AdcReg *adc, int *channels, int seqLen);
void configureADC(AdcReg *adc, AdcConfig cfg);
void adcCommonConfigure(AdcCommonConfig cfg);
void adcConfigureRegularChannel(AdcReg *adc, AdcRegularChn cfg);

#endif /* INC_ADC_H_ */
