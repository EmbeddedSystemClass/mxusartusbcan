/******************************************************************************
* File Name          : ADCTask.h
* Date First Issued  : 02/01/2019
* Description        : ADC w DMA using FreeRTOS/ST HAL
*******************************************************************************/

#ifndef __ADCTASK
#define __ADCTASK

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define ADCSEQSIZE	5	// Number of ADCs each sequence

struct ADCQITEM
{
	uint32_t adc[ADCSEQSIZE];

};


#endif

