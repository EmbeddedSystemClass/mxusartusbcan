/******************************************************************************
* File Name          : adcTask.c
* Date First Issued  : 02/01/2019
* Description        : Handle ADC w DMA using FreeRTOS/ST HAL within a task
*******************************************************************************/

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "adctask.h"

static struct ADCDMATSKBLK* phd = NULL;

/* *************************************************************************
 *  adctask_init(ADC_HandleTypeDef* phadc,\
	uint32_t  notebit1,\
	uint32_t  notebit2,\
	uint32_t* pnoteval,\
	uint16_t  dmact);
 *	@brief	: Setup circular line buffers this uart
 * @param	: phadc = pointer to ADC control block
 * @param	: notebit1 = unique bit for notification @ 1/2 dma buffer
 * @param	: notebit2 = unique bit for notification @ end dma buffer
 * @param	: pnoteval = pointer to word receiving notification word from OS
 * @param	: dmact = number of sequences in 1/2 of circular DMA buffer
 * @return	: 
 * *************************************************************************/
/*
   notebit1 notify at the halfway dma buffer point
     associates with pdma (beginning of dma buffer)
   notebit2 notify at the end of the dma buffer
     associates with pdma + dmact * phadc->Init.NbrOfConversion
*/

struct ADCDMATSKBLK* adctask_init(ADC_HandleTypeDef* phadc,\
	uint32_t  notebit1,\
	uint32_t  notebit2,\
	uint32_t* pnoteval,\
	uint16_t  dmact)
{
	uint16_t* pdma;
	uint64_t* psum;
	struct ADCDMATSKBLK* pblk;
	struct ADCDMATSKBLK* ptmp;

	/* length = total number of uint16_t in dma buffer */
	uint32_t length = dmact * 2 * phadc->Init.NbrOfConversion;

taskENTER_CRITICAL();

	pblk = phd;
	if (pblk == NULL)
	{ // Here, first block
		pblk = (struct ADCDMATSKBLK*)calloc(1, sizeof(struct ADCDMATSKBLK));
		if (pblk == NULL){taskEXIT_CRITICAL();return NULL;}
		pblk->pnext = pblk; // End of list
		phd = pblk;
	}
	else
	{ // Here, add block to list
		ptmp = phd;
		while(ptmp->pnext != ptmp) ptmp++;
		pblk = (struct ADCDMATSKBLK*)calloc(1, sizeof(struct ADCDMATSKBLK));
		if (pblk == NULL){taskEXIT_CRITICAL();return NULL;}
		pblk->pnext = pblk; // End of list
		ptmp->pnext = pblk; // Preceding block points to new block
	}

	/* Get dma buffer allocated */
	pdma = (uint16_t*)calloc(length, sizeof(uint16_t));
	if (pdma == NULL) {taskEXIT_CRITICAL();return NULL;}

	/* Get memory for summed readings.  One for each ADC input.  */
	psum = (uint64_t*)calloc(phadc->Init.NbrOfConversion, sizeof(uint64_t));
	if (psum == NULL) {taskEXIT_CRITICAL();return NULL;}

taskEXIT_CRITICAL();

	/* Populate our control block */
/* The following reproduced for convenience--
struct ADCDMATSKBLK
{
	ADC_HandleTypeDef* phadc;
	uint32_t  notebit1;
	uint32_t  notebit2;
	uint32_t* pnoteval;
	uint16_t* pdma1;
	uint16_t* pdma2;
	osThreadId adctaskHandle;
	uint64_t* psum;
	uint16_t  dmact;
};

*/
	pblk->phadc    = phadc;
	pblk->notebit1 = notebit1;
	pblk->notebit2 = notebit2;
	pblk->pnoteval = pnoteval;
	pblk->dmact    = dmact;
	pblk->pdma1    = pdma;
	pblk->pdma2    = pdma + (dmact * phadc->Init.NbrOfConversion);
	pblk->adctaskHandle = xTaskGetCurrentTaskHandle();
	pblk->psum     = psum;

/**
  * @brief  Enables ADC DMA request after last transfer (Single-ADC mode) and enables ADC peripheral  
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from ADC peripheral to memory.
  * @retval HAL status
  */
	
	HAL_ADC_Start_DMA(pblk->phadc, (uint32_t*)pblk->pdma1, length);
	return pblk;
}
/* *************************************************************************
 * uint64_t* adctask_sum(struct ADCDMATSKBLK* pblk);
 *	@brief	: sum 1/2 of the dma buffer for each ADC in the sequence
 * @param	: pblk = pointer to our control block with all the info
 * *************************************************************************/
uint64_t* adctask_sum(struct ADCDMATSKBLK* pblk)
{
	uint16_t* p;	
	uint16_t* pdat;
	uint32_t n;	
	int i,j;

	/* Was notification an ADC DMA callback? */
	if (((pblk->notebit1 | pblk->notebit2) & *pblk->pnoteval) == 0) return NULL;

	/* Select DMA buffer half just completed. */
	if ((pblk->notebit1 & *pblk->pnoteval) != 0)
	{
		p = pblk->pdma1;
	}
	else
	{
		p = pblk->pdma2;	
	}

	/* Sum dma readings from 1/2 dma buffer. */
	n = pblk->phadc->Init.NbrOfConversion;
	for (i = 0; i < n; i++) // Step thru ADC seq
	{
		pdat = (p + i); // Pt to ADC for 1st sequence
		*(pblk->psum + i) = 0;
		for (j = 0; j < pblk->dmact; j++) // Step thru buffer
		{
			*(pblk->psum + i) += *pdat;
			pdat += n; // Step to next sequence
		}
	}
	return pblk->psum;
}
/* #######################################################################
   ADC DMA interrupt callbacks
   ####################################################################### */
uint32_t Ddma1;
uint32_t Ddma2;
/* *************************************************************************
 * void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
 *	@brief	: Call back from stm32f4xx_hal_adc: Halfway point of dma buffer
 * *************************************************************************/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
Ddma1 += 1;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	struct ADCDMATSKBLK* ptmp;
	ptmp = phd;
	/* Look up this ADC control block from linked list */
	do
	{
		if (ptmp->phadc == hadc) break;
		ptmp++;
	} while (ptmp->pnext != ptmp);
	/* Trigger task that processed 1/2 dma */
	xTaskNotifyFromISR(ptmp->adctaskHandle, 
		ptmp->notebit1,	/* 'or' bit assigned to buffer to notification value. */
		eSetBits,      /* Set 'or' option */
		&xHigherPriorityTaskWoken ); 

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	return;
}
/* *************************************************************************
 * void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
 *	@brief	: Call back from stm32f4xx_hal_adc: End point of dma buffer
 * *************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
Ddma2 += 1;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	struct ADCDMATSKBLK* ptmp;
	ptmp = phd;
	do
	{
		if (ptmp->phadc == hadc) break;
		ptmp++;
	} while (ptmp->pnext != ptmp);
	/* Trigger task that processed 1/2 dma */
	xTaskNotifyFromISR(ptmp->adctaskHandle, 
		ptmp->notebit2,	/* 'or' bit assigned to buffer to notification value. */
		eSetBits,      /* Set 'or' option */
		&xHigherPriorityTaskWoken ); 

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	return;
}


