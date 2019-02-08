/******************************************************************************
* File Name          : ADCTask.c
* Date First Issued  : 02/01/2019
* Description        : ADC w DMA using FreeRTOS/ST HAL
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "ADCTask.h"



static int argument1;
QueueHandle_t ADCQHandle;

static uint16_t* adcbuffptr;


/* *************************************************************************
 * struct SERIALRCVBCB* xSerialTaskRxAdduart(\
		UART_HandleTypeDef* phuart,\
		int8_t    dmaflag,\
		uint32_t  notebit,\
		uint32_t* pnoteval,\
		uint8_t   numline,\
		uint8_t   linesize,\
		char  dmasize);
 *	@brief	: Setup circular line buffers this uart
 * @param	: phuart = pointer to uart control block
 * @param	: dmaflag = 0 for char-by-char mode; 1 = dma mode
 * @param	: notebit = unique bit for notification for this task
 * @param	: pnoteval = pointer to word receiving notification word from OS
 * @param	: numline = number of line buffers in circular line buffer
 * @param	: linesize = number of chars in each line buffer
 * @param	: dmasize = number of chars in total circular DMA buffer
 * @return	: pointer = 'RCVBCB for this uart; NULL = failed
 * *************************************************************************/
struct SERIALRCVBCB* xSerialTaskRxAdduart(\
		UART_HandleTypeDef* phuart,\
		int8_t    dmaflag,\
		uint32_t  notebit,\
		uint32_t* pnoteval,\
		uint8_t   numline,\
		uint8_t   linesize,\
		char  dmasize)
{
	struct SERIALRCVBCB* ptmp1;
	struct SERIALRCVBCB* ptmp2;
	char* pbuf;

	/* There can be a problem with Tasks not started if the calling task gets here first */
	osDelay(10);

taskENTER_CRITICAL();
	/* Add block with circular buffer pointers for this uart/usart to list */
	ptmp1 = (struct SERIALRCVBCB*)calloc(1, sizeof(struct SERIALRCVBCB));

taskEXIT_CRITICAL();
	return ptmp1;	// Success return pointer to this 'BCB
}

/* *************************************************************************
 * void StartADCTask(void* argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartADCTask(void* argument)
{

  /* Infinite loop */
  for(;;)
  {

  }
}
/* *************************************************************************
 * QueueHandle_t xADCTaskCreate(uint16_t queuesize, uint32_t taskpriority);
 * @brief	: Create task and queue for pointer to filtered results
 * @brief	: queuesize = number of items queue will hold
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: QHandle
 * *************************************************************************/
 BaseType_t xADCTaskCreate(uint16_t queuesize, uint32_t taskpriority)
{
/*
BaseType_t xTaskCreate( TaskFunction_t pvTaskCode,
const char * const pcName,
unsigned short usStackDepth,
void *pvParameters,
UBaseType_t uxPriority,
TaskHandle_t *pxCreatedTask );
*/
	return xTaskCreate(StartADCTask, "StartADCTask",\
     96, NULL, taskpriority,\
     &ADCTaskHandle);

	/* FreeRTOS queue for task with data to send. */
	ADCQHandle = xQueueCreate(queuesize, sizeof(adcbuffptr));
	return ADCQHandle;

//	return ADCTaskHandle;
}
/* *************************************************************************
 * char* xADCTaskGetline(struct SERIALRCVBCB* pbcb);
 *	@brief	: Load buffer control block onto queue for sending
 * @param	: pbcb = Pointer to Buffer Control Block
 * @return	: Pointer to line buffer; NULL = no new lines
 * *************************************************************************/
char* xADCTaskGetline(struct SERIALRCVBCB* pbcb)
{
	char* p = NULL;

	/* Check no new lines. */
	if (pbcb->ptake == pbcb->padd) return p;
	p = pbcb->ptake;

	/* Advance 'take' pointer w wraparound check. */
	pbcb->ptake += pbcb->linesize;
	if (pbcb->ptake == pbcb->pend) pbcb->ptake = pbcb->pbegin;

	return p;
}
/* *************************************************************************
 * static void advancebuf(struct SERIALRCVBCB* prtmp);
 * @brief	: Advance to next line buffer
 * *************************************************************************/

/* *************************************************************************
 * static void advanceptr(struct SERIALRCVBCB* prtmp);
 * @brief	: Advance pointer within the active line buffer
 * *************************************************************************/

/* *************************************************************************
 * struct SERIALRCVBCB* getrbcb(UART_HandleTypeDef *phuart);
 * @brief	: Get the rbcb pointer, given the uart handle
 * @return	: NULL = oops!, otherwise pointer to rbcb
 * *************************************************************************/

/* *************************************************************************
 * static void unloaddma(struct SERIALRCVBCB* prbcb);
 * @brief	: DMA: Check for line terminator and store; enter from task poll
 * @param	: prbcb = pointer to buffer control block for uart causing callback
 * *************************************************************************/

/* #######################################################################
   UART interrupt callbacks
   ####################################################################### */
/* *************************************************************************
 * void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *phuart);
 * @brief	: DMA callback at the halfway point in the circular buffer
 * *************************************************************************/
/* NOTE: under interrupt from callback. */

/* DMA Half buffer complete callback (dma only) */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *phuart)
{
	HAL_UART_RxCpltCallback(phuart);
}
/* *************************************************************************
 * void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *phuart);
 * @brief	: DMA callback at the halfway point in the circular buffer
 *				: OR, char-by-char completion of sending
 * *************************************************************************/
/* DMA buffer complete, => OR <= char-by-char complete */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *phuart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;



	/* Trigger Recieve Task to poll dma uarts */
	xTaskNotifyFromISR(ADCTaskHandle, 
		0,	/* 'or' bit assigned to buffer to notification value. */
		eSetBits,      /* Set 'or' option */
		&xHigherPriorityTaskWoken ); 

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	return;
}
/* *************************************************************************
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef *phuart);
 *	@brief	: Call back from receive errror, stm32f4xx_hal_uart
 * *************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *phuart)
{
	/* Look up buffer control block, given uart handle */
	struct SERIALRCVBCB* prtmp = getrbcb(phuart);
	prtmp->errorct += 1;
	return;
}


