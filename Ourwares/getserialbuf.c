/******************************************************************************
* File Name          : getserialbuf.c
* Date First Issued  : 01/12/2019
* Description        : Get a buffer  & control block for SerialTaskSend use
*******************************************************************************/

#include <malloc.h>
#include "getserialbuf.h"

/* Reproduced for convenience from 'SerialTaskSend.h'
struct SERIALSENDTASKBCB
{
	UART_HandleTypeDef* phuart; // Pointer to 'MX uart handle
	osThreadId tskhandle;       // Task handle of originating task
	uint8_t	*pbuf;             // Pointer to byte buffer to be sent
	uint32_t notebit;           // Buffer notification bit
	uint32_t* pnoteval;    // Pointer to notification work in task
	uint16_t size;              // Number of bytes to be sent
	uint16_t maxsize;      // Buffer size 		
};
*/

/* *************************************************************************
 * struct SERIALSENDTASKBCB* getserialbuf(\
		UART_HandleTypeDef* phuart,\
		uint16_t maxsize,\
		uint32_t notebit,\
		uint32_t* pnoteval);
 * @brief	: Create a buffer control block (BCB) for serial sending
 * @param	: phuart = usart handle (pointer)
 * @param	: size = number of uint8_t bytes for this buffer
 * @param	: notebit = single bit used for notification of this buffer
 * @param	: pnoteval = Pointer to Task word that receives notification
 * @return	: pointer to BCB; NULL = failed
 * *************************************************************************/
/*
Construct a list of "struct NOTEBITLIST" items, for each different usart.

The items maintain the bit used by SerialTaskSend to notify the originating
task that the buffer has been sent and is available for reuse.
*/
struct SERIALSENDTASKBCB* getserialbuf(\
		UART_HandleTypeDef* phuart,\
		uint16_t size,\
		uint32_t notebit,\
		uint32_t* pnoteval)
{
/* This MUST be called when a FreeRTOS task starts so that this routine can call
   FreeRTOS to get the task handle.

	To avoid problems of a time tick switching tasks in the middle of this routine the
   FreeRTOS interrupts are locked via "taskENTER_CRITICAL();".

   I suppose suspending the scheduler could be an alternative that would not lock
   interrupts--
	"void vTaskSuspendAll( void );"
	"BaseType_t xTaskResumeAll( void );"
*/
	/* BCB: Buffer control block, passed on queue to SerialTaskSend. See SerialTaskSend.h */
	struct SERIALSENDTASKBCB* pbcb; // calloc'ed bcb pointer

	uint8_t* pbuf;	// callloc'ed byte buffer

taskENTER_CRITICAL();
	/* Get one BCB block */
	pbcb = (struct SERIALSENDTASKBCB*)calloc(1, sizeof(struct SERIALSENDTASKBCB));	
	if (pbcb == NULL) return NULL;

	/* Get byte buffer */
	pbuf = (uint8_t*)calloc(size, sizeof(uint8_t));	
	if (pbuf == NULL) return NULL;

	/* Initialize the BCB. */
	pbcb->phuart    = phuart;  // 'MX uart handle
	pbcb->tskhandle = xTaskGetCurrentTaskHandle();
	pbcb->pbuf      = pbuf;		// Ptr to uint8_t buffer
	pbcb->notebit   = notebit; // Notification bit for this buffer
	pbcb->pnoteval  = pnoteval;// Pointer to Task notification word
	pbcb->size      = 0;		   // No bytes to send at this point.
	pbcb->maxsize   = size;    // Size of uint8_t buffer

	/* Set initial value for noteval in Task to show buffer available. */
	xTaskNotify(pbcb->tskhandle, pbcb->notebit, eSetBits);
//	*pbcb->pnoteval |= notebit; // Needed?

taskEXIT_CRITICAL();

	return pbcb;
}

