/******************************************************************************
* File Name          : SerialTaskReceive.h
* Date First Issued  : 01/21/2019
* Description        : Serial input using FreeRTOS/ST HAL
*******************************************************************************/

#ifndef __SERIALTASKRECEIVE
#define __SERIALTASKRECEIVE

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define MAXNUMSERIAL 6	// Number of uart + usart.

#define LINETERMINATOR ('\n')	// EOL for incoming stream

/* Line buffer control block for one uart */
struct SERIALRCVBCB
{
	struct SERIALRCVBCB* pnext;	// Link to next uart RBCB
	char* pbegin;// Ptr to first line buffer
	char* pend;  // Ptr to last+1 line buffer
	char* padd;  // Ptr to line buffer being filled
	char* ptake; // Ptr to line buffer to take
	char* pwork; // Ptr to next char to be added
	char* pworkend; // Ptr to end of current active line buffer
	UART_HandleTypeDef* phuart;// Pointer to 'MX uart handle
	osThreadId tskhandle;      // Task handle of originating task
	uint32_t  notebit;         // Unique notification bit (within task)
	uint32_t* pnoteval;         // Pointer to word receiving notification 
	char*  pbegindma;       // Pointer to beginning of dma buffer
	char*  penddma;         // Pointer to ebd + 1 of dma buffer
	char*  ptakedma;        // Pointer to last + 1 char taken from dma buffer
	uint32_t  numlinexsize;    // Number of lines * line size (chars)
	uint16_t  linesize;        // Number of chars in each line buffer
	uint16_t  dmasize;         // Number of chars in total circular DMA buffer
	uint8_t   numline;         // Number of line buffers for this uart
	int8_t    dmaflag;         // dmaflag = 0 for char-by-char mode; 1 = dma mode
	uint32_t errorct;				// uart error callback counter
};

/* *************************************************************************/
struct SERIALRCVBCB* xSerialTaskRxAdduart(\
		UART_HandleTypeDef* phuart,\
		int8_t    dmaflag,\
		uint32_t  notebit,\
		uint32_t* pnoteval,\
		uint8_t   numline,\
		uint8_t   linesize,\
		char  dmasize);
/*	@brief	: Setup circular line buffers this uart
 * @param	: phuart = pointer to uart control block
 * @param	: dmaflag = 0 for char-by-char mode; 1 = dma mode
 * @param	: notebit = unique bit for notification for this task
 * @param	: pnoteval = pointer to word receiving notification word from OS
 * @param	: numline = number of line buffers in circular line buffer
 * @param	: linesize = number of chars in each line buffer
 * @param	: dmasize = number of chars in total circular DMA buffer
 * @return	: pointer = 'RCVBCB for this uart; NULL = failed
 * *************************************************************************/
char* xSerialTaskReceiveGetline(struct SERIALRCVBCB* pbcb);
/*	@brief	: Load buffer control block onto queue for sending
 * @param	: pbcb = Pointer to Buffer Control Block
 * @return	: Pointer to line buffer; NULL = no new lines
 * *************************************************************************/
BaseType_t xSerialTaskReceiveCreate(uint32_t taskpriority);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: SerialTaskReceiveHandle
 * *************************************************************************/


#endif

