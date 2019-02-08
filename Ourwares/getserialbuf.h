/******************************************************************************
* File Name          : getserialbuf.h
* Date First Issued  : 01/11/2019
* Description        : Get a buffer & control block for SerialTaskSend use
*******************************************************************************/

#ifndef __GETSERIALBUFY
#define __GETSERIALBUFY

#include "SerialTaskSend.h"

/* Augmented buffer control block */
/*
struct SERIALSENDTASKYCB
{
	struct SERIALSENDTASKBCB* pbcb; // Buffer Control Block
	uint32_t* pnoteval;    // Pointer to notification work in task
	uint16_t maxsize;      // Buffer size 		
//	uint8_t cannum;        // 0 = CAN1, 1 = CAN2
};
*/

/* *************************************************************************/
struct SERIALSENDTASKBCB* getserialbuf(UART_HandleTypeDef* phuart, uint16_t size, uint32_t notebit, uint32_t* pnoteval);
/* @brief	: Create a buffer control block (BCB) for serial sending
 * @param	: phuart = usart handle (pointer)
 * @param	: size = number of uint8_t bytes for this buffer
 * @param	: notebit = single bit used for notification of this buffer
 * @param	: pnoteval = Pointer to Task word that receives notification
 * @return	: pointer to BCB; NULL = failed
 * *************************************************************************/


#endif

