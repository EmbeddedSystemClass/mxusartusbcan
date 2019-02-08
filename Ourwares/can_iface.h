/******************************************************************************
* File Name          : can_iface.h
* Date First Issued  : 05-28-2015/01/02/2019
* Board              : F103 or F4
* Description        : Interface CAN FreeRTOS to STM32CubeMX HAL can driver 
*******************************************************************************/
/* 
'iface is a hack of 'driver'.

Implements the sorted linked list buffer for presenting the highest priority CAN msg
at all times.
*/

#ifndef __CAN_IFACE
#define __CAN_IFACE

#define LDR_RESET	8

#ifndef NULL 
#define NULL	0
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "common_can.h"

// Disable TX RQCPx and RX0, and RX1 interrupts for CAN1 and CAN2 (works in an 'if' statement)
//#define DISABLE_ALLCANINT  do{ __attribute__((__unused__)) int rdbk;CAN_IER(CAN1) &= ~0x13; CAN_IER(CAN2) &= ~0x13; rdbk = CAN_IER(CAN1); rdbk = CAN_IER(CAN2);}while(0) 
// Enable the above interrupts
//#define ENABLE_ALLCANINT   CAN_IER(CAN1) |= 0x13; CAN_IER(CAN1) |= 0x13
 
/*  -- bits -- */
#define	SOFTNART	0x01		// 1 = No retries (including arbitration); 0 = retries
#define NOCANSEND	0x02		// 1 = Do not send to the CAN bus

/* struct CAN_PARAMS holds the values used to setup the CAN registers during 'init' */
// baud rate CAN_BTR[9:0]: brp = (pclk1_freq/(1 + TBS1 + TBS2)) / baudrate;

/* Low level interrupts that each higher priority CAN interrupt triggers. 
   Note: vector.c needs to have the vectors as appropriate for
     CAN1 and CAN2--
       CAN_TX, CAN_RX0, CAN_RX1 setup with names:
	CAN1_TX_IRQHandler; CAN1_RX0_IRQHandler; CAN1_RX1_IRQHandler
	CAN2_TX_IRQHandler; CAN2_RX0_IRQHandler CAN1_RX2_IRQHandler
*/  

/* In the following RX uses 'xw' and TX uses 'xb[]' */
union CAN_X
{
	uint32_t	xw;	// RX (DTW)
	u8	xb[4];	// TX 
};

// retryct xb[0]	// Counter for number of retries for TERR errors
// maxretryct xb[1]	// Maximum number of TERR retry counts
// bits	   xb[2]		// Use these bits to set some conditions (see below)
// nosend  xb[1]	// Do not send: 0 = send; 1 = do NOT send on CAN bus (internal use only)
/*  -- bits -- */
#define	SOFTNART	0x01		// 1 = No retries (including arbitration); 0 = retries
#define NOCANSEND	0x02		// 1 = Do not send to the CAN bus

struct CAN_POOLBLOCK	// Used for common CAN TX/RX linked lists
{
volatile struct CAN_POOLBLOCK* volatile plinknext;	// Linked list pointer (low value id -> high value)
	 struct CANRCVBUF can;		// Msg queued
	 union  CAN_X x;			// Extra goodies that are different for TX and RX

};

/* Here: everything you wanted to know about a CAN module (i.e. CAN1, CAN2, CAN3) */
struct CAN_CTLBLOCK
{
	CAN_HandleTypeDef* phcan; // Ptr to 'MX/ST control block ("handle")

	struct CAN_POOLBLOCK  frii;	// Always present block, i.e. list pointer head

	uint32_t mbx0;	// CAN id last loaded into Mailbox 0

volatile struct CAN_POOLBLOCK  pend;		// Always present block, i.e. list pointer head
volatile struct CAN_POOLBLOCK* volatile pxprv;	// pxprv->plinknext points to msg being sent.  pxprv is NULL if TX is idle.

	uint32_t abortflag;	// 1 = ABRQ0 bit in TSR was set.

	struct CANWINCHPODCOMMONERRORS can_errors;	// A group of error counts
	uint32_t	bogusct;	// Count of bogus CAN IDs rejected
	s8 	ret;		   // Return code from routine call
};

/******************************************************************************/
struct CAN_CTLBLOCK* can_iface_init(CAN_HandleTypeDef *phcan, uint32_t numtx);
/* @brief 	: Setup linked list for TX priority sorted buffering
 * @param	: phcan = Pointer "handle" to HAL control block for CAN module
 * @param	: numtx = number of CAN msgs for TX buffering
 * @return	: Pointer to our knows-all control block for this CAN
 *		:  NULL = calloc failed
 *		:  Pointer->ret = pointer to CAN control block for this CAN unit
*******************************************************************************/
int can_driver_put(struct CAN_CTLBLOCK* pctl, struct CANRCVBUF *pcan, u8 maxretryct, u8 bits);
/* @brief	: Get a free slot and add CAN msg
 * @param	: pctl = pointer to control block for this CAN modules
 * @param	: pcan = pointer to msg: id, dlc, data (common_can.h)
 * @param	: maxretryct =  0 = use TERRMAXCOUNT; not zero = use this value.
 * @param	: bits = Use these bits to set some conditions (see .h file)
 * @return	:  0 = OK; 
 *				: -1 = Buffer overrun (no free slots for the new msg)
 *				: -2 = Bogus CAN id rejected
 ******************************************************************************/

#endif 

