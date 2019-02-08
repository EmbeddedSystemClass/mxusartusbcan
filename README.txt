README.txt
02/04/2019


A.  TASKS 

/*********************** CanTask01 ************************************************
** Set to Idle (-3) FreeRTOS priority
** List stack usage of tasks
** Test yprintf and sending with multiple serial buffers to one uart
** Test floating point linking and working
***********************************************************************************/

*********************** DefaultTask *******************************************
** Test usb-cdc (ttyACM0) sending
** Test sending fixed CAN msg
*******************************************************************************

*********************** Task02 ************************************************
** Test sscanf: incoming ascii on usart6
 * Test ADC: three inputs plus internal temperature and voltage reference
 * Multiple bits on TastNotifyWait plus yprintf
*******************************************************************************

********************* Task03 **************************************************
** Receive CAN msgs placed on queue by CAN callback in 'iface.c'
*******************************************************************************

CanRxTask

CanTxTask

SerialTaskReceive

SerialTaskSend

B.  QUEUES



C.  SEMAPHORES
