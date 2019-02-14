/******************************************************************************
* File Name          : morse.h
* Date First Issued  : 02/13/2019
* Description        : Morse code
*******************************************************************************/

#ifndef __MORSE
#define __MORSE

/* ************************************************************************* */
void morse_string(char* p);
/*	@brief	: Send a character string as Morse code
 * @param	: p = pointer to string
 * *************************************************************************/
void morse_number(uint32_t n);
/*	@brief	: Send a character string as Morse code
 * @param	: nx = number to send
 * *************************************************************************/
void morse_trap(uint8_t x);
/*	@brief	: Send a character string as Morse code
 * @param	: x = trap number to flash
 * *************************************************************************/

#endif

