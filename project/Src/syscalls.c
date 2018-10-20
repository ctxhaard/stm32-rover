/*
 * syscalls.c
 *
 *  Created on: Oct 12, 2018
 *      Author: ctomasin
 */

// NOTE: sembra che sull STM32F0 non ci sia ITM!!!
#if 0

#include "main.h"
#include "stm32f0xx_hal.h"


int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
	   ITM_SendChar( *ptr++ );
	}

	return len;
}

#endif
