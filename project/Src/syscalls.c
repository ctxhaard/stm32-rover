/*
 * syscalls.c
 *
 *  Created on: Oct 12, 2018
 *      Author: ctomasin
 */

#include "usart.h"
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

//#define DEBUG

int _write(int file, char *data, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }
#ifdef DEBUG
   HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
   return (status == HAL_OK ? len : 0);
#else
   return len;
#endif   // return # of bytes written - as best we can tell
}
