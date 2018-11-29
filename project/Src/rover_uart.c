/*
 * rover.c
 *
 *  Created on: Oct 31, 2018
 *      Author: ctomasin
 */

#include "usart.h"
#include "rover.h"

#include <string.h>

#define RECEIVE_BUF_SIZE 256
#define READ_SIZE        16

osThreadId uartTaskHandle;

osMailQDef(command_q,10,struct command_t);

enum {
	READ_STATUS_BEFORE,
	READ_STATUS_BEGIN_R_FOUND,
	READ_STATUS_COMMAND_FOUND,
	READ_STATUS_END_R_FOUND,
};

static volatile int rxNextReadSize = READ_SIZE;
static uint8_t rxBuffer[RECEIVE_BUF_SIZE];
static volatile int rxIndex = 0;
static volatile int rxStatus = READ_STATUS_BEFORE;
static volatile int rxCommandBegin = 0;

/**
 * Input from esp8266
 * */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	osSignalSet(uartTaskHandle,SIGNAL_FLAG_UART);
}

static
void do_parse_input()
{
	int index;
	for(index = rxIndex; (index - rxIndex) < rxNextReadSize && index < RECEIVE_BUF_SIZE; ++index) {
		switch (rxStatus) {
		case READ_STATUS_BEFORE:
			if ( rxBuffer[ index ] == '\r' )
				rxStatus = READ_STATUS_BEGIN_R_FOUND;
			break;
		case READ_STATUS_BEGIN_R_FOUND:
			if ( rxBuffer[ index ] == '\n' ) {
				rxStatus = READ_STATUS_COMMAND_FOUND;
				rxCommandBegin = (index + 1);
			} else{
				rxStatus = READ_STATUS_BEFORE;
				rxIndex = 0;
				rxCommandBegin = 0;
				rxNextReadSize = READ_SIZE;
				return;
			}
			break;
		case READ_STATUS_COMMAND_FOUND:
			if ( rxBuffer [ index ] == '\r')
				rxStatus = READ_STATUS_END_R_FOUND;
			break;
		case READ_STATUS_END_R_FOUND:
			if ( rxBuffer [ index ] == '\n') {
				int rxCommandEnd = (index - 1);
				// il comando va da rxCommandBegin a (index - 2)
				struct command_t *pCmd;
				pCmd = (struct command_t*)osMailAlloc(command_q_id, 0); // ISR devono mettere 0 come timeout
				if (pCmd) {
					pCmd->size = (rxCommandEnd - rxCommandBegin);
					memcpy(pCmd->command,&rxBuffer[rxCommandBegin], (rxCommandEnd - rxCommandBegin));
					osMailPut(command_q_id, pCmd);
					osSignalSet(defaultTaskHandle,SIGNAL_FLAG_COMMAND);
					memcpy(rxBuffer,rxBuffer+rxCommandEnd+1,(rxIndex + rxNextReadSize - rxCommandEnd));
				}
			}
			rxStatus = READ_STATUS_BEFORE;
			rxCommandBegin = 0;
			rxNextReadSize = READ_SIZE;
			break;
		default:
			break;
		}
	}
	if (index >= RECEIVE_BUF_SIZE) {
		rxStatus = READ_STATUS_BEFORE;
		rxIndex = 0;
		rxCommandBegin = 0;
		rxNextReadSize = READ_SIZE;
	} else if ((index - rxIndex) >= rxNextReadSize) {
		rxIndex += rxNextReadSize;
		rxNextReadSize = 1;
	}
}


/**
 * UART task
 * */
void StartUartTask(void const * argument)
{
	command_q_id = osMailCreate(osMailQ(command_q), NULL);
	for(;;)
	{
		osDelay(1); // TODO: si può togliere o bisogna aumentarlo???
		HAL_UART_Receive_IT(&huart2,rxBuffer + rxIndex, rxNextReadSize);
		osEvent event = osSignalWait(SIGNAL_FLAG_UART, /*osWaitForever*/1000);
		if (event.status == osEventSignal) {
			do_parse_input();
		}
	}
}
