/*
 * rover.c
 *
 *  Created on: Oct 31, 2018
 *      Author: ctomasin
 */

#include "usart.h"
#include "rover.h"

#include <string.h>

#define RECEIVE_BUF_SIZE 1024

osThreadId uartTaskHandle;

osMailQDef(command_q,10,struct command_t);

enum {
	READ_STATUS_BEFORE,
	READ_STATUS_BEGIN_R_FOUND,
	READ_STATUS_COMMAND_FOUND,
	READ_STATUS_END_R_FOUND,
};

static
uint8_t rxBuffer[RECEIVE_BUF_SIZE];

static
uint8_t *pRead = rxBuffer;

static
void do_parse_input(UART_HandleTypeDef *huart)
{
	static volatile int rxStatus = READ_STATUS_BEFORE;
	static uint8_t *pCmdBegin = rxBuffer;

	for(; pRead < huart->pRxBuffPtr; pRead++) {
		switch (rxStatus) {
		case READ_STATUS_BEFORE:
			if ( *pRead == '\r' )
				rxStatus = READ_STATUS_BEGIN_R_FOUND;
			break;
		case READ_STATUS_BEGIN_R_FOUND:
			if ( *pRead == '\n' ) {
				rxStatus = READ_STATUS_COMMAND_FOUND;
				pCmdBegin = pRead + 1;
			} else{
				rxStatus = READ_STATUS_BEFORE;
				pCmdBegin = rxBuffer;
				return;
			}
			break;
		case READ_STATUS_COMMAND_FOUND:
			if ( *pRead == '\r')
				rxStatus = READ_STATUS_END_R_FOUND;
			break;
		case READ_STATUS_END_R_FOUND:
			if ( *pRead == '\n') {
				uint8_t *pCmdEnd =  pRead - 1;
				// il comando va da rxCommandBegin a (index - 2)
				struct command_t *pCmd;
				pCmd = (struct command_t*)osMailAlloc(command_q_id, 0); // ISR devono mettere 0 come timeout
				if (pCmd) {
					int cmdLen = (pCmdEnd - pCmdBegin);
					int reminderLen = ( huart->pRxBuffPtr - pCmdEnd);

					pCmd->size = cmdLen;
					memcpy(pCmd->command, pCmdBegin, cmdLen);
					osMailPut(command_q_id, pCmd);
					osSignalSet(defaultTaskHandle,SIGNAL_FLAG_COMMAND);
					memcpy(rxBuffer, pCmdEnd, reminderLen);
					huart->pRxBuffPtr = rxBuffer + reminderLen;
					huart->RxXferCount = sizeof ( rxBuffer ) - reminderLen;
					pRead = rxBuffer;
				}
			}
			rxStatus = READ_STATUS_BEFORE;
			pCmdBegin = rxBuffer;
			break;
		default:
			break;
		}
	}
}

/**
 * Input from esp8266
 * */
void rover_RxCallback(UART_HandleTypeDef *huart)
{
	do_parse_input(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	puts("Buffer full!!!\n");
	pRead = rxBuffer;
	osSignalSet(uartTaskHandle,SIGNAL_FLAG_UART);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	puts("Transfer error!!!\n");
	pRead = rxBuffer;
	osSignalSet(uartTaskHandle,SIGNAL_FLAG_UART);
}


HAL_StatusTypeDef rover_Receive_IT(UART_HandleTypeDef *huart)
{
  uint16_t* tmp;
  uint16_t  uhMask = huart->Mask;
  uint16_t  uhdata;

  /* Check that a Rx process is ongoing */
  if(huart->RxState == HAL_UART_STATE_BUSY_RX)
  {
    uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      tmp = (uint16_t*) huart->pRxBuffPtr ;
      *tmp = (uint16_t)(uhdata & uhMask);
      huart->pRxBuffPtr +=2U;
    }
    else
    {
      *huart->pRxBuffPtr++ = (uint8_t)(uhdata & (uint8_t)uhMask);
      rover_RxCallback(huart);
    }

    if(--huart->RxXferCount == 0U)
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupt*/
      CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

      /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
      CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

      /* Rx process is completed, restore huart->RxState to Ready */
      huart->RxState = HAL_UART_STATE_READY;

      HAL_UART_RxCpltCallback(huart);

      return HAL_OK;
    }

    return HAL_OK;
  }
  else
  {
    /* Clear RXNE interrupt flag */
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

    return HAL_BUSY;
  }
}

static void UART_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef* huart = (UART_HandleTypeDef*)(hdma->Parent);
  huart->RxXferCount = 0U;
  huart->TxXferCount = 0U;

  HAL_UART_ErrorCallback(huart);
}

static void UART_EndRxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* At end of Rx process, restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;
}

void rover_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its;
  uint32_t errorflags;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
      rover_Receive_IT(huart);
      return;
    }
  }

  /* If some errors occur */
  cr3its = READ_REG(huart->Instance->CR3);
  if(   (errorflags != RESET)
     && (   ((cr3its & USART_CR3_EIE) != RESET)
         || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)) )
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);

      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);

      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);

      huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if(((isrflags & USART_ISR_ORE) != RESET) &&
       (((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET)))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);

      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    /* Call UART Error Call back function if need be --------------------------*/
    if(huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver ---------------------------------------------------*/
      if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
      {
        rover_Receive_IT(huart);
      }

      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
         consider error as blocking */
      if (((huart->ErrorCode & HAL_UART_ERROR_ORE) != RESET) ||
          (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)))
      {
        /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        UART_EndRxTransfer(huart);

        /* Disable the UART DMA Rx request if enabled */
        if (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
        {
          CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

          /* Abort the UART DMA Rx channel */
          if(huart->hdmarx != NULL)
          {
            /* Set the UART DMA Abort callback :
               will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
            huart->hdmarx->XferAbortCallback = UART_DMAAbortOnError;

            /* Abort DMA RX */
            if(HAL_DMA_Abort_IT(huart->hdmarx) != HAL_OK)
            {
              /* Call Directly huart->hdmarx->XferAbortCallback function in case of error */
              huart->hdmarx->XferAbortCallback(huart->hdmarx);
            }
          }
          else
          {
            /* Call user error callback */
            HAL_UART_ErrorCallback(huart);
          }
        }
        else
        {
          /* Call user error callback */
          HAL_UART_ErrorCallback(huart);
        }
      }
      else
      {
        /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
        HAL_UART_ErrorCallback(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
      }
    }
    return;

  } /* End if some error occurs */

#if !defined(STM32F030x6) && !defined(STM32F030x8)&& !defined(STM32F070xB)&& !defined(STM32F070x6)&& !defined(STM32F030xC)
  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
  if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
  {
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);
    /* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_WakeupCallback(huart);
    return;
  }
#endif /* !defined(STM32F030x6) && !defined(STM32F030x8)&& !defined(STM32F070xB)&& !defined(STM32F070x6)&& !defined(STM32F030xC) */

  /* UART in mode Transmitter ------------------------------------------------*/
  if(((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
  {
    UART_Transmit_IT(huart);
    return;
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
    UART_EndTransmit_IT(huart);
    return;
  }

}

/**
 * UART task
 * */
void StartUartTask(void const * argument)
{
	puts("starting UART task\n");
	command_q_id = osMailCreate(osMailQ(command_q), NULL);

	for(;;)
	{
		if (HAL_OK != HAL_UART_Receive_IT(&huart2, rxBuffer, sizeof(rxBuffer))) {
			puts("cannot start receiving data\n");
		}
		osSignalWait(SIGNAL_FLAG_UART, osWaitForever);
	}
}
