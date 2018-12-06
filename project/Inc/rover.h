/*
 * rover.h
 *
 *  Created on: Oct 31, 2018
 *      Author: ctomasin
 */

#ifndef ROVER_H_
#define ROVER_H_

#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

extern osThreadId frontSensorPulseTaskHandle;
extern osThreadId uartTaskHandle;
extern osThreadId defaultTaskHandle;

extern QueueHandle_t distanceQueueHandle;

osMailQId(command_q_id);

struct command_t {
	uint8_t size;
	char command[6];
};

#define SIGNAL_FLAG_BTN (1 << 0)
#define SIGNAL_FLAG_UART (1 << 1)
#define SIGNAL_FLAG_PROX (1 << 2)
#define SIGNAL_FLAG_COMMAND (1 << 3)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void rover_IRQHandler(UART_HandleTypeDef *huart);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void rover_tasks_init();

void default_task_loop();

void StartFrontSensorPulseTask(void const * argument);

void StartUartTask(void const * argument);

#endif /* ROVER_H_ */
