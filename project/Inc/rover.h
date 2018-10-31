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

extern volatile int _start;
extern volatile int edge_num;
//extern volatile uint32_t dist_mm;

extern osThreadId frontSensorPulseTaskHandle;
extern osThreadId bluetoothTaskHandle;
extern osThreadId defaultTaskHandle;

extern QueueHandle_t distanceQueueHandle;

#define SIGNAL_FLAG_BTN (1 << 0)
#define SIGNAL_FLAG_BT (1 << 1)
#define SIGNAL_FLAG_PROX (1 << 2)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void rover_tasks_init();

void default_task_loop();

void StartFrontSensorPulseTask(void const * argument);

void StartBluetoothTask(void const * argument);

#endif /* ROVER_H_ */
