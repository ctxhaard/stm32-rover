/*
 * rover.c
 *
 *  Created on: Oct 31, 2018
 *      Author: ctomasin
 */

#include "rover.h"
#include "l298n.h"

osThreadId sensorsTaskHandle;
osThreadId bluetoothTaskHandle;

osMessageQId(distanceQueueHandle);

/**
 * Blue pushbutton interrupt management
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (B1_Pin == GPIO_Pin) {
		// NOTE: blue pushbutton pressed
		_start = !_start;
		osSignalSet(defaultTaskHandle,SIGNAL_FLAG_BTN);
	}
}

/**
 * Input capture to measure pulse duration on proximity sensor ECHO signal
 * */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	  if (htim->Instance==TIM16 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

		  if (edge_num++ == 1) {
			  uint32_t capval = __HAL_TIM_GET_COMPARE(htim,TIM_CHANNEL_1);
			  //uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			  // un incremento di 1 (a 48MHz) corrisponde a 0.0071 mm unita'
			  // PSC a 20 ==> 0.143 mm / tick
			  uint32_t distmm = (capval * 0.143) / 2; // diviso 2 perche' andata e ritorno

			  osMessagePut(distanceQueueHandle, distmm, 0);
			  osSignalSet(sensorsTaskHandle,SIGNAL_FLAG_PROX);
		  }
		  __HAL_TIM_SET_COUNTER(htim,0);
	  }
}

/**
 * FreeRTOS method to keep the processor in low power mode when idle
 * */
__weak void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	__WFE();
}

/**
 * Specific tasks initialization; Default task is initialized by ST CubeMX in freertos.c
 * */
void rover_tasks_init()
{
	// coda di ricezione delle misurazioni dei sensori di prossimità
	osMessageQDef(sensors_mq, 5, uint32_t);
	distanceQueueHandle = osMessageCreate(osMessageQ(sensors_mq), defaultTaskHandle);

	osThreadDef(sensorsTask, StartSensorsTask, osPriorityNormal, 0, 128);
	sensorsTaskHandle = osThreadCreate(osThread(sensorsTask), NULL);

	osThreadDef(bluetoothTask, StartBluetoothTask, osPriorityNormal, 0, 128);
	bluetoothTaskHandle = osThreadCreate(osThread(bluetoothTask), NULL);
}

/**
 * Default task main loop; task function in prepared by ST Cube MX into freertos.c
 * */
void default_task_loop()
{
	for(;;)
	{
		osEvent event = osMessageGet(distanceQueueHandle,osWaitForever);
		if (event.status == osEventMessage) {
			uint32_t dist_mm = event.value.v;
			if (!_start) {
				l298n_roll();
			} else {
				int power = (dist_mm - 200);
				l298n_power(power,1,power,1);
			}
		}

	}
}

/**
 * Proximity sensors task: produces the impulse on the TRIGGER signal
 * */
void StartSensorsTask(void const * argument)
{
	for(;;)
	{
		HAL_GPIO_WritePin(TRIG1_GPIO_Port,TRIG1_Pin, GPIO_PIN_SET);
		osDelay(1); // basterebbero 100 us
		edge_num = 0;
		HAL_GPIO_WritePin(TRIG1_GPIO_Port,TRIG1_Pin, GPIO_PIN_RESET);
		osSignalWait(SIGNAL_FLAG_PROX, osWaitForever);
	}
}

/**
 * Bluetooth task
 * */
void StartBluetoothTask(void const * argument)
{
	// TODO: ottenere i comandi dal bluetooth
	for(;;)
	{
		osDelay(1);
	}
}
