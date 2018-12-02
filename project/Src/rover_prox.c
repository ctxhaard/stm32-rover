#include "rover.h"

volatile int edge_num = 0;

osThreadId frontSensorPulseTaskHandle;
osMessageQId(distanceQueueHandle);

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
			  osSignalSet(frontSensorPulseTaskHandle,SIGNAL_FLAG_PROX);
			  osSignalSet(defaultTaskHandle,SIGNAL_FLAG_PROX);
		  }
		  __HAL_TIM_SET_COUNTER(htim,0);
	  }
}

/**
 * Proximity sensors task: produces the impulse on the TRIGGER signal
 * */
void StartFrontSensorPulseTask(void const * argument)
{
	printf("starting Front sensor task\n");
	for(;;)
	{
		HAL_GPIO_WritePin(TRIG1_GPIO_Port,TRIG1_Pin, GPIO_PIN_SET);
		osDelay(1); // basterebbero 100 us
		edge_num = 0;
		HAL_GPIO_WritePin(TRIG1_GPIO_Port,TRIG1_Pin, GPIO_PIN_RESET);
		osSignalWait(SIGNAL_FLAG_PROX, osWaitForever);
	}
}
