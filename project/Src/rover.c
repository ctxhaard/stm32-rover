/*
 * rover.c
 *
 *  Created on: Oct 31, 2018
 *      Author: ctomasin
 */

#include "rover.h"
#include "l298n.h"
#include "usart.h"

#include <string.h>
#include <stdlib.h>

#define RECEIVE_BUF_SIZE 12
#define READ_SIZE         7  // size of smaller command i.e \r\nend\r\n

osThreadId frontSensorPulseTaskHandle;
osThreadId uartTaskHandle;

osMessageQId(distanceQueueHandle);

struct command_t {
	uint8_t size;
	char command[6];
};

osMailQDef(command_q,10,struct command_t);
osMailQId(command_q_id);

enum {
	READ_STATUS_BEFORE,
	READ_STATUS_BEGIN_R_FOUND,
	READ_STATUS_COMMAND_FOUND,
	READ_STATUS_END_R_FOUND,
};

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
			  osSignalSet(frontSensorPulseTaskHandle,SIGNAL_FLAG_PROX);
			  osSignalSet(defaultTaskHandle,SIGNAL_FLAG_PROX);
		  }
		  __HAL_TIM_SET_COUNTER(htim,0);
	  }
}

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
	int index = rxIndex;
	for(index = 0; (index - rxIndex) < rxNextReadSize && index < RECEIVE_BUF_SIZE; ++index) {
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
			}
			break;
		case READ_STATUS_COMMAND_FOUND:
			if ( rxBuffer [ index ] == '\r')
				rxStatus = READ_STATUS_END_R_FOUND;
			break;
		case READ_STATUS_END_R_FOUND:
			if ( rxBuffer [ index ] == '\n') {
				int rxCommandEnd = (index - 2);
				// il comando va da rxCommandBegin a (index - 2)
				struct command_t *pCmd;
				pCmd = (struct command_t*)osMailAlloc(command_q_id, 0); // ISR devono mettere 0 come timeout
				pCmd->size = (rxCommandEnd - rxCommandBegin);
				memcpy(pCmd->command,&rxBuffer[rxCommandBegin], (rxCommandEnd - rxCommandBegin));
				osMailPut(command_q_id, pCmd);
				osSignalSet(defaultTaskHandle,SIGNAL_FLAG_COMMAND);
			}
			rxStatus = READ_STATUS_BEFORE;
//			rxCommandBegin = 0;
//			rxNextReadSize = READ_SIZE;
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
	osSignalSet(uartTaskHandle,SIGNAL_FLAG_UART);
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
	//__WFE();
}

/**
 * Specific tasks initialization; Default task is initialized by ST CubeMX in freertos.c
 * */
void rover_tasks_init()
{
	// coda di ricezione delle misurazioni dei sensori di prossimità
	osMessageQDef(sensors_mq, 5, uint32_t);
	distanceQueueHandle = osMessageCreate(osMessageQ(sensors_mq), defaultTaskHandle);

	osThreadDef(frontTask, StartFrontSensorPulseTask, osPriorityNormal, 0, 128);
	frontSensorPulseTaskHandle = osThreadCreate(osThread(frontTask), NULL);

	osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
	uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);
}

enum {
	CMD_STOP = 0,
	CMD_0_45 = 1,
	CMD_45_90 = 2,
	CMD_90_135 = 3,
	CMD_135_180 = 4,
	CMD_180_225 = 5,
	CMD_225_270 = 6,
	CMD_270_315 =7,
	CMD_315_360 = 8
};


static int commandGetCmd(const struct command_t *pCmd) {
	int result= CMD_STOP;
	if (pCmd->size > 2 && pCmd->command[1] == ':'){
		char *tail;
		const char *v = pCmd->command;
		long int val = strtol(v,&tail,0);
		if (v != tail) {
			result = val;
		}
		//result = atoi(pCmd->command);
	}
	return result;
}

static int commandGetForce(const struct command_t *pCmd) {
	int result = 0;
	if (pCmd->size > 2 && pCmd->command[1] == ':'){
		//result = atoi(&pCmd->command[2]);
		char *tail;
		const char *v = &pCmd->command[2];
		long int val = strtol(v,&tail,0);
		if (v != tail) {
			result = val;
		}
	}
	return result;
}


#define MIN_DISTANCE_MM 100

static uint32_t last_dist_mm = 0;
static int last_cmd = CMD_STOP;
static int last_cmd_force = 0;

void do_control()
{
	if (!_start || CMD_STOP == last_cmd || last_cmd_force == 0) {
		l298n_roll();
	} else if (last_dist_mm <= MIN_DISTANCE_MM && (CMD_315_360 == last_cmd || CMD_0_45 == last_cmd)) {
		l298n_roll();
	} else {
		int power_r = 0;
		int dir_r = MOTOR_DIR_FORWARD;
		int power_l = 0;
		int dir_l = MOTOR_DIR_FORWARD;
		switch (last_cmd) {
		case CMD_0_45:
			// TODO: assegnare i valori appropriati
			dir_l = MOTOR_DIR_FORWARD;
			dir_r = MOTOR_DIR_FORWARD;
			power_l = last_cmd_force;
			power_r = last_cmd_force;
			break;
		case CMD_45_90:
			// TODO: assegnare i valori appropriati
			dir_l = MOTOR_DIR_FORWARD;
			dir_r = MOTOR_DIR_FORWARD;
			power_l = last_cmd_force;
			power_r = 0;
			break;
		case CMD_90_135:
			// TODO: assegnare i valori appropriati
			dir_l = MOTOR_DIR_REVERSE;
			dir_r = MOTOR_DIR_REVERSE;
			power_l = last_cmd_force;
			power_r = 0;
			break;
		case CMD_135_180:
			// TODO: assegnare i valori appropriati
			dir_l = MOTOR_DIR_REVERSE;
			dir_r = MOTOR_DIR_REVERSE;
			power_l = last_cmd_force;
			power_r = last_cmd_force;
			break;
		case CMD_180_225:
			// TODO: assegnare i valori appropriati
			dir_l = MOTOR_DIR_REVERSE;
			dir_r = MOTOR_DIR_REVERSE;
			power_l = last_cmd_force;
			power_r = last_cmd_force;
			break;
		case CMD_225_270:
			// TODO: assegnare i valori appropriati
			dir_l = MOTOR_DIR_REVERSE;
			dir_r = MOTOR_DIR_REVERSE;
			power_l = 0;
			power_r = last_cmd_force;
			break;
		case CMD_270_315:
			dir_l = 1;
			dir_r = 1;
			break;
		case CMD_315_360:
			dir_l = 1;
			dir_r = 1;
			power_l = last_cmd_force;
			power_r = last_cmd_force;
			break;
		}
		l298n_power(power_l, dir_l, power_r, dir_r);
	}
}


/**
 * Default task main loop; task function in prepared by ST Cube MX into freertos.c
 * */
void default_task_loop()
{
	for(;;)
	{
		osSignalWait(0, osWaitForever);
		osEvent cmdEvent = osMailGet(command_q_id,0);
		osEvent proxyEvent = osMessageGet(distanceQueueHandle,0);

		if (cmdEvent.status == osEventMessage) {
			struct command_t *pCmd =(struct command_t*)cmdEvent.value.p;
			last_cmd = commandGetCmd(pCmd);
			last_cmd_force = commandGetForce(pCmd);
			last_cmd_force /= 2; // valori 0 - 200 => 0 - 100; sopra 200 verranno portati a 100 da do_command

			osMailFree(command_q_id,cmdEvent.value.p);
		}
		if (proxyEvent.status == osEventMessage) {
			last_dist_mm = proxyEvent.value.v;
		}
		do_control();
	}
}

/**
 * Proximity sensors task: produces the impulse on the TRIGGER signal
 * */
void StartFrontSensorPulseTask(void const * argument)
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
void StartUartTask(void const * argument)
{
	command_q_id = osMailCreate(osMailQ(command_q), NULL);
	for(;;)
	{
		osDelay(1);
		HAL_UART_Receive_IT(&huart2,rxBuffer + rxIndex, rxNextReadSize);
		osSignalWait(SIGNAL_FLAG_UART, /*osWaitForever*/1000);
	}
}
