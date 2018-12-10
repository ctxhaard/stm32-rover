/*
 * rover.c
 *
 *  Created on: Oct 31, 2018
 *      Author: ctomasin
 */

#include "rover.h"
#include "l298n.h"

#include <stdlib.h>

volatile int _start = 0;

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
	printf("starting main task\n");
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


#define MIN_DISTANCE_MM 200

static uint32_t last_dist_mm = 0;
static int last_cmd = CMD_STOP;
static int last_cmd_force = 0;

void do_control()
{
	if (!_start || CMD_STOP == last_cmd || last_cmd_force == 0) {
		puts("roll(1)\n");
		l298n_roll();
	} else if (last_dist_mm <= MIN_DISTANCE_MM
			&& (CMD_315_360 == last_cmd || CMD_0_45 == last_cmd
					|| CMD_45_90 == last_cmd || CMD_270_315 == last_cmd)) {
		puts("roll(2)\n");
		l298n_brake();
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
			dir_l = MOTOR_DIR_FORWARD;
			dir_r = MOTOR_DIR_REVERSE;
			power_l = last_cmd_force;
			power_r = last_cmd_force;
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
			dir_r = MOTOR_DIR_FORWARD;
			power_l = last_cmd_force;
			power_r = last_cmd_force;
			break;
		case CMD_270_315:
			dir_l = MOTOR_DIR_FORWARD;
			dir_r = MOTOR_DIR_FORWARD;
			power_l = 0;
			power_r = last_cmd_force;
			break;
		case CMD_315_360:
			dir_l = MOTOR_DIR_FORWARD;
			dir_r = MOTOR_DIR_FORWARD;
			power_l = last_cmd_force;
			power_r = last_cmd_force;
			break;
		}
		printf("go: %d:%d, %d:%d\n", power_l, dir_l, power_r, dir_r);
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

		if (cmdEvent.status == osEventMail) {
			struct command_t *pCmd =(struct command_t*)cmdEvent.value.p;
//			printf("command: %s\n", pCmd->command);
			last_cmd = commandGetCmd(pCmd);
			last_cmd_force = commandGetForce(pCmd);
			last_cmd_force /= 2; // valori 0 - 200 => 0 - 100; sopra 200 verranno portati a 100 da do_command
			printf("cmd: %d frc: %d\n", last_cmd, last_cmd_force);

			osMailFree(command_q_id,cmdEvent.value.p);
		}
		if (proxyEvent.status == osEventMessage) {
			last_dist_mm = proxyEvent.value.v;
		}
		do_control();
	}
}
