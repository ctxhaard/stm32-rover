/*
 * l298n.h
 *
 *  Created on: Oct 12, 2018
 *      Author: ctomasin
 */

#ifndef L298N_H_
#define L298N_H_

#include "stm32f0xx_hal.h"

enum {
	MOTOR_DIR_FORWARD = 1,
	MOTOR_DIR_REVERSE = 0,
};

void l298n_init(TIM_HandleTypeDef *);

/**
 * @param left_power value 0 - 100
 * @param left_dir MOTOR_DIR_FORWARD or MOTOR_DIR_REVERSE
 * @param right_power value 0 - 100
 * @param right_dir MOTOR_DIR_FORWARD or MOTOR_DIR_REVERSE
 * */
void l298n_power(int left_power,int left_dir, int right_power, int right_dir);
void l298n_roll();
void l298n_brake();

#endif /* L298N_H_ */
