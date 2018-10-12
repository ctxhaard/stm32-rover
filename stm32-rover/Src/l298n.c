/*
 * l298n.c
 *
 *  Created on: Oct 12, 2018
 *      Author: ctomasin
 */
#include "l298n.h"

#define MIN(a,b) ((a) > (b) ? (b) : (a))

#define POWER_MAX 200
#define POWER_MIN 100

TIM_HandleTypeDef *_phtim;

inline int real_power(int power) {
	int result = MIN(POWER_MAX, POWER_MIN + power);
	if (result <= POWER_MIN) result = 0;
	return result;
}

void l298n_init(TIM_HandleTypeDef *phtim)
{
	_phtim = phtim;

	// left motor enable
	HAL_TIM_PWM_Start(phtim,TIM_CHANNEL_3);
	// right motor enable
	HAL_TIM_PWM_Start(phtim,TIM_CHANNEL_4);

	l298n_roll();
}

void l298n_power(int left_power,int left_dir, int right_power, int right_dir)
{
	__HAL_TIM_SET_COMPARE(_phtim,TIM_CHANNEL_4, real_power(left_power));
	HAL_GPIO_WritePin(GPIOA,IN1_Pin, left_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,IN2_Pin, left_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(_phtim,TIM_CHANNEL_3, real_power(right_power));
	HAL_GPIO_WritePin(GPIOA,IN3_Pin, right_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,IN4_Pin, right_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void l298n_roll()
{
	__HAL_TIM_SET_COMPARE(_phtim, TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(_phtim, TIM_CHANNEL_4,0);

	HAL_GPIO_WritePin(GPIOA,IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin,GPIO_PIN_RESET);
}

void l298n_brake()
{
	__HAL_TIM_SET_COMPARE(_phtim, TIM_CHANNEL_3, POWER_MAX);
	__HAL_TIM_SET_COMPARE(_phtim, TIM_CHANNEL_4, POWER_MAX);

	HAL_GPIO_WritePin(GPIOA,IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin,GPIO_PIN_SET);
}
