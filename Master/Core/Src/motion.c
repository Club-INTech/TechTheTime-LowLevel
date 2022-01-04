#include "motion.h"

#include <math.h>

static TIM_HandleTypeDef *left_encoder_handler = NULL;
static TIM_HandleTypeDef *right_encoder_handler = NULL;
static TIM_HandleTypeDef *pwm_handler = NULL;

static int64_t error_I = 0;
static uint32_t previous_position = 0;

void Motion_Init(TIM_HandleTypeDef *_left_encoder_handler, TIM_HandleTypeDef *_right_encoder_handler, TIM_HandleTypeDef *_pwm_handler)
{
	left_encoder_handler = _left_encoder_handler;
	right_encoder_handler = _right_encoder_handler;
	pwm_handler = _pwm_handler;
}

uint32_t Motion_Get_Left_Ticks()
{
	return __HAL_TIM_GET_COUNTER(left_encoder_handler);
}

uint32_t Motion_Get_Right_Ticks()
{
	return __HAL_TIM_GET_COUNTER(right_encoder_handler);
}

void Motion_Update_Left_PWM(int64_t pwm)
{
	if (pwm > 0) {
		__HAL_TIM_SET_COMPARE(pwm_handler, TIM_CHANNEL_1, pwm);
		__HAL_TIM_SET_COMPARE(pwm_handler, TIM_CHANNEL_2, 0);
	}
	else {
		__HAL_TIM_SET_COMPARE(pwm_handler, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(pwm_handler, TIM_CHANNEL_2, -pwm);
	}
}

void Motion_Update_Right_PWM(int64_t pwm)
{
 	if (pwm > 0) {
		__HAL_TIM_SET_COMPARE(pwm_handler, TIM_CHANNEL_3, pwm);
		__HAL_TIM_SET_COMPARE(pwm_handler, TIM_CHANNEL_4, 0);
	}
	else {
		__HAL_TIM_SET_COMPARE(pwm_handler, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(pwm_handler, TIM_CHANNEL_4, -pwm);
	}
}

// b position désirée, c position actuelle (ici la position désirée est celle de la roue jumelle
int64_t Motion_Compute_PID(uint32_t setpoint, uint32_t position, double kp, double kd, double ki)
{
	int64_t error_P = ((int64_t) setpoint) - ((int64_t) position);
	error_I += error_P;
	int64_t error_D = position - previous_position;
	previous_position = position;
	int64_t pwm = kp * error_P + kd * error_D + ki * error_I;
	if (pwm > UINT32_MAX) {
		return UINT32_MAX;
	}
	else if (-pwm > UINT32_MAX){
		return -((int64_t) UINT32_MAX);
	}
	else {
	    return pwm;
	}
}
