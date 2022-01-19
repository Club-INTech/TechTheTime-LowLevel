#pragma once

#include <stdint.h>

#include "stm32g4xx_hal.h"

typedef int32_t Motion_Tick;
typedef int32_t Motion_PWM;

typedef enum {
	MOTION_CHANNEL_FORWARD_LEFT = TIM_CHANNEL_1,
	MOTION_CHANNEL_FORWARD_RIGHT = TIM_CHANNEL_2,
	MOTION_CHANNEL_BACKWARD_LEFT = TIM_CHANNEL_3,
	MOTION_CHANNEL_BACKWARD_RIGHT = TIM_CHANNEL_4
} Motion_Channel;

typedef enum {
	MOTION_MOVEMENT_TYPE_FORWARD,
	MOTION_MOVEMENT_TYPE_BACKWARD,
	MOTION_MOVEMENT_TYPE_CLOCKWISE,
	MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE
} Motion_MovementType;

void Motion_Init(TIM_HandleTypeDef *, TIM_HandleTypeDef *, TIM_HandleTypeDef *);
Motion_PWM Motion_Get_Left_Ticks(Motion_MovementType);
Motion_PWM Motion_Get_Right_Ticks(Motion_MovementType);
void Motion_Update_Left_PWM(Motion_PWM, Motion_Channel, Motion_Channel);
void Motion_Update_Right_PWM(Motion_PWM, Motion_Channel, Motion_Channel);
Motion_PWM Motion_Compute_PID(Motion_Tick, Motion_Tick, double, double, double);
double Motion_PWM_Base_Right(Motion_Tick, Motion_MovementType);
double Motion_PWM_Base_Left(Motion_Tick, Motion_MovementType);
void Motion_Translation_Forward(Motion_Tick);
void Motion_Translation_Backward(Motion_Tick);
void Motion_Rotation_clockwise(Motion_Tick);
void Motion_Rotation_counter_clockwise(Motion_Tick);

