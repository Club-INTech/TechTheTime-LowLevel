#pragma once

#include <stdint.h>

#include "stm32g4xx_hal.h"

typedef uint32_t Motion_Channel;

typedef enum {
	MOTION_MOVEMENT_TYPE_FORWARD,
	MOTION_MOVEMENT_TYPE_BACKWARD,
	MOTION_MOVEMENT_TYPE_CLOCKWISE,
	MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE
} Motion_MovementType;

void Motion_Init(TIM_HandleTypeDef *, TIM_HandleTypeDef *, TIM_HandleTypeDef *);
uint16_t Motion_Get_Left_Ticks(Motion_MovementType);
uint16_t Motion_Get_Right_Ticks(Motion_MovementType);
void Motion_Update_Left_PWM(int64_t, Motion_Channel, Motion_Channel);
void Motion_Update_Right_PWM(int64_t, Motion_Channel, Motion_Channel);
int64_t Motion_Compute_PID(uint32_t, uint32_t, double, double, double);
double Motion_PWM_Base_Right(uint32_t, Motion_MovementType);
double Motion_PWM_Base_Left(uint32_t, Motion_MovementType);
void Motion_Translation_Forward(uint32_t);
void Motion_Translation_Backward(uint32_t);
void Motion_Rotation_clockwise(uint32_t);
void Motion_Rotation_counter_clockwise(uint32_t);

