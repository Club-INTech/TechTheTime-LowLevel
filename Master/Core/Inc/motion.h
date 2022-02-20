#pragma once

#include <stdint.h>

#include "stm32g4xx_hal.h"

#include <order/motion.h>

typedef Shared_Tick Motion_Tick;  //rajouter un truc pour faire l'alias ?
typedef int32_t Motion_PWM;
typedef Shared_PID_K Motion_PID_K;

/* unsigned int channels[] = { faire tableau pour relier Motion_Channel à MO

}; */

typedef enum {
	MOTION_CHANNEL_FORWARD_LEFT = TIM_CHANNEL_3,
	MOTION_CHANNEL_FORWARD_RIGHT = TIM_CHANNEL_2,
	MOTION_CHANNEL_BACKWARD_LEFT = TIM_CHANNEL_4,
	MOTION_CHANNEL_BACKWARD_RIGHT = TIM_CHANNEL_1
} Motion_Channel;

typedef enum {
	MOTION_MOVEMENT_TYPE_NONE,
	MOTION_MOVEMENT_TYPE_FORWARD,
	MOTION_MOVEMENT_TYPE_BACKWARD,
	MOTION_MOVEMENT_TYPE_CLOCKWISE,
	MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE
} Motion_MovementType;

typedef struct {
	double kp, kd, ki;
	Motion_Tick previous_position, sum_error;
} Motion_PID_Profile;



void Motion_Init_Arg(Motion_MovementType, Motion_Tick);
void Motion_Init(TIM_HandleTypeDef *, TIM_HandleTypeDef *, TIM_HandleTypeDef *);
Motion_Tick Motion_Get_Left_Ticks(void);
Motion_Tick Motion_Get_Right_Ticks(void);
void Motion_Update_Left_PWM(Motion_PWM, Motion_Channel, Motion_Channel);
void Motion_Update_Right_PWM(Motion_PWM, Motion_Channel, Motion_Channel);
Motion_PWM Motion_Compute_PID(Motion_Tick, Motion_Tick, Motion_PID_Profile *);
double Motion_PWM_Base_Right(Motion_Tick, Motion_MovementType);
double Motion_PWM_Base_Left(Motion_Tick, Motion_MovementType);
void Motion_Translation_Forward(Motion_Tick);
void Motion_Translation_Backward(Motion_Tick);
void Motion_Rotation_Clockwise(Motion_Tick);
void Motion_Rotation_Counter_Clockwise(Motion_Tick);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);

