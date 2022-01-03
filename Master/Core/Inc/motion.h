#pragma once

#include <stdint.h>

#include "stm32g4xx_hal.h"

void Motion_Init(TIM_HandleTypeDef *, TIM_HandleTypeDef *, TIM_HandleTypeDef *);
uint32_t Motion_Get_Left_Ticks();
uint32_t Motion_Get_Right_Ticks();
void Motion_Update_Left_Setpoint(int64_t);
void Motion_Update_Right_Setpoint(int64_t);
int64_t Motion_Compute_PID(uint32_t, uint32_t, uint64_t, uint64_t, uint64_t);
