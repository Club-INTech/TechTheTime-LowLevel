#pragma once

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "stm32l4xx_hal.h"

void Service_Init(I2C_HandleTypeDef*, TIM_HandleTypeDef*, TIM_HandleTypeDef*);
void Service_Process(void);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
