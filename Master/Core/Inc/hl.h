#pragma once

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void HL_Init(UART_HandleTypeDef *);
void HL_Send_Measure(void);

#ifdef __cplusplus
}
#endif // __cplusplus
