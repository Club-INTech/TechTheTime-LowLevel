#pragma once

#include "stm32g4xx_hal.h"

extern uint8_t has_received_order;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void HL_Init(UART_HandleTypeDef *);
void HL_Send_Measure(void);
void HL_Interrupt(void);

#ifdef __cplusplus
}
#endif // __cplusplus
