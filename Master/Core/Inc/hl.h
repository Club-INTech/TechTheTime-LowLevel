#pragma once

#include "stm32g4xx_hal.h"

void HL_Init(UART_HandleTypeDef *);
void HL_Write_Byte(uint8_t);
