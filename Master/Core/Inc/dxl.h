#pragma once

#include <stdint.h>
#include <math.h>

#include "stm32g4xx.h"

void DXL_Init(UART_HandleTypeDef *);
void DXL_Transmit(uint8_t *, uint16_t);
void DXL_Torque_On(uint8_t);
void DXL_Torque_Off(uint8_t);
void DXL_Light_On(uint8_t);
void DXL_Light_Off(uint8_t);
void DXL_Update_Id(uint8_t, uint8_t);
void DXL_Sync_Position(uint8_t *, uint8_t);
