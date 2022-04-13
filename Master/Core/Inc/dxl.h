#pragma once

#include <stdint.h>

#include "crc_calculation.h"

void DXL_Init(UART_HandleTypeDef *);
void DXL_Transmit(uint8_t *, uint16_t);
void DXL_Torque_On(uint8_t);
void DXL_Torque_Off(uint8_t);
void DXL_Position(uint8_t, uint32_t);
void DXL_Light_On(uint8_t);
void DXL_Light_Off(uint8_t);
void DXL_Update_Id(uint8_t, uint8_t);
