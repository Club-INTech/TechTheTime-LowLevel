#pragma once

#include <stdint.h>

#include "stm32g4xx_hal.h"

uint16_t update_crc(uint16_t, uint8_t *, uint16_t);
void update_crc_packet (uint8_t *, uint16_t);
