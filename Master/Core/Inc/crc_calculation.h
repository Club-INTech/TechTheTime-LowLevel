#pragma once

#include <stdint.h>

#include "stm32g4xx_hal.h"

uint16_t update_crc(uint16_t, unsigned char *, uint16_t);
void update_crc_packet (unsigned char *, uint16_t);
