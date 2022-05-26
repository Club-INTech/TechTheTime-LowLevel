#pragma once

#include "stm32l4xx.h"

void Sick_Init(ADC_HandleTypeDef *);
float Sick_Measure (void);
