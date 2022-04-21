#include "motion.h"
#include "motion_config.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <order/motion.h>
#include <order/type.h>
static ADC_HandleTypeDef * ADC_ptr = NULL;

void Res_Init(ADC_HandleTypeDef * Adc_handler){
	ADC_ptr = Adc_handler;
}

uint8_t giveRes(void){
	HAL_ADC_Start(ADC_ptr);
	int Rbot=2000;
	int V=5;
	int Ur =HAL_ADC_GetValue(ADC_ptr)* 5/32768;
	int Rcarre = Rbot*V/Ur - Rbot;
	return (uint8_t) Rcarre ;
}



