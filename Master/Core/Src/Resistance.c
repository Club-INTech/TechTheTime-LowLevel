#include "motion.h"
#include "motion_config.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <order/motion.h>
#include <order/type.h>

#include "Resistance.h"
#include "order/Resistance.h"
static ADC_HandleTypeDef * ADC_ptr = NULL;

void Res_Init(ADC_HandleTypeDef * Adc_handler){
	ADC_ptr = Adc_handler;
	HAL_ADC_Start(ADC_ptr);
}

uint8_t giveRes(void){
	HAL_ADC_Start(ADC_ptr);
	int Rbot=2000;
	float V=3.3;
	HAL_ADC_PollForConversion(ADC_ptr,1000);

	float Ur =3.3 - HAL_ADC_GetValue(ADC_ptr)*3.3/256;
	int Rcarre = -Rbot*Ur /(Ur-V);
	HAL_Delay(10);
	return (uint8_t) Rcarre ;
}
