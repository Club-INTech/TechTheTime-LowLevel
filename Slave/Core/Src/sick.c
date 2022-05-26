#include "sick.h"



static ADC_HandleTypeDef *handler_sick = NULL;



static double resistorValue = 0;



void Sick_Init(ADC_HandleTypeDef *_handler_sick) {
	handler_sick = _handler_sick;
}

float Sick_Measure (void) {
	HAL_ADC_Start(handler_sick);
	HAL_Delay(100); //
	HAL_ADC_Stop(handler_sick);
	double tension = HAL_ADC_GetValue(handler_sick) / UINT32_MAX * 3.3;
	double minCurrent = 0.004;
	double maxCurrent = 0.020;
	double minDistance = 100;
	double maxDistance = 400;
	double minVoltage = minCurrent * resistorValue;
	double maxVoltage = maxCurrent * resistorValue;
	return  (float) ((maxDistance - minDistance) / (maxVoltage - minVoltage) * tension + (maxVoltage * minDistance - minVoltage*maxDistance)/(maxVoltage - minVoltage));
}
