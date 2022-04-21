#include "motion.h"
#include "motion_config.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <order/motion.h>
#include <order/type.h>

uint8_t isJumperOn(void){
	return (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) ? 0 : 1;
}
