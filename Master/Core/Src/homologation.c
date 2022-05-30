#include "homologation.h"
#include "motion.h"

#include "motion.h"
#include "motion_config.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <order/motion.h>
#include <order/type.h>

Shared_Tick left_tick = 0;
Shared_Tick right_tick = 0;

int script_check_stop(void) {
	left_tick = Motion_Get_Left_Ticks();
	right_tick = Motion_Get_Right_Ticks();
	HAL_Delay(10);
	if (left_tick == (Motion_Get_Left_Ticks()) && right_tick == (Motion_Get_Right_Ticks()) && HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4) != GPIO_PIN_SET) {
		return 1;
	}
	else {
		return 0;
	}
}

void script_homologation(void) {
	Motion_Set_Forward_Translation_Setpoint(5000);
	//while ((script_check_stop()) != 1);
	HAL_Delay (5000);
	Motion_Set_Backward_Translation_Setpoint(5000);
	HAL_Delay(5000);
	/*
	Motion_Set_Clockwise_Rotation_Setpoint(4184);
	//while ((script_check_stop()) != 1);
	HAL_Delay(5000);
	Motion_Set_Forward_Translation_Setpoint(5000);
	*/
}

void script_carredef_Yellow(void) { //rajouter delay
	Motion_Set_Forward_Translation_Setpoint(13661);
	while ((script_check_stop()) != 1);
	Motion_Set_Clockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(11313);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(1917);
	while ((script_check_stop()) != 1);
	Motion_Set_Backward_Translation_Setpoint(1917);
	while ((script_check_stop()) != 1);
	Motion_Set_Clockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(4314);
	while ((script_check_stop()) != 1);
	Motion_Set_Counterclockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(1917);
	while ((script_check_stop()) != 1);
	Motion_Set_Backward_Translation_Setpoint(1917);
}

void script_global_bras_Yellow(void) {
	Motion_Set_Forward_Translation_Setpoint(4841);
	while ((script_check_stop()) != 1);
	Motion_Set_Clockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(527);
	while ((script_check_stop()) != 1);
	Motion_Set_Counterclockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(1438);
	while ((script_check_stop()) != 1);
	//take_palet_ground(9)
	//reverse_palet_ground(9)
	Motion_Set_Forward_Translation_Setpoint(671);
	while ((script_check_stop()) != 1);
	//take_palet_ground(11)
	//take_palet_ground(7)
	Motion_Set_Forward_Translation_Setpoint(96);
	while ((script_check_stop()) != 1);
	Motion_Set_Counterclockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(3115);
	while ((script_check_stop()) != 1);
	//drop 9
	Motion_Set_Counterclockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(1438);
	while ((script_check_stop()) != 1);
	Motion_Set_Clockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	//reverse_palet_ground(7)
	//drop 7
	Motion_Set_Clockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(2876);
	while ((script_check_stop()) != 1);
	Motion_Set_Counterclockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	//reverse_palet_ground(11)
	//drop 11
	Motion_Set_Clockwise_Rotation_Setpoint(4184);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(2588);
	while ((script_check_stop()) != 1);
	Motion_Set_Clockwise_Rotation_Setpoint(2092);
	while ((script_check_stop()) != 1);
	Motion_Set_Forward_Translation_Setpoint(8484);
	while ((script_check_stop()) != 1);
}


