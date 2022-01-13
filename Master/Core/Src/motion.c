#include "motion.h"

#include <math.h>


static TIM_HandleTypeDef *left_encoder_handler = NULL;
static TIM_HandleTypeDef *right_encoder_handler = NULL;
static TIM_HandleTypeDef *pwm_handler = NULL;

static int64_t error_I = 0;
static uint32_t previous_position = 0;

static uint32_t pwm_base = 0;

void Motion_Init(TIM_HandleTypeDef *_left_encoder_handler, TIM_HandleTypeDef *_right_encoder_handler, TIM_HandleTypeDef *_pwm_handler)
{
	left_encoder_handler = _left_encoder_handler;
	right_encoder_handler = _right_encoder_handler;
	pwm_handler = _pwm_handler;
}

uint16_t Motion_Get_Left_Ticks(Motion_MovementType motion)
{
	uint16_t counter_value = __HAL_TIM_GET_COUNTER(left_encoder_handler);
	switch ( motion ) {
	case MOTION_MOVEMENT_TYPE_FORWARD :
	case MOTION_MOVEMENT_TYPE_CLOCKWISE :
		return counter_value;
	case MOTION_MOVEMENT_TYPE_BACKWARD :
	case MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE :
		return -((int16_t) counter_value);
	default:
		return 0;
	}
}

uint16_t Motion_Get_Right_Ticks(Motion_MovementType motion)
{
	uint16_t counter_value = __HAL_TIM_GET_COUNTER(right_encoder_handler);
	switch ( motion ) {
	case MOTION_MOVEMENT_TYPE_FORWARD :
	case MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE :
		return counter_value;
	case MOTION_MOVEMENT_TYPE_BACKWARD :
	case MOTION_MOVEMENT_TYPE_CLOCKWISE :
		return -((int16_t) counter_value);
	default:
		return 0;
	}
}

void Motion_Update_Left_PWM(int64_t pwm, Motion_Channel Channel_a, Motion_Channel Channel_b)
{
	if (pwm > 0) {    // tim_channel_1, tim_channel_2
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_a, pwm);
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_b, 0);
	}
	else {
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_a, 0);
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_b, -pwm);
	}
}

void Motion_Update_Right_PWM(int64_t pwm, Motion_Channel Channel_a, Motion_Channel Channel_b)
{
 	if (pwm > 0) {     // tim_channel_3, tim_channel_4
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_a, pwm);
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_b, 0);
	}
	else {
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_a, 0);
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_b, -pwm);
	}
}


int64_t Motion_Compute_PID(uint32_t setpoint, uint32_t position, double kp, double kd, double ki)
{
	int64_t error_P = ((int64_t) setpoint) - ((int64_t) position);
	error_I += error_P;
	int64_t error_D = position - previous_position;
	previous_position = position;
	int64_t pwm = kp * error_P + kd * error_D + ki * error_I;
	if (pwm > UINT32_MAX) {
		return UINT32_MAX;
	}
	else if (-pwm > UINT32_MAX){
		return -((int64_t) UINT32_MAX);
	}
	else {
	    return pwm;
	}
}


double Motion_PWM_Base_Right(uint32_t finalsetpoint, Motion_MovementType motion)
{
	double beta =  (finalsetpoint - Motion_Get_Right_Ticks(motion)) / finalsetpoint;
	return beta;
}

double Motion_PWM_Base_Left(uint32_t finalsetpoint, Motion_MovementType motion) // retourner int64 non ? comment faire la conversion
{
	double alpha =  (finalsetpoint - Motion_Get_Left_Ticks(motion)) / finalsetpoint;
	return alpha;
}

void Motion_Translation_Forward(uint32_t finalsetpoint)
{
    double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_FORWARD);
    double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_FORWARD);

// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	int64_t left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), 1, 0, 0);
	int64_t right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), 1, 0, 0);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * left_pwm_setpoint + ( 1 - alpha) * pwm_base, TIM_CHANNEL_1, TIM_CHANNEL_2);
	Motion_Update_Right_PWM(beta * right_pwm_setpoint + ( 1 - beta) * pwm_base, TIM_CHANNEL_1, TIM_CHANNEL_2);

}

void Motion_Translation_Backward(uint32_t finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_BACKWARD);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_BACKWARD);

// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	int64_t left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), 1, 0, 0);
	int64_t right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), 1, 0, 0);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * left_pwm_setpoint + (1 - alpha) * pwm_base, TIM_CHANNEL_2, TIM_CHANNEL_1);
	Motion_Update_Right_PWM(beta * right_pwm_setpoint + (1 - beta) * pwm_base, TIM_CHANNEL_2, TIM_CHANNEL_1);

}

void Motion_Rotation_clockwise(uint32_t finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_CLOCKWISE);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_CLOCKWISE);

	  // Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	  	int64_t left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), 1, 0, 0);
	  	int64_t right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), 1, 0, 0);

	  	// On met à jour les PWM aux moteurs
	  	Motion_Update_Left_PWM(alpha * left_pwm_setpoint + (1 - alpha) * pwm_base, TIM_CHANNEL_1, TIM_CHANNEL_2);
	  	Motion_Update_Right_PWM(beta * right_pwm_setpoint + (1 - beta) * pwm_base, TIM_CHANNEL_4, TIM_CHANNEL_3);
}

void Motion_Rotation_counter_clockwise(uint32_t finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE);

	  // Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	  	int64_t left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), 1, 0, 0);
	  	int64_t right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), 1, 0, 0);

	  	// On met à jour les PWM aux moteurs
	  	Motion_Update_Left_PWM(alpha * left_pwm_setpoint + (1 - alpha) * pwm_base, TIM_CHANNEL_2, TIM_CHANNEL_1);
	  	Motion_Update_Right_PWM(beta * right_pwm_setpoint + (1 - beta) * pwm_base, TIM_CHANNEL_3, TIM_CHANNEL_4);
}





