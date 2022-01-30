#include "motion.h"

#include <math.h>
#include <stdlib.h>



#define MOTION_PWM_MAX UINT16_MAX
#define MOTION_ENCODER_OFFSET (3 * 1024)

static TIM_HandleTypeDef *left_encoder_handler = NULL;
static TIM_HandleTypeDef *right_encoder_handler = NULL;
static TIM_HandleTypeDef *pwm_handler = NULL;

static Motion_PID_Profile
		left_profile = {
				.kp = 1,
				.kd = 0,
				.ki = 0,
				.previous_position = 0,
				.sum_error = 0
		},
		right_profile = {
				.kp = 1,
				.kd = 0,
				.ki = 0,
				.previous_position = 0,
				.sum_error = 0
		},
		movement_profile = {
				.kp = 1,
				.kd = 0,
				.ki = 0,
				.previous_position = 0,
				.sum_error = 0
		};

static Motion_PWM pwm_base = 2000;  //max = 10000; (valeur d'autoreload de mon compteur
static Motion_MovementType motion_arg = MOTION_MOVEMENT_TYPE_FORWARD;
static Motion_Tick finalsetpoint_arg = 0;

void Motion_Init_Arg(Motion_MovementType motion, Motion_Tick finalsetpoint) {
	motion_arg = motion;
	finalsetpoint_arg = finalsetpoint;

	left_profile.previous_position = 0;
	right_profile.previous_position = 0;
	movement_profile.previous_position = 0;

	left_profile.sum_error = 0;
	right_profile.sum_error = 0;
	movement_profile.sum_error = 0;


	switch(motion) {
	case MOTION_MOVEMENT_TYPE_FORWARD:
		__HAL_TIM_SET_COUNTER(left_encoder_handler, MOTION_ENCODER_OFFSET);
		__HAL_TIM_SET_COUNTER(right_encoder_handler, MOTION_ENCODER_OFFSET);
		break;
	case MOTION_MOVEMENT_TYPE_BACKWARD:
		__HAL_TIM_SET_COUNTER(left_encoder_handler, UINT16_MAX - MOTION_ENCODER_OFFSET);
		__HAL_TIM_SET_COUNTER(right_encoder_handler, UINT16_MAX - MOTION_ENCODER_OFFSET);
		break;
	default:
		break;
	}

	return;
}

void Motion_Init(TIM_HandleTypeDef *_left_encoder_handler, TIM_HandleTypeDef *_right_encoder_handler, TIM_HandleTypeDef *_pwm_handler)
{
	left_encoder_handler = _left_encoder_handler;
	right_encoder_handler = _right_encoder_handler;
	pwm_handler = _pwm_handler;

	HAL_TIM_Encoder_Start(left_encoder_handler, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(right_encoder_handler, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(pwm_handler);
	HAL_TIM_PWM_Start(pwm_handler, MOTION_CHANNEL_FORWARD_LEFT);
	HAL_TIM_PWM_Start(pwm_handler, MOTION_CHANNEL_FORWARD_RIGHT);
	HAL_TIM_PWM_Start(pwm_handler, MOTION_CHANNEL_BACKWARD_LEFT);
	HAL_TIM_PWM_Start(pwm_handler, MOTION_CHANNEL_BACKWARD_RIGHT);
}

Motion_Tick Motion_Get_Left_Ticks(Motion_MovementType motion)
{
	uint16_t counter_value = __HAL_TIM_GET_COUNTER(left_encoder_handler);
	switch ( motion ) {
	// Dans le cas d'un mouvement en translation avant, on retourne la valeur du compteur telle quelle afin de gérer de longues distances
	case MOTION_MOVEMENT_TYPE_FORWARD :
		return counter_value - MOTION_ENCODER_OFFSET;
	// Dans le cas d'un mouvement en translation arrière, on joue sur le fonctionnement du complément à 2 pour considérer la plage de valeur entre 0 et -(2^16 - 1)
	case MOTION_MOVEMENT_TYPE_BACKWARD :
		return (~counter_value + 1) - MOTION_ENCODER_OFFSET;
	// Dans le cas d'un mouvement en rotation, on interpréte la valeur du compteur comme un entier signé (c'est permis grâce au fonctionnement du complément à 2)
	case MOTION_MOVEMENT_TYPE_CLOCKWISE :
		return (int16_t) counter_value;
	// Idem pour la rotation dans le sens inverse, sauf que l'on inverse le signe
	case MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE :
		return -((int16_t) counter_value);
	default:
		return 0;
	}
}

Motion_Tick Motion_Get_Right_Ticks(Motion_MovementType motion)
{
	uint16_t counter_value = __HAL_TIM_GET_COUNTER(right_encoder_handler);
	switch ( motion ) {
	// Dans le cas d'un mouvement en translation avant, on retourne la valeur du compteur telle quelle afin de gérer de longues distances
	case MOTION_MOVEMENT_TYPE_FORWARD :
		return counter_value - MOTION_ENCODER_OFFSET;
	// Dans le cas d'un mouvement en translation arrière, on joue sur le fonctionnement du complément à 2 pour considérer la plage de valeur entre 0 et -(2^16 - 1) et on retourne la valeur opposée
	case MOTION_MOVEMENT_TYPE_BACKWARD :
		return (~counter_value + 1) - MOTION_ENCODER_OFFSET;
	// Dans le cas d'un mouvement en rotation, on interpréte la valeur du compteur comme un entier signé (c'est permis grâce au fonctionnement du complément à 2)
	case MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE :
		return (int16_t) counter_value;
	// Idem pour la rotation dans le sens inverse, sauf que l'on inverse le signe
	case MOTION_MOVEMENT_TYPE_CLOCKWISE :
		return -((int16_t) counter_value);
	default:
		return 0;
	}
}

void Motion_Update_Left_PWM(Motion_PWM pwm, Motion_Channel Channel_a, Motion_Channel Channel_b)
{
	if (pwm > 0) {    // tim_channel_1, tim_cHAL_TIM_Base_Start_IT(hannel_2
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_a, pwm);
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_b, 0);
	}
	else {
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_a, 0);
		__HAL_TIM_SET_COMPARE(pwm_handler, Channel_b, -pwm);
	}
}

void Motion_Update_Right_PWM(Motion_PWM pwm, Motion_Channel Channel_a, Motion_Channel Channel_b)
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

Motion_PWM Motion_Compute_PID(Motion_Tick setpoint, Motion_Tick position, Motion_PID_Profile *profile)
{
	Motion_PWM error_P = (Motion_PWM) setpoint - position;
	profile->sum_error += error_P;
	Motion_PWM error_D = profile->previous_position - position;
	profile->previous_position = position;
	Motion_PWM pwm = profile->kp * error_P + profile->kd * error_D + profile->ki * profile->sum_error;
	if (pwm > MOTION_PWM_MAX) {
		return MOTION_PWM_MAX;
	}
	else if (-pwm > MOTION_PWM_MAX){
		return -((Motion_PWM) MOTION_PWM_MAX);
	}
	else {
	    return pwm;
	}
}


double Motion_PWM_Base_Right(Motion_Tick finalsetpoint, Motion_MovementType motion)
{
	double beta =  (double) Motion_Get_Right_Ticks(motion) / finalsetpoint;
	return beta;
}

double Motion_PWM_Base_Left(Motion_Tick finalsetpoint, Motion_MovementType motion) // retourner int64 non ? comment faire la conversion
{
	double alpha =  (double) Motion_Get_Left_Ticks(motion) / finalsetpoint;
	return alpha;
}

void Motion_Translation_Forward(Motion_Tick finalsetpoint)
{
    double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_FORWARD);
    double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_FORWARD);

    // Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), &left_profile);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), &right_profile);
	Motion_Tick avg_position = (Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_FORWARD) + Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_FORWARD)) / 2;
	Motion_PWM translation_pwm_setpoint = Motion_Compute_PID(finalsetpoint, avg_position, &movement_profile);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * translation_pwm_setpoint + ( 1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_FORWARD_LEFT, MOTION_CHANNEL_BACKWARD_LEFT);
	Motion_Update_Right_PWM(beta * translation_pwm_setpoint + ( 1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_FORWARD_RIGHT, MOTION_CHANNEL_BACKWARD_RIGHT);
}

/*void Motion_Translation_Backward(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_BACKWARD);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_BACKWARD);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), 1, 0, 0);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), 1, 0, 0);
	Motion_Tick avg_position = (Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD) + Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD)) / 2;
	Motion_PWM translation_pwm_setpoint = Motion_Compute_PID(finalsetpoint, avg_position, 1, 0, 0); // TODO

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * translation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_BACKWARD_LEFT, MOTION_CHANNEL_FORWARD_LEFT);
	Motion_Update_Right_PWM(beta * translation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_BACKWARD_RIGHT, MOTION_CHANNEL_FORWARD_RIGHT);
}

void Motion_Rotation_Clockwise(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_CLOCKWISE);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_CLOCKWISE);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), 1, 0, 0);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), 1, 0, 0);
	Motion_Tick avg_position = (Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE) + Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE)) / 2;
	Motion_PWM rotation_pwm_setpoint = Motion_Compute_PID(finalsetpoint, avg_position, 1, 0, 0);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * rotation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_FORWARD_LEFT, MOTION_CHANNEL_BACKWARD_LEFT);
	Motion_Update_Right_PWM(beta * rotation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_BACKWARD_RIGHT, MOTION_CHANNEL_FORWARD_RIGHT);
}

void Motion_Rotation_Counter_Clockwise(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), 1, 0, 0);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), 1, 0, 0);
	Motion_Tick avg_position = (Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE) + Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE)) / 2;
	Motion_PWM rotation_pwm_setpoint = Motion_Compute_PID(finalsetpoint, avg_position, 1, 0, 0);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * rotation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_BACKWARD_LEFT, MOTION_CHANNEL_FORWARD_LEFT);
	Motion_Update_Right_PWM(beta * rotation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_FORWARD_RIGHT, MOTION_CHANNEL_BACKWARD_RIGHT);
}*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		  if (htim->Instance == pwm_handler->Instance)
		  {
			  switch(motion_arg) {
			  case MOTION_MOVEMENT_TYPE_FORWARD:
				  Motion_Translation_Forward(finalsetpoint_arg);
			  break;
			 /* case MOTION_MOVEMENT_TYPE_BACKWARD:
				  Motion_Translation_Backward(finalsetpoint_arg);
			  break;
			  case MOTION_MOVEMENT_TYPE_CLOCKWISE:
				  Motion_Rotation_Clockwise(finalsetpoint_arg);
			  break;
			  case MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE:
				  Motion_Rotation_Counter_Clockwise(finalsetpoint_arg);
			  break;*/
			  default:
			  break;
			  }
		  }
	  }
