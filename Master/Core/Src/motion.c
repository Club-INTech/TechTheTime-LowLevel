#include "motion.h"
#include "motion_config.h"

#include <math.h>
#include <stdlib.h>

#include <order/motion.h>

#define MOTION_PWM_MAX 9000
#define MOTION_ENCODER_OFFSET 5000
#define MOTION_PWM_BASE (MOTION_PWM_MAX * MOTION_POWER_BASE)

static TIM_HandleTypeDef *left_encoder_handler = NULL;
static TIM_HandleTypeDef *right_encoder_handler = NULL;
static TIM_HandleTypeDef *pwm_handler = NULL;

static Motion_PID_Profile
	left_profile = {
		.kp = MOTION_LEFT_KP,
		.kd = MOTION_LEFT_KD,
		.ki = MOTION_LEFT_KI
	},
	right_profile = {
		.kp = MOTION_RIGHT_KP,
		.kd = MOTION_RIGHT_KD,
		.ki = MOTION_RIGHT_KI
	},
	translation_profile = {
		.kp = MOTION_TRANSLATION_KP,
		.kd = MOTION_TRANSLATION_KD,
		.ki = MOTION_TRANSLATION_KI
	},
	rotation_profile = {
		.kp = MOTION_ROTATION_KP,
		.kd = MOTION_ROTATION_KD,
		.ki = MOTION_ROTATION_KI
	};

static Motion_PWM pwm_base = MOTION_PWM_BASE;
static Motion_MovementType motion_arg = MOTION_MOVEMENT_TYPE_NONE;
static Motion_Tick finalsetpoint_arg = 0;

void Motion_Init_Arg(Motion_MovementType motion, Motion_Tick finalsetpoint) {
	motion_arg = MOTION_MOVEMENT_TYPE_NONE;
	finalsetpoint_arg = finalsetpoint;

	left_profile.previous_position = 0;
	right_profile.previous_position = 0;
	translation_profile.previous_position = 0;
	rotation_profile.previous_position = 0;

	left_profile.sum_error = 0;
	right_profile.sum_error = 0;
	translation_profile.sum_error = 0;
	rotation_profile.sum_error = 0;

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
		__HAL_TIM_SET_COUNTER(left_encoder_handler, 0);
		__HAL_TIM_SET_COUNTER(right_encoder_handler, 0);
		break;
	}

	motion_arg = motion;
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

Motion_Tick Motion_Get_Left_Ticks(void)
{
	uint16_t counter_value = __HAL_TIM_GET_COUNTER(left_encoder_handler);
	switch ( motion_arg ) {
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
		return (int16_t) counter_value;
	}
}

Motion_Tick Motion_Get_Right_Ticks(void)
{
	uint16_t counter_value = __HAL_TIM_GET_COUNTER(right_encoder_handler);
	switch ( motion_arg ) {
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
		return (int16_t) counter_value;
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
	return finalsetpoint != 0 ? fmax(0, fmin((double) Motion_Get_Right_Ticks() / finalsetpoint, 1)) : 1;
}

double Motion_PWM_Base_Left(Motion_Tick finalsetpoint, Motion_MovementType motion) // retourner int64 non ? comment faire la conversion
{
	return finalsetpoint != 0 ? fmax(0, fmin((double) Motion_Get_Left_Ticks() / finalsetpoint, 1)) : 1;
}

void Motion_Translation_Forward(Motion_Tick finalsetpoint)
{
    double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_FORWARD);
    double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_FORWARD);

    // Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(), Motion_Get_Left_Ticks(), &left_profile);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(), Motion_Get_Right_Ticks(), &right_profile);
	Motion_Tick avg_position = (Motion_Get_Left_Ticks() + Motion_Get_Right_Ticks()) / 2;
	Motion_PWM translation_pwm_setpoint = Motion_Compute_PID(finalsetpoint, avg_position, &translation_profile);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * translation_pwm_setpoint + ( 1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_FORWARD_LEFT, MOTION_CHANNEL_BACKWARD_LEFT);
	Motion_Update_Right_PWM(beta * translation_pwm_setpoint + ( 1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_FORWARD_RIGHT, MOTION_CHANNEL_BACKWARD_RIGHT);
}

void Motion_Translation_Backward(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_BACKWARD);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_BACKWARD);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(), Motion_Get_Left_Ticks(), &left_profile);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(), Motion_Get_Right_Ticks(), &right_profile);
	Motion_Tick avg_position = (Motion_Get_Left_Ticks() + Motion_Get_Right_Ticks()) / 2;
	Motion_PWM translation_pwm_setpoint = Motion_Compute_PID(finalsetpoint, avg_position, &translation_profile); // TODO

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * translation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_BACKWARD_LEFT, MOTION_CHANNEL_FORWARD_LEFT);
	Motion_Update_Right_PWM(beta * translation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_BACKWARD_RIGHT, MOTION_CHANNEL_FORWARD_RIGHT);
}

void Motion_Rotation_Clockwise(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_CLOCKWISE);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_CLOCKWISE);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(), Motion_Get_Left_Ticks(), &left_profile);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(), Motion_Get_Right_Ticks(), &right_profile);
	Motion_Tick avg_position = (Motion_Get_Left_Ticks() + Motion_Get_Right_Ticks()) / 2;
	Motion_PWM rotation_pwm_setpoint = Motion_Compute_PID(finalsetpoint, avg_position, &rotation_profile);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * rotation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_FORWARD_LEFT, MOTION_CHANNEL_BACKWARD_LEFT);
	Motion_Update_Right_PWM(beta * rotation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_BACKWARD_RIGHT, MOTION_CHANNEL_FORWARD_RIGHT);
}

void Motion_Rotation_Counter_Clockwise(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(), Motion_Get_Left_Ticks(), &left_profile);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(), Motion_Get_Right_Ticks(), &right_profile);
	Motion_Tick avg_position = (Motion_Get_Left_Ticks() + Motion_Get_Right_Ticks()) / 2;
	Motion_PWM rotation_pwm_setpoint = Motion_Compute_PID(finalsetpoint, avg_position, &rotation_profile);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * rotation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_BACKWARD_LEFT, MOTION_CHANNEL_FORWARD_LEFT);
	Motion_Update_Right_PWM(beta * rotation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_FORWARD_RIGHT, MOTION_CHANNEL_BACKWARD_RIGHT);
}

//
// Order implementations
//

void Motion_Set_Forward_Translation_Setpoint(Shared_Tick setpoint) {
	Motion_Init_Arg(MOTION_MOVEMENT_TYPE_FORWARD, setpoint);
}

void Motion_Release(void) {
	Motion_Init_Arg(MOTION_MOVEMENT_TYPE_NONE, 0);
}

//
// Interrupt routines
//

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == pwm_handler->Instance)
  {
	  switch(motion_arg) {
		  case MOTION_MOVEMENT_TYPE_FORWARD:
			  Motion_Translation_Forward(finalsetpoint_arg);
			  break;
		  case MOTION_MOVEMENT_TYPE_BACKWARD:
			  Motion_Translation_Backward(finalsetpoint_arg);
			  break;
		  case MOTION_MOVEMENT_TYPE_CLOCKWISE:
			  Motion_Rotation_Clockwise(finalsetpoint_arg);
			  break;
		  case MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE:
			  Motion_Rotation_Counter_Clockwise(finalsetpoint_arg);
			  break;
	  default:
		  break;
	  }
  }
}
