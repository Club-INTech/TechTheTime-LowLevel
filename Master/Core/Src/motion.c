#include "motion.h"
#include "motion_config.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <order/motion.h>
#include <order/type.h>

#define MOTION_PWM_MAX (UINT16_MAX * MOTION_POWER_MAX)
#define MOTION_PWM_BASE (UINT16_MAX * MOTION_POWER_BASE)
#define MOTION_ENCODER_OFFSET 5000

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
static Motion_Tick distance_arg = 0;
static Motion_Tick offset_arg = 0;

static void Motion_Start_Motor(void) {
	HAL_TIM_Base_Start_IT(pwm_handler);
	HAL_TIM_PWM_Start(pwm_handler, MOTION_CHANNEL_FORWARD_LEFT);
	HAL_TIM_PWM_Start(pwm_handler, MOTION_CHANNEL_FORWARD_RIGHT);
	HAL_TIM_PWM_Start(pwm_handler, MOTION_CHANNEL_BACKWARD_LEFT);
	HAL_TIM_PWM_Start(pwm_handler, MOTION_CHANNEL_BACKWARD_RIGHT);
}

static void Motion_Stop_Motor(void) {
	HAL_TIM_Base_Stop_IT(pwm_handler);
	HAL_TIM_PWM_Stop(pwm_handler, MOTION_CHANNEL_FORWARD_LEFT);
	HAL_TIM_PWM_Stop(pwm_handler, MOTION_CHANNEL_FORWARD_RIGHT);
	HAL_TIM_PWM_Stop(pwm_handler, MOTION_CHANNEL_BACKWARD_LEFT);
	HAL_TIM_PWM_Stop(pwm_handler, MOTION_CHANNEL_BACKWARD_RIGHT);
}

static void Motion_Rewind_Profile(Motion_PID_Profile *profile_ptr) {
  memset(profile_ptr->error_buf, 0, sizeof profile_ptr->error_buf);
  profile_ptr->last_error_ptr = profile_ptr->error_buf;
  profile_ptr->error_sum = 0;
}

static void Motion_Update_Profile(Motion_PID_Profile *profile_ptr, Motion_Tick error) {
	profile_ptr->last_error_ptr++;
	if (profile_ptr->last_error_ptr == profile_ptr->error_buf + MOTION_ERROR_BUFFER_SIZE)
		profile_ptr->last_error_ptr = profile_ptr->error_buf;

	profile_ptr->error_sum += error - *profile_ptr->last_error_ptr;
	*profile_ptr->last_error_ptr = error;
}

void Motion_Init_Arg(Motion_MovementType motion, Motion_Tick finalsetpoint) {
	Motion_Stop_Motor();

	motion_arg = motion;
	finalsetpoint_arg = finalsetpoint;

	Motion_Rewind_Profile(&left_profile);
	Motion_Rewind_Profile(&right_profile);
	Motion_Rewind_Profile(&translation_profile);
	Motion_Rewind_Profile(&rotation_profile);

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

	if (motion != MOTION_MOVEMENT_TYPE_NONE) Motion_Start_Motor();
}

void Motion_Init(TIM_HandleTypeDef *_left_encoder_handler, TIM_HandleTypeDef *_right_encoder_handler, TIM_HandleTypeDef *_pwm_handler)
{
	left_encoder_handler = _left_encoder_handler;
	right_encoder_handler = _right_encoder_handler;
	pwm_handler = _pwm_handler;

	HAL_TIM_Encoder_Start(left_encoder_handler, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(right_encoder_handler, TIM_CHANNEL_ALL);
	Motion_Start_Motor();
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
		return (uint16_t)(~counter_value + 1u) - MOTION_ENCODER_OFFSET;
	// Dans le cas d'un mouvement en rotation, on interpréte la valeur du compteur comme un entier signé (c'est permis grâce au fonctionnement du complément à 2)
	case MOTION_MOVEMENT_TYPE_CLOCKWISE :
		return (int16_t) counter_value;
	// Idem pour la rotation dans le sens inverse, sauf que l'on inverse le signe
	case MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE :
		return -((int16_t) counter_value);
	case MOTION_MOVEMENT_TYPE_FREE :
		return (int16_t) counter_value;
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
		return (uint16_t)(~counter_value + 1u) - MOTION_ENCODER_OFFSET;
	// Dans le cas d'un mouvement en rotation, on interpréte la valeur du compteur comme un entier signé (c'est permis grâce au fonctionnement du complément à 2)
	case MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE :
		return (int16_t) counter_value;
	// Idem pour la rotation dans le sens inverse, sauf que l'on inverse le signe
	case MOTION_MOVEMENT_TYPE_CLOCKWISE :
		return -((int16_t) counter_value);
	case MOTION_MOVEMENT_TYPE_FREE :
		return (int16_t) counter_value;
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
	if (pwm > 0) {    // tim_channel_1, tim_cHAL_TIM_Base_Start_IT(hannel_2
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
	Motion_Tick error = setpoint - position, error_delta = error - *profile->last_error_ptr;
	Motion_Update_Profile(profile, error);

	Motion_PWM pwm = profile->kp * error + profile->kd * error_delta + profile->ki * profile->error_sum;
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

void Motion_Release(void) {
	Motion_Init_Arg(MOTION_MOVEMENT_TYPE_NONE, 0);
}

void Motion_Set_Forward_Translation_Setpoint(Shared_Tick setpoint) {
	Motion_Init_Arg(MOTION_MOVEMENT_TYPE_FORWARD,setpoint);
}

void Motion_Set_Backward_Translation_Setpoint(Shared_Tick setpoint) {
	Motion_Init_Arg(MOTION_MOVEMENT_TYPE_BACKWARD, setpoint);
}

void Motion_Set_Clockwise_Rotation_Setpoint(Shared_Tick setpoint) {
	Motion_Init_Arg(MOTION_MOVEMENT_TYPE_CLOCKWISE, setpoint);
}

void Motion_Set_Counterclockwise_Rotation_Setpoint(Shared_Tick setpoint) {
	Motion_Init_Arg(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE, setpoint);
}

void Motion_Start_Joystick(void) {
	distance_arg = 0;
	offset_arg = 0;
	Motion_Init_Arg(MOTION_MOVEMENT_TYPE_FREE, 0);
}

void Motion_Set_Joystick(Shared_Tick distance, Shared_Tick offset) {
	distance_arg = distance;
	offset_arg = offset;
}

void Motion_Set_Translation_PID(Shared_PID_K kp, Shared_PID_K ki, Shared_PID_K kd) {
	translation_profile.kp = FROM_SHARED_PID_K_FIXED_POINT(kp);
	translation_profile.kd = FROM_SHARED_PID_K_FIXED_POINT(kd);
	translation_profile.ki = FROM_SHARED_PID_K_FIXED_POINT(ki);
}

void Motion_Set_Rotation_PID(Shared_PID_K kp, Shared_PID_K ki, Shared_PID_K kd) {
	rotation_profile.kp = FROM_SHARED_PID_K_FIXED_POINT(kp);
	rotation_profile.kd = FROM_SHARED_PID_K_FIXED_POINT(kd);
	rotation_profile.ki = FROM_SHARED_PID_K_FIXED_POINT(ki);
}

void Motion_Set_Left_PID(Shared_PID_K kp, Shared_PID_K ki, Shared_PID_K kd) {
	left_profile.kp = FROM_SHARED_PID_K_FIXED_POINT(kp);
	left_profile.kd = FROM_SHARED_PID_K_FIXED_POINT(kd);
	left_profile.ki = FROM_SHARED_PID_K_FIXED_POINT(ki);
}

void Motion_Set_Right_PID(Shared_PID_K kp, Shared_PID_K ki, Shared_PID_K kd) {
	right_profile.kp = FROM_SHARED_PID_K_FIXED_POINT(kp);
	right_profile.kd = FROM_SHARED_PID_K_FIXED_POINT(kd);
	right_profile.ki = FROM_SHARED_PID_K_FIXED_POINT(ki);
}

void Motion_Joystick(Motion_Tick distance, Motion_Tick offset) {
	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(distance + offset, Motion_Get_Left_Ticks(), &left_profile);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(distance - offset, Motion_Get_Right_Ticks(), &right_profile);

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(left_pwm_setpoint, MOTION_CHANNEL_FORWARD_LEFT, MOTION_CHANNEL_BACKWARD_LEFT);
	Motion_Update_Right_PWM(right_pwm_setpoint, MOTION_CHANNEL_FORWARD_RIGHT, MOTION_CHANNEL_BACKWARD_RIGHT);
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
	  case MOTION_MOVEMENT_TYPE_FREE:
		  Motion_Joystick(distance_arg, offset_arg);
	  default:
		  break;
	  }
	}
}
