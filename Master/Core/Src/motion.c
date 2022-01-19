#include "motion.h"

#include <math.h>
#include <stdlib.h>

#define MOTION_PWM_MAX UINT16_MAX

static TIM_HandleTypeDef *left_encoder_handler = NULL;
static TIM_HandleTypeDef *right_encoder_handler = NULL;
static TIM_HandleTypeDef *pwm_handler = NULL;

static Motion_PWM error_I = 0;
static Motion_PWM pwm_base = 0;
static Motion_Tick previous_position = 0;

void Motion_Init(TIM_HandleTypeDef *_left_encoder_handler, TIM_HandleTypeDef *_right_encoder_handler, TIM_HandleTypeDef *_pwm_handler)
{
	left_encoder_handler = _left_encoder_handler;
	right_encoder_handler = _right_encoder_handler;
	pwm_handler = _pwm_handler;

	HAL_TIM_Encoder_Start(left_encoder_handler, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(right_encoder_handler, TIM_CHANNEL_ALL);
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
		return counter_value;
	// Dans le cas d'un mouvement en translation arrière, on joue sur le fonctionnement du complément à 2 pour considérer la plage de valeur entre 0 et -(2^16 - 1)
	case MOTION_MOVEMENT_TYPE_BACKWARD :
		return ~counter_value + 1;
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
		return counter_value;
	// Dans le cas d'un mouvement en translation arrière, on joue sur le fonctionnement du complément à 2 pour considérer la plage de valeur entre 0 et -(2^16 - 1) et on retourne la valeur opposée
	case MOTION_MOVEMENT_TYPE_BACKWARD :
		return ~counter_value + 1;
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
	if (pwm > 0) {    // tim_channel_1, tim_channel_2
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


Motion_PWM Motion_Compute_PID(Motion_Tick setpoint, Motion_Tick position, double kp, double kd, double ki)
{
	Motion_PWM error_P = (Motion_PWM) setpoint - position;
	error_I += error_P;
	Motion_PWM error_D = position - previous_position;
	previous_position = position;
	Motion_PWM pwm = kp * error_P + kd * error_D + ki * error_I;
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
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), 1, 0, 0);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_FORWARD), 1, 0, 0);
	Motion_PWM translation_pwm_setpoint = 0; // TODO

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * translation_pwm_setpoint + ( 1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_FORWARD_LEFT, MOTION_CHANNEL_BACKWARD_LEFT);
	Motion_Update_Right_PWM(beta * translation_pwm_setpoint + ( 1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_FORWARD_RIGHT, MOTION_CHANNEL_BACKWARD_RIGHT);
}

void Motion_Translation_Backward(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_BACKWARD);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_BACKWARD);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), 1, 0, 0);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_BACKWARD), 1, 0, 0);
	Motion_PWM translation_pwm_setpoint = 0; // TODO

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * translation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_BACKWARD_LEFT, MOTION_CHANNEL_FORWARD_LEFT);
	Motion_Update_Right_PWM(beta * translation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_BACKWARD_RIGHT, MOTION_CHANNEL_FORWARD_RIGHT);
}

void Motion_Rotation_clockwise(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_CLOCKWISE);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_CLOCKWISE);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), 1, 0, 0);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_CLOCKWISE), 1, 0, 0);
	Motion_PWM rotation_pwm_setpoint = 0; // TODO

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * rotation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_FORWARD_LEFT, MOTION_CHANNEL_BACKWARD_LEFT);
	Motion_Update_Right_PWM(beta * rotation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_BACKWARD_RIGHT, MOTION_CHANNEL_FORWARD_RIGHT);
}

void Motion_Rotation_counter_clockwise(Motion_Tick finalsetpoint) {
	double alpha = Motion_PWM_Base_Left(finalsetpoint, MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE);
	double beta = Motion_PWM_Base_Right(finalsetpoint, MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE);

	// Lorsqu'on corrige le PWM au moteur grâce au PID, on donne la position d'une codeuse en consigne à l'autre pour chacune des deux
	Motion_PWM left_pwm_setpoint = Motion_Compute_PID(Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), 1, 0, 0);
	Motion_PWM right_pwm_setpoint = Motion_Compute_PID(Motion_Get_Left_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), Motion_Get_Right_Ticks(MOTION_MOVEMENT_TYPE_COUNTERCLOCKWISE), 1, 0, 0);
	Motion_PWM rotation_pwm_setpoint = 0; // TODO

	// On met à jour les PWM aux moteurs
	Motion_Update_Left_PWM(alpha * rotation_pwm_setpoint + (1 - alpha) * pwm_base + left_pwm_setpoint, MOTION_CHANNEL_BACKWARD_LEFT, MOTION_CHANNEL_FORWARD_LEFT);
	Motion_Update_Right_PWM(beta * rotation_pwm_setpoint + (1 - beta) * pwm_base + right_pwm_setpoint, MOTION_CHANNEL_FORWARD_RIGHT, MOTION_CHANNEL_BACKWARD_RIGHT);
}
