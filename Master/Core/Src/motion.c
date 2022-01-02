#include "motion.h"
#include <math.h>

static int64_t error_I = 0;
static int64_t previous_error = 0;
static int64_t previous_tension = 0;

void motion1(int64_t a)
{

	if (a > 0) {
		TIM3->CCR1 = a;
	}

	else {
		TIM3->CCR2 = - a;
	}

}

void motion2(int64_t a)
{
	if (a > 0) {
		TIM3->CCR3 = a;

	}
	else {
		TIM3->CCR4 = - a;
	}

}




int64_t Asservir_position(uint32_t b, uint32_t c, uint64_t kp, uint64_t kd, uint64_t ki)                  // b position désirée, c position actuelle (ici la position désirée est celle de la roue jumelle
{
	//float c = previous_tension * distance_max /  5;  -> quand on demande une distance                           // calcul de la position désirée
	int64_t error_P = b - c;
	error_I += error_P;
	int64_t error_D = error_P - previous_error;
	previous_error = error_P;
	int64_t pwm = kp * error_P + kd * error_D + ki * error_I ;
	if (pwm > pow(2,32)- 1 ) {
		return pow(2,32) -1;
	}
	else if (pwm < - pow(2,32) + 1){
		return - pox (2,32) +1;
	}
	else {
	    return pwm;
	}

}









