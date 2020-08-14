/*
 * PID.c
 *
 *  Created on: 2020. 8. 12.
 *      Author: NB
 */


#include "PID.h"
#include <stdlib.h>
#include <string.h>
#include <float.h> // Float min, max
#include <math.h> // isnan, isinf

PID_Handle_t *PID_Create(){
	return malloc(sizeof(PID_Handle_t));
}
int32_t PID_Initialize(PID_Handle_t *this, float Kp, float Ki, float Kd, float min, float max){
	if(!this)
		return -1;
	memset(this, 0, sizeof(PID_Handle_t));
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->output_min = min;
	this->output_max = max;

	return 0;
}
int32_t PID_SetInterface(PID_Handle_t *this, PID_GetFunc GetInput, PID_GetFunc GetTime, PID_SetFunc SetOutput){
	if(!this)
		return -1;

	this->GetInput = GetInput;
	this->GetTime = GetTime;
	this->SetOutput = SetOutput;

	this->curr_time = this->GetTime(); // Prevent Peak signal
	return 0;

}

inline static float PID_Saturate(float in, float min, float max){
	if(in < min){
		return min;
	} else if(max < in){
		return max;
	} else {
		return in;
	}
}

void PID_SetTarget(PID_Handle_t *this, float target){
	if(!this)
		return;
	
	this->target = target;
}

float PID_Iterate(PID_Handle_t *this){
	if(!this){
		return NaN;
	}
	/* Sense */
	this->curr_in = this->GetInput();
	this->curr_time = this->GetTime();

	/* Compute */
	this->curr_error = this->target - this->curr_in;
	float dt = this->curr_time - this->prev_time;

	float p_term = this->Kp * this->curr_error;
	this->i_term += (this->Ki * this->curr_error) * dt;
	this->i_term = PID_Saturate(this->i_term, this->output_min, this->output_max);
	float d_term = (this->Kd * (this->curr_error - this->prev_error)) / dt;

	this->output = p_term + this->i_term + d_term;
	this->output = PID_Saturate(this->output, this->output_min, this->output_max);

	this->prev_in = this->curr_in;
	this->prev_error = this->curr_error;
	this->prev_time = this->curr_time;

	/* Actuate */
	this->SetOutput(this->output);
	return this->output;
}

void PID_GetFunc_Template(){
	float ret = 0.0f;
	return ret;
}

void PID_SetFunc_Template(float in){
	if(isnan(in)){
		return ;
	}
	(void) in;
}