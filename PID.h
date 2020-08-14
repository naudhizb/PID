/*
 * PID.h
 *
 *  Created on: 2020. 8. 12.
 *      Author: NB
 */

#ifndef PID_H_
#define PID_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef float (*PID_GetFunc)(void);
typedef void (*PID_SetFunc)(float);

typedef struct {
	/* Configuration */
	float Kp;
	float Ki;
	float Kd;
	float output_max;
	float output_min;

	/* Context */
	uint32_t status;
	float prev_error;
	float curr_error;
	float i_term;
	float prev_time;
	float curr_time;

	/* In */
	float prev_in;
	float curr_in;
	float target;

	/* Out */
	float output;

	/* Interface */
	PID_GetFunc GetInput;
	PID_GetFunc GetTime;
	PID_SetFunc SetOutput;

} PID_Handle_t;

PID_Handle_t *PID_Create();
int32_t PID_Initialize(PID_Handle_t *this, float Kp, float Ki, float Kd, float min, float max);
int32_t PID_SetInterface(PID_Handle_t *this, PID_GetFunc GetInput, PID_GetFunc GetTime, PID_SetFunc SetOutput);
void PID_SetTarget(PID_Handle_t *this, float target);
float PID_Iterate(PID_Handle_t *this);

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
