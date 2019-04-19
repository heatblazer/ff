/*
 * motor.h
 *
 *  Created on: Apr 16, 2019
 *      Author: ilian
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include <stdint.h>

typedef enum MotorInitStates
{
	NotInitialized=0,
	InitOK=1,
	RequestInit=2,
	ErrorGrip = 3,
	ErrorRotation = 4
} MotorStates;

typedef struct motor_context_t
{
	MotorStates initState;
	uint32_t encRotation;
	uint32_t encGrip;
	uint8_t dac1_byte;
	uint8_t dac2_byte;
	float dac1_volt;
	float dac2_volt;
	DAC_HandleTypeDef* hdac;
} motor_ctx;


extern void InitMotor(motor_ctx* motor);

extern void Open(motor_ctx* motor);




#endif /* INC_MOTOR_H_ */
