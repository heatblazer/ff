/*
 * motor.h
 *
 *  Created on: Apr 16, 2019
 *      Author: ilian
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "defs.h"

typedef enum MotorInitStates_t
{
	NotInitialized=0,
	InitOK=1,
	RequestInit=2,
	ErrorGrip = 3,
	ErrorRotation = 4,
} MotorStates;


typedef enum MotorRequests_t
{
	RequestFree,
	Open,
	Close,
	RotateL,
	RotateR,
	RequestOk,
	RequestFailed// error or unknown data
} MotorRequests;



typedef struct motor_context_t
{
	// to be deleted
	struct
	{
		uint32_t tim2DiffCounter;
	} debugValues;

	MotorRequests motorRequests;
	MotorStates initState;
	volatile uint32_t targetRot;
	volatile uint32_t targetGrip;
	volatile uint32_t movePosition; // rotate left right variable
	uint32_t encRotation;
	uint32_t encGrip;
	uint32_t nextIrq;
	uint8_t fast;
	volatile uint8_t dac1_volt;
	volatile uint8_t dac2_volt;
	volatile uint16_t adc[4];
	DAC_HandleTypeDef* hdac;
} motor_ctx;


extern void InitMotor(motor_ctx* motor);

extern void OpenMotor(motor_ctx* motor);

extern void CloseMotor(motor_ctx* motor);

extern void RotateLeft(motor_ctx* motor, int movestep, int volt);

extern void RotateRight(motor_ctx* motor, int movestep, int volt);

extern void MotorStatus(motor_ctx* motor);


#endif /* INC_MOTOR_H_ */
