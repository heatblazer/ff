/*
 * motor.c
 *
 *  Created on: Apr 16, 2019
 *      Author: ilian
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "motor.h"

//
//	Details:{Instance = 0x40010000, Init = {Prescaler = 2, CounterMode = 0, Period = 2500, ClockDivision = 0, RepetitionCounter = 0, AutoReloadPreload = 0}, Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED, hdma = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, Lock = HAL_UNLOCKED, State = HAL_TIM_STATE_READY}


//	  if (HAL_GPIO_ReadPin(GPIOC,ROT_INDEX_Pin) == 1)
//	  		{HAL_GPIO_WritePin(GPIOC, LED_Pin, 1);}
//	  else {HAL_GPIO_WritePin(GPIOC, LED_Pin, 0);}

#if 0
dac1_byte = (uint8_t) ((dac1_volt / 3.3) * 255);
	dac2_byte = (uint8_t) ((dac2_volt / 3.3) * 255);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dac1_byte);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, dac2_byte);
#endif


extern UART_HandleTypeDef huart6;


#define DELAY_LOOP 2000

void InitMotor(motor_ctx* motor)
{
	if (motor->initState != 2)
		return ;
	if (motor->initState == 2)
	{

		motor->dac2_volt = 23;
		HAL_DAC_SetValue(&motor->hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, (uint8_t)motor->dac2_volt);
		HAL_GPIO_WritePin(GPIOA, SLEEP_GRIP_Pin, 1);
	    HAL_GPIO_WritePin(GPIOA, DIR_GRIP_Pin, 0);
	    TIM2->CNT = 10000;
	    motor->encGrip = 0;
	    motor->targetGrip = 10000 * 1000;

	    for (;;)
	    {
		  HAL_GPIO_WritePin(GPIOA, STEP_GRIP_Pin, 1);
	      for(int i=0; i < 10; i++)
	    	  __asm__ __volatile__ ("nop\n\t");
	  	  HAL_GPIO_WritePin(GPIOA, STEP_GRIP_Pin, 0);
	  	 // motor->encGrip -= 1;
	  	  motor->targetGrip -= 625;
	  	  for(uint32_t i=0; i < DELAY_LOOP; i++);
	  	  if (TIM2->CNT > (motor->targetGrip/1000))
	  		  if (((TIM2->CNT + 20) - (motor->targetGrip/1000)) >= 100)
	  			  break;

	    }

	    motor->targetGrip = 10000 * 1000;
	    TIM2->CNT = 10000;
	    HAL_GPIO_WritePin(GPIOA, DIR_GRIP_Pin, 1);
	    for (;;)
	    {
	  	  HAL_GPIO_WritePin(GPIOA, STEP_GRIP_Pin, 1);
	      for(int i=0; i < 10; i++)
	    	  __asm__ __volatile__ ("nop\n\t");

	      HAL_GPIO_WritePin(GPIOA, STEP_GRIP_Pin, 0);
	      motor->targetGrip += 625;
	  	  for(uint32_t i=0; i < DELAY_LOOP; i++);
	  	  if ((motor->targetGrip/1000) > TIM2->CNT)
	  		  if ((((motor->targetGrip/1000) + 20) - TIM2->CNT) >= 100)
	  			  break;
	    }// for

	    if (TIM2->CNT < 17000)
		{
			motor->initState = 3;
			return;
		}


	    HAL_GPIO_WritePin(GPIOA, DIR_GRIP_Pin, 0);
	    for(int i=0; i < 500; i++)
	    {
	    	HAL_GPIO_WritePin(GPIOA, STEP_GRIP_Pin, 1);
	    	for(int i=0; i < 10; i++)
		    	  __asm__ __volatile__ ("nop\n\t");
		    HAL_GPIO_WritePin(GPIOA, STEP_GRIP_Pin, 0);
		    for(uint32_t i=0; i < DELAY_LOOP; i++);
	    }

		//HAL_GPIO_WritePin(GPIOA, SLEEP_GRIP_Pin, 0);
		TIM2->CNT = 10000;
		motor->targetGrip = 10000 * 1000;
		motor->dac1_volt = 23;

		HAL_DAC_SetValue(motor->hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, (uint8_t)motor->dac1_volt);
		HAL_GPIO_WritePin(GPIOB, SLEEP_ROT_Pin, 1);// enable rotor power

		uint32_t rotcounter = 0;

		HAL_GPIO_WritePin(GPIOB, DIR_ROT_Pin, 1);

		for(;;)
		{
			rotcounter++;
			HAL_GPIO_WritePin(GPIOA, STEP_ROT_Pin, 1);
			for(int i=0; i < 10; i++)
				  __asm__ __volatile__ ("nop\n\t");
			HAL_GPIO_WritePin(GPIOA, STEP_ROT_Pin, 0);

			for(int i=0; i < DELAY_LOOP * 5; i++) ;

			if (HAL_GPIO_ReadPin(GPIOC, ROT_INDEX_Pin) == 0)
			{
				motor->initState = 1;
				break;
			}
			if (rotcounter >= 5000)
			{
				motor->initState = 4;
				HAL_GPIO_WritePin(GPIOB, SLEEP_ROT_Pin, 0);// turn off motor
				return;
			}
		}

		motor->dac1_volt = 8;
		HAL_DAC_SetValue(motor->hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, (uint8_t)motor->dac1_volt);
		TIM5->CNT = 1000000;
	}
}



void OpenMotor(motor_ctx* motor)
{
	if (motor->initState != 1 || motor->motorRequests != RequestFree)
		return;

	motor->dac2_volt = 120; // to be decided later

	HAL_DAC_SetValue(&motor->hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, motor->dac2_volt);
	HAL_GPIO_WritePin(GPIOA, DIR_GRIP_Pin, 1);
//	HAL_GPIO_WritePin(GPIOA, SLEEP_GRIP_Pin, 1);
	motor->motorRequests = Open;
}

void CloseMotor(motor_ctx* motor)
{
	if (motor->initState != 1 || motor->motorRequests != RequestFree)
		return;

	motor->dac2_volt = 120;
	HAL_DAC_SetValue(motor->hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, motor->dac2_volt);
    HAL_GPIO_WritePin(GPIOA, DIR_GRIP_Pin, 0);
//	HAL_GPIO_WritePin(GPIOA, SLEEP_GRIP_Pin, 1);
	motor->motorRequests = Close;

}

void RotateLeft(motor_ctx* motor, int movestep, int volt)
{
	if (motor->initState != 1 || motor->motorRequests != RequestFree)
		return;


	if (volt >= 150)
		volt = 150;
	else if (volt < 0) // if no arg or neg
		volt = 150;

	if (movestep < 0)
		movestep = 6000;
	else if (movestep > 30000)
		movestep = 30000;

	HAL_GPIO_WritePin(GPIOB, DIR_ROT_Pin, 0);
	HAL_DAC_SetValue(motor->hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, volt);
	motor->movePosition = TIM5->CNT + movestep;
	motor->targetRot = TIM5->CNT * 1000;
	motor->motorRequests = RotateL;
}

void RotateRight(motor_ctx* motor, int movestep, int volt)
{
	if (motor->initState != 1 || motor->motorRequests != RequestFree)
		return;

	if (volt >= 150)
		volt = 150;
	else if (volt < 0) // if no arg or neg
		volt = 80;

	if (movestep < 0)
		movestep = 6000;
	else if (movestep > 30000)
		movestep = 30000;

	HAL_GPIO_WritePin(GPIOB, DIR_ROT_Pin, 1);
	HAL_DAC_SetValue(motor->hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, volt);
	motor->movePosition = TIM5->CNT - movestep;
	motor->targetRot = TIM5->CNT * 1000;
	motor->motorRequests = RotateR;
}



void MotorStatus(motor_ctx* motor)
{
	if (!motor)
		return;

	 char msg[256] = {0};
	snprintf(msg, sizeof(msg), "Status:init state:(%d), request:(%d)"
			"adc0:(%u), adc1(%u), volt(%u), Temperature(%u)"
			",TIM2:(%u), TIM5:(%u)\r\n",
			motor->initState,
			motor->motorRequests,
			motor->adc[0],motor->adc[1],motor->adc[2],motor->adc[3],
			TIM2->CNT, TIM5->CNT);
	FF_UPrint(&huart6, msg, sizeof(msg));
	memset(msg, 0, sizeof(msg));
}



