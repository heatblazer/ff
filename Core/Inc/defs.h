/*
 * defs.h
 *
 *  Created on: Apr 19, 2019
 *      Author: ilian
 */

#ifndef INC_DEFS_H_
#define INC_DEFS_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"


#define RS485SW_Pin GPIO_PIN_13
#define RS485SW_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOC
#define ROT_INDEX_Pin GPIO_PIN_15
#define ROT_INDEX_GPIO_Port GPIOC
#define STEP_GRIP_Pin GPIO_PIN_6
#define STEP_GRIP_GPIO_Port GPIOA
#define STEP_ROT_Pin GPIO_PIN_7
#define STEP_ROT_GPIO_Port GPIOA
#define MS1_ROT_Pin GPIO_PIN_1
#define MS1_ROT_GPIO_Port GPIOB
#define MS2_ROT_Pin GPIO_PIN_2
#define MS2_ROT_GPIO_Port GPIOB
#define MS3_ROT_Pin GPIO_PIN_10
#define MS3_ROT_GPIO_Port GPIOB
#define SLEEP_ROT_Pin GPIO_PIN_12
#define SLEEP_ROT_GPIO_Port GPIOB
#define DIR_ROT_Pin GPIO_PIN_13
#define DIR_ROT_GPIO_Port GPIOB
#define MS1_GRIP_Pin GPIO_PIN_14
#define MS1_GRIP_GPIO_Port GPIOB
#define MS2_GRIP_Pin GPIO_PIN_15
#define MS2_GRIP_GPIO_Port GPIOB
#define MS3_GRIP_Pin GPIO_PIN_8
#define MS3_GRIP_GPIO_Port GPIOA
#define SLEEP_GRIP_Pin GPIO_PIN_9
#define SLEEP_GRIP_GPIO_Port GPIOA
#define DIR_GRIP_Pin GPIO_PIN_10
#define DIR_GRIP_GPIO_Port GPIOA
#define GRIP_INDEX_Pin GPIO_PIN_4
#define GRIP_INDEX_GPIO_Port GPIOB
#define CAN_SILENT_Pin GPIO_PIN_7
#define CAN_SILENT_GPIO_Port GPIOB



#endif /* INC_DEFS_H_ */
