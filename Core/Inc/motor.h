/*
 * motor.h
 *
 *  Created on: Apr 2, 2019
 *      Author: ilian
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <stm32_hal_legacy.h>

 typedef struct motor_context_t
 {
	 float dac1_volt;
	 float dac2_volt;
	 uint8_t dac1_byte;
	 uint8_t dac2_byte;
	 uint16_t index_rot;
	 uint16_t index_grip;
	 volatile uint16_t adc[4];
	 int  encGrip; // Tim 2
	 int  encRot; // Tim 5
	 struct /* represents the Uart device */
	 {
		 UART_HandleTypeDef handle;
		 GPIO_InitTypeDef  gpio;
		 uint8_t rxData[30];
		 uint8_t txData[30];
	 } uart;
 } motor_ctx;


// init motor context
extern motor_ctx get_motor_context(float dac1_v, float dac2_v, uint32_t baudrate, int stopbits, int parity);

 extern void transmit_motor_data(motor_ctx* ctx);

 extern void recieve_motor_data(motor_ctx* ctx);

 extern void start_motor(motor_ctx* ctx);



#ifdef __cplusplus
 }
#endif


#endif /* INC_MOTOR_H_ */
