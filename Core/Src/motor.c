#include "motor.h"
#include "stm32f4xx_hal_uart.h"
#include <string.h>

/**
 *  volatile uint16_t adc[4];
	 float dac1_volt = 0.5;
	 float dac2_volt = 0.5;
	 uint8_t dac1_byte;
	 uint8_t dac2_byte;
	 uint16_t index_rot = 0;
	 uint16_t index_grip=0;
	 uint8_t rxData[30];
	 uint8_t txData[30];
	 int  encGrip; // Tim 2
	 int  encRot; // Tim 5
 */

static uint8_t checksum(uint8_t* data, size_t size)
{
	size_t s;
	uint8_t check = 0xff;
	for(s = 0; s < size-1; s++)
	{
		check ^= data[s];
	}
	data[s++] = check;// write the check at the last byte
}


static void set_transmit_mode(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);  // RS 485 transmit mode
}

static void set_receive_mode(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // RS 485 receive mode
}

/*
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
*/
motor_ctx get_motor_context(float dac1_v, float dac2_v, uint32_t baudrate, int stopbits, int parity)
{
	(void)stopbits;
	(void) parity;
	motor_ctx ctx;
	memset(&ctx, 0, sizeof(ctx));
	ctx.dac1_volt = dac1_v;
	ctx.dac2_volt = dac2_v;
	ctx.uart.gpio.Pin = GPIO_PIN_2;
	ctx.uart.gpio.Mode = GPIO_MODE_AF_PP;
	ctx.uart.gpio.Alternate = GPIO_AF7_USART2;
	ctx.uart.gpio.Speed = GPIO_SPEED_HIGH;
	ctx.uart.gpio.Pull = GPIO_NOPULL;
	//void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
	HAL_GPIO_Init(GPIOA, &ctx.uart.gpio);
	ctx.uart.gpio.Pin = GPIO_PIN_3;
	ctx.uart.gpio.Mode = GPIO_MODE_AF_OD;
	HAL_GPIO_Init(GPIOA, &ctx.uart.gpio);
	ctx.uart.handle.Instance = USART2;
	ctx.uart.handle.Init.BaudRate = baudrate; // 9600
	ctx.uart.handle.Init.WordLength = UART_WORDLENGTH_8B;
	ctx.uart.handle.Init.StopBits = UART_STOPBITS_1;
	ctx.uart.handle.Init.Parity = UART_PARITY_NONE;
	ctx.uart.handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	ctx.uart.handle.Init.Mode = UART_MODE_TX_RX;
	if (HAL_UART_Init(&ctx.uart.handle) != HAL_OK)
		asm("bkpt 255");
	return ctx;
}


void transmit_motor_data(motor_ctx* ctx)
{

}

void recieve_motor_data(motor_ctx* ctx)
{

}

void start_motor(motor_ctx* ctx)
{
	// do the logic here
}
