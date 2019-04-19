/*
 * utils.c
 *
 *  Created on: Apr 14, 2019
 *      Author: ilian
 */

#include "utils.h"

#define FF_IS_DIGIT(x) ((x>='0') && (x <= '9'))


#if 0
char c[64];
// memset(txData, 0, 32);
memset(c, 0 ,sizeof(c));
snprintf(c, sizeof(c), "RotEnc %u Cnt %u razl %u\n\r" , encRot, positionRot, (unsigned int) razlika);

/*          ---- tova trqbva da stane funkciq

HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 1);
HAL_UART_Transmit(&huart6, (uint8_t *)c, strlen(c), 10);  // 5 e duljina na izprashtaniq paket, 10 e milisekundi timeout
HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 0);
*/
#if 0
HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 1);
HAL_UART_Transmit_IT(&huart6, (uint8_t *)c, strlen(c));  // 5 e duljina na izprashtaniq paket, 10 e milisekundi timeout
HAL_Delay(DELAY_TIME);
#endif
#endif

#define FF_UART_DELAY_TIME 100

void FF_UPrint(UART_HandleTypeDef* uart, const char* message, size_t size)
{
	if (!message || uart)
		return;
	HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 1);
	HAL_UART_Transmit_IT(uart, (uint8_t*)message, size);  // 5 e duljina na izprashtaniq paket, 10 e milisekundi timeout
	HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 0);
	HAL_Delay(FF_UART_DELAY_TIME);
}


void FF_parse_args(const char* input, int* out_arg1, int* out_arg2)
{


	int arg1, arg2;
	char dbuff[16] = {0}, *pDbuff;
	pDbuff = dbuff;
	const char* it = input;
	const char* digit= 0x00;
	while (*it != '\0')
	{
		if (FF_IS_DIGIT(*it))
		{
			digit = it;
			while (*digit != '\0' && FF_IS_DIGIT(*digit))
			{
				*pDbuff++ = *digit++;
			}

			arg1 = atoi(dbuff);
			*out_arg1 = arg1;
			memset(dbuff, 0, sizeof(dbuff));
			digit++;

			pDbuff = &dbuff[0];

			while(!FF_IS_DIGIT(*digit) && *digit != '\0') digit++;

			if (*digit == '\0')
			{
				*out_arg2 = -1;
				return;
			}
			else
			{
				while (FF_IS_DIGIT(*digit))
				{
					*pDbuff++ = *digit++;
				}

				arg2 = atoi(dbuff);
				*out_arg2 = arg2;
				memset(dbuff, 0, sizeof(dbuff));
			}

			break;
		}
		it++;
	}
	if (*it == '\0') // did not match any digits
	{
		*out_arg1 = -1;
		*out_arg2 = -1;
	}
}

