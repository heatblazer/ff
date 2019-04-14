/*
 * utils.c
 *
 *  Created on: Apr 14, 2019
 *      Author: ilian
 */

#include "utils.h"
#include <stdlib.h> // atoi
#include <string.h>

#define FF_IS_DIGIT(x) ((x>='0') && (x <= '9'))

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

