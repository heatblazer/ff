/*
 * utils.h
 *
 *  Created on: Apr 14, 2019
 *      Author: ilian
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "defs.h"


#ifdef __cplusplus
extern "C" {
#endif


extern void FF_parse_args(const char* input, int* out_arg1, int* out_arg2);

extern void FF_UPrint(UART_HandleTypeDef* uart, const char* message, size_t size);


#ifdef __cplusplus
};
#endif


#endif /* INC_UTILS_H_ */
