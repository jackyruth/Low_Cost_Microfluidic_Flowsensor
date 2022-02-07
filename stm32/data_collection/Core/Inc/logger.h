/*
 * logger.h
 *
 *  Created on: Nov 30, 2021
 *      Author: charl
 */


#include "constants.h"
#include "string.h"
#ifndef INC_PRIVATE_LOGGER_H_
#define INC_PRIVATE_LOGGER_H_

extern char print_data [PRINT_DATA_SIZE];


#define log_test(fmt, ...) sprintf(print_data, fmt"\r\n", __VA_ARGS__); \
		HAL_UART_Transmit(&huart1, (uint8_t*) print_data, strlen(print_data), HAL_MAX_DELAY);\
		memset(print_data, 0, PRINT_DATA_SIZE);

#if LOG_MODULE_MODE > 1
#define log_error(fmt, ...) sprintf(print_data, fmt"\r\n", __VA_ARGS__); \
		HAL_UART_Transmit(&huart1, (uint8_t*) print_data, strlen(print_data), HAL_MAX_DELAY);\
		memset(print_data, 0, PRINT_DATA_SIZE);
#else
#define log_error(fmt, ...)
#endif

#if LOG_MODULE_MODE > 2
#define log_info(fmt, ...) sprintf(print_data, fmt"\r\n", __VA_ARGS__); \
		HAL_UART_Transmit(&huart1, (uint8_t*) print_data, strlen(print_data), HAL_MAX_DELAY);\
		memset(print_data, 0, PRINT_DATA_SIZE);
#else
#define log_info(fmt, ...)
#endif

#if LOG_MODULE_MODE > 3
#define log_debug(fmt, ...) sprintf(print_data, fmt"\r\n", __VA_ARGS__); \
		HAL_UART_Transmit(&huart1, (uint8_t*) print_data, strlen(print_data), HAL_MAX_DELAY);\
		memset(print_data, 0, PRINT_DATA_SIZE);
#else
#define log_debug(fmt, ...)
#endif



#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#endif /* INC_PRIVATE_LOGGER_H_ */
