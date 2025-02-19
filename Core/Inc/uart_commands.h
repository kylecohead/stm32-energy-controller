/*
 * uart_commands.h
 *
 *  Created on: Feb 19, 2025
 *      Author: 25964917
 */

#ifndef INC_UART_COMMANDS_H_
#define INC_UART_COMMANDS_H_

#include "main.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void start_listening_for_commands(void);

#endif /* INC_UART_COMMANDS_H_ */
