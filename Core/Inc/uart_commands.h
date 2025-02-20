/*
 * uart_commands.h
 *
 *  Created on: Feb 19, 2025
 *      Author: 25964917
 */

#ifndef INC_UART_COMMANDS_H_
#define INC_UART_COMMANDS_H_

extern UART_HandleTypeDef huart2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART_wait_for_commands();

#endif /* INC_UART_COMMANDS_H_ */
