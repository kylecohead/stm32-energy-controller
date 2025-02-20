/*
 * uart_commands.c
 *
 *  Created on: Feb 19, 2025
 *      Author: 25964917
 */

#include "uart_commands.h"
#include "string.h"
#include <stdint.h>

// Define receive buffer
uint8_t cmd_buffer[1000] = "";
size_t cmd_buffer_index = 0;

// Define current command buffer
uint8_t current_cmd[1000] = "";
uint16_t current_cmd_length = 0;

// processed command flag, 1: no commands to process, 0: command to be processed
int processed_cmd_flag = 1;

//UART receive interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (cmd_buffer[cmd_buffer_index] == '\n' || cmd_buffer[cmd_buffer_index] == '\r')
	{
		size_t i = cmd_buffer_index;
		while (cmd_buffer[i] == '\r' || cmd_buffer[i] == '\r')
		{
			i-- ;
		}
		strncpy(&current_cmd, cmd_buffer, i+1);
		bzero(current_cmd, i+2);

		current_cmd_length = i+1;
		processed_cmd_flag = 1;
		cmd_buffer_index = 0;
	}
	else
	{
		cmd_buffer_index ++;
	}
	UART_wait_for_commands();

}

void UART_wait_for_commands(void) {
	// Start listening for first byte
	HAL_UART_Receive_IT(&huart2, cmd_buffer + cmd_buffer_index, 1);
}

