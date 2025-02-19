/*
 * uart_commands.c
 *
 *  Created on: Feb 19, 2025
 *      Author: 25964917
 */

#include "uart_commands.h"
#include <stdint.h>


// Define receive buffer
uint8_t rx_buffer[50];
volatile uint8_t rx_index = 0;

//UART receive interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		// Check for full command
		if (rx_index == 0 && rx_buffer[0] == '*') {
			rx_index++;
		} else if (rx_index > 0) {
			if (rx_buffer[rx_index] == '\n') {
				rx_buffer[rx_index] = '\0'; // Null terminate the string
				if (strcmp((char*) rx_buffer, "*Load#") == 0) {
					// Command to turn on LED D2
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				}
				rx_index = 0; // Reset for next command
			} else if (rx_index < 49) {
				rx_index++;
			} else {
				rx_index = 0; // Buffer overflow, reset for safety
			}
		}
		// Restart listening for next byte
		HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
	}
}

void start_listening_for_commands(void) {
	// Start listening for first byte
	HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

