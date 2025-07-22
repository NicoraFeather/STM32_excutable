//
// Created by lak19 on 2025/7/14.
//

#ifndef UART_COMMAND_H
#define UART_COMMAND_H

#include "usart.h"
#include "../../Lib/Inc/functional.h"
void Decode_Command(uint16_t size);
void USART_Parse_Command(char *str, uint8_t motor_n);
#endif //UART_COMMAND_H
