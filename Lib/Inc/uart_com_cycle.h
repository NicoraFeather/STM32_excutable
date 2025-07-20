//
// Created by lak19 on 2025/7/20.
//

#ifndef UART_COM_CYCLE_H
#define UART_COM_CYCLE_H


#include "../../Core/Inc/main.h"
#include <string.h>

uint8_t Command_Write(uint8_t *data, uint8_t length);

uint8_t Command_GetCommand(uint8_t *command);

#endif //UART_COM_CYCLE_H
