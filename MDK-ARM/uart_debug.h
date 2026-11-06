#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "main.h"
#include <stdbool.h>

#define DEBUG_BUF_SIZE  2048

void Debug_Init(UART_HandleTypeDef *huart);
void Debug_Print(const char *str);
void Debug_Printf(const char *fmt, ...);
void Debug_Process(void);  
void Debug_TxCpltCallback(UART_HandleTypeDef *huart);

#endif