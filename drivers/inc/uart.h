#ifndef UART_H
#define UART_H

#include <stdio.h>
#include "stm32f4xx_hal.h"


#ifdef DEBUG_PRINT_ON_UART
#define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

void USART2_UART_Init(void);

#endif /* UART_H */

