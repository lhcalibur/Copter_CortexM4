#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "uart.h"
int main(void)
{
	HAL_Init();
	USART2_UART_Init();
	vTaskStartScheduler();
	while(1);	
	return 0;
}


