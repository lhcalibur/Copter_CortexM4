#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>

#include "system.h"
#include "config.h"
#include "uart.h"
#include "stabilizer.h"

static bool isInit = false;

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;


/* Private functions */
static void systemTask(void *arg);


void systemLaunch(void)
{
	xTaskCreate(systemTask, SYSTEM_TASK_NAME,
			SYSTEM_TASK_STACKSIZE, NULL,
			SYSTEM_TASK_PRI, NULL);

}

static void systemTask(void *arg)
{
	bool pass = true;

	USART2_UART_Init();
	systemInit();

	stabilizerInit();

}

void systemInit(void)
{
	if(isInit)
		return;

	canStartMutex = xSemaphoreCreateMutex();
	xSemaphoreTake(canStartMutex, portMAX_DELAY);

	isInit = true;
}
