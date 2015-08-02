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

/* Private variable */
static bool selftestPassed = false;
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

	//Start the firmware
	if(pass)
	{
		selftestPassed = true;
		systemStart();
	}
	else
	{
		
	}
	for(; ;);


}

void systemInit(void)
{
	if(isInit)
		return;

	canStartMutex = xSemaphoreCreateMutex();
	xSemaphoreTake(canStartMutex, portMAX_DELAY);

	isInit = true;
}

void systemWaitStart(void)
{
	//This permits to guarantee that the system task is initialized before other
	//tasks waits for the start event.
	while(!isInit)
		vTaskDelay(2);

	xSemaphoreTake(canStartMutex, portMAX_DELAY);
	xSemaphoreGive(canStartMutex);
}

/* Global system variables */
void systemStart(void)
{
  xSemaphoreGive(canStartMutex);
}
