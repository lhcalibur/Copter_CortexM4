#include "FreeRTOS.h"
#include "task.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_imu_6axes.h"

#include <stdbool.h>

#include "imu.h"

#define IMU_STARTUP_TIME_MS   1000

// TODO: Fix __errno linker error with math lib
//int __attribute__((used)) __errno;

static bool isInit = false;
static bool isMagPresent;
static bool isBaroPresent;


void imuInit(void)
{
	if(isInit)
		return;

	isMagPresent = false;
	isBaroPresent = false;

	// Wait for sensors to startup
	while (xTaskGetTickCount() < M2T(IMU_STARTUP_TIME_MS));


	/* Initialize the IMU 6-axes */
	BSP_IMU_6AXES_Init();


	isInit = true;

}

