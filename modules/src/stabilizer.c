#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_imu_6axes.h"
#include "x_nucleo_iks01a1_magneto.h"

#include <stdbool.h>
#include <math.h>

#include "config.h"
#include "stabilizer.h"
#include "system.h"
#include "imu.h"
#include "imu_type.h"
#include "imu_filter.h"
#include "uart.h"

// TODO: Fix __errno linker error with math lib
int __attribute__((used)) __errno;


static bool isInit = false;



//char dataOut[256];
volatile Axes_TypeDef ACC_Value;         /*!< Acceleration Value */
volatile Axes_TypeDef GYR_Value;         /*!< Gyroscope Value */
volatile Axes_TypeDef MAG_Value;         /*!< Magnetometer Value */

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;


static void stabilizerTask(void* param);

void stabilizerInit(void)
{
	if(isInit)
		return;
	imuInit();

	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
			STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);



	isInit = true;
}

static void stabilizerTask(void* param)
{
	uint32_t lastWakeTime;
	//uint32_t data[6];
	//Wait for the system to be fully started to start stabilization loop
	systemWaitStart();

	lastWakeTime = xTaskGetTickCount ();

	for( ; ;)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
		BSP_IMU_6AXES_X_GetAxes((Axes_TypeDef *)&ACC_Value);
		BSP_IMU_6AXES_G_GetAxes((Axes_TypeDef *)&GYR_Value);
		BSP_MAGNETO_M_GetAxes((Axes_TypeDef *)&MAG_Value);


		//data[0] = ACC_Value.AXIS_X;
		//data[1] = ACC_Value.AXIS_Y;
		//data[2] = ACC_Value.AXIS_Z;
		//data[3] = GYR_Value.AXIS_X;
		//data[4] = GYR_Value.AXIS_Y;
		//data[5] = GYR_Value.AXIS_Z;

		acc.x = ACC_Value.AXIS_X / 1000.0;
		acc.y = ACC_Value.AXIS_Y / 1000.0;
		acc.z = ACC_Value.AXIS_Z / 1000.0;
		gyro.x = GYR_Value.AXIS_X / 1000.0;
		gyro.y = GYR_Value.AXIS_Y / 1000.0;
		gyro.z = GYR_Value.AXIS_Z / 1000.0;
		mag.x = MAG_Value.AXIS_X / 1000.0f;
		mag.y = MAG_Value.AXIS_Y / 1000.0f;
		mag.z = MAG_Value.AXIS_Z / 1000.0f;

		gyro.x = gyro.x * M_PI / 180.0;
		gyro.y = gyro.y * M_PI / 180.0;
		gyro.z = gyro.z * M_PI / 180.0;

		//filterUpdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
		filterUpdate_mars(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,mag.x,mag.y,mag.z);

		EulerUpdate(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
		printf("%4.5f %4.5f %4.5f\n",eulerRollActual,eulerPitchActual,eulerYawActual);



		//printf("%4.5f %4.5f %4.5f %4.5f %4.5f %4.5f\n",acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z);
		//printf("%4.5f %4.5f %4.5f\n",gyro.x,gyro.y,gyro.z);
		//printf("%ld %ld %ld\n",MAG_Value.AXIS_X,MAG_Value.AXIS_Y,MAG_Value.AXIS_Z);
      
		//printf("ACC_X: %d, ACC_Y: %d, ACC_Z: %d\n", (int)data[0], (int)data[1], (int)data[2]);

		//printf("GYR_X: %d, GYR_Y: %d, GYR_Z: %d\n", (int)data[3], (int)data[4], (int)data[5]);

	}



}
