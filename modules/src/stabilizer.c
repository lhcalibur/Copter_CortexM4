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
#include "mavlink_types.h"

mavlink_system_t mavlink_system;
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
extern int __io_putchar(int ch) __attribute__((weak));
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
#include "mavlink.h"

#define TARGET_SYSTEM           20       /* XXX what should these really be? */
#define TARGET_COMPONENT        MAV_COMP_ID_IMU


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

static float radRollActual;
static float radPitchActual;
static float radYawActual;


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
	uint16_t heartb = 0;
	//uint32_t data[6];
	//Wait for the system to be fully started to start stabilization loop
	systemWaitStart();

	lastWakeTime = xTaskGetTickCount ();

	for( ; ;)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 400Hz
		heartb ++;
		while (heartb >= 500) {					// 1Hz
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
			heartb = 0;
		}


		BSP_IMU_6AXES_X_GetAxes((Axes_TypeDef *)&ACC_Value);
		BSP_IMU_6AXES_G_GetAxes((Axes_TypeDef *)&GYR_Value);
		BSP_MAGNETO_M_GetAxes((Axes_TypeDef *)&MAG_Value);

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
		//filterUpdate_mars(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,mag.x,mag.y,mag.z);
		MahonyAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,mag.x,mag.y,mag.z);

		//EulerUpdate(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
		sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

		radRollActual = eulerRollActual * M_PI / 180.0f;
		radPitchActual = eulerPitchActual * M_PI / 180.0f;
		radYawActual = eulerYawActual * M_PI / 180.0f;
		mavlink_msg_attitude_send(MAVLINK_COMM_0, lastWakeTime, \
				radRollActual, radPitchActual, radYawActual, \
				gyro.x, gyro.y, gyro.z);
		//printf("%4.5f %4.5f %4.5f\n",radRollActual,radPitchActual,radYawActual);
		//printf("%4.5f %4.5f %4.5f\n",eulerRollActual,eulerPitchActual,eulerYawActual);
		
	}
}



void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	if (chan > MAVLINK_COMM_NUM_BUFFERS)
		return;
	__io_putchar(ch);
}

