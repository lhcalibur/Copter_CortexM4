#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

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
		while (heartb >= 400) {					// 1Hz
			MAVLINK(mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);)
			heartb = 0;
		}
		imuRead(&gyro, &acc, &mag);
		if (imu6IsCalibrated())
		{

			//filterUpdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
			//filterUpdate_mars(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,mag.x,mag.y,mag.z);
			MahonyAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,mag.x,mag.y,mag.z);

			//EulerUpdate(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
			sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

			radRollActual = eulerRollActual * M_PI / 180.0f;
			radPitchActual = eulerPitchActual * M_PI / 180.0f;
			radYawActual = eulerYawActual * M_PI / 180.0f;
			MAVLINK(mavlink_msg_attitude_send(MAVLINK_COMM_0, lastWakeTime, \
					radRollActual, radPitchActual, radYawActual, \
					gyro.x, gyro.y, gyro.z);)
		}
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

