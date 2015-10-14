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
//#include "sensfusion6.h"
#include "uart.h"
#include "mavlink_types.h"

mavlink_system_t mavlink_system;
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
extern int __io_putchar(int ch) __attribute__((weak));
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
#include "mavlink.h"

#define TARGET_SYSTEM           20       /* XXX what should these really be? */
#define TARGET_COMPONENT        MAV_COMP_ID_IMU
/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define HEART_UPDATE_RATE_DIVIDER IMU_UPDATE_FREQ

#define ATTITUDE_UPDATE_RATE_DIVIDER  5
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz


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
	//uint32_t tempTime;
	uint16_t heartbCounter = 0;
	uint16_t attitudeCounter = 0;
	uint16_t altHoldCounter = 0;
	//uint32_t data[6];
	//Wait for the system to be fully started to start stabilization loop
	systemWaitStart();
	
	lastWakeTime = xTaskGetTickCount ();

		


	for( ; ;)
	{
		//tempTime = lastWakeTime;
		vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
		heartbCounter ++;
		/*
		if (lastWakeTime < tempTime) {
			tempTime = (0 - tempTime) + lastWakeTime;
		} else {
			tempTime = lastWakeTime - tempTime;
		}
		*/
		while (heartbCounter >= HEART_UPDATE_RATE_DIVIDER) {					// 1Hz
			MAVLINK(mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);)
			heartbCounter = 0;
		}
		imuRead(&gyro, &acc, &mag);
		if (imu6IsCalibrated())
		{
			// 250HZ
			if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
			{
				MahonyAHRSupdateIMU(gyro.y, gyro.x, gyro.z, acc.y, acc.x, acc.z);
				//filterUpdate_mars(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,mag.x,mag.y,mag.z);
				//MahonyAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,mag.x,mag.y,mag.z);
				//MahonyAHRSupdate(gyro.y, gyro.x, gyro.z, acc.y, acc.x, acc.z,mag.y,mag.x,mag.z);
				//filterUpdate_mars(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,mag.x,mag.y,mag.z);
				//MahonyAHRSupdate(gyro.y, gyro.x, gyro.z, acc.y, acc.x, acc.z,mag.y,mag.x,mag.z);

				sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
				radRollActual = eulerRollActual * M_PI / 180.0f;
				radPitchActual = eulerPitchActual * M_PI / 180.0f;
				radYawActual = eulerYawActual * M_PI / 180.0f;



				//float yh, xh;
#define yh (mag.y * cos(radRollActual) - mag.z * sin(radRollActual))
#define xh (mag.x*cos(radPitchActual) + mag.y*sin(radRollActual)*sin(radPitchActual) + mag.z * cos(radRollActual)*sin(radPitchActual))
				radYawActual = atan2(-yh,xh);


				MAVLINK(mavlink_msg_attitude_send(MAVLINK_COMM_0, lastWakeTime, \
							radRollActual, radPitchActual, radYawActual, \
							gyro.x, gyro.y, gyro.z);)

				attitudeCounter = 0;
			}
			// 100HZ
			if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))
			{
				//stabilizerAltHoldUpdate();
				altHoldCounter = 0;
			}
		}

	}
}



void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	if (chan > MAVLINK_COMM_NUM_BUFFERS)
		return;
	__io_putchar(ch);
}

