#include "stm32f4xx_nucleo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_imu_6axes.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"

#include <stdbool.h>
#include <math.h>

#include "imu.h"
#include "filter.h"
#include "uart.h"

#define IMU_STARTUP_TIME_MS   1000

#define GYRO_NBR_OF_AXES 3
//#define GYRO_X_SIGN      (-1)
//#define GYRO_Y_SIGN      (-1)
//#define GYRO_Z_SIGN      (-1)

#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)


#define IMU_NBR_OF_BIAS_SAMPLES  128

#define GYRO_VARIANCE_BASE        2000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

//#define IMU_TAKE_ACCEL_BIAS

typedef struct
{
  Axis3i16   bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} BiasObj;

BiasObj    gyroBias;
#ifdef IMU_TAKE_ACCEL_BIAS
BiasObj    accelBias;
#endif

static int32_t    varianceSampleTime;
static Axis3i16   gyroMpu;
static Axis3i16   accelMpu;
static Axis3i16   accelLPF;
//static Axis3i16   accelLPFAligned;
static Axis3i32   mag;
static Axis3i32   accelStoredFilterValues;
static uint8_t    imuAccLpfAttFactor;


// TODO: Fix __errno linker error with math lib
//int __attribute__((used)) __errno;

static bool isInit = false;

static void imuBiasInit(BiasObj* bias);
static void imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut);
static bool imuFindBiasValue(BiasObj* bias);
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                              Axis3i32* storedValues, int32_t attenuation);
//static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out);



void imuInit(void)
{
	if(isInit)
		return;

	// Wait for sensors to startup
	while (xTaskGetTickCount() < M2T(IMU_STARTUP_TIME_MS));


	/* Initialize the IMU 6-axes */
	BSP_IMU_6AXES_Init();
	BSP_IMU_6AXES_X_Set_ODR(500.0f);
	BSP_IMU_6AXES_X_Set_FS(8.0f);
	BSP_IMU_6AXES_G_Set_ODR(500.0f);
	BSP_IMU_6AXES_G_Set_FS(2000.0f);
	BSP_MAGNETO_Init();
	BSP_PRESSURE_Init();


	imuBiasInit(&gyroBias);
#ifdef IMU_TAKE_ACCEL_BIAS
	imuBiasInit(&accelBias);
#endif
	varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
	imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;


	isInit = true;

}

void imuRead(Axis3f* gyroOut, Axis3f* accOut, Axis3f* magOut)
{
	float G_sensitivity = 0.0f;
	float X_sensitivity = 0.0f;

	BSP_IMU_6AXES_X_GetAxesRaw((AxesRaw_TypeDef *)&accelMpu);
	BSP_IMU_6AXES_G_GetAxesRaw((AxesRaw_TypeDef *)&gyroMpu);
	BSP_MAGNETO_M_GetAxes((Axes_TypeDef *)&mag);
	BSP_IMU_6AXES_G_GetSensitivity(&G_sensitivity);	
	BSP_IMU_6AXES_X_GetSensitivity(&X_sensitivity);


	imuAddBiasValue(&gyroBias, &gyroMpu);
#ifdef IMU_TAKE_ACCEL_BIAS
	if (!accelBias.isBiasValueFound)
	{
		imuAddBiasValue(&accelBias, &accelMpu);
	}
#endif
	if (!gyroBias.isBiasValueFound)
	{
		imuFindBiasValue(&gyroBias);
		if (gyroBias.isBiasValueFound)
		{
			//ledseqRun(SYS_LED, seq_calibrated);
#ifndef IMU_TAKE_ACCEL_BIAS
			BSP_LED_On(LED2);
#endif
		}
	}

#ifdef IMU_TAKE_ACCEL_BIAS
	if (gyroBias.isBiasValueFound &&
			!accelBias.isBiasValueFound)
	{
		Axis3i32 mean;

		imuCalculateBiasMean(&accelBias, &mean);
		accelBias.bias.x = mean.x;
		accelBias.bias.y = mean.y;
		accelBias.bias.z = mean.z - (1 / X_sensitivity * 1000.0f);
		accelBias.isBiasValueFound = true;
		BSP_LED_On(LED2);
	}
#endif
	imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues, (int32_t)imuAccLpfAttFactor);

	//imuAccAlignToGravity(&accelLPF, &accelLPFAligned);





	/*
	accOut->x = ACC_Value.AXIS_X / 1000.0;
	accOut->y = ACC_Value.AXIS_Y / 1000.0;
	accOut->z = ACC_Value.AXIS_Z / 1000.0;
	gyroOut->x = GYR_Value.AXIS_X / 1000.0;
	gyroOut->y = GYR_Value.AXIS_Y / 1000.0;
	gyroOut->z = GYR_Value.AXIS_Z / 1000.0;
	magOut->x = MAG_Value.AXIS_X / 1000.0f;
	magOut->y = MAG_Value.AXIS_Y / 1000.0f;
	magOut->z = MAG_Value.AXIS_Z / 1000.0f;

	gyroOut->x = gyroOut->x * M_PI / 180.0;
	gyroOut->y = gyroOut->y * M_PI / 180.0;
	gyroOut->z = gyroOut->z * M_PI / 180.0;
	*/


	// Re-map outputs
	
	gyroOut->x = (gyroMpu.x - gyroBias.bias.x) * G_sensitivity / 1000.0f;
	gyroOut->y = (gyroMpu.y - gyroBias.bias.y) * G_sensitivity / 1000.0f;
	gyroOut->z = (gyroMpu.z - gyroBias.bias.z) * G_sensitivity / 1000.0f;

	/*
	gyroOut->x = (gyroMpu.x) * G_sensitivity / 1000.0f;
	gyroOut->y = (gyroMpu.y) * G_sensitivity / 1000.0f;
	gyroOut->z = (gyroMpu.z) * G_sensitivity / 1000.0f;
	*/

	gyroOut->x = - gyroOut->x * M_PI / 180.0f;
	gyroOut->y = gyroOut->y * M_PI / 180.0f;
	gyroOut->z = gyroOut->z * M_PI / 180.0f;

#ifdef IMU_TAKE_ACCEL_BIAS
	/*
	accOut->x = (accelLPFAligned.x - accelBias.bias.x) * X_sensitivity / 1000.0f;
	accOut->y = (accelLPFAligned.y - accelBias.bias.y) * X_sensitivity / 1000.0f;
	accOut->z = (accelLPFAligned.z - accelBias.bias.z) * X_sensitivity / 1000.0f;
	*/
	accOut->x = - (accelLPF.x - accelBias.bias.x) * X_sensitivity / 1000.0f;
	accOut->y = (accelLPF.y - accelBias.bias.y) * X_sensitivity / 1000.0f;
	accOut->z = (accelLPF.z - accelBias.bias.z) * X_sensitivity / 1000.0f;

#else
	//accOut->x = (accelLPFAligned.x) * X_sensitivity / 1000.0f;
	//accOut->y = (accelLPFAligned.y) * X_sensitivity / 1000.0f;
	//accOut->z = (accelLPFAligned.z) * X_sensitivity / 1000.0f;
	accOut->x =  - (accelLPF.x) * X_sensitivity;
	accOut->y = (accelLPF.y) * X_sensitivity;
	accOut->z = (accelLPF.z) * X_sensitivity;

#endif
	magOut->x = (float)mag.x / 10000.0f;
	magOut->y = (float)mag.y / 10000.0f;
	magOut->z = (float)mag.z / 10000.0f;


}

bool imu6IsCalibrated(void)
{
	bool status;

	status = gyroBias.isBiasValueFound;
#ifdef IMU_TAKE_ACCEL_BIAS
	status &= accelBias.isBiasValueFound;
#endif

	return status;
}

static void imuBiasInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;


}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal)
{
  bias->bufHead->x = dVal->x;
  bias->bufHead->y = dVal->y;
  bias->bufHead->z = dVal->z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool imuFindBiasValue(BiasObj* bias)
{
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    Axis3i32 variance;
    Axis3i32 mean;

    imuCalculateVarianceAndMean(bias, &variance, &mean);
    DEBUG_PRINT("%ld, %ld, %ld\n", variance.x, variance.y, variance.z);
    //DEBUG_PRINT("%ld, %ld, %ld\n", mean.x, mean.y, mean.z);
    

    if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = mean.x;
      bias->bias.y = mean.y;
      bias->bias.z = mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}


/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
/*
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out)
{
  Axis3i16 rx;
  Axis3i16 ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}
*/
bool imuHasBarometer(void)
{
	return BSP_PRESSURE_isInitialized();
}

