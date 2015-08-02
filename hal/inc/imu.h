#ifndef IMU_H_
#define IMU_H_

/**
 * IMU update frequency dictates the overall update frequency.
 */
#define IMU_UPDATE_FREQ   500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)


void imuInit(void);

#endif /* IMU_H_ */
