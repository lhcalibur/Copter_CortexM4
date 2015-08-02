#ifndef IMU_FILTER_H_
#define IMU_FILTER_H_

void filterUpdate_imu(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);

void filterUpdate_mars(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);

void EulerUpdate(float *roll, float *pitch, float *yaw);

#endif /* IMU_FILTER_H_ */
