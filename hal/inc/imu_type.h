#ifndef IMU_TYPE_H_
#define IMU_TYPE_H_

typedef struct {
         int16_t x;
         int16_t y;
         int16_t z;
} Axis3i16;

typedef struct {
         int32_t x;
         int32_t y;
         int32_t z;
} Axis3i32;


typedef struct {
	float x;
	float y;
	float z;
} Axis3f;



#endif /* IMU_TYPE_H_ */
