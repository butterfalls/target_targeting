#ifndef _MPU6050_H_
#define _MPU6050_H_


int MPU6050_DMP_Init(void);



int MPU6050_DMP_Get_Data(float *Pitch,float *Roll,float *Yaw);

#endif
