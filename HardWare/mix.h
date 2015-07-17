#ifndef _MIX_H
#define _MIX_H

#include "math.h"

#define _time_constant_z  1.0f//1.0
#define _k1_z  (3/_time_constant_z)
#define _k2_z  (3/_time_constant_z*_time_constant_z)
#define _k3_z  (1/_time_constant_z*_time_constant_z*_time_constant_z)

#define _time_constant_xy  1.0f
#define _k1_xy (3 / _time_constant_xy)
#define _k2_xy (3 / _time_constant_xy*_time_constant_xy)
#define _k3_xy (1 / _time_constant_xy*_time_constant_xy*_time_constant_xy)

#define M 4

typedef  struct
{
    float w;
    float x;
    float y;
    float z;
}quaternion;

typedef  struct
{
    float x;
    float y;
    float z;
}Vector3f;

typedef  struct
{
    Vector3f a;
    Vector3f b;
    Vector3f c;
}Matrix3f;

typedef  struct
{
    float x;
    float y;
    float z;
	  Matrix3f pitch;
}GY;

typedef  struct{
    signed alt;
    signed lat;
    signed lng;
}Location;


#define GRAVITY_MSS 9.80665f

float math_rsqrt(float number);
void quaternion_loadIdentity(quaternion * q);
void quaternion_normalize(quaternion *q);
void mix_gyrAcc_crossMethod(const float gyr[3],const float acc[3]);
void mix_gyrAccMag_crossMethod(quaternion * attitude,const float gyr[3],const float acc[3],const float mag[3],float interval);
Vector3f get_accel_ef(Matrix3f * Mat,Vector3f*T);

void InertialNav_update(float dt);
void  update(float sample, unsigned  int timestamp);
float slope(void);

float angle_pitch(float b[M]);
float angle_roll(float c[M]);
float   angle_yaw(float a[M]);

#endif