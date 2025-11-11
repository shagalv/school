#ifndef _ANGLE_H
#define _ANGLE_H

extern float servo_angle;//舵机角度设置，全局变量
extern float wheel_angle;//对应的车轮角度

float servo_angle2wheel_angle(float servo_angle);
float speed_rate(float wheel_angle);

#endif