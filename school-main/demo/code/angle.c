#include "zf_common_headfile.h"
#include "main.h"
#include <math.h>
#include <float.h>
#include "angle.h"

//车长轴距需要自行测量修改
static const float CAR_LENGTH = 180.0f;        // 车长
static const float WHEELBASE = 150.0f;         // 轴距

/*******************************************************************************
* 函 数 名         : servo_angle2wheel_angle
* 函数功能         : 将舵机的偏转角度与实际轮子角度对应上	
* 输    入         : servo_angle
* 输    出         : wheel_angle
*******************************************************************************/
//有建模结果说明，舵机偏转角度与实际轮子偏转角度呈线性关系，这个具体的值通过实际调试确定
float k=1.1
float servo_angle2wheel_angle(float servo_angle)
{
    return servo_angle*k;
}

/*******************************************************************************
* 函 数 名         : servo_angle2wheel_angle
* 函数功能         : 根据轮子转向角度计算出左右轮关系	
* 输    入         : wheel_angle，wheel_angle>0表示车子向右转
* 输    出         : speed_rate，设定右轮速度为设定值不变，左轮根据角度值得到的与右轮速度的比例系数
*******************************************************************************/
float speed_rate(float wheel_angle)
{
    float rate;
    rate = CAR_LENGTH/(CAR_LENGTH-(WHEELBASE*tan(wheel_angle)));
    return rate;
}