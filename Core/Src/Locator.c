/**
  ******************************************************************************
  * @file           : Locator.c
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/5/3
  ******************************************************************************
  */
#include "Locator.h"
#include "arm_math.h"
#include "retarget.h"
locater_def locater;
//locater_def locater = {.pos_x_base=0.0f,.pos_y_base=0.0f,.pos_x = 1.f};  // 上电在坐标系中初始位置
/**
 * @brief 对串口3接收到的原始数据进行解包，解包后的数据存在locater里
 * @param data
 * @param loc
 */
void locatorAndToF_Data_Rec(uint8_t *data, locater_def *loc,float *TOF_Distance)
{
    /** 总共18+4位数据 **/
    loc_Receive_Union Union_loc;
    if(data[0] == 0xBB && data[1] == 0xCC)
    {
        loc->pos_x_last = loc->pos_x;
        loc->pos_y_last = loc->pos_y;
        loc->lastAngle = loc->angle;

        for(int i = 0; i < 16; i++)
        {
            Union_loc.data_8[i] = data[i+2];
        }
        *TOF_Distance = Union_loc.data_f[0];
//        loc->pos_x = Union_loc.data_f[1] + loc->pos_x_base - 0.9* arm_cos_f32(loc->angle * 3.1415926 / 180) + 0.9;
//        loc->pos_y = Union_loc.data_f[2] + loc->pos_y_base - 0.9* arm_sin_f32(loc->angle * 3.1415926 / 180);
        loc->pos_x = Union_loc.data_f[1];
        loc->pos_y = Union_loc.data_f[2];

        loc->angle = Union_loc.data_f[3];

        loc->speed_x = 500*(loc->pos_x - loc->pos_x_last);
        loc->speed_y = 500*(loc->pos_y - loc->pos_y_last);
        loc->angular_speed = 500*(loc->angle - loc->lastAngle);

        if((loc->angle - loc->lastAngle) < -180.0f)
        {
            loc->circleNum++;
        }
        else if((loc->angle - loc->lastAngle) > 180.0f)
        {
            loc->circleNum--;
        }
        loc->continuousAngle = (float)loc->circleNum * 360.0f + loc->angle;
    }
}