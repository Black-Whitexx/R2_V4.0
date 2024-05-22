/**
  ******************************************************************************
  * @file           : Locator.h
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/5/3
  ******************************************************************************
  */

#ifndef R2_MASTER_V3_LOCATOR_H
#define R2_MASTER_V3_LOCATOR_H
#include "stm32g4xx_hal.h"

typedef struct
{
    float pos_x;
    float pos_y;

    float pos_x_last;
    float pos_y_last;

    float speed_x;
    float speed_y;

    float angle;
    float angular_speed;

    float continuousAngle;
    float lastAngle;
    float offset_angle;
    int circleNum;

    float pos_x_base;
    float pos_y_base;

}locater_def;

typedef union
{
    uint8_t data_8[12];
    int32_t data_32[3];
    float data_f[3];
}loc_Receive_Union;

extern locater_def locater;

void locater_Data_Rec(uint8_t *data, locater_def *loc);
#endif //R2_MASTER_V3_LOCATOR_H
