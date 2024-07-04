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

    float Tof_dis;
}locater_def;

typedef union
{
    uint8_t data_8[20];
    int32_t data_32[5];
    float data_f[5];
}loc_Receive_Union;



void locatorAndToF_Data_Rec(const uint8_t *data, locater_def *loc,float *TOF_Distance1,float *TOF_Distance2);
#endif //R2_MASTER_V3_LOCATOR_H
