/**
  ******************************************************************************
  * @file           : Chassis.h
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/4/27
  ******************************************************************************
  */

#ifndef R2_MASTER_V3_CHASSIS_H
#define R2_MASTER_V3_CHASSIS_H
#include "stm32g4xx_hal.h"

#define angle2rad 0.0174532925f
#define l 0.37863f //车轮外接圆半径
#define D 0.153f //车轮直径
#define pi 3.1415926f

typedef struct
{
    float x;
    float y;
    float angle;
    float last_angle;
}PointStruct;

void SGW2Wheels(float vel_x, float vel_y, float omega, float theta);
void Chassis_Move(PointStruct *target_point);
void Chassis_Move_OfVision(PointStruct *target_point);
void Chassis_Move_OfDT35(PointStruct *target_point);
float Distance_Calc(PointStruct point, float x, float y);
void Set_Point(PointStruct *point,float x,float y,float angle);
void SetStore_Points(PointStruct *pointW,PointStruct *pointR);

#endif //R2_MASTER_V3_CHASSIS_H
