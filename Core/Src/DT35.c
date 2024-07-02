/**
  ******************************************************************************
  * @file           : DT35.c
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/16
  ******************************************************************************
  */
#include "DT35.h"
#include "Chassis.h"
#include "PID.h"

DT35_Struct DT35_Data;
PID_t DT35_Run;
PointStruct DT32_Points;
/** 用于存储比赛5个放球点 **/
PointStruct DT35_AimPoints[5]= {
        //TODO:修改表达
        {.y = 46.46f,.x = 34.f,.angle = 0.0f},
        {.y = 124.3f,.x = 34.f,.angle = 0.0f},
        {.y = 199.7f,.x = 34.f,.angle = 0.0f},
        {.y = 275.2f,.x = 34.f,.angle = 0.0f},
        {.y = 349.60f,.x = 34.f,.angle = 0.0f}
};
float DT35_Area2 = 161.f;
void DT35_Rec(uint8_t *data,DT35_Struct *DT35_data)
{
    DT35_data->DT35_2 = (float)(data[1] << 24 | data[2] << 16 | data[3] << 8 | data[4] ) / 10000;
    DT35_data->DT35_3 = (float)(data[5] << 24 | data[6] << 16 | data[7] << 8 | data[8] ) / 10000;
    DT35_data->DT35_1 = (float)(data[9] << 24 | data[10] << 16 | data[11] << 8 | data[12] ) / 10000;
//    printf("%f,%f,%f\n",DT35_data->forward,DT35_data->Left,DT35_data->Right);
}