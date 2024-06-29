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
        {.x = 46.46f,.y = 46.452f,.angle = 90.0f},
        {.x = 124.3f,.y = 46.452f,.angle = 90.0f},
        {.x = 199.4f,.y = 46.452f,.angle = 90.0f},
        {.x = 274.2f,.y = 46.452f,.angle = 90.0f},
        {.x = 349.60f,.y = 46.452f,.angle = 90.0f}
};

void DT35_Rec(uint8_t *data,DT35_Struct *DT35_data)
{
    DT35_data->Right = (float)(data[1] << 24 | data[2] << 16 | data[3] << 8 | data[4] ) / 10000;
    DT35_data->back = (float)( data[5] << 24 | data[6] << 16 | data[7] << 8 | data[8] ) / 10000;
    DT35_data->Left = (float)( data[9] << 24 | data[10] << 16 | data[11] << 8 | data[12] ) / 10000;
//    printf("%f,%f,%f\n",DT35_data->forward,DT35_data->Left,DT35_data->Right);
}