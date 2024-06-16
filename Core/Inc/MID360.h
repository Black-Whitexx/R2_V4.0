/**
  ******************************************************************************
  * @file           : MID360.h
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/5/4
  ******************************************************************************
  */

#ifndef R2_MASTER_V3_MID360_H
#define R2_MASTER_V3_MID360_H
#include "stm32g4xx_hal.h"
#include "Vision.h"

typedef struct {
    float locx;
    float locy;
    float locz;
    float yaw;
}RaDar_Data_t;

//#define r 0.28432f
#define r 0.23408f

void RaDar_Data_Rec(uint8_t* data,RaDar_Data_t* RaDar_data,VisionStruct* Vision_data);

extern RaDar_Data_t LiDar;
extern VisionStruct Vision_Data;

#endif //R2_MASTER_V3_MID360_H
