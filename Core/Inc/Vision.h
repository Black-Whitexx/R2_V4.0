/**
  ******************************************************************************
  * @file           : Vision.h
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/5/24
  ******************************************************************************
  */
#ifndef R2_MASTER_V3_VISION_H
#define R2_MASTER_V3_VISION_H

#include "stm32g4xx_hal.h"

typedef struct
{
    uint8_t flag;
    float vision_x;
    float vision_y;
}VisionStruct;

void Vision_Send(uint8_t cmd);

#endif //R2_MASTER_V3_VISION_H
