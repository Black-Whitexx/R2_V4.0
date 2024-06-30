/**
  ******************************************************************************
  * @file           : DT35.h
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/16
  ******************************************************************************
  */

#ifndef MY_G474_DT35_H
#define MY_G474_DT35_H

#include "stm32g4xx_hal.h"
#include "retarget.h"

typedef struct
{
    float offset;
    float Right;
    float Back;
    float Forward;
}DT35_Struct;

extern DT35_Struct DT35_Data;

void DT35_Rec(uint8_t *data,DT35_Struct *DT35_Data);

#endif //MY_G474_DT35_H
