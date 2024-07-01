//
// Created by 11294 on 2024/6/1.
//

#include "VL53-100.h"

float TOF_Dis[10];
float TOF_dis1;

void TOF(uint8_t *data,float *dis)
{
    if(data[29] == 32)
    {
        *dis = (float)(data[30]-48)*100.0f+(float)(data[31]-48)*10.0f+(float)(data[32]-48);
    }
    else
    {
        *dis = 1000.0f+(float)(data[30]-48)*100.0f+(float)(data[31]-48)*10.0f+(float)(data[32]-48);
    }
    *dis= (-149.25003f + (float)*dis*1.00857f);
}