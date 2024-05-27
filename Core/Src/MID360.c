/**
  ******************************************************************************
  * @file           : MID360.c
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/5/4
  ******************************************************************************
  */
#include "MID360.h"
#include "retarget.h"
#include "arm_math.h"

#define pi 3.1415926f
RaDar_Data_t LiDar;

void RaDar_Data_Rec(uint8_t* data,RaDar_Data_t* RaDar_data,VisionStruct* Vision_data)
{
    if(data[0] == 0xAE){
        RaDar_data->locx = (float)(data[1] | data[2] << 8 | data[3] << 16 | data[4] << 24) / 1000.0f;
        RaDar_data->locy = (float)(data[5] | data[6] << 8 | data[7] << 16 | data[8] << 24) / 1000.0f;
        RaDar_data->yaw = (float)(data[9] | data[10] << 8 | data[11] << 16 | data[12] << 24) * 180.0f / pi / 1000.0f;

        RaDar_data->locy = RaDar_data->locy + r * arm_cos_f32(RaDar_data->yaw * 3.1415926f / 180.0f) - r;
        RaDar_data->locx = RaDar_data->locx - r * arm_sin_f32(RaDar_data->yaw * 3.1415926f / 180.0f);

        Vision_data->flag = data[13];
        Vision_data->vision_x = (float)( data[17] | data[18] << 8 | data[19] << 16 | data[20] << 24 );
        Vision_data->vision_y = (float)( data[21] | data[22] << 8 | data[23] << 16 | data[24] << 24 );

        Vision_Rec(Vision_data);
//        printf("%d,%d,%d,%d\n",data[14],data[15],data[16],data[17]);
//        printf("%f,%f,%f,%f\n",RaDar_data->locx,RaDar_data->locy,RaDar_data->locz,RaDar_data->yaw);
    }
}