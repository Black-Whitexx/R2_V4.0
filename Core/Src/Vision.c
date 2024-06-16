/**
  ******************************************************************************
  * @file           : Vision.c
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/5/24
  ******************************************************************************
  */
#include "Vision.h"
#include "PID.h"
#include "Chassis.h"
#include "MID360.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "retarget.h"
#include "main.h"

VisionStruct Vision_Data;
PID_t VisionPID_X,VisionRun2;
PointStruct Vision_Points[256];
uint8_t Vision_State,VisionFlag,Color;

//void Vision_Rec(VisionStruct* visionData)
//{
//    int16_t suctionSpeed = 0;
//    if(visionData->flag == 1)//在黄区找到球了
//    {
////        printf("1,1\n");
//        cnt = 0;
//        Set_Point(&Vision_Points[0],-visionData->vision_y / 1000.0f + LiDar.locx,visionData->vision_x / 1000.0f + LiDar.locy,90,0);
//    }
//    else if(visionData->flag == 2)
//    {
//        switch ((int16_t)(visionData->vision_y)) {
//            case 1:/** 看到蓝色球了，平台回正，把夹爪放下去 **/
////                printf("2,1\n");
//                Vision_State = Vision_Right;
//                break;
//            case 2:/** 进来的是无效球，停车等待球吐出去 **/
////                printf("2,2\n");
//                Toggle_Pos = Toggle_Mid;
//                Vision_State = Vision_Delay;
//                break;
//            case 0:
//                if( Vision_State != Vision_Right && Vision_State != Vision_FindBall && Vision_State != Vision_Delay && Vision_State != Vision_Delay2 )
//                {
//                    xTaskResumeFromISR(Vision_TaskHandle);
//                }
//                break;
//            default:
//                break;
//        }
//    }
//    else if(visionData->flag == 3)
//    {
//        switch ((int16_t)(visionData->vision_y)) {
//            case 6:/** 进来的是有效球，进入TakeRightBall_State **/
////                printf("3,6\n");
//                str_flag = 0;
//                Vision_State = Vision_GetRightBall;
//                State = TakeRightBall_State;
//                break;
//            case 7:/** 车内没球 **/
//                if(Vision_State == Vision_Delay2)
//                {
//                    Vision_flag = 1;
//                }
//                break;
//            case 2:/** 紫球还在里面 **/
////                printf("3,2\n");
//                suctionSpeed = 0;//停下5065
//                xQueueOverwriteFromISR(SuctionSpeed_QueueHandle,&suctionSpeed,0);
//                Toggle_Pos = Toggle_Mid;
//                Vision_State = Vision_Delay2;
//                break;
//            case 3:/** 视野里没有球，向后退 **/
////                printf("3,3\n");
//                Set_Point(&Vision_Points[0],-1.43f,1.49f,90,0);
//                Vision_State = Vision_FindBall;
//                break;
//            default:
//                break;
//        }
//    }
////    else if(visionData->flag == 4)//在绿区做好框的决策
////    {
////        Store_Flag = 1;
//////        switch ((uint8_t)visionData->vision_x) {
//////            case 1:
//////                Set_Point(&Aim_Points[AimPoints_Index],0,0,90,0);
//////                break;
//////            case 2:
//////                Set_Point(&Aim_Points[AimPoints_Index],0,0,90,0);
//////                break;
//////            case 3:
//////                Set_Point(&Aim_Points[AimPoints_Index],0,0,90,0);
//////                break;
//////            case 4:
//////                Set_Point(&Aim_Points[AimPoints_Index],0,0,90,0);
//////                break;
//////            case 5:
//////                Set_Point(&Aim_Points[AimPoints_Index],0,0,90,0);
//////                break;
//////            default:
//////                break;
//////        }
////    }
//}
void Vision_Send(uint8_t cmd)
{
    HAL_UART_Transmit(&huart2,&cmd, sizeof(cmd),0xFFFFF);
}



