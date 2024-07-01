/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdcan.h"
#include "gpio.h"
#include "DT35.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t str_flag;
extern int16_t Wheels_VelOut[0];
PointStruct Target_Point;
extern int16_t Toggle_Pos;
extern int16_t Slope_Pos;
int32_t VESC_Speed = 0;
uint8_t Sheild_Flag = 0;//屏蔽控制权交给chassis任务
extern float DT35_Area2;
float LiDar_x_last,LiDar_y_last = 0;
float Locator_Start[2];
float MutiPos_x;
float MutiPos_y;
float Locator_Addup[2];
uint8_t offset_angle = 0;

uint8_t Camp = BLUE;

//extern locater_def locater;
PointStruct Start_Point = {.x = 2.68f, .y = 9.62f, .angle = 0.0f};//3区调试用�????????????
PointStruct Watch_Point = {.x = 4.03f, .y = 9.62f, .angle = 0.0f};//3区调试用�????????????
//定义环类型，用于�????�取不同数据作为反馈�??????????????????-

/* USER CODE END Variables */
osThreadId Debug_TaskHandle;
osThreadId NRF_TaskHandle;
osThreadId ChassisTaskHandle;
osThreadId ClawTaskHandle;
osThreadId SuctionTaskHandle;
osThreadId VisionComTaskHandle;
osThreadId CloseLoopTaskHandle;
osMessageQId NRF_RX_QueueHandle;
osMessageQId VisionData_QueueHandle;
osMessageQId ControlQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DebugTask(void const * argument);
void NRFTask(void const * argument);
void chassis(void const * argument);
void claw(void const * argument);
void suction(void const * argument);
void visioncom(void const * argument);
void closeloop(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of NRF_RX_Queue */
  osMessageQDef(NRF_RX_Queue, 1, RemoteRXSturct);
  NRF_RX_QueueHandle = osMessageCreate(osMessageQ(NRF_RX_Queue), NULL);

  /* definition and creation of VisionData_Queue */
  osMessageQDef(VisionData_Queue, 1, VisionStruct);
  VisionData_QueueHandle = osMessageCreate(osMessageQ(VisionData_Queue), NULL);

  /* definition and creation of ControlQueue */
  osMessageQDef(ControlQueue, 8, ControlMsgStruct);
  ControlQueueHandle = osMessageCreate(osMessageQ(ControlQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Debug_Task */
  osThreadDef(Debug_Task, DebugTask, osPriorityNormal, 0, 1024);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);

  /* definition and creation of NRF_Task */
  osThreadDef(NRF_Task, NRFTask, osPriorityNormal, 0, 1024);
  NRF_TaskHandle = osThreadCreate(osThread(NRF_Task), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, chassis, osPriorityNormal, 0, 2048);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of ClawTask */
  osThreadDef(ClawTask, claw, osPriorityNormal, 0, 2048);
  ClawTaskHandle = osThreadCreate(osThread(ClawTask), NULL);

  /* definition and creation of SuctionTask */
  osThreadDef(SuctionTask, suction, osPriorityNormal, 0, 2048);
  SuctionTaskHandle = osThreadCreate(osThread(SuctionTask), NULL);

  /* definition and creation of VisionComTask */
  osThreadDef(VisionComTask, visioncom, osPriorityNormal, 0, 2048);
  VisionComTaskHandle = osThreadCreate(osThread(VisionComTask), NULL);

  /* definition and creation of CloseLoopTask */
  osThreadDef(CloseLoopTask, closeloop, osPriorityNormal, 0, 2048);
  CloseLoopTaskHandle = osThreadCreate(osThread(CloseLoopTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_DebugTask */
/**
  * @brief  此任务用于一些参数和外设的初始化以及使用串口和LED调试
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DebugTask */
void DebugTask(void const * argument)
{
  /* USER CODE BEGIN DebugTask */
    /** 等待NRF校准 **/
    //包含部分初始化内�??????????????
//    while (NRF24L01_Check()) {
//        printf("no\n");
//    }
    //NRF24L01_RX_Mode();
    PID_Set(&Wheels[0], 7.2f, 0.1f, 1.0f, 10000);
    PID_Set(&Wheels[1], 7.2f, 0.1f, 1.0f, 10000);
    PID_Set(&Wheels[2], 7.2f, 0.1f, 1.0f, 10000);
    PID_Set(&Wheels[3], 7.2f, 0.1f, 1.0f, 10000);

    PID_Set(&Slope_Speed_t, 4.7f, 0.08f, 0.2f, 10000);
    PID_Set(&Slope_Position_t, 2.2f, 0, 0.0f, 0);
    PID_Set(&Toggle_Speed_t, 7.2f, 0.5f, 2.0f, 10000);
    PID_Set(&Toggle_Position_t, 0.8f, 0, 0.8f, 0);

    PID_Set(&Translation_PID, 1.80f, 0.0f, 0.3f, 0.0f);
    PID_Set(&Turn_PID, 0.035f, 0.0f, 0.2f, 0.0f);

    PID_Set(&VisionRun2, 1.8f, 0.0000f, 0.5f, 0.0f);
    PID_Set(&DT35_Run, 0.02f, 0.0f, 0.0f, 0.0f);

    PID_Set(&VisionPID_X, 0.0020f, 0.0f, 0.001f, 0.0f);
    PID_Set(&Chassis_GetBall_PID, 1.05f, 0.0f, 0.3f, 0.0f);
    /* Infinite loop */
    for (;;) {
        //printf("%.2f %.2f %.2f\n", DT35_Data.DT35_2, DT35_Data.DT35_3,DT35_Data.DT35_1);
//        printf("%.2f %.2f \n", locater.pos_x, locater.pos_y);
//        printf("%.3f %.3f\n", LiDar.locx, LiDar.locy);
        //printf("%f %f\n",MutiPos_x,MutiPos_y);
//        printf("%s\n", USART5_Buffer);
        //printf("%f\n", LiDar.yaw);
        //printf("%.2f,%.2f,%.2f,%.4f\n",locater.pos_x,locater.pos_y,locater.angle,locater.Tof_dis);
        //printf("%x",USART5_Buffer[1]);
        //
         //printf("%f %f %f\n",TOF_dis1,MutiPos_x,MutiPos_y);
        osDelay(100);
    }
  /* USER CODE END DebugTask */
}

/* USER CODE BEGIN Header_NRFTask */
/**
* @brief 此函数用于接收遥控器的数�?????????????????????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRFTask */
void NRFTask(void const * argument)
{
  /* USER CODE BEGIN NRFTask */
    /** 定义存储数据变量 **/
//    static uint8_t rc_data[RX_PLOAD_WIDTH] = {0};//接收数组缓冲
//    static RemoteRXSturct RemoteRX;//用于存遥控器传来的数
//    uint8_t cmd;
//    int16_t suctionSpeed = 0;
    if(Camp == RED){
        offset_angle = 45;
    }
    else if(Camp == BLUE){
        offset_angle = -45;
    }
    uint8_t QueueBuffer;
    osDelay(5000);
    QueueBuffer = 0;
    xQueueSend(VisionData_QueueHandle, &QueueBuffer, 100);
    /* Infinite loop */
    for (;;) {
//        if (NRF24L01_RxPacket(rc_data) == 0)  //接收遥控器数据，若收到返0，若没收到返1
//        {
//            /** 读取左右摇杆值，限制�?????????????????????-128~128 **/
//            RemoteRX.lx = (int16_t) -(rc_data[1] - 128);
//            RemoteRX.ly = (int16_t) -(rc_data[2] - 128);
//            RemoteRX.rx = (int16_t) -(rc_data[3] - 128);
//            RemoteRX.ry = (int16_t) -(rc_data[4] - 128);
//            /** 接收遥控器按键命 **/
//            RemoteRX.command = rc_data[5];
//            /** 设置摇杆值的死区 **/
//            if (abs(RemoteRX.rx) < 2) RemoteRX.rx = 0;
//            if (abs(RemoteRX.ry) < 2) RemoteRX.ry = 0;
//            if (abs(RemoteRX.lx) < 2) RemoteRX.lx = 0;
//            if (abs(RemoteRX.ly) < 2) RemoteRX.ly = 0;
//            /** 对遥控器进行滤波，原因是遥控器有莫名其妙的电平跳 **/
//            if (RemoteRX.rx == -5) RemoteRX.rx = 0;
//            if (RemoteRX.ry == -4) RemoteRX.ry = 0;
//            if (RemoteRX.lx == -4) RemoteRX.lx = 0;
//            if (RemoteRX.ly == -4) RemoteRX.ly = 0;
//
//            /**将NRF数据更新到四轮全�??????????????????变量�??????????????????**/
//            SGW2Wheels((float) RemoteRX.rx * 3 / 128, (float) RemoteRX.ry * 3 / 128, (float) RemoteRX.lx * 3 / 128, 0);
//            printf("%d",RemoteRX.command);
//            /** 对遥控器按键命令进行响应 **/
//            switch (RemoteRX.command) {
//                case Left_Up_Up:
//                    break;
//                case Left_Up:
//                    printf("start");
//                    QueueBuffer = 0;
//                    xQueueSend(VisionData_QueueHandle, &QueueBuffer, 100);
//                    break;
//                case Right_Up:
//
//                    break;
//                case Right_Down:
//
//                    break;
//                case Right_Right:
//
//                    break;
//                case Left_Left:
//
//                    break;
//                case Left_Right:
//
//                    break;
//                case Right_Left:
//
//                    break;
//                case Right_Up_Up:
//                    break;
//                default:
//                    break;
//            }
//        printf("%d,%d\n",RemoteRX.command,RemoteRX.ly);/** 用于调试遥控器按 **/
//            RemoteRX.command = 0; //重置命令
//
//        }
        //printf("detecting\n");
        LED0_Flashing;
        osDelay(500);
    }
  /* USER CODE END NRFTask */
}

/* USER CODE BEGIN Header_chassis */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis */
void chassis(void const * argument)
{
  /* USER CODE BEGIN chassis */
    ControlMsgStruct ControlQueueBuf;
    uint8_t Stop_Flag = 1;
    uint8_t CloseLoopStatus = 0;
    PointStruct target_point = {0, 0, 0};
    float offset_x = 0;
    uint8_t QueueBuffer = 0;
    uint8_t number = 0;
    uint8_t AlignStatus = 0;
    VisionStruct visiondatabuf;
    uint8_t StartPointNumber = 0;
    /* Infinite loop */
    for (;;) {
        if (xQueuePeek(ControlQueueHandle, &ControlQueueBuf, 0) == pdTRUE) {
            if (ControlQueueBuf.Device == CHASSIS) {
                xQueueReceive(ControlQueueHandle, &ControlQueueBuf, 0);
                switch (ControlQueueBuf.Command) {
                    case CHASSIS_RUN:
                        Stop_Flag = 0;
                        break;
                    case CHASSIS_STOP:
                        Stop_Flag = 1;
                        break;
                    case CloseLoop_START:
                        //printf("STart");
                        CloseLoopStatus = CloseLoop_START;
                        target_point.x = ControlQueueBuf.data[0];
                        target_point.y = ControlQueueBuf.data[1];
                        target_point.angle = ControlQueueBuf.data[2];
                        break;
                    case CloseLoop_MID360:
                        CloseLoopStatus = CloseLoop_MID360;
                        target_point.x = ControlQueueBuf.data[0];
                        target_point.y = ControlQueueBuf.data[1];
                        target_point.angle = ControlQueueBuf.data[2];
                        AlignStatus = (int) ControlQueueBuf.data[3];
                        //Sheild_Flag = 1;
                        break;
                    case CloseLoop_Mid360AndDT35:
                        CloseLoopStatus = CloseLoop_Mid360AndDT35;
                        target_point.x = ControlQueueBuf.data[0];
                        target_point.y = ControlQueueBuf.data[1];
                        target_point.angle = ControlQueueBuf.data[2];
                        number = (int) ControlQueueBuf.data[3];
                        break;
                    case CloseLoop_DT35:
                        printf("CLDT35_GET");
                        CloseLoopStatus = CloseLoop_DT35;
                        target_point.x = ControlQueueBuf.data[0];
                        target_point.y = ControlQueueBuf.data[1];
                        target_point.angle = ControlQueueBuf.data[2];
                        break;
                    case CloseLoop_Ball:
                        CloseLoopStatus = CloseLoop_Ball;
                        offset_x = ControlQueueBuf.data[0];
                        break;
//                  case CloseLoop_Left:
//                      CloseLoopStatus = CloseLoop_Left;
//                      offset_x = ControlQueueBuf.data[0];
//                      break;
//                  case CloseLoop_Right:
//                      CloseLoopStatus = CloseLoop_Right;
//                      offset_x = ControlQueueBuf.data[0];
//                      break;
                    case GoForwardSlowly:
                        CloseLoopStatus = GoForwardSlowly;
                        break;
                    case CHASSIS_TURN:
                        CloseLoopStatus = CHASSIS_TURN;
                        break;
                    default:
                        break;
                }
            }
        }
        /** 判断程序**/
        /** 这一部分是奥里给 **/
        if (Stop_Flag == 1) {
            Car_Stop;
            //printf("stop");
        }
        else {
            if (!(target_point.x == 0 && target_point.y == 0)) {
                //printf("in\n");
                //printf("%d",CloseLoopStatus);
                if(CloseLoopStatus == CloseLoop_START){
                    //printf("startrunning");
                    /** 纵向 **/
                    if(StartPointNumber == 0) {
                        //printf("detecting1\n");
                        Chassis_Move_OfVision(&target_point,&VisionRun2,2.5f);
                        if (fabsf(target_point.y-MutiPos_y) < 0.05f) {
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_START, Run1to3_Points[1].x,
                                          Run1to3_Points[1].y, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgInit(&ControlQueueBuf);
                            StartPointNumber = 1;
                            printf("1ok");
                        }
                    }
                    /** 横向 **/
                    else if(StartPointNumber == 1) {
                        //printf("%f %f",target_point.x,target_point.y);
                        Chassis_Move_OfVision(&target_point,&VisionRun2,2.5f);
                            if (fabsf(target_point.x-MutiPos_x) < 0.05f ) {
                                ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_START, Run1to3_Points[2].x,
                                              Run1to3_Points[2].y, 0, 0);
                                xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                                ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                                xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                                ControlMsgInit(&ControlQueueBuf);
                                StartPointNumber = 2;
                                printf("2ok");
                            }
                    }
                    /**纵向 **/
                    else if(StartPointNumber == 2) {
                        Chassis_Move_OfVision(&target_point,&VisionRun2,2.5f);
                        //printf("%f %f",target_point.x,target_point.y);
                        if ((target_point.y -MutiPos_y) < 0.1f &&
                            fabsf(LiDar.yaw - target_point.angle) < 3) {
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_MID360, Start_Point.x,
                                          Start_Point.y, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_CLOSE, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Mid, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, SUCTION, SUCTION_ON, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, SUCTION, StopVESC, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_OFF, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgInit(&ControlQueueBuf);
                            StartPointNumber = 3;
                            printf("3ok");
                        }
                    }
                }
                if (CloseLoopStatus == CloseLoop_MID360) {
                    Chassis_Move_OfVision(&target_point,&Chassis_GetBall_PID,1.f);
                    //printf("%f %f\n",MutiPos_x,MutiPos_y);
                    if (Distance_Calc(target_point,MutiPos_x,MutiPos_y) < 0.3f &&
                        fabsf(LiDar.yaw - target_point.angle) < 3) {
                        printf("send1normal\n");
                        //printf("%f",LiDar.yaw );
                        //Sheild_Flag = 0;
                        Vision_Send(0xCC);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_STOP, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgInit(&ControlQueueBuf);
                    }
                }
                else if (CloseLoopStatus == CloseLoop_Mid360AndDT35) {
                    Chassis_Move_OfVision(&target_point,&VisionRun2,2.5f);
                    //printf("MAD");
                    if (fabsf(target_point.x - LiDar.locx) < 1.f &&
                        fabsf(LiDar.yaw - target_point.angle) < 3.f) {
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_DT35, DT35_AimPoints[number - 1].x,
                                      DT35_AimPoints[number - 1].y, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgInit(&ControlQueueBuf);
                        printf("MID2DT,ok\n");
                    }
                }
                else if (CloseLoopStatus == CloseLoop_DT35) {
                    Chassis_Move_OfDT35(&target_point);
                    //printf("InDT\n");
//                    printf("%f %f",DT35_Data.back,DT35_Data.Right);
                    //Chassis_Move_OfDT35(&target_point);
                    printf("%f\n",Distance_Calc(target_point, DT35_Data.DT35_1, DT35_Data.DT35_2));
                    if (Distance_Calc(target_point, DT35_Data.DT35_1, DT35_Data.DT35_2) < 5.0f) {
                        ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_OPEN, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_STOP, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        osDelay(500);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_MID360, Watch_Point.x, Watch_Point.y,
                                      Watch_Point.angle, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                        ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_CLOSE, 0, 0, 0, 0);
//                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                        ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Mid, 0, 0, 0, 0);
//                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, SUCTION, SUCTION_OFF, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, SUCTION, StopVESC, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_OFF, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        Vision_Send(0xCC);
                        printf("DT35send1\n");
                    }
                }
                    /** 对球闭环 **/
                else if (CloseLoopStatus == CloseLoop_Ball) {
                    if(AlignStatus == 1) {
                        float Speed_xy = -PID_Realise(&VisionPID_X, 0, offset_x, 1.0f, 3);
                        float omega = PID_Realise(&Turn_PID, 0, LiDar.yaw, 0.5f, 0.5f);
                        float Speed_y = Speed_xy;
                        SGW2Wheels(-0.1f, Speed_y, omega, 0);
                        if (fabsf(offset_x) < 30) {
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, GoForwardSlowly, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgInit(&ControlQueueBuf);
                        }
                    }
                    else if (AlignStatus == 5){
                        float const_speed = 0.2f;
                        float Speed_xy = -PID_Realise(&VisionPID_X, 0, offset_x, 1.0f, 3);
                        float omega = PID_Realise(&Turn_PID, (float)offset_angle, LiDar.yaw, 0.5f, 0.5f);
                        float Speed_x = Speed_xy;
                        float Speed_y = -Speed_xy;
                        SGW2Wheels(Speed_x, Speed_y, omega, 0);
                        if (fabsf(offset_x) < 30) {
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, GoForwardSlowly, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgInit(&ControlQueueBuf);
                        }
                    }
                    else if (AlignStatus == 6){
                        float const_speed = 0.1f;
                        float Speed_xy = -PID_Realise(&VisionPID_X, 0, offset_x, 1.0f, 3);
                        float omega = PID_Realise(&Turn_PID, (float)-offset_angle, LiDar.yaw, 0.5f, 0.5f);
                        float Speed_y = Speed_xy + const_speed;
                        float Speed_x = Speed_xy - const_speed;
                        SGW2Wheels(Speed_x, Speed_y, omega, 0);
                        if (fabsf(offset_x) < 30) {
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, GoForwardSlowly, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgInit(&ControlQueueBuf);
                        }
                    }
                } else if (CloseLoopStatus == GoForwardSlowly) {
                    SGW2Wheels(-1.41f, 0, 0, 0);
                }
            }
        }
        osDelay(1);
    }
  /* USER CODE END chassis */
}

/* USER CODE BEGIN Header_claw */
/**
* @brief Function implementing the ClawTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_claw */
void claw(void const * argument)
{
  /* USER CODE BEGIN claw */
    ControlMsgStruct ControlQueueBuf;
    /* Infinite loop */
    for (;;) {
        if (xQueuePeek(ControlQueueHandle, &ControlQueueBuf, 0) == pdTRUE) {
            if (ControlQueueBuf.Device == CLAW) {
                xQueueReceive(ControlQueueHandle, &ControlQueueBuf, 0);
                switch (ControlQueueBuf.Command) {
                    case CLAW_OPEN:
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
                        //printf("open");
                        break;
                    case CLAW_CLOSE:
                        printf("close");
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
                        break;
                    case Toggle_Up:
                        Toggle_Pos = 3100;
                        break;
                    case Toggle_Mid:
                        Toggle_Pos = 1500;
                        break;
                    case Toggle_Down:
                        Toggle_Pos = 200;
                        break;
                    default:
                        break;
                }
            }
        }
        osDelay(1);
    }
  /* USER CODE END claw */
}

/* USER CODE BEGIN Header_suction */
/**
* @brief Function implementing the SuctionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_suction */
void suction(void const * argument)
{
  /* USER CODE BEGIN suction */
    ControlMsgStruct ControlQueueBuf;
    /* Infinite loop */
    for (;;) {
        if (xQueuePeek(ControlQueueHandle, &ControlQueueBuf, 0) == pdTRUE) {
            if (ControlQueueBuf.Device == SUCTION) {
                xQueueReceive(ControlQueueHandle, &ControlQueueBuf, 0);
                switch (ControlQueueBuf.Command) {
                    case SUCTION_ON:
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
                        break;
                    case SUCTION_OFF:
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
                        break;
                    case Slope_ON:
                        Slope_Pos = 1500;
                        break;
                    case Slope_OFF:
                        Slope_Pos = 0;
                        break;
                    case Slope_OUT:
                        Slope_Pos = 3000;
                        break;
                    case OpenVESC:
                        VESC_Speed = -7000;
                        break;
                    case StopVESC:
                        VESC_Speed = 0;
                        break;
                    default:
                        break;
                }
            }
        }
        osDelay(1);
    }
  /* USER CODE END suction */
}

/* USER CODE BEGIN Header_visioncom */
/**
* @brief Function implementing the VisionComTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_visioncom */
void visioncom(void const * argument)
{
  /* USER CODE BEGIN visioncom */
    VisionStruct visiondata;
    uint8_t status;//用于小状态切�????????????
    float v_x, v_y;
    ControlMsgStruct ControlQueueBuf;
    /* Infinite loop */
    for (;;) {
        if (xQueueReceive(VisionData_QueueHandle, &visiondata, 0) == pdTRUE) {
            if (Sheild_Flag == 0) {
                if (visiondata.flag == 0) {
                    //并不是从1�????????????3，在3区调试用
                    printf("0\n");
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_START, Run1to3_Points[0].x, Run1to3_Points[0].y,
                                  0, 0);
//                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_MID360, Start_Point.x, Start_Point.y,
//                                 90, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_CLOSE, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Down, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, SUCTION_OFF, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, StopVESC, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_OFF, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgInit(&ControlQueueBuf);
                }
                    /** 跑球的点，中间区 **/
                else if (visiondata.flag == 1) {
                    printf("1\n");
                    ControlMsgSet(&ControlQueueBuf, SUCTION, SUCTION_ON, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_OPEN, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Mid, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    printf("%f %f",-visiondata.vision_y / 1000.0f + MutiPos_x,visiondata.vision_x / 1000.0f + MutiPos_y);
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_MID360,
                                  -visiondata.vision_y / 1000.0f + MutiPos_x,
                                  visiondata.vision_x / 1000.0f + MutiPos_y, 0, 1);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, OpenVESC, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_ON, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgInit(&ControlQueueBuf);
                }
                    /** 跑球的点，左边区 **/
                else if (visiondata.flag == 5) {
                    printf("5\n");
                    ControlMsgSet(&ControlQueueBuf, SUCTION, SUCTION_ON, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_OPEN, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Down, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_MID360,
                                  -visiondata.vision_y / 1000.0f + MutiPos_x,
                                  visiondata.vision_x / 1000.0f + MutiPos_y, (float)offset_angle, 5);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, OpenVESC, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_ON, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgInit(&ControlQueueBuf);
                } else if (visiondata.flag == 6) {
                    printf("6\n");
                    ControlMsgSet(&ControlQueueBuf, SUCTION, SUCTION_ON, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_OPEN, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Down, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_MID360,
                                  -visiondata.vision_y / 1000.0f +MutiPos_x,
                                  visiondata.vision_x / 1000.0f + MutiPos_y, (float)-offset_angle, 6);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, OpenVESC, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_ON, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgInit(&ControlQueueBuf);
                } else if (visiondata.flag == 2) {
                    if (visiondata.vision_y == 0) {
                        /** 对球 **/
//                        printf("align");
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_Ball, visiondata.vision_x, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgInit(&ControlQueueBuf);
//                        if(TOF_dis1 < 180.f){
//                            ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_STOP, 0, 0, 0, 0);
//                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                            ControlMsgInit(&ControlQueueBuf);
//                        }
                    } else if (visiondata.vision_y == 1) {
                        if(TOF_dis1 < 180.f) {
                            /** 正确的球即将进入车内 **/
                            printf("rightball\n");
//                  ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_MID360,Start_Point.x,Start_Point.y,Start_Point.angle,0);
//                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                  ControlMsgSet(&ControlQueueBuf,CHASSIS,CHASSIS_RUN,0,0,0,0);
//                      xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_STOP, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_OPEN, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_OFF, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Down, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            osDelay(50);
                            ControlMsgSet(&ControlQueueBuf, SUCTION, SUCTION_OFF, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgSet(&ControlQueueBuf, SUCTION, StopVESC, 0, 0, 0, 0);
                            xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                            ControlMsgInit(&ControlQueueBuf);
                        }
                    }
                } else if (visiondata.flag == 3) {
                    if (visiondata.vision_y == 2) {
                        /** 停车把球排出�???????????? **/
                        printf("3,2\n");
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_STOP, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, SUCTION, StopVESC, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Mid, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_ON, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        osDelay(100);
                        ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_OUT, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        osDelay(100);
                        ControlMsgSet(&ControlQueueBuf, SUCTION, Slope_ON, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgInit(&ControlQueueBuf);
                    } else if (visiondata.vision_y == 6) {
                        /** 夹球 **/
                        printf("3,6\n");
//                      ControlMsgSet(&ControlQueueBuf,CHASSIS,CHASSIS_STOP,0,0,0,0);
//                      xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                      ControlMsgSet(&ControlQueueBuf,SUCTION,Slope_OFF,0,0,0,0);
//                      xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                      ControlMsgSet(&ControlQueueBuf,CLAW,Toggle_Down,0,0,0,0);
//                      xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                      ControlMsgSet(&ControlQueueBuf,CLAW,CLAW_OPEN,0,0,0,0);
//                      xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                      osDelay(1000);
                        osDelay(200);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_MID360, Watch_Point.x, Watch_Point.y,
                                      Watch_Point.angle, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, SUCTION, StopVESC, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, CLAW, CLAW_CLOSE, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        osDelay(200);
                        ControlMsgSet(&ControlQueueBuf, CLAW, Toggle_Up, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
//                      ControlMsgSet(&ControlQueueBuf,SUCTION,SUCTION_OFF,0,0,0,0);
//                      xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgInit(&ControlQueueBuf);
                    } else if (visiondata.vision_y == 3) {
                        /** 后�??再次找球 **/
//                        ControlMsgSet(&ControlQueueBuf, CHASSIS, GoForwardSlowly, 0, 0, 0, 0);
//                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        printf("3,3\n");
                        osDelay(1000);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_MID360, Watch_Point.x, Watch_Point.y,
                                      Watch_Point.angle, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgInit(&ControlQueueBuf);
                    }
                    else if (visiondata.vision_y == 7) {
                        /** 继续吸球 **/
                        ControlMsgSet(&ControlQueueBuf, SUCTION, OpenVESC, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                        xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                        ControlMsgInit(&ControlQueueBuf);;
                    }
                }
                else if (visiondata.flag == 4) {
                    printf("put");
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CloseLoop_Mid360AndDT35,
                                  Frame_Points[(int) (visiondata.vision_x - 1)].x,
                                  Frame_Points[(int) (visiondata.vision_x - 1)].y, 0, visiondata.vision_x);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgSet(&ControlQueueBuf, CHASSIS, CHASSIS_RUN, 0, 0, 0, 0);
                    xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                    ControlMsgInit(&ControlQueueBuf);
                }
            } else {
                //printf("disabled");
            }

            osDelay(1);
        }
    }
  /* USER CODE END visioncom */
}

/* USER CODE BEGIN Header_closeloop */
/**
* @brief Function implementing the CloseLoopTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_closeloop */
void closeloop(void const * argument)
{
  /* USER CODE BEGIN closeloop */
    /* Infinite loop */
    for (;;) {
        /** 沟槽的大疆，不能分开单独控制�?????????个ID下的4个电�????????? **/
        /** 四轮闭环**/
        //SGW2Wheels(0.f,0.5f,0,0);
        Wheels_VelOut[0] = (int16_t) PID_Realise(&Wheels[0], -Wheels_vel[0], Motor_Info[0].speed, M3508_CURRENT_MAX, 5);
        Wheels_VelOut[1] = (int16_t) PID_Realise(&Wheels[1], -Wheels_vel[1], Motor_Info[1].speed, M3508_CURRENT_MAX, 5);
        Wheels_VelOut[2] = (int16_t) PID_Realise(&Wheels[2], -Wheels_vel[2], Motor_Info[2].speed, M3508_CURRENT_MAX, 5);
        Wheels_VelOut[3] = (int16_t) PID_Realise(&Wheels[3], -Wheels_vel[3], Motor_Info[3].speed, M3508_CURRENT_MAX, 5);
        Set_Current(&hfdcan1, 0x200, Wheels_VelOut[0], Wheels_VelOut[1], Wheels_VelOut[2], Wheels_VelOut[3]);
        /** 夹爪和分球板闭环**/
        float Slope_Position = PID_Realise(&Slope_Position_t, Slope_Pos, Motor_Info[6].actual_total_angle, 2000, 10.0f);
        int16_t Slope_Speed = (int16_t) PID_Realise(&Slope_Speed_t, Slope_Position, Motor_Info[6].speed,
                                                    M2006_CURRENT_MAX, 5);
        float Toggle_Position = PID_Realise(&Toggle_Position_t, Toggle_Pos, Motor_Info[7].actual_total_angle, 1000,
                                            5.0f);
        int16_t Toggle_Speed = (int16_t) PID_Realise(&Toggle_Speed_t, Toggle_Position, Motor_Info[7].speed,
                                                     M3508_CURRENT_MAX, 5);
        Set_Current(&hfdcan2, 0x1FF, 0, 0, Slope_Speed, Toggle_Speed);
        Vesc_SetSpeed(&hfdcan1, VESC_ID, VESC_Speed);
        /** 插�?? **/
        float LiDar_x = LiDar.locx;
        float LiDar_y = LiDar.locy;
        float Locator_x = 0.707f * (locater.pos_x/100.f - locater.pos_y/100.f);
        float Locator_y = 0.707f * (locater.pos_x/100.f + locater.pos_y/100.f);
//        float Locator_x = locater.pos_x/100.f;
//        float Locator_y = locater.pos_y/100.f;
        //printf("1:%.2f,%.2f\n",Locator_Start[0],Locator_Start[1]);
        if(!((LiDar_x == LiDar_x_last) && (LiDar_y == LiDar_y_last))){
            /** 该更新了 **/
            /** 码盘起始重置 **/
            Locator_Addup[0] = 0;
            Locator_Addup[1] = 0;
            Locator_Start[0] = Locator_x;
            Locator_Start[1] = Locator_y;
            //printf("2:%.2f,%.2f\n",Locator_Start[0],Locator_Start[1]);
        }
        /** 码盘插�?? **/
        Locator_Addup[0] = Locator_x - Locator_Start[0];
        Locator_Addup[1] = Locator_y - Locator_Start[1];
        MutiPos_x = LiDar_x + Locator_Addup[0];
        if(Camp == RED) {
            MutiPos_y = LiDar_y + Locator_Addup[1];
        }
        else if(Camp == BLUE){
            MutiPos_y = -(LiDar_y + Locator_Addup[1]);
        }
//        MutiPos_x = Locator_Addup[0];
//        MutiPos_y = Locator_Addup[1];
//        MutiPos_x = locater.pos_x;
//        MutiPos_y = locater.pos_y;
        //printf("%f %f\n",locater.pos_x,MutiPos_x);
        LiDar_x_last = LiDar_x;
        LiDar_y_last = LiDar_y;
        osDelay(1);
    }
  /* USER CODE END closeloop */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

