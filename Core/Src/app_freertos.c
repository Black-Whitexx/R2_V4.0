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
//定义闭环类型，用于�?�取不同数据作为反馈�??????

/* USER CODE END Variables */
osThreadId Debug_TaskHandle;
osThreadId NRF_TaskHandle;
osThreadId ChassisTaskHandle;
osThreadId ClawTaskHandle;
osThreadId SuctionTaskHandle;
osThreadId VisionComTaskHandle;
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
  osMessageQDef(ControlQueue, 16, ControlMsgStruct);
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
  osThreadDef(ChassisTask, chassis, osPriorityNormal, 0, 1024);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of ClawTask */
  osThreadDef(ClawTask, claw, osPriorityNormal, 0, 1024);
  ClawTaskHandle = osThreadCreate(osThread(ClawTask), NULL);

  /* definition and creation of SuctionTask */
  osThreadDef(SuctionTask, suction, osPriorityBelowNormal, 0, 1024);
  SuctionTaskHandle = osThreadCreate(osThread(SuctionTask), NULL);

  /* definition and creation of VisionComTask */
  osThreadDef(VisionComTask, visioncom, osPriorityIdle, 0, 1024);
  VisionComTaskHandle = osThreadCreate(osThread(VisionComTask), NULL);

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
    //包含部分初始化内�??
    while (NRF24L01_Check()) {
        printf("no\n");
    }
    NRF24L01_RX_Mode();
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

    PID_Set(&VisionRun2, 1.55f, 0.0f, 0.3f, 0.0f);
    PID_Set(&DT35_Run, 0.020f, 0.0f, 0.0f, 0.0f);

    PID_Set(&VisionPID_X, 0.0020f, 0.0f, 0.001f, 0.0f);
    /* Infinite loop */
    for (;;) {
        LED0_Flashing;
//    printf("LiDar:X:%f,Y:%f,Angle:%f ",LiDar.locx,LiDar.locy,LiDar.yaw);
//    printf("%f,%f\n",DT35_Data.forward,DT35_Data.Right);
//      printf("%d\n",VisionFlag);
        osDelay(500);
    }
  /* USER CODE END DebugTask */
}

/* USER CODE BEGIN Header_NRFTask */
/**
* @brief 此函数用于接收遥控器的数�?????????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRFTask */
void NRFTask(void const * argument)
{
  /* USER CODE BEGIN NRFTask */
    /** 定义存储数据变量 **/
    static uint8_t rc_data[RX_PLOAD_WIDTH] = {0};//接收数组缓冲
    static RemoteRXSturct RemoteRX;//用于存遥控器传来的数
    uint8_t cmd;
    int16_t suctionSpeed = 0;
    uint8_t QueueBuffer;
    /* Infinite loop */
    for (;;) {
        if (NRF24L01_RxPacket(rc_data) == 0)  //接收遥控器数据，若收到返0，若没收到返1
        {
            /** 读取左右摇杆值，限制�?????????-128~128 **/
            RemoteRX.lx = (int16_t) -(rc_data[1] - 128);
            RemoteRX.ly = (int16_t) -(rc_data[2] - 128);
            RemoteRX.rx = (int16_t) -(rc_data[3] - 128);
            RemoteRX.ry = (int16_t) -(rc_data[4] - 128);
            /** 接收遥控器按键命 **/
            RemoteRX.command = rc_data[5];
            /** 设置摇杆值的死区 **/
            if (abs(RemoteRX.rx) < 2) RemoteRX.rx = 0;
            if (abs(RemoteRX.ry) < 2) RemoteRX.ry = 0;
            if (abs(RemoteRX.lx) < 2) RemoteRX.lx = 0;
            if (abs(RemoteRX.ly) < 2) RemoteRX.ly = 0;
            /** 对遥控器进行滤波，原因是遥控器有莫名其妙的电平跳 **/
            if (RemoteRX.rx == -5) RemoteRX.rx = 0;
            if (RemoteRX.ry == -4) RemoteRX.ry = 0;
            if (RemoteRX.lx == -4) RemoteRX.lx = 0;
            if (RemoteRX.ly == -4) RemoteRX.ly = 0;

            /**将NRF数据更新到四轮全�??????变量�??????**/
            SGW2Wheels((float) RemoteRX.rx * 3 / 128, (float) RemoteRX.ry * 3 / 128, (float) RemoteRX.lx * 3 / 128, 0);

            /** 对遥控器按键命令进行响应 **/
            switch (RemoteRX.command) {
                case Left_Up_Up://切换成手动模�?????????

                    break;
                case Left_Up://�?????????键启�?????????
                    xQueueSend(VisionData_QueueHandle, &QueueBuffer, 100);
                    QueueBuffer = 0;
                    break;
                case Right_Up:

                    break;
                case Right_Down:

                    break;
                case Right_Right:

                    break;
                case Left_Left:

                    break;
                case Left_Right:

                    break;
                case Right_Left:

                    break;
                case Right_Up_Up:
                    break;
                default:
                    break;
            }
//        printf("%d,%d\n",RemoteRX.command,RemoteRX.ly);/** 用于调试遥控器按 **/
            RemoteRX.command = 0; //重置命令
        }
        osDelay(100);
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
    PointStruct target_point = {0,0,0};
    float offset_x = 0;
  /* Infinite loop */
  for(;;)
  {
      if (xQueueReceive(ControlQueueHandle, &ControlQueueBuf, 0) == pdTRUE) {
          if(ControlQueueBuf.Device == CHASSIS){
              switch(ControlQueueBuf.Command){
                  case CHASSIS_RUN:
                      Stop_Flag = 0;
                      break;
                  case CHASSIS_STOP:
                      Stop_Flag = 1;
                      break;
                  case CloseLoop_MID360:
                      CloseLoopStatus = CloseLoop_MID360;
                      target_point.x = ControlQueueBuf.data[0];
                      target_point.y = ControlQueueBuf.data[1];
                      target_point.angle = ControlQueueBuf.data[2];
                      break;
                  case CloseLoop_DT35:
                      CloseLoopStatus = CloseLoop_DT35;
                      target_point.x = ControlQueueBuf.data[0];
                      target_point.y = ControlQueueBuf.data[1];
                      target_point.angle = ControlQueueBuf.data[2];
                      break;
                  case CloseLoop_Middle:
                      CloseLoopStatus = CloseLoop_Middle;
                      offset_x = ControlQueueBuf.data[0];
                      break;
                  case CloseLoop_Left:
                      CloseLoopStatus = CloseLoop_Left;
                      offset_x = ControlQueueBuf.data[0];
                      break;
                  case CloseLoop_Right:
                      CloseLoopStatus = CloseLoop_Right;
                      offset_x = ControlQueueBuf.data[0];
                      break;
                  case GoForwardSlowly:
                      CloseLoopStatus = GoForwardSlowly;
                      break;
                  default:
                      break;
              }
          }
      }
      /** 闭环程序 **/
      /**跑点位置闭环 **/
      if(Stop_Flag == 1){
          Car_Stop;
      }
      else{
          if(!(target_point.x == 0 && target_point.y == 0 && target_point.angle == 0)){
              if (CloseLoopStatus == CloseLoop_MID360) {
                  Chassis_Move_OfVision(&target_point);
                  /** 跑点到位检测 **/
                  if(Distance_Calc(target_point, LiDar.locx, LiDar.locy) < 0.1f){
                      Vision_Send(0x01);
                  }
              } else if (CloseLoopStatus == CloseLoop_DT35) {
                  Chassis_Move_OfDT35(&target_point);
              }
                  /** 对球闭环 **/
              else if (CloseLoopStatus == CloseLoop_Left) {
                  float Speed_xy = -PID_Realise(&VisionPID_X, 0, offset_x, 1.0f, 3);
                  float omega = PID_Realise(&Turn_PID, 135, LiDar.yaw, 0.5f, 0.5f);
                  float Speed_x = Speed_xy;
                  float Speed_y = Speed_xy;
                  SGW2Wheels(Speed_x, Speed_y, omega, 0);
              } else if (CloseLoopStatus == CloseLoop_Right) {
                  float Speed_xy = -PID_Realise(&VisionPID_X, 0, offset_x, 1.0f, 3);
                  float omega = PID_Realise(&Turn_PID, 45, LiDar.yaw, 0.5f, 0.5f);
                  float Speed_x = Speed_xy;
                  float Speed_y = -Speed_xy;
                  SGW2Wheels(Speed_x, Speed_y, omega, 0);
              } else if (CloseLoopStatus == CloseLoop_Middle) {
                  float Speed_xy = -PID_Realise(&VisionPID_X, 0, offset_x, 1.0f, 3);
                  float omega = PID_Realise(&Turn_PID, 135, LiDar.yaw, 0.5f, 0.5f);
                  float Speed_x = Speed_xy;
                  SGW2Wheels(Speed_x, 0, omega, 0);
                  /** 球对正检测 **/
                  if(offset_x < 7){
                      ControlMsgSet(&ControlQueueBuf,CHASSIS,GoForwardSlowly,0,0,0,0);
                      xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                      ControlMsgInit(&ControlQueueBuf);
                  }
              }
              else if(CloseLoopStatus == GoForwardSlowly){
                  SGW2Wheels(0, 0.5f, 0, 0);
              }
          }
      }

      /** 四轮闭环输出**/
      Wheels_VelOut[0] = (int16_t) PID_Realise(&Wheels[0], -Wheels_vel[0], Motor_Info[0].speed, M3508_CURRENT_MAX, 5);
      Wheels_VelOut[1] = (int16_t) PID_Realise(&Wheels[1], -Wheels_vel[1], Motor_Info[1].speed, M3508_CURRENT_MAX, 5);
      Wheels_VelOut[2] = (int16_t) PID_Realise(&Wheels[2], -Wheels_vel[2], Motor_Info[2].speed, M3508_CURRENT_MAX, 5);
      Wheels_VelOut[3] = (int16_t) PID_Realise(&Wheels[3], -Wheels_vel[3], Motor_Info[3].speed, M3508_CURRENT_MAX, 5);
      Set_Current(&hfdcan1, 0x200, Wheels_VelOut[0], Wheels_VelOut[1], Wheels_VelOut[2], Wheels_VelOut[3]);

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
   float Toggle_Pos = 0;
  /* Infinite loop */
  for(;;)
  {
      if (xQueueReceive(ControlQueueHandle, &ControlQueueBuf, 0) == pdTRUE) {
          if(ControlQueueBuf.Device == CLAW){
              switch(ControlQueueBuf.Command){
                  case CLAW_OPEN:
                      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
                      break;
                  case CLAW_CLOSE:
                      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
                      break;
                  case Toggle_Up:
                      Toggle_Pos = 3100;
                      break;
                  case Toggle_Mid:
                      Toggle_Pos = 1500;
                      break;
                  case Toggle_Down:
                      Toggle_Pos = 700;
                      break;
                  default:
                      break;
              }
          }
      }
      float Toggle_Position = PID_Realise(&Slope_Position_t, Toggle_Pos, Motor_Info[6].actual_total_angle, 2000, 10.0f);
      int16_t Toggle_Speed = (int16_t) PID_Realise(&Slope_Speed_t, Toggle_Position, Motor_Info[6].speed,
                                                  M2006_CURRENT_MAX, 5);
      Set_Current(&hfdcan2, 0x1FF, 0, 0, Toggle_Speed, 20000);
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
    float Slope_Pos = 0;
    int32_t VESC_Speed = 0;
    /* Infinite loop */
  for(;;)
  {
      if (xQueueReceive(ControlQueueHandle, &ControlQueueBuf, 0) == pdTRUE) {
        if(ControlQueueBuf.Device == SUCTION){
            switch(ControlQueueBuf.Command){
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
                    VESC_Speed = 7000;
                    break;
                case StopVESC:
                    VESC_Speed = 0;
                    break;
                default:
                    break;
            }
        }
      }
      float Slope_Position = PID_Realise(&Slope_Position_t, Slope_Pos, Motor_Info[6].actual_total_angle, 2000, 10.0f);
      int16_t Slope_Speed = (int16_t) PID_Realise(&Slope_Speed_t, Slope_Position, Motor_Info[6].speed,
                                                  M2006_CURRENT_MAX, 5);
      Vesc_SetSpeed(&hfdcan1, VESC_ID, VESC_Speed);
      Set_Current(&hfdcan2, 0x1FF, 0, 0, Slope_Speed, 20000);
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
    uint8_t status;//用于小状态切换
    float v_x, v_y;
    PointStruct Start_Point = {.x = 1.62f, .y = 9.63f, .angle = 90.0f};//3区调试用点
    ControlMsgStruct ControlQueueBuf;
    /* Infinite loop */
  for(;;) {
      if (xQueueReceive(VisionData_QueueHandle, &visiondata, 0) == pdTRUE) {
          /** 开始 **/
          if (visiondata.flag == 0) {
              //并不是从1到3，在3区调试用
              ControlMsgSet(&ControlQueueBuf,CHASSIS,CHASSIS_RUN,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_MID360,Start_Point.x,Start_Point.y,Start_Point.angle,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,CLAW,CLAW_CLOSE,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,CLAW,Toggle_Mid,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,SUCTION_OFF,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,StopVESC,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,Slope_OFF,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgInit(&ControlQueueBuf);
          }
          /** 跑球的点，中间区 **/
          else if (visiondata.flag == 1) {
              ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_MID360,-visiondata.vision_y / 1000.0f + LiDar.locx,visiondata.vision_x / 1000.0f + LiDar.locy,90,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,SUCTION_ON,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,OpenVESC,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,Slope_ON,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgInit(&ControlQueueBuf);
          }
              /** 跑球的点，左边区 **/
          else if (visiondata.flag == 5) {
              ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_MID360,-visiondata.vision_y / 1000.0f + LiDar.locx,visiondata.vision_x / 1000.0f + LiDar.locy,135,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,SUCTION_ON,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,OpenVESC,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,Slope_ON,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgInit(&ControlQueueBuf);
          }
          else if (visiondata.flag == 6) {
              ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_MID360,-visiondata.vision_y / 1000.0f + LiDar.locx,visiondata.vision_x / 1000.0f + LiDar.locy,45,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,SUCTION_ON,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,OpenVESC,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgSet(&ControlQueueBuf,SUCTION,Slope_ON,0,0,0,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgInit(&ControlQueueBuf);
          }
          else if (visiondata.flag == 2) {
              if(visiondata.vision_y == 0){
                  /** 对球 **/
                  ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_Middle,visiondata.vision_x,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgInit(&ControlQueueBuf);
              }
              else if(visiondata.vision_y == 1){
                  /** 正确的球即将进入车内 **/
                  ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_MID360,Start_Point.x,Start_Point.y,Start_Point.angle,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgSet(&ControlQueueBuf,SUCTION,SUCTION_OFF,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgSet(&ControlQueueBuf,SUCTION,StopVESC,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgSet(&ControlQueueBuf,CLAW,Toggle_Down,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgInit(&ControlQueueBuf);
              }
          }
          else if (visiondata.flag == 3) {
              if(visiondata.vision_y == 2){
                  /** 停车把球排出去 **/
                  ControlMsgSet(&ControlQueueBuf,CHASSIS,CHASSIS_STOP,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgSet(&ControlQueueBuf,SUCTION,StopVESC,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgSet(&ControlQueueBuf,SUCTION,Slope_ON,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  osDelay(100);
                  ControlMsgSet(&ControlQueueBuf,SUCTION,Slope_OUT,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  osDelay(100);
                  ControlMsgSet(&ControlQueueBuf,SUCTION,Slope_ON,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgInit(&ControlQueueBuf);
              }
              else if(visiondata.vision_x == 6){
                  /** 夹球 **/
                  ControlMsgSet(&ControlQueueBuf,CLAW,CLAW_CLOSE,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  osDelay(100);
                  ControlMsgSet(&ControlQueueBuf,CLAW,Toggle_Up,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgInit(&ControlQueueBuf);
              }
              else if(visiondata.vision_x == 3){
                  /** 后退再次找球 **/
                  ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_MID360,Start_Point.x,Start_Point.y,Start_Point.angle,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgInit(&ControlQueueBuf);
              }
              else if(visiondata.vision_x == 7){
                  /** 继续吸球 **/
                  ControlMsgSet(&ControlQueueBuf,SUCTION,OpenVESC,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgSet(&ControlQueueBuf,CHASSIS,CHASSIS_RUN,0,0,0,0);
                  xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
                  ControlMsgInit(&ControlQueueBuf);
              }
          }
          else if (visiondata.flag == 4) {
              ControlMsgSet(&ControlQueueBuf,CHASSIS,CloseLoop_DT35,Frame_Points[(int)(visiondata.vision_x-1)].x,Frame_Points[(int)(visiondata.vision_x-1)].x,90,0);
              xQueueSend(ControlQueueHandle, &ControlQueueBuf, 100);
              ControlMsgInit(&ControlQueueBuf);
          }

          osDelay(100);
      }
  }
  /* USER CODE END visioncom */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

