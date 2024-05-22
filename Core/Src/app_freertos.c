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

/* USER CODE END Variables */
osThreadId Debug_TaskHandle;
osThreadId RoboRun_TaskHandle;
osThreadId NRF_TaskHandle;
osThreadId Control_TaskHandle;
osThreadId HandleBall_TaskHandle;
osThreadId Suction_TaskHandle;
osThreadId Run1to3_TaskHandle;
osMessageQId NRF_RX_QueueHandle;
osMessageQId SuctionSpeed_QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DebugTask(void const * argument);
void RoboRunTask(void const * argument);
void NRFTask(void const * argument);
void ControlTask(void const * argument);
void HandleBallTask(void const * argument);
void SuctionTask(void const * argument);
void Run1to3Task(void const * argument);

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

  /* definition and creation of SuctionSpeed_Queue */
  osMessageQDef(SuctionSpeed_Queue, 1, int16_t);
  SuctionSpeed_QueueHandle = osMessageCreate(osMessageQ(SuctionSpeed_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Debug_Task */
  osThreadDef(Debug_Task, DebugTask, osPriorityNormal, 0, 1024);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);

  /* definition and creation of RoboRun_Task */
  osThreadDef(RoboRun_Task, RoboRunTask, osPriorityNormal, 0, 1024);
  RoboRun_TaskHandle = osThreadCreate(osThread(RoboRun_Task), NULL);

  /* definition and creation of NRF_Task */
  osThreadDef(NRF_Task, NRFTask, osPriorityNormal, 0, 1024);
  NRF_TaskHandle = osThreadCreate(osThread(NRF_Task), NULL);

  /* definition and creation of Control_Task */
  osThreadDef(Control_Task, ControlTask, osPriorityNormal, 0, 1024);
  Control_TaskHandle = osThreadCreate(osThread(Control_Task), NULL);

  /* definition and creation of HandleBall_Task */
  osThreadDef(HandleBall_Task, HandleBallTask, osPriorityNormal, 0, 1024);
  HandleBall_TaskHandle = osThreadCreate(osThread(HandleBall_Task), NULL);

  /* definition and creation of Suction_Task */
  osThreadDef(Suction_Task, SuctionTask, osPriorityNormal, 0, 512);
  Suction_TaskHandle = osThreadCreate(osThread(Suction_Task), NULL);

  /* definition and creation of Run1to3_Task */
  osThreadDef(Run1to3_Task, Run1to3Task, osPriorityNormal, 0, 1024);
  Run1to3_TaskHandle = osThreadCreate(osThread(Run1to3_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  vTaskSuspend(RoboRun_TaskHandle);
//  vTaskSuspend(NRF_TaskHandle);
//  vTaskSuspend(Control_TaskHandle);
//  vTaskSuspend(HandleBall_TaskHandle);
//  vTaskSuspend(Suction_TaskHandle);
  vTaskSuspend(Run1to3_TaskHandle);
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
  while (NRF24L01_Check())
  {
      printf("no\n");
  }
  NRF24L01_RX_Mode();
  PID_Set(&Wheels[0],7.2f,0.1f,1.0f,10000);
  PID_Set(&Wheels[1],7.2f,0.1f,1.0f,10000);
  PID_Set(&Wheels[2],7.2f,0.1f,1.0f,10000);
  PID_Set(&Wheels[3],7.2f,0.1f,1.0f,10000);

  PID_Set(&Slope_Speed_t,7.2f,0.15f,2.2f,10000);
  PID_Set(&Slope_Position_t,1.85f,0,0.8f,0);
  PID_Set(&Toggle_Speed_t,7.2f,0.5f,2.0f,10000);
  PID_Set(&Toggle_Position_t,0.8f,0,0.8f,0);

  PID_Set(&Left_Speed_t,3,0.14f,2.2f,10000);
  PID_Set(&Right_Speed_t,3,0.14f,2.2f,10000);

  PID_Set(&Translation_PID,1.70f,0.0f,0.0f,0.0f);
  PID_Set(&Turn_PID,0.06f,0.0f,0.0f,0.0f);

  /* Infinite loop */
  for(;;)
  {
    LED0_Flashing;
//    printf("%d,%d,%d,%d\n",Motor_Info[0].speed,Motor_Info[1].speed,Motor_Info[2].speed,Motor_Info[3].speed);
//    printf("%f,%f,%f\n",Motor_Info[7].actual_total_angle,Slope_Position_t.target,Slope_Speed_t.PID_total_out);
//    printf("X:%f,Y:%f,Angle:%f\n",locater.pos_x,locater.pos_y,locater.angle);
//    printf("err:%f,ki:%f,i:%f,i_out:%f,out:%f\n",Wheels[0].err,Wheels[0].integral,Wheels[0].Ki,Wheels[0].i_out,Wheels[0].PID_total_out);
    printf("X:%f,Y:%f,Angle:%f\n",LiDar.locx,LiDar.locy,LiDar.yaw);
//    printf("%d,%d\n",Motor_Info[5].speed,-7000);
    osDelay(100);
  }
  /* USER CODE END DebugTask */
}

/* USER CODE BEGIN Header_RoboRunTask */
/**
* @brief 此任务用于R2跑点或�?�遥�?????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RoboRunTask */
void RoboRunTask(void const * argument)
{
  /* USER CODE BEGIN RoboRunTask */
    RemoteRXSturct NRFRX_Data;
  /* Infinite loop */
  for(;;)
  {
    if(Control_Mode == AutoRun_Mode)//跑点模式
    {
        if( Distance_Calc(Aim_Points[AimPoints_Index],LiDar.locx,LiDar.locy) < 0.01f && fabsf(LiDar.yaw - Aim_Points[AimPoints_Index].angle) < 0.5f )
        {
            if( Aim_Points[AimPoints_Index].num > 0 )
            {
                Aim_Points[AimPoints_Index].num --;
                AimPoints_Index ++;
                Aim_Points[AimPoints_Index].num = Aim_Points[AimPoints_Index-1].num;
            }
            else
            {
                AimPoints_Index ++;

                Wheels_vel[0] = 0;
                Wheels_vel[1] = 0;
                Wheels_vel[2] = 0;
                Wheels_vel[3] = 0;
                if(State == Run2Get_State)
                {
                    Control_Mode = Manual_Mode;
                }
                else if(State == TakeRightBall_State)
                {
                    Cmd = 0x01;
                    HAL_UART_Transmit(&huart3,&Cmd, sizeof(Cmd),0xFFFFF);//发送命令让摄像头回传哪个框
                    vTaskSuspend(RoboRun_TaskHandle);
                }
                else if(State == Run2Store_State)
                {
                    Control_Mode = Manual_Mode;
                }
                else
                {
                    vTaskSuspend(RoboRun_TaskHandle);
                }
            }
        }
        else
        {
            printf("X:%f,Y:%f,Angle:%f\n",LiDar.locx,LiDar.locy,LiDar.yaw);
            Chassis_Move(&Aim_Points[AimPoints_Index]);
        }
    }
    else//遥控模式
    {
        if( xQueueReceive(NRF_RX_QueueHandle, &NRFRX_Data, 0) == pdTRUE )//读取队列中NRF传来的数据，从NRF任务中写
        {
            SGW2Wheels((float)NRFRX_Data.rx * 3 / 128,(float)NRFRX_Data.ry * 3 / 128,(float)NRFRX_Data.lx * 3 / 128,0);
        }
    }
    osDelay(5);
  }
  /* USER CODE END RoboRunTask */
}

/* USER CODE BEGIN Header_NRFTask */
/**
* @brief 此函数用于接收遥控器的数�?????
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
    int16_t suctionSpeed = 0;//VESC速度
    /* Infinite loop */
  for(;;)
  {
      if (NRF24L01_RxPacket(rc_data) == 0)  //接收遥控器数据，若收到返0，若没收到返1
      {
          /** 读取左右摇杆值，限制�???????????-128~128 **/
          RemoteRX.lx = (int16_t)-(rc_data[1] - 128);
          RemoteRX.ly = (int16_t)-(rc_data[2] - 128);
          RemoteRX.rx = (int16_t)-(rc_data[3] - 128);
          RemoteRX.ry = (int16_t)-(rc_data[4] - 128);
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

          xQueueOverwrite(NRF_RX_QueueHandle, &RemoteRX);/** 把遥控器的数据放入队列，用于遥控 **/

          /** 对遥控器按键命令进行响应 **/
          switch (RemoteRX.command) {
              case Left_Up_Up:
                  Control_Mode = Manual_Mode;
                  vTaskResume(RoboRun_TaskHandle);
                  break;
              case Right_Up_Up:
                  Control_Mode = AutoRun_Mode;
                  vTaskResume(RoboRun_TaskHandle);
                  break;
              case Right_Up:
                  State = Store_State;
                  break;
              case Right_Down:
                  State = TakeWrongBall_State;
                  break;
              case Right_Right:
                  State = TakeRightBall_State;
                  break;
              case Right_Left:
                  State = TakeWrongBall_State;
                  break;
              case Left_Up://一键启动开始
                  SUCTION_ON; //伸出吸球机构
                  Toggle_Pos = Toggle_Mid; //夹爪翻到中位
                  osDelay(500);//避免夹爪提前打开
                  CLAW_OFF;
                  Set_Point(&Aim_Points[AimPoints_Index],-1.37f,1.52f,0,1); //写入一个起始点
                  Set_Point(&Aim_Points[AimPoints_Index+1],0,0,0,0); //写入一个起始点
                  State = Run2Get_State;
                  vTaskResume(RoboRun_TaskHandle);
                  break;
              case Left_Down:
                  Set_Point(&Aim_Points[AimPoints_Index],-1.37f,1.52f,90,0);
                  vTaskResume(RoboRun_TaskHandle);
                  break;
              case Left_Right:
                  Set_Point(&Aim_Points[AimPoints_Index],0.0f,0.0f,0,0);
                  vTaskResume(RoboRun_TaskHandle);
                  break;
              case Left_Left:
                  break;
              default:
                  break;
          }
//          printf("%d,%d\n",RemoteRX.command,RemoteRX.ly);/** 用于调试遥控器按 **/
          RemoteRX.command = 0; //重置命令
      }
    osDelay(10);
  }
  /* USER CODE END NRFTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
* @brief 此任务用于�?�过运动状�?�的判断切换跑点模式或�?�视觉追踪模�?????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask */
void ControlTask(void const * argument)
{
  /* USER CODE BEGIN ControlTask */
    int16_t suctionSpeed = 0;//VESC速度
  /* Infinite loop */
  for(;;)
  {
        switch (State) {
            case Run2Get_State:
                break;
            case TakeRightBall_State://取正确的球
                suctionSpeed = 7000;
                xQueueOverwrite(SuctionSpeed_QueueHandle,&suctionSpeed);
                Toggle_Pos = Toggle_Down;
                osDelay(1500);
                CLAW_ON;//关闭夹爪
                suctionSpeed = 0;
                xQueueOverwrite(SuctionSpeed_QueueHandle,&suctionSpeed);
//                Set_Point(&Aim_Points[AimPoints_Index],0,0,0,0);//写入一个目标点
//                vTaskResume(RoboRun_TaskHandle);
                Toggle_Pos = Toggle_Up;
                osDelay(500);
                SUCTION_OFF;
                while (State == TakeRightBall_State){}
                break;
            case TakeWrongBall_State:
                suctionSpeed = 7000;
                xQueueOverwrite(SuctionSpeed_QueueHandle,&suctionSpeed);
                Slope_Pos = Slope_Left;
                Left_TargetSpe = Left_Spe;
                osDelay(1500);
                osDelay(1500);
                suctionSpeed = 0;
                xQueueOverwrite(SuctionSpeed_QueueHandle,&suctionSpeed);
                Slope_Pos = 0;
                Left_TargetSpe = 0;
                State = Default_State;
                break;
            case Store_State://放球
                CLAW_OFF;//打开夹爪
                osDelay(1000);
//                Set_Point(&Aim_Points[AimPoints_Index],0,0,0,0);//写入一个目标点
//                vTaskResume(RoboRun_TaskHandle);
                Toggle_Pos = Toggle_Mid;
                SUCTION_ON;
                while(State == Store_State){}
                break;
            //跑到点之�??
            case Find_State:
                //�??启视觉找球任�??
                SUCTION_ON;
                break;
            default:
                break;
        }
    osDelay(10);
  }
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_HandleBallTask */
/**
* @brief 此任务用于控制电机实现左右拨球或者夹�?????/放球，控�?????3�?????2006和一�?????3508电机、两个气�?????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HandleBallTask */
void HandleBallTask(void const * argument)
{
  /* USER CODE BEGIN HandleBallTask */
  int16_t Right_Speed,Left_Speed,Slope_Speed,Toggle_Speed;
  float Slope_Temp,Toggle_Temp;
  /* Infinite loop */
  for(;;)
  {
    Left_Speed = (int16_t)PID_Realise(&Left_Speed_t,Left_TargetSpe,Motor_Info[5].speed,M2006_CURRENT_MAX,5);
    Right_Speed = (int16_t)PID_Realise(&Right_Speed_t,Right_TargetSpe,Motor_Info[4].speed,M2006_CURRENT_MAX,5);

    Slope_Temp = PID_Realise(&Slope_Position_t,Slope_Pos,Motor_Info[6].actual_total_angle,2000,10.0f);
    Slope_Speed = (int16_t)PID_Realise(&Slope_Speed_t,Slope_Temp,Motor_Info[6].speed,M2006_CURRENT_MAX,5);

    Toggle_Temp = PID_Realise(&Toggle_Position_t,Toggle_Pos,Motor_Info[7].actual_total_angle,1000,5.0f);
    Toggle_Speed = (int16_t)PID_Realise(&Toggle_Speed_t,Toggle_Temp,Motor_Info[7].speed,M3508_CURRENT_MAX,5);

    Set_Current(&hfdcan2,0x1FF,Right_Speed,Left_Speed,Slope_Speed,Toggle_Speed);
    osDelay(10);
  }

  /* USER CODE END HandleBallTask */
}

/* USER CODE BEGIN Header_SuctionTask */
/**
* @bri
 * ef 此任务用于吸球，控制VESC电调驱动�?????�?????5065电机
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SuctionTask */
void SuctionTask(void const * argument)
{
  /* USER CODE BEGIN SuctionTask */
    int16_t suctionSpeed = 0;//VESC速度
  /* Infinite loop */
  for(;;)
  {
      if(xQueuePeek(SuctionSpeed_QueueHandle,&suctionSpeed,0) == pdTRUE)
      {
          Vesc_SetSpeed(&hfdcan1,VESC_ID,suctionSpeed);
      }
      osDelay(5);
  }
  /* USER CODE END SuctionTask */
}

/* USER CODE BEGIN Header_Run1to3Task */
/**
* @brief 此任务用于比赛开始时R2�?????1区跑�?????3�?????
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Run1to3Task */
void Run1to3Task(void const * argument)
{
  /* USER CODE BEGIN Run1to3Task */
  uint8_t index = 0;
  /* Infinite loop */
  for(;;)
  {
      if( Distance_Calc(Run1to3_Points[index],LiDar.locx,LiDar.locy) < 0.01f && fabsf(LiDar.yaw - Run1to3_Points[index].angle) < 0.5f )
      {
              index++;
              Wheels_vel[0] = 0;
              Wheels_vel[1] = 0;
              Wheels_vel[2] = 0;
              Wheels_vel[3] = 0;
              if(index == 5)
              {
                  vTaskSuspend(Run1to3_TaskHandle);
              }
      }
      else
      {
          printf("X:%f,Y:%f,Angle:%f\n",LiDar.locx,LiDar.locy,LiDar.yaw);
          Chassis_Move(&Run1to3_Points[index]);
      }
    osDelay(5);
  }
  /* USER CODE END Run1to3Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

