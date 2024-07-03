/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "DT35.h"
#include "Locator.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "fdcan.h"
#include "Screen.h"
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//float LiDar_x_last,LiDar_y_last = 0;
//float Locator_Start[2];
//float MutiPos_x;
//float MutiPos_y;
//float Locator_Addup[2];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern int16_t Wheels_VelOut[0];
extern int16_t Toggle_Pos;
extern int16_t Slope_Pos;
extern int32_t VESC_Speed;
//extern locater_def locater;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void Motor_CLoseloop(){
//    Wheels_VelOut[0] = (int16_t) PID_Realise(&Wheels[0], -Wheels_vel[0], Motor_Info[0].speed, M3508_CURRENT_MAX, 5);
//    Wheels_VelOut[1] = (int16_t) PID_Realise(&Wheels[1], -Wheels_vel[1], Motor_Info[1].speed, M3508_CURRENT_MAX, 5);
//    Wheels_VelOut[2] = (int16_t) PID_Realise(&Wheels[2], -Wheels_vel[2], Motor_Info[2].speed, M3508_CURRENT_MAX, 5);
//    Wheels_VelOut[3] = (int16_t) PID_Realise(&Wheels[3], -Wheels_vel[3], Motor_Info[3].speed, M3508_CURRENT_MAX, 5);
//    Set_Current(&hfdcan1, 0x200, Wheels_VelOut[0], Wheels_VelOut[1], Wheels_VelOut[2], Wheels_VelOut[3]);
//    /** ��צ�ͷ����ջ�**/
//    float Slope_Position = PID_Realise(&Slope_Position_t, Slope_Pos, Motor_Info[6].actual_total_angle, 2000, 10.0f);
//    int16_t Slope_Speed = (int16_t) PID_Realise(&Slope_Speed_t, Slope_Position, Motor_Info[6].speed,
//                                                M2006_CURRENT_MAX, 5);
//    float Toggle_Position = PID_Realise(&Toggle_Position_t, Toggle_Pos, Motor_Info[7].actual_total_angle, 1000,
//                                        5.0f);
//    int16_t Toggle_Speed = (int16_t) PID_Realise(&Toggle_Speed_t, Toggle_Position, Motor_Info[7].speed,
//                                                 M3508_CURRENT_MAX, 5);
//    Set_Current(&hfdcan2, 0x1FF, 0, 0, Slope_Speed, Toggle_Speed);
//    Vesc_SetSpeed(&hfdcan1, VESC_ID, VESC_Speed);
//}

//void Position_InterPolation(){
//    float LiDar_x = LiDar.locx;
//    float LiDar_y = LiDar.locy;
//    float Locator_x = locater.pos_x/100.f;
//    float Locator_y = locater.pos_y/100.f;
//    if(!(LiDar_x == LiDar_x_last && LiDar_y == LiDar_y_last)){
//        /** �ø����� **/
//        /** ������ʼ���� **/
//        Locator_Addup[0] = 0;
//        Locator_Addup[1] = 0;
//        Locator_Start[0] = Locator_x;
//        Locator_Start[1] = Locator_y;
//    }
//    /** ���̲�ֵ **/
//    Locator_Addup[0] = Locator_x - Locator_Start[0];
//    Locator_Addup[1] = Locator_y - Locator_Start[1];
//    MutiPos_x = LiDar_x + Locator_Addup[0];
//    MutiPos_y = LiDar_y + Locator_Addup[1];
//    MutiPos_x = 1;
//    MutiPos_y = LiDar_y + Locator_Addup[1];
//
//}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */
extern osMessageQId SuctionSpeed_QueueHandle;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
//    Motor_CLoseloop();
//    Position_InterPolation();
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
//    if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //判断是否是空闲中�???????????????????
//    {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);                     //清除空闲中断标志（否则会�???????????????????直不断进入中断）
    HAL_UART_DMAStop(&huart1);                        //停止DMA接收

    VOFA_SetPID(&Slope_Speed_t,&Slope_Position_t);          //对串口接收数据进行解�????????????????

    HAL_UART_Receive_DMA(&huart1, USART1_Buffer, 255);   //重启串口接收中断，开始DMA传输
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�???????????????????
//    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
//  if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))   //判断是否是空闲中�???????????????????
// {
      __HAL_UART_CLEAR_IDLEFLAG(&huart2);                     //清除空闲中断标志（否则会�???????????????????直不断进入中断）
      HAL_UART_DMAStop(&huart2);                        //停止DMA接收

      RaDar_Data_Rec(USART2_Buffer, &LiDar, &Vision_Data);

      HAL_UART_Receive_DMA(&huart2, USART2_Buffer, 50);   //重启串口接收中断，开始DMA传输
      __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�???????????????????
// }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);                     //清除空闲中断标志（否则会�???????????????????直不断进入中断）
    HAL_UART_DMAStop(&huart3);                        //停止DMA接收
    Unpack_Screen_CMD(USART3_Buffer);
    HAL_UART_Receive_DMA(&huart3, USART3_Buffer, 30);   //重启串口接收中断，开始DMA传输
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�???????????????????
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt / UART4 wake-up interrupt through EXTI line 34.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);                     //清除空闲中断标志（否则会�?????直不断进入中断）
    HAL_UART_DMAStop(&huart4);                        //停止DMA接收

    if( USART4_Buffer[0] == 0x06)
    {
        DT35_Rec(USART4_Buffer,&DT35_Data);          //对DT35的数据进行解�?????
    }

    HAL_UART_Receive_DMA(&huart4, USART4_Buffer, 255);             //重启串口接收中断，开始DMA传输
    __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt / UART5 wake-up interrupt through EXTI line 35.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
    static uint8_t i=0,j=0,k=0;
    int16_t suctionSpeed = 0;
//    if(RESET != __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))   //判断是否是空闲中�???????????????????
//    {
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);                     //清除空闲中断标志（否则会�???????????????????直不断进入中断）
    HAL_UART_DMAStop(&huart5);                        //停止DMA接收

    locatorAndToF_Data_Rec(USART5_Buffer, &locater,&TOF_dis1);
    //printf("%.2f,%.2f,%.2f,%.4f\n",locater.pos_x,locater.pos_y,locater.angle,locater.Tof_dis);

    HAL_UART_Receive_DMA(&huart5, USART5_Buffer, 255);   //重启串口接收中断，开始DMA传输
    __HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�???????????????????
//    }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt, DAC2 and DAC4 channel underrun error interrupts.
  */
void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC_IRQn 0 */

  /* USER CODE END TIM7_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_DAC_IRQn 1 */

  /* USER CODE END TIM7_DAC_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 0.
  */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */

  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */

  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN3 interrupt 0.
  */
void FDCAN3_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN3_IT0_IRQn 0 */

  /* USER CODE END FDCAN3_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan3);
  /* USER CODE BEGIN FDCAN3_IT0_IRQn 1 */

  /* USER CODE END FDCAN3_IT0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == htim2.Instance){
    }
}
/* USER CODE END 1 */
