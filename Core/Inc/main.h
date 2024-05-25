/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include "PID.h"
#include "My_Can.h"
#include "24l01.h"
#include "math.h"
#include "stdlib.h"
#include "Chassis.h"
#include "Locator.h"
#include "MID360.h"
#include "VESC.h"
#include "usart.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t USART1_Buffer[256];
extern uint8_t USART2_Buffer[256];
extern uint8_t USART3_Buffer[256];
extern uint8_t USART4_Buffer[256];
extern uint8_t USART5_Buffer[256];

extern PID_t Wheels[4];//轮子转�??
extern float Wheels_vel[4];//轮子转�??
extern PID_t Translation_PID, Turn_PID;//平动的PID结构体，转动的PID结构�??????

extern PointStruct Aim_Points[256],Frame_Points[5];//目标点们
extern uint8_t AimPoints_Index;//目标点序�?????

extern MotorInfo_t Motor_Info[MOTOR_NUM];//大疆电机返回的数据数�????????

extern uint8_t Control_Mode;
extern uint8_t State,Cmd;

extern uint8_t cnt;

extern PointStruct Run1to3_Points[5];//用于存储比赛�?????始从1区跑到三区的目标�?????,有五个点

extern PID_t Slope_Speed_t,Slope_Position_t,Toggle_Speed_t,Toggle_Position_t;
extern PID_t Left_Speed_t,Right_Speed_t;

extern float Left_TargetSpe,Right_TargetSpe,Slope_Pos,Toggle_Pos;

extern PID_t VisionPID_X,VisionPID_Y,VisionRun2;

extern PointStruct Vision_Points[10];
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_CE_Pin GPIO_PIN_3
#define SPI4_CE_GPIO_Port GPIOE
#define SPI4_NSS_Pin GPIO_PIN_4
#define SPI4_NSS_GPIO_Port GPIOE
#define SPI4_IRQ_Pin GPIO_PIN_13
#define SPI4_IRQ_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_12
#define LED0_GPIO_Port GPIOE
#define SOLE_P1_Pin GPIO_PIN_11
#define SOLE_P1_GPIO_Port GPIOD
#define SOLE_N1_Pin GPIO_PIN_12
#define SOLE_N1_GPIO_Port GPIOD
#define SOLE_P2_Pin GPIO_PIN_13
#define SOLE_P2_GPIO_Port GPIOD
#define SOLE_N2_Pin GPIO_PIN_14
#define SOLE_N2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define SUCTION_ON   HAL_GPIO_WritePin(SOLE_P1_GPIO_Port,SOLE_N1_Pin|SOLE_P2_Pin,GPIO_PIN_RESET); HAL_GPIO_WritePin(SOLE_P1_GPIO_Port,SOLE_P2_Pin,GPIO_PIN_SET)
#define SUCTION_OFF  HAL_GPIO_WritePin(SOLE_P1_GPIO_Port,SOLE_N1_Pin|SOLE_P2_Pin,GPIO_PIN_RESET); HAL_GPIO_WritePin(SOLE_P1_GPIO_Port,SOLE_N1_Pin,GPIO_PIN_SET)
#define CLAW_ON      HAL_GPIO_WritePin(SOLE_P1_GPIO_Port,SOLE_P1_Pin|SOLE_N2_Pin,GPIO_PIN_RESET); HAL_GPIO_WritePin(SOLE_P1_GPIO_Port,SOLE_P1_Pin,GPIO_PIN_SET)
#define CLAW_OFF     HAL_GPIO_WritePin(SOLE_P1_GPIO_Port,SOLE_P1_Pin|SOLE_N2_Pin,GPIO_PIN_RESET); HAL_GPIO_WritePin(SOLE_P1_GPIO_Port,SOLE_N2_Pin,GPIO_PIN_SET)

#define LED0_Flashing HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin)

#define Manual_Mode 0x01
#define AutoRun_Mode 0x00

#define Default_State 0x00
#define Run2Get_State 0xFF
#define Find_State 0xFE
#define TakeRightBall_State 0xFD
#define Run2Store_State 0xFC
#define Store_State 0xFB
#define TakeWrongBall_State 0xFA

#define Left_Spe 7000 //左边电机旋转
#define Right_Spe (-7000) //右边电机旋转
#define Toggle_Down 0 //夹爪翻下�??
#define Toggle_Mid 1300 //夹爪归中
#define Toggle_Up 3000 //夹爪翻上�??
#define Slope_Left (-450.0f) //平台向左倾斜
#define Slope_Right 450.0f //平台向右倾斜
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
