/**
  ******************************************************************************
  * @file           : PID.h
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/4/27
  ******************************************************************************
  */

#ifndef R2_MASTER_V3_PID_H
#define R2_MASTER_V3_PID_H
#include "stm32g4xx_hal.h"

#define M3508_CURRENT_MAX 16384
#define M6020_CURRENT_MAX 30000
#define M2006_CURRENT_MAX 10000

/**
  * @brief  PID参数结构体定义
  */
typedef struct {
    float target;//定义目标值
    float current;//定义实际值
    float err;//定义偏差值
    float err_last;//定义上一个偏差值
    float Kp,Ki,Kd,FB_Gain;//定义比例、积分、微分系数
    float p_out;//定义P项输出
    float i_out;//定义I项输出
    float d_out;//定义D项输出
    float PID_total_out;//定义PID执行输出
    float integral;//定义积分值
    float differentiation;//定义微分值
/*-----------------------积分限幅---------------------------*/
    float integral_limit;//定义积分限幅值
    float FB_Now;
    float FB_Last;
    float FB_Error;
}PID_t;

void PID_Set(PID_t *PID, float kp, float ki, float kd, float integral_limit,float FB_Gain);
float PID_Realise(PID_t *PID, float target, float current, float max_output, float DeadZone, float ki_gain);

#endif //R2_MASTER_V3_PID_H
