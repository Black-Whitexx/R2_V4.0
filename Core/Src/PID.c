/**
  ******************************************************************************
  * @file           : PID.c
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/4/27
  ******************************************************************************
  */
#include "PID.h"
#include "math.h"
#include "retarget.h"

PID_t Slope_Speed_t,Slope_Position_t,Toggle_Speed_t,Toggle_Position_t;
PID_t Left_Speed_t,Right_Speed_t;
/**
  * @brief  PID参数设置，全是float，去除了积分分离，感觉有点冗余而且不会用
  * @param  *PID 要设置的目标PID结构体地址
  */
void PID_Set(PID_t *PID, float kp, float ki, float kd, float integral_limit)
{
    PID->integral_limit = integral_limit;
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->target = 0;
    PID->current = 0;
    PID->err = 0;
    PID->err_last = 0;
    PID->integral = 0;
    PID->differentiation = 0;
    PID->p_out = 0;
    PID->i_out = 0;
    PID->d_out = 0;
    PID->PID_total_out = 0;
}

/**
  * @brief  PID输出释放
  * @param  *PID 要计算输出的目标PID结构体
  * @param  target PID目标
  * @param  current PID输入（当前值）
  * @param  max_output PID输出限幅（绝对值）
  * @return PID输出
  */
float PID_Realise(PID_t *PID, float target, float current, float max_output, float DeadZone)
{
    PID->target = target;
    PID->current = current;
    PID->err = (PID->target - PID->current);

    //输入死区控制
    if(fabsf(PID->err) < DeadZone)
        PID->err = 0;

    //计算积分项
    PID->integral = PID->integral + PID->err;
    if(PID->integral_limit != 0)//如果开启了积分限幅，基本上一定开启
    {
        if(PID->integral < -PID->integral_limit)
            PID->integral = -PID->integral_limit;
        else if(PID->integral > PID->integral_limit)
            PID->integral = PID->integral_limit;
    }

    //计算微分量
    PID->differentiation = PID->err - PID->err_last;

    PID->p_out = PID->Kp * PID->err;
    PID->i_out = PID->Ki * PID->integral;
    PID->d_out = PID->Kd * PID->differentiation;

    PID->PID_total_out = PID->p_out + PID->i_out + PID->d_out;

    PID->err_last = PID->err;

    //输出限幅控制
    if(PID->PID_total_out > max_output)//输出限幅 正向
        PID->PID_total_out = max_output;
    else if(PID->PID_total_out < -max_output)//输出限幅 负向
        PID->PID_total_out = -max_output;

    return PID->PID_total_out;
}
