/**
  ******************************************************************************
  * @file           : Chassis.c
  * @author         : 86153
  * @brief          : 右上角是1号轮，逆时针旋转序号变大
  * @attention      : None
  * @date           : 2024/4/27
  ******************************************************************************
  */
#include "Chassis.h"
#include "arm_math.h"
#include "PID.h"
#include "retarget.h"
#include "MID360.h"
#include "DT35.h"
#include "main.h"
extern PID_t VisionRun2,DT35_Run;
extern uint8_t Camp;
extern float DT35_Forward,DT35_CloseBall;
PID_t Wheels[4];//轮子PID结构体
float Wheels_vel[4];//轮子转速
extern float MutiPos_x,MutiPos_y;
uint8_t AimPoints_Index;//目标点序号
PointStruct Aim_Points[256];//目标点们
PID_t Translation_PID, Turn_PID;//平动的PID结构体，转动的PID结构体
uint8_t cnt;

/** 用于存储比赛5个放球点 **/
PointStruct Frame_Points[5]= {
        {.x = 4.85f,.y = 11.20f,.angle = 0.0f},
        {.x = 4.85f,.y = 10.45f,.angle = 0.0f},
        {.x = 4.85f,.y = 9.70f,.angle = 0.0f},
        {.x = 4.85f,.y = 8.95f,.angle = 0.0f},
        {.x = 4.85f,.y = 8.20f,.angle = 0.0f}
};
/** 用于存储比赛开始从1区跑到三区的目标点,有五个点 **/
PointStruct Run1to3_Points[3] = {
        {.x = 0.f,.y = 6.f,.angle = 0.0f},
        {.x = 3.75f,.y = 6.f,.angle = 0.0f},
        {.x = 3.75f,.y = 8.13f,.angle = 0.0f},
};
/**
 * @brief 四全向轮底层解算函数
 * @param vel_x 世界坐标系中车的x轴速度 m/s
 * @param vel_y 世界坐标系中车的y轴速度 m/s
 * @param omega 旋转角速度 rad/s
 */
void SGW2Wheels(float vel_x_, float vel_y_, float omega, float theta)
{
    /********直接计算方法********/
    theta = theta * angle2rad;
    float sin_ang = arm_sin_f32(theta);
    float cos_ang = arm_cos_f32(theta);
    float vel_x ;
    float vel_y;
    if(Camp == RED){
        vel_x = vel_y_;
        vel_y = -vel_x_;
    }
    else if(Camp == BLUE){
        vel_x = -vel_y_;
        vel_y = -vel_x_;
    }
    Wheels_vel[0] = ( ((-cos_ang - sin_ang) * vel_x + (-sin_ang + cos_ang) * vel_y ) / (float)sqrt(2) + l * omega ) * 60 * 19 / pi / D;
    Wheels_vel[1] = ( ((-cos_ang + sin_ang) * vel_x + (-sin_ang - cos_ang) * vel_y ) / (float)sqrt(2) + l * omega ) * 60 * 19 / pi / D;
    Wheels_vel[2] = ( ((cos_ang + sin_ang) * vel_x + (sin_ang - cos_ang) * vel_y ) / (float)sqrt(2) + l * omega ) * 60 * 19 / pi / D;
    Wheels_vel[3] = ( ((cos_ang - sin_ang) * vel_x + (sin_ang + cos_ang) * vel_y ) / (float)sqrt(2) + l * omega ) * 60 * 19 / pi / D;
}
/***
 * @brief 底盘跑点函数
 * @param target_point 目标点结构体
 */
void Chassis_Move(PointStruct *target_point)
{
    float xSpeed = 0.0f,ySpeed = 0.0f;
    float dis = 0.0f;//当前点与目标点的距离
    float vel = 0.0f, omega = 0.0f;//速度,角速度
    float err_x = (target_point->x - LiDar.locx);//x差值
    float err_y = (target_point->y - LiDar.locy);//y差值
    float delta_angle = (target_point->angle - LiDar.yaw);//角度差值
    float max_out = 0.0f,allErr_x = 0.0f,allErr_y = 0.0f;
    static float all_dis = 0.0f;

    if( cnt == 0 )
    {
        allErr_x = (target_point->x - LiDar.locx);
        allErr_y = (target_point->y - LiDar.locy);
        arm_sqrt_f32(allErr_x * allErr_x + allErr_y * allErr_y,&all_dis);
        cnt = 1;
    }
    //计算向量长度
    arm_sqrt_f32(err_x * err_x + err_y * err_y,&dis);

    if( dis / all_dis >= 0.9 ) max_out = 1.5f;
    else if( dis / all_dis >= 0.8 && dis / all_dis < 0.9 ) max_out = 2.0f;
    else if( dis / all_dis >= 0.7 && dis / all_dis < 0.8 ) max_out = 2.5f;
    else if( dis / all_dis < 0.7 ) max_out = 3.0f;

    //计算平动速度向量
    //vel = PID_Realise(&Translation_PID, 0, -dis, max_out, 0.01f);
    //速度向量取绝对值
    arm_abs_f32(&vel, &vel, 1);
    //计算角速度
    //omega = PID_Realise(&Turn_PID, 0, -delta_angle, 1.3f, 0.1f);

    //线速度分解为x和y的分量
    xSpeed = vel * arm_cos_f32(atan2f(err_y, err_x));
    ySpeed = vel * arm_sin_f32(atan2f(err_y, err_x));

    //将车身x，y速度转换为轮子的x，y速度
    SGW2Wheels(xSpeed, ySpeed, omega, LiDar.yaw);
}
/***
 * @brief 底盘跑点函数
 * @param target_point 目标点结构体
 */
void Chassis_Move_OfVision(PointStruct *target_point,PID_t *pid,float max)
{
    float xSpeed = 0.0f,ySpeed = 0.0f;
    float dis = 0.0f;//当前点与目标点的距离
    float vel = 0.0f, omega = 0.0f;//速度,角速度
    float err_x = (target_point->x - MutiPos_x);//x差值
    float err_y = (target_point->y - MutiPos_y);//y差值
    float delta_angle = (target_point->angle - LiDar.yaw);//角度差值
    float max_out = 0.0f,allErr_x = 0.0f,allErr_y = 0.0f;
    static float all_dis = 0.0f;
    float ki_gain_threshold =1.5f ;
    float ki_gain = 1;
    //计算向量长度
    arm_sqrt_f32(err_x * err_x + err_y * err_y,&dis);
    //计算平动速度向量
    if(dis>ki_gain_threshold){
        ki_gain = 0;
    }
    else if(dis<=ki_gain_threshold){
        ki_gain = 1 - dis/ki_gain_threshold;
    }
    vel = PID_Realise(pid, 0, -dis, 1.5f, 0.01f,ki_gain);
    //速度向量取绝对值
    arm_abs_f32(&vel, &vel, 1);
    //计算角速度
    omega = PID_Realise(&Turn_PID, 0, -delta_angle, 1.2f, 0.1f,1);

    //线速度分解为x和y的分量
    xSpeed = vel * arm_cos_f32(atan2f(err_y, err_x));
    ySpeed = vel * arm_sin_f32(atan2f(err_y, err_x));

    //将车身x，y速度转换为轮子的x，y速度
    SGW2Wheels(xSpeed, ySpeed, omega, LiDar.yaw);
}
/***
 * @brief 底盘跑点函数
 * @param target_point 目标点结构体
 */
void Chassis_Move_OfDT35(PointStruct *target_point)
{
    float xSpeed = 0.0f,ySpeed = 0.0f;
    float dis = 0.0f;//当前点与目标点的距离
    float vel = 0.0f, omega = 0.0f;//速度,角速度
    float err_y = (target_point->y - DT35_Forward);//x差值
    float err_x = (target_point->x - DT35_CloseBall);//y差值
    float delta_angle = (target_point->angle - LiDar.yaw);//角度差值
    float max_out = 0.0f,allErr_x = 0.0f,allErr_y = 0.0f;
    float ki_gain_threshold =1.5f ;
    float ki_gain = 1;
//    static float all_dis = 0.0f;
//        allErr_x = (target_point->x - DT35_Data.Right);
//        allErr_y = (target_point->y - DT35_Data.Forward);
//        arm_sqrt_f32(allErr_x * allErr_x + allErr_y * allErr_y,&all_dis);
//        cnt = 1;
    //计算向量长度

//    if( dis / all_dis >= 0.9 ) max_out = 1.5f;
//    else if( dis / all_dis >= 0.8 && dis / all_dis < 0.9 ) max_out = 2.0f;
//    else if( dis / all_dis >= 0.7 && dis / all_dis < 0.8 ) max_out = 2.5f;
//    else if( dis / all_dis < 0.7 ) max_out = 3.0f;
        //max_out = 0.1f;
    arm_sqrt_f32(err_x * err_x + err_y * err_y,&dis);
    //计算平动速度向量
    if(dis>ki_gain_threshold){
        ki_gain = 0;
    }
    else if(dis<=ki_gain_threshold){
        ki_gain = 1 - dis/ki_gain_threshold;
    }
    vel = PID_Realise(&DT35_Run, 0, -dis, 1.5f, 0.01f,ki_gain);
    //速度向量取绝对值
    arm_abs_f32(&vel, &vel, 1);
    //计算角速度
    omega = PID_Realise(&Turn_PID, 0, -delta_angle, 1.2f, 0.1f,1);

    //线速度分解为x和y的分量
    xSpeed = vel * arm_cos_f32(atan2f(err_y, err_x));
    ySpeed = vel * arm_sin_f32(atan2f(err_y, err_x));

    //将车身x，y速度转换为轮子的x，y速度
    SGW2Wheels(-xSpeed, -ySpeed, omega, 0);

    //printf("%f %f",forward,close_ball);
}
/**
 * @brief 计算两点间距离
 * @param point 目标点结构体
 * @param x 当前x坐标
 * @param y 当前y坐标
 * @return
 */
float Distance_Calc(PointStruct point, float x, float y)
{
    return sqrtf((point.x - x) * (point.x - x) + (point.y - y) * (point.y - y));
}

/**
 * @brief 用于往目标点队列里发送目标点
 * @param point 目标点结构体
 * @param x x坐标
 * @param y y坐标
 * @param angle 车身偏航角
 */
void Set_Point(PointStruct *point,float x,float y,float angle)
{
    point->x = x;
    point->y = y;
    point->angle = angle;
}

void SetStore_Points(PointStruct *pointW,PointStruct *pointR)
{
    pointW->x = pointR->x;
    pointW->y = pointR->y;
    pointW->angle = pointR->angle;
}