#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "bsp_rc.h"
#include "motor.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "kalman_filter.h"
#include "motor.h"

#define JOINT_CAN (1)  //fdcan1
#define WHEEL_CAN (2)  //fdcan2

#define J0_DIRECTION (-1)//左前
#define J1_DIRECTION (-1)//左后
#define J2_DIRECTION (1)//右前
#define J3_DIRECTION (1)//右后

#define W0_DIRECTION ( -1)
#define W1_DIRECTION (  1)

// 2. 关节0点偏移量 (已在水平位置标零，无需再加有的零点残差小数)
//#define J0_ANGLE_OFFSET     (M_PI)  // 左腿 Phi1 标零时对应 180° (π)
//#define J1_ANGLE_OFFSET     (0.0f)  // 左腿 Phi4 标零时对应 0° (0)
//#define J2_ANGLE_OFFSET     (0.0f)  // 右腿 Phi4 标零时对应 0° (π)
//#define J3_ANGLE_OFFSET     (M_PI)  // 右腿 Phi1 标零时对应 180° (0)
#define J0_ANGLE_OFFSET (-1.7442730f - 0.0201330185f)
#define J1_ANGLE_OFFSET (            -0.0416097641f)
#define J2_ANGLE_OFFSET (            -0.0692214966f)
#define J3_ANGLE_OFFSET ( 1.7442730f - 0.0719060898f)
//状态向量
typedef struct
{
   float s;//自然坐标系下机器人水平方向移动距离
	 float s_dot;
	 float yaw;//偏航角
	 float yaw_dot;
	 float theta_l;//腿倾斜角（自然坐标系法向）
	 float theta_l_dot;
	float theta_r;
	float theta_r_dot;
	float theta_body;//机体倾斜角（自然坐标系法向）
	float theta_body_dot;

}State_t;

//控制向量
typedef struct
{
float T_wl;
float  T_wr;
float T_bl; 
float	 T_br;

}control_t;

//腿部状态
typedef struct Leg
{
	    struct rod
    {
        float Phi0;    // rad(Phi0)：是虚拟腿（从髋关节连到车轮中心的直线）相对于**车体（机座）**的夹角
        float dPhi0;   // rad/s
        float ddPhi0;  // rad/s^2
			
			  float L0;    // 虚拟腿长m
        float dL0;   // m/s
        float ddL0;  // m/s^2
			

			  float T_b;    // 虚拟腿部转矩
	  
			  float F_b;    // 沿虚拟腿部力

			  float F;      // N根据关节反馈力矩和雅可比矩阵反接出来的支持力，只由关节提供
			  float Tp;  // N*m       旋转扭矩

//        float Theta;    // rad
//        float dTheta;   // rad/s
//        float ddTheta;  // rad/s^2

//        float F;   // N         根据关节反馈力矩和雅可比矩阵反接出来的支持力，只由关节提供
//        float Tp;  // N*m       旋转扭矩
//				float F_spring;          //弹簧在竖直方向上的分力
//			  float F_costheta;      //支持力在竖直上的分量
//			  float Tp_sintheta;     //旋转力在竖直上的分量
    } rod;    //连杆虚拟腿
		
	    struct joint
    {
        float T1, T2;        // N*m
        float Phi1, Phi4;    // rad
        float dPhi1, dPhi4;  // rad/s
			
    } joint;
		
		 struct wheel
    {
        float Angle;     // rad
	
        float Velocity;  // rad/s

    } wheel;
		 float J[2][2];           // 雅可比矩阵
	
}Leg_t;
 //反馈
typedef struct
{
  //  Body_t body;
   // World_t world;
    Leg_t leg[2];             // 0-左 1-右
	  //float two_leg_err;        //两腿之间的差值
    State_t state[2];  // 0-左 1-右
   // ChassisSpeedVector_t speed_vector;
	  //Gimbal_t gimbal;
	  //uint8_t tell_gimbal_thing;       //反馈云台标志位 0.自由运动  1.小陀螺
} Fdb_t;

/**
 * @brief 期望
 */
typedef struct
{
  //  Body_t body;
    State_t leg_state[2];  // 0-左 1-右
    float rod_L0[2];          // 0-左 1-右
    float rod_Angle[2];       // 0-左 1-右
   // ChassisSpeedVector_t speed_vector;
} Ref_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */

 
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
//	  Imu_t * imu;     // imu数据
//	  ChassisMode_e mode;    // 底盘模式
//    ChassisMode_e last_mode;
    /*-------------------- Motors --------------------*/
    // 平衡底盘有2个驱动轮电机和4个关节电机
    Motor_s joint_motor[4];
    Motor_s wheel_motor[2];  // 驱动轮电机 0-左轮，1-右轮
    /*-------------------- Values --------------------*/
	
    Ref_t ref;  // 期望值
    Fdb_t fdb;  // 状态值
  	//Cmd_t cmd;  // 控制量

    int8_t step;           // 底盘运行步骤号
    uint32_t step_time;    // (ms)底盘步骤运行时间
	
//	  PID_t pid;  // PID控制器
//	  LPF_t lpf;  // 低通滤波器
	
    uint32_t last_time;  // (ms)上一次更新时间
    uint32_t duration;   // (ms)任务周期
		
//		LQR_OUT_t lqr_out;

} Chassis_s;

void ChassisInit(void);
void UpdateLegStatus(void);

#endif