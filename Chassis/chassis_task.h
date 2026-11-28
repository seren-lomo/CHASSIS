#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "bsp_rc.h"
#include "motor.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "kalman_filter.h"
#include "bsp_task_timing.h"
#include "motor.h"


#define CHASSIS_RC_DEADLINE    20 // 摇杆死区
///////////////////////PID
//气弹簧阻尼 
//初期调试先给小一点的刚度和限幅，防止发散打车
#define LEG_KP          (2000.0f)   // 初期可从 800 调起，稳了再加到 1500~2300
#define LEG_KD          (50.0f)    // 物理速度阻尼系数 (N / (m/s))

//横滚角
#define ROLL_KP         (0.0f)   // 横滚角刚度 (初期先设 0，站稳后再慢慢加)
#define ROLL_KD         (15.0f)    // 横滚角阻尼
#define ROLL_KI          0.0f
#define MAX_OUT_ROLL     180.0
#define MAX_IOUT_ROLL     0.0f




///////////////////////


#define JOINT_CAN (1)  //fdcan1
#define WHEEL_CAN (2)  //fdcan2

#define J0_DIRECTION (-1)//左前
#define J1_DIRECTION (-1)//左后
#define J2_DIRECTION (1)//右前
#define J3_DIRECTION (1)//右后

#define W0_DIRECTION ( 1)//右
#define W1_DIRECTION ( -1)

#define WHEEL_MASS           (0.5f)      // (kg)轮子重量  
#define WHEEL_RADIUS         (0.055f)    // (m)轮子半径 
/* Physical motor mapping: J0=ID3 left thigh/Phi4, J1=ID4 left Phi1.
 * J2=ID2 right Phi4, J3=ID1 right Phi1.
 * Provisional calibration: thigh Phi4=0, shank at +90 degrees.
 * Phi1_ref=2*atan2(0.250,0.210); validate over the full leg travel.
 * theta_transform computes direction*(pos+offset).
 */
#define J0_ANGLE_OFFSET (            -0.0201330185f) 	//ID3，左大腿 Phi4
#define J1_ANGLE_OFFSET (-1.7442730f - 0.0416097641f)  //	ID4，左虚拟杆 Phi1
#define J2_ANGLE_OFFSET (            -0.0692214966f) //	ID2，右大腿 Phi4
#define J3_ANGLE_OFFSET ( 1.7442730f - 0.0719060898f) //	ID1，右虚拟杆 Phi1	

#define MIN_LEG_LENGTH       ( 0.135f)         //最短腿长  
#define MAX_LEG_LENGTH       (0.357f)          //最大腿长  
#define BODY_MASS_FN         75.0f       //加云台更改  不要轮毂小腿下半部分簧上质量  观察实际腿长 L0 与目标腿长 ref.rod_L0 的差值
#define MAX_JOINT_TOR   (35.0f)    // 初期调试先限幅 15 Nm，正常后再放开到 40 Nm                                        //实际 L0 始终小于目标值（车身被压低） 前馈偏小，调大


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
//滤波
typedef struct LPF
{
    LowPassFilter_t support_force_filter[2];
	  LowPassFilter_t VX_filter;//键鼠信号阶跃滤波
    LowPassFilter_t x_acc_lpf;//位移加速度低通滤波
	  LowPassFilter_t dtheta;//腿部运动滤波

} LPF_t;

typedef struct LQR
{

/* Variables visible in Watch. */
     float wbr_x[10];
     float wbr_xref[10];
     float wbr_K[4][10];
     float wbr_u[4];
     int wbr_lqr_valid;//wbr_lqr_valid == 1：输入状态全部有限且腿长在合法查表区间内

}LQR_t;

typedef struct
{
    pid_type_def roll_angle;//横滚角
    pid_type_def leg_length_length[2];
   // pid_type_def leg_length_speed[2];
   // pid_type_def chassis_follow_gimbal;
   // pid_type_def leg_angle_angle;
   // pid_type_def stand_up;
   // pid_type_def wheel_stop[2];
//	  pid_type_def leg_tp;
//    pid_type_def leg_chase_L_to_R;
//	  pid_type_def leg_chase_R_to_L;

} PID_t;


typedef struct Body {
    float phi;       // 车体前后俯仰角 (对应建模中的 theta_b, rad)
    float phi_dot;   // 车体前后俯仰角速度 (rad/s)
    float roll;      // IMU 原始 roll
    float roll_dot;
    float pitch;     // IMU 原始 pitch
    float pitch_dot;
    float yaw;
    float yaw_dot;
	
	  float gx, gy, gz;  //重力加速度在机体坐标系下的分量，用于消除重力加速度对加速度计的影响

	   float x_accel;  // 机体坐标系下x轴加速度，除重力分量
    float y_accel;  // 机体坐标系下y轴加速度
    float z_accel;  // 机体坐标系下z轴加速度

} Body_t;

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

        float Theta;    // rad
        float dTheta;   // rad/s
        float ddTheta;  // rad/s^2

//        float F;   // N         根据关节反馈力矩和雅可比矩阵反接出来的支持力，只由关节提供
//        float Tp;  // N*m       旋转扭矩
				float F_spring;          //弹簧在竖直方向上的分力
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
    Body_t body;
   // World_t world;
    Leg_t leg[2];             // 0-左 1-右
	  //float two_leg_err;        //两腿之间的差值
    State_t state;  // 0-左 1-右
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

typedef struct Cmd
{
    struct leg
    {
        struct rod_cmd
        {
            float F;   // N
            float Tp;  // N*m
        } rod;
        struct joint_cmd
        {
            float T[2];    // N*m
            float Pos[2];  // rad
        } joint;
        struct wheel_cmd
        {
            float T;  // N*m
        } wheel;
    } leg[2];  // 0-左 1-右
} Cmd_t;



typedef struct __Imu
{
    float angle[3];  // rad 欧拉角数据
    float gyro[3];   // rad/s 陀螺仪数据
    float accel[3];  // m/s^2 加速度计数据
} Imu_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */

 
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
	  Imu_t * imu;     // imu数据
//	  ChassisMode_e mode;    // 底盘模式
//    ChassisMode_e last_mode;
    /*-------------------- Motors --------------------*/
    // 平衡底盘有2个驱动轮电机和4个关节电机
    Motor_s joint_motor[4];
    Motor_s wheel_motor[2];  // 驱动轮电机 0-右轮，1-左轮
    /*-------------------- Values --------------------*/
	
    Ref_t ref;  // 期望值
    Fdb_t fdb;  // 状态值
  	Cmd_t cmd;  // 控制量

    int8_t step;           // 底盘运行步骤号
    uint32_t step_time;    // (ms)底盘步骤运行时间
	
	  PID_t pid;  // PID控制器
	  LPF_t lpf;  // 低通滤波器
	
    uint32_t last_time;  // (ms)上一次更新时间
    uint32_t duration;   // (ms)任务周期
		
		LQR_t lqr;

} Chassis_s;

typedef struct
{
    TaskTiming_s timing;
    uint32_t initialized;
    uint32_t enable_attempts[4];
    uint32_t enabled_mask;     /* Bit 0..3 = CAN motor ID 1..4, recent state==1. */
} ChassisDebug_s;

extern volatile ChassisDebug_s g_chassis_debug;
void chassis_task(void *argument);
void ChassisInit(void);
void UpdateLegStatus(void);

#endif
