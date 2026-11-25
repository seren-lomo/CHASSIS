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

#define J0_DIRECTION (-1)
#define J1_DIRECTION (-1)
#define J2_DIRECTION (1)
#define J3_DIRECTION (1)
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
	
//    Ref_t ref;  // 期望值
//    Fdb_t fdb;  // 状态值
//  	Cmd_t cmd;  // 控制量

    int8_t step;           // 底盘运行步骤号
    uint32_t step_time;    // (ms)底盘步骤运行时间
	
//	  PID_t pid;  // PID控制器
//	  LPF_t lpf;  // 低通滤波器
	
    uint32_t last_time;  // (ms)上一次更新时间
    uint32_t duration;   // (ms)任务周期
		
//		LQR_OUT_t lqr_out;

} Chassis_s;
#endif