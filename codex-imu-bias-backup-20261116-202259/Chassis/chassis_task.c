#include "chassis_task.h"
#include "bsp_rc.h"
#include "IMU_task.h"
#include "motor.h"
#include "CAN_receive.h"
#include "stdbool.h"
#include "bsp_delay.h"
#include "bsp_can.h"
#include "stdio.h"
#include "fdcan.h"
#include "usart.h"
#include "user_lib.h"
#include "chassis_cal.h"
#include "string.h"
#include "cmsis_os2.h"
#include "CAN_cmd_damiao.h"

 int BODY_PITCH_DIRECTION =-1;
static Imu_t chassis_imu;
Chassis_s CHASSIS = {.imu = &chassis_imu};
volatile ChassisDebug_s g_chassis_debug; 
void UpdateLegStatus(void);
void UpdateBodyStatus(void);
void UpdateMotorStatus(void);
void  ChassisSendCmd(void);
void chassis_task(void *argument)
{
    uint32_t next_tick;
    uint32_t last_enable_tick;
    uint8_t first_enable = 1U;
    (void)argument;
    ChassisInit();
    next_tick = osKernelGetTickCount();//记录当前 tick，作为周期调度的基准
    last_enable_tick = next_tick;
    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        uint8_t i;
        TaskTimingBegin(&g_chassis_debug.timing);//// 开始测耗时
        UpdateMotorStatus();
        g_chassis_debug.enabled_mask = 0U;
        for (i = 0U; i < 4U; i++)                 //计算"已使能电机掩码"
        {
            if (CHASSIS.joint_motor[i].fdb.state == 1U &&
                CAN_DM_MEASURE[i].id == CHASSIS.joint_motor[i].id &&
                !CHASSIS.joint_motor[i].offline)
                g_chassis_debug.enabled_mask |= 1U << i;
        }
				
        /* Retry disabled motors at 10 Hz, without clearing motor faults. */
        if (first_enable || (uint32_t)(now - last_enable_tick) >= 100U)//使能重试机制
        {
            first_enable = 0U;
            last_enable_tick = now;
            for (i = 0U; i < 4U; i++)
            {
                if (CHASSIS.joint_motor[i].fdb.state == 0U)
                {
                    DmEnable(&CHASSIS.joint_motor[i]);
                    g_chassis_debug.enable_attempts[i]++;
                }
            }
        }
        /* The body angle is needed by UpdateLegStatus in the same cycle. */
        if (g_imu_debug.ready && g_imu_debug.timing.loop_count != 0U)//只在 IMU 数据就绪且至少跑过一次时才更新


        UpdateBodyStatus();
        UpdateLegStatus();
        ChassisSendCmd();
        FDCAN1_UpdateDebug();
        TaskTimingEnd(&g_chassis_debug.timing);
        TaskWaitNext1ms(&next_tick, &g_chassis_debug.timing);//// 睡到下一个 1 ms 边界
    }
}

void ChassisInit(void)
{
	 	CHASSIS.rc = get_remote_control_point();  //获取遥控器指针 
	  /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.joint_motor[0], 1, JOINT_CAN, DM_8009, J3_DIRECTION, 1, DM_MODE_MIT);//右后
    MotorInit(&CHASSIS.joint_motor[1], 2, JOINT_CAN, DM_8009, J2_DIRECTION, 1, DM_MODE_MIT);//右前
    MotorInit(&CHASSIS.joint_motor[2], 3, JOINT_CAN, DM_8009, J0_DIRECTION, 1, DM_MODE_MIT);//左前
    MotorInit(&CHASSIS.joint_motor[3], 4, JOINT_CAN, DM_8009, J1_DIRECTION, 1, DM_MODE_MIT);//左后
	  
	  MotorInit(&CHASSIS.wheel_motor[0], 1, WHEEL_CAN, DJI_M3508, W0_DIRECTION, 15.76470588235294, 0);//注意电流控制要转换成扭矩控制
    MotorInit(&CHASSIS.wheel_motor[1], 2, WHEEL_CAN, DJI_M3508, W1_DIRECTION, 15.76470588235294, 0);//反馈数据是转子的转速
	 
    /* Enabling/retries belong to the nonblocking periodic task. */
    g_chassis_debug.initialized = 1U;
}



//更新腿部状态
void UpdateLegStatus(void)
{
   uint8_t i = 0;
  //正视
	//左腿   以关节中心建立坐标系 左关节(phi1) 水平 上转 -pi   ->  0    右关节(phi4) 水平上转  0  ->  -pi        
  //                                      水平 下转  pi ->  0                  水平下转   0  ->  pi
  /* ID3 drives ED (Phi4); ID4 is the other input (Phi1). */
	CHASSIS.fdb.leg[0].joint.Phi1=theta_transform(CHASSIS.joint_motor[3].fdb.pos,J1_ANGLE_OFFSET,J1_DIRECTION,1);
	CHASSIS.fdb.leg[0].joint.Phi4 =theta_transform(CHASSIS.joint_motor[2].fdb.pos,J0_ANGLE_OFFSET,J0_DIRECTION, 1);

	CHASSIS.fdb.leg[1].joint.Phi1 =theta_transform(CHASSIS.joint_motor[0].fdb.pos, J3_ANGLE_OFFSET, J3_DIRECTION, 1);
  CHASSIS.fdb.leg[1].joint.Phi4 =theta_transform(CHASSIS.joint_motor[1].fdb.pos, J2_ANGLE_OFFSET, J2_DIRECTION, 1);
	
	CHASSIS.fdb.leg[0].joint.dPhi1 = CHASSIS.joint_motor[3].fdb.vel * (J1_DIRECTION);
  CHASSIS.fdb.leg[0].joint.dPhi4 = CHASSIS.joint_motor[2].fdb.vel * (J0_DIRECTION);
  CHASSIS.fdb.leg[1].joint.dPhi1 = CHASSIS.joint_motor[0].fdb.vel * (J3_DIRECTION);
  CHASSIS.fdb.leg[1].joint.dPhi4 = CHASSIS.joint_motor[1].fdb.vel * (J2_DIRECTION);
	
	  // =====更新关节电机力矩=====
  CHASSIS.fdb.leg[0].joint.T1 = CHASSIS.joint_motor[3].fdb.tor * (J1_DIRECTION);
  CHASSIS.fdb.leg[0].joint.T2 = CHASSIS.joint_motor[2].fdb.tor * (J0_DIRECTION);
  CHASSIS.fdb.leg[1].joint.T1 = CHASSIS.joint_motor[0].fdb.tor * (J3_DIRECTION);
  CHASSIS.fdb.leg[1].joint.T2 = CHASSIS.joint_motor[1].fdb.tor * (J2_DIRECTION);
	
		// =====更新驱动轮角速度===== 反馈转子速度除以减速比
	CHASSIS.fdb.leg[0].wheel.Velocity = CHASSIS.wheel_motor[0].fdb.vel * (W0_DIRECTION)/CHASSIS.wheel_motor[0].reduction_ratio;//rad/s
  CHASSIS.fdb.leg[1].wheel.Velocity = CHASSIS.wheel_motor[1].fdb.vel * (W1_DIRECTION)/CHASSIS.wheel_motor[1].reduction_ratio;//rad/s
	
		// =====更新摆杆姿态=====
  float L0_Phi0[2];//存L0 Phi0
  float dL0_dPhi0[2];

	for (i = 0; i < 2; i++) {
        float last_dL0 = CHASSIS.fdb.leg[i].rod.dL0;
        float last_dPhi0 = CHASSIS.fdb.leg[i].rod.dPhi0;
     //   float last_dTheta = CHASSIS.fdb.leg[i].rod.dTheta;
        
		    // 更新腿长和摆角位置信息  通过关节电机角度正解到L0和phi0
		    GetL0AndPhi0(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, L0_Phi0);
		    //如果不考虑关节电机的phi1和phi4 那么L0 和  phi0的关系就是
        //屁股往前看               phi0都是  0-->>pi/2-->>pi
        CHASSIS.fdb.leg[i].rod.L0 = L0_Phi0[0]  ;
        CHASSIS.fdb.leg[i].rod.Phi0 = L0_Phi0[1];
		
		    CHASSIS.fdb.leg[i].rod.Theta = M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;//腿倾斜角
		
        CHASSIS.fdb.state.theta_l=CHASSIS.fdb.leg[0].rod.Theta;//写入状态
				CHASSIS.fdb.state.theta_r=CHASSIS.fdb.leg[1].rod.Theta;
		
				 // 计算雅可比矩阵  根据phi1和phi4计算出末端的雅可比矩阵J
        CalcJacobian(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, CHASSIS.fdb.leg[i].J);
		    
		     // 一阶微分   通过雅可比矩阵求解出l0的微分和phi0的微分 同时求解出theta的微分
        GetdL0AnddPhi0(CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.dPhi1, CHASSIS.fdb.leg[i].joint.dPhi4,dL0_dPhi0);
        
		    //计算得到弹簧前馈补偿力 
		  //  CHASSIS.fdb.leg[i].rod.F_spring=LegController_CalcSpringForce(CHASSIS.fdb.leg[i].rod.L0);
		
        CHASSIS.fdb.leg[i].rod.dL0 = dL0_dPhi0[0];
        CHASSIS.fdb.leg[i].rod.dPhi0 = dL0_dPhi0[1];
       // CHASSIS.fdb.leg[i].rod.dTheta = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;
				//CHASSIS.fdb.leg[i].rod.dTheta = LowPassFilterCalc(&CHASSIS.lpf.dtheta, CHASSIS.fdb.leg[i].rod.dTheta);

//        // 更新加速度信息  一阶微分--->>二阶微分  
//        float accel = (CHASSIS.fdb.leg[i].rod.dL0 - last_dL0) / (CHASSIS.duration * MS_TO_S);
//        CHASSIS.fdb.leg[i].rod.ddL0 = accel;

//        accel = (CHASSIS.fdb.leg[i].rod.dPhi0 - last_dPhi0) / (CHASSIS.duration * MS_TO_S);
//        CHASSIS.fdb.leg[i].rod.ddPhi0 = accel;

//        accel = (CHASSIS.fdb.leg[i].rod.dTheta - last_dTheta) / (CHASSIS.duration * MS_TO_S);
//        CHASSIS.fdb.leg[i].rod.ddTheta = accel;


        
		    // 计算支撑力 根据反馈的两个关节电机的力矩  以及计算得出的雅可比矩阵  可以得到等效摆杆地面的支持力 以及转动力
        float F[2];//F   T
        GetLegForce(CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.T1, CHASSIS.fdb.leg[i].joint.T2, F);
				
				CHASSIS.fdb.leg[i].rod.F=F[0];  
				CHASSIS.fdb.leg[i].rod.Tp=F[1];

				float F0 = F[0];
        float Tp = F[1];
				
		//		float P = F0 * cosf(theta) +( Tp * sinf(theta) / CHASSIS.fdb.leg[i].rod.L0)+CHASSIS.fdb.leg[i].rod.F_spring;

//				
//        CHASSIS.fdb.leg[i].Fn = P;

//        if (CHASSIS.fdb.leg[i].Fn< TAKE_OFF_FN_THRESHOLD) {
//            CHASSIS.fdb.leg[i].touch_time = 0;
//            CHASSIS.fdb.leg[i].take_off_time += CHASSIS.duration;
//        } 
//				else {
//            CHASSIS.fdb.leg[i].touch_time += CHASSIS.duration;
//            CHASSIS.fdb.leg[i].take_off_time = 0;
//        }
    }
	
}


////////更新机体imu
void UpdateBodyStatus(void)
{
	
	CHASSIS.fdb.body.roll=CHASSIS.imu->angle[0];
	CHASSIS.fdb.body.roll_dot=CHASSIS.imu->gyro[0];
	
  //机体系pitch就是左右倾斜的角
	CHASSIS.fdb.body.pitch = CHASSIS.imu->angle[1];
	CHASSIS.fdb.body.pitch_dot=CHASSIS.imu->gyro[1];
	
	CHASSIS.fdb.body.yaw=CHASSIS.imu->angle[2];
	CHASSIS.fdb.body.yaw_dot=CHASSIS.imu->gyro[2];
	
		 // 更新加速度反馈数据，记录下来方便使用
    float ax = CHASSIS.imu->accel[0];
    float ay = CHASSIS.imu->accel[1];
    float az = CHASSIS.imu->accel[2];
	
		    // 计算几个常用的三角函数值，减少重复计算
    float cos_roll = cosf(CHASSIS.fdb.body.roll);
    float sin_roll = sinf(CHASSIS.fdb.body.roll);
    float cos_pitch = cosf(CHASSIS.fdb.body.pitch);
    float sin_pitch = sinf(CHASSIS.fdb.body.pitch);
    float cos_yaw = cosf(CHASSIS.fdb.body.yaw);
    float sin_yaw = sinf(CHASSIS.fdb.body.yaw);
		
				// 计算重力加速度在各个轴上的分量
    CHASSIS.fdb.body.gx = -GRAVITY * sin_pitch;
    CHASSIS.fdb.body.gy = GRAVITY * sin_roll * cos_pitch;
    CHASSIS.fdb.body.gz = GRAVITY * cos_roll * cos_pitch;
		
		    // 消除重力加速度的影响，获取机体坐标系下的加速度
    CHASSIS.fdb.body.x_accel = ax - CHASSIS.fdb.body.gx;
    CHASSIS.fdb.body.y_accel = ay - CHASSIS.fdb.body.gy;
    CHASSIS.fdb.body.z_accel = az - CHASSIS.fdb.body.gz;

    CHASSIS.fdb.body.phi     = BODY_PITCH_DIRECTION * (CHASSIS.fdb.body.roll);
    CHASSIS.fdb.body.phi_dot = BODY_PITCH_DIRECTION * CHASSIS.fdb.body.roll_dot;

 
        CHASSIS.fdb.state.theta_body     = CHASSIS.fdb.body.phi;
        CHASSIS.fdb.state.theta_body_dot = CHASSIS.fdb.body.phi_dot;
        CHASSIS.fdb.state.yaw            = CHASSIS.fdb.body.yaw;
        CHASSIS.fdb.state.yaw_dot        = CHASSIS.fdb.body.yaw_dot;
 

}
void SendJointMotorCmd(void)
{
			// 3. 发送 0 力矩指令（既让腿保持无力可手掰，又触发达妙电机回传下一帧位置数据）
      for (uint8_t i = 0; i < 4; i++) {
          CHASSIS.joint_motor[i].set.tor = 0.0f;
      }
      DmMitCtrlTorque(&CHASSIS.joint_motor[0]); 
      DmMitCtrlTorque(&CHASSIS.joint_motor[1]); 
      DmMitCtrlTorque(&CHASSIS.joint_motor[2]); 
      DmMitCtrlTorque(&CHASSIS.joint_motor[3]);

}
//void SendWheelMotorCmd(void);

 void ChassisSendCmd(void) 
 {
	      SendJointMotorCmd();
       // SendWheelMotorCmd();
 }

 /**
 * @brief  更新底盘电机数据
 * @param  none
 */


void UpdateMotorStatus(void)
{
    for (uint8_t i = 0; i < 4; i++) 
	  {
     GetMotorMeasure(&CHASSIS.joint_motor[i]);//获取关节电机数据
    }
    for (uint8_t i = 0; i < 2; i++) 
		{
		GetMotorMeasure(&CHASSIS.wheel_motor[i]);	//获取3508数据在  转速是rad/s
    }
}
