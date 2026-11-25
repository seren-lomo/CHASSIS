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
#include "wbr_lqr.h"
#include <float.h>
#include <math.h>

 int BODY_PITCH_DIRECTION =-1;
 float dt_s = 0.001f;
static Imu_t chassis_imu;
static LowPassFilter_t s_dot_filter;

Chassis_s CHASSIS = {.imu = &chassis_imu};
volatile ChassisDebug_s g_chassis_debug; 



enum
{
    STAND_OFF,
    STAND_SUPPORT,
    STAND_BALANCE,
    STAND_RISING,
    STAND_FIXED_BALANCE,
    STAND_FIXED_SUPPORT_HOLD  /* Mode only; request stays STAND_FIXED_BALANCE. */
};

enum
{
    STAND_OK,
    STAND_IMU_INVALID,
    STAND_MOTOR_OFFLINE,
    STAND_POSE_OUT_OF_RANGE,
    STAND_LQR_INVALID,
    STAND_INVALID_REQUEST,
    STAND_RISE_TIMEOUT,
    STAND_FIXED_NOT_READY
};

enum
{
    STAND_POSE_BODY_PHI  = 1U << 0,
    STAND_POSE_BODY_ROLL = 1U << 1,
    STAND_POSE_LEFT_ANGLE = 1U << 2,   /* Right leg uses the next bit. */
    STAND_POSE_LEFT_LENGTH = 1U << 4,
    STAND_POSE_LEFT_JACOBIAN = 1U << 6,
    STAND_POSE_LEFT_DATA = 1U << 8,
    STAND_POSE_HOLD_BODY_RATE = 1U << 10
};

enum
{
    STAND_POSE_RUNNING,
    STAND_POSE_ENTERING,
    STAND_POSE_SUPPORTED,
    STAND_POSE_FIXED_ENTRY
};

enum
{
    STAND_RISE_IDLE,
    STAND_RISE_LIFT,
    STAND_RISE_RECENTER
};

typedef struct
{
    float body_phi_rad;
    float leg_theta_rad;
} StandPoseLimits_s;

/* Keep the phase limits together so the check and snapshot use the same values. */
//最大机体前后俯仰角（body_phi_rad）和左右腿倾斜角（leg_theta_rad）
static const StandPoseLimits_s stand_pose_limits[] = {
    [STAND_POSE_RUNNING]   = {0.2618f, 0.5236f},
    [STAND_POSE_ENTERING]  = {0.13962634f, 0.26179939f}, /* 8 deg body, 15 deg leg */
    [STAND_POSE_SUPPORTED] = {0.2618f, 0.6109f},
    [STAND_POSE_FIXED_ENTRY] = {0.13962634f, 0.17453293f} /* 8 deg body, 10 deg leg */
};

/* Limit the effect of wheel slip on stationary balance position feedback. */
static const float stand_position_error_limit_m = 0.35f;//0.05
/* Keep position feedback active while reducing wheel-odometry influence in the fixed-height test. */
static const float stand_fixed_translation_scale = 0.5f;
/* Reduce fixed-height position return torque after a large displacement. */
static const float stand_fixed_position_error_limit_m = 0.12f;

//加减初始后驱的系数
static const float stand_fixed_common_leg_wheel_scale = 0.65f;
static const float stand_fixed_body_wheel_scale = 1.25f;
static const float stand_fixed_leg_pitch_rate_damping = 1.0f; /* N m per rad/s, per leg. */


static const float stand_support_hold_body_rate_limit_rad_s = 0.5f;

/* Supported lift, measured as leg-pivot height relative to the wheel axle. */
static const float stand_rise_height_target_m = 0.020f;
static const float stand_rise_min_lift_m = 0.008f;
static const float stand_rise_theta_limit_rad = 0.10472f; /* 6 degrees */
static const float stand_rise_theta_rate_rad_s = 0.35f;
static const float stand_rise_extend_rate_mps = 0.015f;
static const float stand_rise_retract_rate_mps = 0.080f;
static const float stand_rise_ready_time_s = 0.030f;
static const uint32_t stand_rise_phase_timeout_ticks = 4000U;

typedef struct
{
    uint32_t sequence;
    uint32_t tick;
    uint16_t reason_mask;
    uint8_t request;
    uint8_t mode;
    uint8_t entering_balance;
    float body_phi_rad;
    float body_roll_rad;
    float leg_theta_rad[2];     /* Left, right. */
    float leg_length_m[2];
    float wheel_current_cmd[2]; /* Left, right; previous CAN command in raw counts. */
    float body_phi_limit_rad;
    float leg_theta_limit_rad;
    float position_error_m;
    float position_error_used_m;
} StandPoseSnapshot_s;

typedef struct
{
    float length_ref_m;
    float length_m;
    float angle_ref_rad;
    float angle_rad;
    float height_gain_m;
    float axial_force_request_n;
    float axial_force_cmd_n;
    float axial_force_feedback_n;
    float spring_force_n;
    float swing_cmd_nm;
    float swing_feedback_nm;
} StandRiseLegSnapshot_s;

typedef struct
{
    uint32_t sequence;
    uint32_t tick;
    uint8_t phase;
    uint8_t force_limited_mask;
    StandRiseLegSnapshot_s leg[2]; /* Left, right. */
    float joint_torque_cmd_nm[4];  /* Motor index order. */
    float joint_torque_feedback_nm[4];
} StandRiseTimeoutSnapshot_s;

volatile StandPoseSnapshot_s chassis_stand_pose_snapshot;
volatile StandRiseTimeoutSnapshot_s chassis_stand_rise_timeout_snapshot;

typedef struct
{
    uint32_t tick;
    float blend;
    float position_error_m;
    float position_error_used_m;
    float speed_mps;
    float speed_error_used_mps;
    float body_phi_rad;
    float body_rate_rad_s;
    float leg_theta_rad[2];    /* Left, right. */
    float leg_rate_rad_s[2];
    float leg_length_m[2];
    float leg_length_ref_m[2];
    float leg_swing_cmd_nm[2];
    float wheel_speed_mps[2];  /* Positive is forward. */
    float wheel_request_nm[2]; /* Model output, before the CAN current limit. */
    float wheel_terms_nm[2][5]; /* s/ds, yaw/rate, left leg, right leg, body. */
    float current_limit;
    float current_cmd[2];     /* Left, right, signed hardware counts. */
    float current_feedback[2]; /* Left, right, signed hardware counts. */
    float leg_swing_feedback_nm[2];
    uint8_t force_limited_mask;
} StandBalanceSample_s;

typedef struct
{
    uint32_t sequence;
    uint8_t mode;
    uint8_t exit_fault;
    uint8_t backward_valid;
    uint8_t current_limit_mask; /* First limited sample, bit 0 left, bit 1 right. */
    StandBalanceSample_s entry;
    StandBalanceSample_s first_backward;
    StandBalanceSample_s first_current_limit;
    StandBalanceSample_s recent;
} StandBalanceTrace_s;

volatile StandBalanceTrace_s chassis_stand_balance_trace;
static const float stand_wheel_torque_per_count = 0.000480637608f;

/* With the robot secured: 1 supports, 2 rises then balances, 4 balances at fixed length. */
volatile uint8_t chassis_stand_request = STAND_OFF;
volatile uint8_t chassis_stand_mode = STAND_OFF;
volatile uint8_t chassis_stand_fault = STAND_OK;
volatile float chassis_stand_support_n = BODY_MASS_FN;
volatile float chassis_stand_requested_f0[2];
volatile uint8_t chassis_stand_force_limited_mask;
volatile float chassis_stand_position_error_m;
volatile float chassis_stand_position_error_used_m;
volatile uint8_t chassis_stand_wheel_speed_limited_mask; /* bit 0: left, bit 1: right */
volatile uint8_t chassis_stand_rise_phase;
volatile uint8_t chassis_stand_rise_timeout_phase;
volatile float chassis_stand_rise_height_gain_m[2];

static float stand_support_blend;
static float stand_balance_blend;
static float stand_speed_error_used_mps;
static uint32_t stand_rise_phase_tick;
static float stand_rise_start_height_m[2];
static float stand_rise_ready_s;

void UpdateLegStatus(void);
void UpdateBodyStatus(void);
void UpdateMotorStatus(void);
void  ChassisSendCmd(void);
void BodyMotionObserve(void);
static void ConsoleNormal(void);
static void StopStand(uint8_t fault);

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
		   
			dt_s = (g_chassis_debug.timing.loop_count > 1U)
           ? g_chassis_debug.timing.period_us * 1e-6f
           : 0.001f;//周期测量
			
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
        if (g_imu_debug.ready && g_imu_debug.valid_attitude)
        {
            UpdateBodyStatus();
            UpdateLegStatus();
            if (!CHASSIS.wheel_motor[0].offline && !CHASSIS.wheel_motor[1].offline)
            {
                BodyMotionObserve();
            }
            ConsoleNormal();
        }
        else
        {
            CHASSIS.lqr.wbr_lqr_valid = 0;
            StopStand(chassis_stand_request != STAND_OFF ? STAND_IMU_INVALID : STAND_OK);
        }
        ChassisSendCmd();
        FDCAN1_UpdateDebug();
        FDCAN2_UpdateDebug();
        TaskTimingEnd(&g_chassis_debug.timing);
        TaskWaitNext1ms(&next_tick, &g_chassis_debug.timing);//// 睡到下一个 1 ms 边界
    }
}





///////////////////////
void ChassisInit(void)
{
	 	CHASSIS.rc = get_remote_control_point();  //获取遥控器指针 
	  /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.joint_motor[0], 1, JOINT_CAN, DM_8009, J3_DIRECTION, 1, DM_MODE_MIT);//右后
    MotorInit(&CHASSIS.joint_motor[1], 2, JOINT_CAN, DM_8009, J2_DIRECTION, 1, DM_MODE_MIT);//右前
    MotorInit(&CHASSIS.joint_motor[2], 3, JOINT_CAN, DM_8009, J0_DIRECTION, 1, DM_MODE_MIT);//左前
    MotorInit(&CHASSIS.joint_motor[3], 4, JOINT_CAN, DM_8009, J1_DIRECTION, 1, DM_MODE_MIT);//左后
	  //0 右 id1
	  MotorInit(&CHASSIS.wheel_motor[0], 1, WHEEL_CAN, DJI_M3508, W0_DIRECTION, 15.76470588235294, 0);//注意电流控制要转换成扭矩控制
    MotorInit(&CHASSIS.wheel_motor[1], 2, WHEEL_CAN, DJI_M3508, W1_DIRECTION, 15.76470588235294, 0);//反馈数据是转子的转速
	 
	
	 //pid roll 弹簧
	 float roll_pid[3] = {ROLL_KP , ROLL_KI , ROLL_KD };
   PID_init(&CHASSIS.pid.roll_angle, PID_POSITION, roll_pid, MAX_OUT_ROLL,MAX_IOUT_ROLL);
    /* Enabling/retries belong to the nonblocking periodic task. */
    g_chassis_debug.initialized = 1U;
	/* 0.5 作为初步调试值，之后根据噪声和响应调整 */
    LowPassFilterInit(&s_dot_filter, 0.5f);
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
	CHASSIS.fdb.leg[0].wheel.Velocity = CHASSIS.wheel_motor[1].fdb.vel * (W1_DIRECTION)/CHASSIS.wheel_motor[1].reduction_ratio;//rad/s
  CHASSIS.fdb.leg[1].wheel.Velocity = CHASSIS.wheel_motor[0].fdb.vel * (W0_DIRECTION)/CHASSIS.wheel_motor[0].reduction_ratio;//rad/s右
	
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
		

		
				 // 计算雅可比矩阵  根据phi1和phi4计算出末端的雅可比矩阵J
        CalcJacobian(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, CHASSIS.fdb.leg[i].J);
		    
		     // 一阶微分   通过雅可比矩阵求解出l0的微分和phi0的微分 同时求解出theta的微分
        GetdL0AnddPhi0(CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.dPhi1, CHASSIS.fdb.leg[i].joint.dPhi4,dL0_dPhi0);
        
		    //计算得到弹簧前馈补偿力 
		    CHASSIS.fdb.leg[i].rod.F_spring=LegController_CalcSpringForce(CHASSIS.fdb.leg[i].rod.L0);
		
        CHASSIS.fdb.leg[i].rod.dL0 = dL0_dPhi0[0];
        CHASSIS.fdb.leg[i].rod.dPhi0 = dL0_dPhi0[1];
				
        CHASSIS.fdb.leg[i].rod.dTheta = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;
			//	CHASSIS.fdb.leg[i].rod.dTheta = LowPassFilterCalc(&CHASSIS.lpf.dtheta, CHASSIS.fdb.leg[i].rod.dTheta);
        

       
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
	        CHASSIS.fdb.state.theta_l=CHASSIS.fdb.leg[0].rod.Theta;//写入状态
				  CHASSIS.fdb.state.theta_r=CHASSIS.fdb.leg[1].rod.Theta;
					CHASSIS.fdb.state.theta_l_dot=CHASSIS.fdb.leg[0].rod.dTheta;
				  CHASSIS.fdb.state.theta_r_dot=CHASSIS.fdb.leg[1].rod.dTheta;
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


/////////////////////
/**
 * @brief  机体运动状态观测器
 * @param  none
 */
float s_dot_raw;
void BodyMotionObserve(void)
{
/* 沿用旧版的轮轴角速度补偿写法 */
//       float v_left = WHEEL_RADIUS *
//               (CHASSIS.fdb.leg[0].wheel.Velocity
//                    - CHASSIS.fdb.leg[0].rod.dTheta
//                    + CHASSIS.fdb.body.phi_dot);

//			float v_right = WHEEL_RADIUS *
//							(CHASSIS.fdb.leg[1].wheel.Velocity
//										- CHASSIS.fdb.leg[1].rod.dTheta
//										+ CHASSIS.fdb.body.phi_dot);
	       float v_left = WHEEL_RADIUS *
               CHASSIS.fdb.leg[0].wheel.Velocity;
                  

			float v_right = WHEEL_RADIUS *
							CHASSIS.fdb.leg[1].wheel.Velocity;
			
        s_dot_raw = 0.5f * (v_left + v_right);
			//滤波
            CHASSIS.fdb.state.s_dot = LowPassFilterCalc(&s_dot_filter, s_dot_raw);
            CHASSIS.fdb.state.s += CHASSIS.fdb.state.s_dot * dt_s;

}

//LQR 注单位：m、m/s、rad、rad/s？？？待验证
static void CalcLQR(bool fixed_balance)
{
   const State_t *state = &CHASSIS.fdb.state;
   float x_error[10];
	 CHASSIS.lqr.wbr_x[0] = state->s;
   CHASSIS.lqr.wbr_x[1] = state->s_dot;

     CHASSIS.lqr.wbr_x[2] = state->yaw;
     CHASSIS.lqr.wbr_x[3] = state->yaw_dot;

    CHASSIS.lqr.wbr_x[4] = state->theta_l;
     CHASSIS.lqr.wbr_x[5] = state->theta_l_dot;

     CHASSIS.lqr.wbr_x[6] = state->theta_r;
     CHASSIS.lqr.wbr_x[7] = state->theta_r_dot;

     CHASSIS.lqr.wbr_x[8] = state->theta_body;
     CHASSIS.lqr.wbr_x[9] = state->theta_body_dot;

    for (uint8_t i = 0U; i < 10U; ++i)
    {
        x_error[i] = CHASSIS.lqr.wbr_x[i] - CHASSIS.lqr.wbr_xref[i];
    }
    chassis_stand_position_error_m = x_error[0];
    if (isfinite(x_error[0]))
    {
        float position_limit = fixed_balance ? stand_fixed_position_error_limit_m
                                             : stand_position_error_limit_m;
        x_error[0] = fp32_constrain(x_error[0], -position_limit, position_limit);
    }
    if (fixed_balance)
    {
        x_error[0] *= stand_fixed_translation_scale;
        x_error[1] *= stand_fixed_translation_scale;
    }
    chassis_stand_position_error_used_m = x_error[0];
    stand_speed_error_used_mps = x_error[1];
	
	float Ll_lqr = fp32_constrain(CHASSIS.fdb.leg[0].rod.L0, 0.135f, 0.357f);
  float Lr_lqr = fp32_constrain(CHASSIS.fdb.leg[1].rod.L0, 0.135f, 0.357f);
	
    CHASSIS.lqr.wbr_lqr_valid = WbrLqrStep(
        Ll_lqr, Lr_lqr, x_error, CHASSIS.lqr.wbr_K, CHASSIS.lqr.wbr_u,
        fixed_balance ? stand_fixed_common_leg_wheel_scale : 1.0f,
        fixed_balance ? stand_fixed_body_wheel_scale : 1.0f,
        fixed_balance ? stand_fixed_leg_pitch_rate_damping : 0.0f);
	

}

/**
 * @brief 腿部力矩控制
   F沿腿长支持力 地面给
   F_pd腿长虚拟弹簧阻尼
   F0前馈
 */
//注：后期加跳跃，离地
float F_roll, l0_err, F_pd;
//力矩优先级保护
/* Find the axial-force interval that fits both joints with the chosen swing torque. */
static float LimitStandLegForce(float force, float swing, float J[2][2], float joint_torque_limit)
{
    float min_force = -FLT_MAX;
    float max_force = FLT_MAX;
    for (uint8_t joint = 0U; joint < 2U; joint++)
    {
        float axial = J[0][joint];
        float swing_torque = J[1][joint] * swing;
        if (fabsf(axial) > 1.0e-6f)
        {
            float lower = (-joint_torque_limit - swing_torque) / axial;
            float upper = ( joint_torque_limit - swing_torque) / axial;
            min_force = fmaxf(min_force, fminf(lower, upper));
            max_force = fminf(max_force, fmaxf(lower, upper));
        }
    }
    return fp32_constrain(force, min_force, max_force);
}

static void LegTorqueController(float support_force, float joint_torque_limit)
{
    bool rising = chassis_stand_mode == STAND_RISING;

    F_roll = fp32_constrain(PID_calc(&CHASSIS.pid.roll_angle, CHASSIS.fdb.body.pitch, 0.0f),
                           -80.0f, 80.0f);
    chassis_stand_force_limited_mask = 0U;

    for (uint8_t i = 0U; i < 2U; i++)
    {
        Leg_t *leg = &CHASSIS.fdb.leg[i];
        float roll_force = (i == 0U) ? -F_roll : F_roll;
        float force;//沿腿支持力
        float support_swing;//非平衡态的腿摆回正力矩
        float swing;//虚拟摆动力矩
        float swing_gain;//中摆动力矩对关节力矩的最大放大系数

        CHASSIS.ref.rod_L0[i] = fp32_constrain(CHASSIS.ref.rod_L0[i], MIN_LEG_LENGTH, MAX_LEG_LENGTH);
        l0_err = CHASSIS.ref.rod_L0[i] - leg->rod.L0;//腿长位置误差
        F_pd = fp32_constrain(LEG_KP * l0_err - LEG_KD * leg->rod.dL0, -150.0f, 150.0f);
        force = stand_support_blend * (support_force + roll_force + F_pd - leg->rod.F_spring);
        if (rising)
        {
            support_swing = fp32_constrain(10.0f * (leg->rod.Theta - CHASSIS.ref.rod_Angle[i])
                                           - leg->rod.dPhi0, -3.0f, 3.0f);
        }
        else
        {
            support_swing = fp32_constrain(3.0f * leg->rod.Theta - leg->rod.dPhi0, -1.2f, 1.2f);
        }
        swing = (1.0f - stand_balance_blend) * stand_support_blend * support_swing
                - stand_balance_blend * CHASSIS.lqr.wbr_u[2U + i];

        swing_gain = fmaxf(fabsf(leg->J[1][0]), fabsf(leg->J[1][1]));
        if (swing_gain > 1.0e-6f)
        {
            float max_swing = joint_torque_limit / swing_gain;
            swing = fp32_constrain(swing, -max_swing, max_swing);
        }
        chassis_stand_requested_f0[i] = force;
        CHASSIS.cmd.leg[i].rod.F = LimitStandLegForce(force, swing, leg->J, joint_torque_limit);
        if (fabsf(CHASSIS.cmd.leg[i].rod.F - force) > 0.01f)
        {
            chassis_stand_force_limited_mask |= 1U << i;
        }
        CHASSIS.cmd.leg[i].rod.Tp = swing;
        CalcVmc(CHASSIS.cmd.leg[i].rod.F, swing, leg->J, CHASSIS.cmd.leg[i].joint.T);
    }
}

static void StopStand(uint8_t fault)
{
    if (chassis_stand_mode == STAND_BALANCE || chassis_stand_mode == STAND_FIXED_BALANCE)
    {
        chassis_stand_balance_trace.exit_fault = fault;
    }
    if (fault != STAND_OK)
    {
        chassis_stand_fault = fault;
        chassis_stand_request = STAND_OFF;
    }
    chassis_stand_mode = STAND_OFF;
    chassis_stand_rise_phase = STAND_RISE_IDLE;
    stand_support_blend = 0.0f;
    stand_balance_blend = 0.0f;
    chassis_stand_force_limited_mask = 0U;
    memset(&CHASSIS.cmd, 0, sizeof(CHASSIS.cmd));
    for (uint8_t i = 0U; i < 2U; i++)
    {
        chassis_stand_requested_f0[i] = 0.0f;
        CHASSIS.wheel_motor[i].set.tor = 0.0f;
    }
    for (uint8_t i = 0U; i < 4U; i++)
    {
        CHASSIS.joint_motor[i].set.tor = 0.0f;
    }
}

static uint16_t StandPoseFaultMask(const StandPoseLimits_s *limits)
{
    uint16_t mask = 0U;

    if (!(fabsf(CHASSIS.fdb.body.phi) <= limits->body_phi_rad))
    {
        mask |= STAND_POSE_BODY_PHI;
    }
    if (!(fabsf(CHASSIS.fdb.body.pitch) <= 0.1745f))
    {
        mask |= STAND_POSE_BODY_ROLL;
    }

    for (uint8_t i = 0U; i < 2U; i++)
    {
        const Leg_t *leg = &CHASSIS.fdb.leg[i];
        float determinant = leg->J[0][0] * leg->J[1][1] - leg->J[0][1] * leg->J[1][0];
        if (!(fabsf(leg->rod.Theta) <= limits->leg_theta_rad))
        {
            mask |= (uint16_t)(STAND_POSE_LEFT_ANGLE << i);
        }
        if (!(leg->rod.L0 >= MIN_LEG_LENGTH && leg->rod.L0 <= MAX_LEG_LENGTH))
        {
            mask |= (uint16_t)(STAND_POSE_LEFT_LENGTH << i);
        }
        if (!isfinite(determinant) || fabsf(determinant) <= 1.0e-5f)
        {
            mask |= (uint16_t)(STAND_POSE_LEFT_JACOBIAN << i);
        }
        if (!(isfinite(leg->rod.dL0) && isfinite(leg->rod.dPhi0) &&
              isfinite(leg->rod.F_spring) && isfinite(CHASSIS.ref.rod_L0[i])))
        {
            mask |= (uint16_t)(STAND_POSE_LEFT_DATA << i);
        }
    }
    return mask;
}

static bool StandSupportHoldRateValid(void)
{
    return fabsf(CHASSIS.fdb.body.phi_dot) <= stand_support_hold_body_rate_limit_rad_s &&
           fabsf(CHASSIS.fdb.body.pitch_dot) <= stand_support_hold_body_rate_limit_rad_s;
}

static void CaptureStandPoseFault(uint16_t mask, uint8_t request, bool entering_balance,
                                  const StandPoseLimits_s *limits)
{
    volatile StandPoseSnapshot_s *snapshot = &chassis_stand_pose_snapshot;
    snapshot->tick = osKernelGetTickCount();
    snapshot->reason_mask = mask;
    snapshot->request = request;
    snapshot->mode = chassis_stand_mode;
    snapshot->entering_balance = entering_balance;
    snapshot->body_phi_rad = CHASSIS.fdb.body.phi;
    snapshot->body_roll_rad = CHASSIS.fdb.body.pitch;
    for (uint8_t i = 0U; i < 2U; i++)
    {
        snapshot->leg_theta_rad[i] = CHASSIS.fdb.leg[i].rod.Theta;
        snapshot->leg_length_m[i] = CHASSIS.fdb.leg[i].rod.L0;
    }
    snapshot->wheel_current_cmd[0] = CHASSIS.wheel_motor[1].set.curr;
    snapshot->wheel_current_cmd[1] = CHASSIS.wheel_motor[0].set.curr;
    snapshot->body_phi_limit_rad = limits->body_phi_rad;
    snapshot->leg_theta_limit_rad = limits->leg_theta_rad;
    snapshot->position_error_m = chassis_stand_position_error_m;
    snapshot->position_error_used_m = chassis_stand_position_error_used_m;
    snapshot->sequence++;
}

static void HoldStandLegLengths(void)
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        CHASSIS.ref.rod_L0[i] = CHASSIS.fdb.leg[i].rod.L0;
    }
}

static void CaptureStandRiseTimeout(uint32_t now)
{
    volatile StandRiseTimeoutSnapshot_s *snapshot = &chassis_stand_rise_timeout_snapshot;
    snapshot->tick = now;
    snapshot->phase = chassis_stand_rise_phase;
    snapshot->force_limited_mask = chassis_stand_force_limited_mask;

    /* The command fields still hold the last rising tick, before support takes over. */
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        const Leg_t *leg = &CHASSIS.fdb.leg[i];
        volatile StandRiseLegSnapshot_s *sample = &snapshot->leg[i];
        sample->length_ref_m = CHASSIS.ref.rod_L0[i];
        sample->length_m = leg->rod.L0;
        sample->angle_ref_rad = CHASSIS.ref.rod_Angle[i];
        sample->angle_rad = leg->rod.Theta;
        sample->height_gain_m = chassis_stand_rise_height_gain_m[i];
        sample->axial_force_request_n = chassis_stand_requested_f0[i];
        sample->axial_force_cmd_n = CHASSIS.cmd.leg[i].rod.F;
        sample->axial_force_feedback_n = leg->rod.F;
        sample->spring_force_n = leg->rod.F_spring;
        sample->swing_cmd_nm = CHASSIS.cmd.leg[i].rod.Tp;
        sample->swing_feedback_nm = leg->rod.Tp;
    }
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        snapshot->joint_torque_cmd_nm[i] = CHASSIS.joint_motor[i].set.tor;
        snapshot->joint_torque_feedback_nm[i] = CHASSIS.joint_motor[i].fdb.tor;
    }
    snapshot->sequence++;
}

static void BeginStandRise(uint32_t now)
{
    chassis_stand_rise_phase = STAND_RISE_LIFT;
    chassis_stand_rise_timeout_phase = STAND_RISE_IDLE;
    stand_rise_phase_tick = now;
    stand_rise_ready_s = 0.0f;
    HoldStandLegLengths();
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        const Leg_t *leg = &CHASSIS.fdb.leg[i];
        stand_rise_start_height_m[i] = leg->rod.L0 * cosf(leg->rod.Theta);
        CHASSIS.ref.rod_Angle[i] = leg->rod.Theta;
        chassis_stand_rise_height_gain_m[i] = 0.0f;
    }
    chassis_stand_fault = STAND_OK;
}

static bool UpdateStandRise(uint32_t now)
{
    const float step_s = fp32_constrain(dt_s, 0.0f, 0.005f);
    const float theta_step = stand_rise_theta_rate_rad_s * step_s;
    const bool recentering = chassis_stand_rise_phase == STAND_RISE_RECENTER;
    float projection[2];
    bool lifted = stand_support_blend >= 1.0f;
    bool legs_upright = true;

    for (uint8_t i = 0U; i < 2U; ++i)
    {
        const Leg_t *leg = &CHASSIS.fdb.leg[i];
        projection[i] = cosf(leg->rod.Theta);
        float lift = leg->rod.L0 * projection[i] - stand_rise_start_height_m[i];
        chassis_stand_rise_height_gain_m[i] = lift;
        lifted = lifted && lift >= stand_rise_min_lift_m;
        legs_upright = legs_upright && fabsf(leg->rod.Theta) <= stand_rise_theta_limit_rad;
    }

    for (uint8_t i = 0U; i < 2U; ++i)
    {
        const float height_target = stand_rise_start_height_m[i] + stand_rise_height_target_m;
        float length_target = height_target / projection[i];
        if (recentering && lifted)
        {
            CHASSIS.ref.rod_Angle[i] -= fp32_constrain(CHASSIS.ref.rod_Angle[i],
                                                      -theta_step, theta_step);
            /* Shorten with the planned angle, preserving clearance at the measured angle. */
            float min_length = (stand_rise_start_height_m[i] + stand_rise_min_lift_m) / projection[i];
            length_target = fmaxf(height_target / cosf(CHASSIS.ref.rod_Angle[i]), min_length);
        }
        length_target = fp32_constrain(length_target, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
        float retract_step = recentering && lifted ? -stand_rise_retract_rate_mps * step_s : 0.0f;
        CHASSIS.ref.rod_L0[i] += fp32_constrain(length_target - CHASSIS.ref.rod_L0[i],
                                               retract_step, stand_rise_extend_rate_mps * step_s);
    }

    bool ready = lifted;
    if (recentering)
    {
        ready = ready && legs_upright &&
                StandPoseFaultMask(&stand_pose_limits[STAND_POSE_ENTERING]) == 0U;
    }
    stand_rise_ready_s = ready ? stand_rise_ready_s + step_s : 0.0f;
    if (stand_rise_ready_s < stand_rise_ready_time_s)
    {
        return false;
    }
    if (recentering)
    {
        return true;
    }

    chassis_stand_rise_phase = STAND_RISE_RECENTER;
    stand_rise_phase_tick = now;
    stand_rise_ready_s = 0.0f;
    return false;
}

static void ConsoleNormal(void)
{
    uint8_t request = chassis_stand_request;
    bool fixed_balance = request == STAND_FIXED_BALANCE;
    bool holding_support = fixed_balance && chassis_stand_mode == STAND_FIXED_SUPPORT_HOLD;
    uint8_t next_mode = holding_support ? STAND_FIXED_SUPPORT_HOLD : request;
    float support_force = chassis_stand_support_n;
    bool balance_requested = request == STAND_BALANCE || fixed_balance;
    bool entering_balance = balance_requested && !holding_support && chassis_stand_mode != request;
    /* The fixed-height test and its support fallback use the rated joint torque. */
    float joint_torque_limit = fixed_balance ? fminf(MAX_JOINT_TOR, 34.0f) : MAX_JOINT_TOR;
    uint8_t pose_phase = STAND_POSE_RUNNING;
    const StandPoseLimits_s *pose_limits;
    uint16_t pose_fault;

    CalcLQR(fixed_balance);
    if (request == STAND_OFF)
    {
        StopStand(STAND_OK);
        return;
    }
    if ((request != STAND_SUPPORT && !balance_requested) ||
        !isfinite(support_force) || support_force < 0.0f)
    {
        StopStand(STAND_INVALID_REQUEST);
        return;
    }
    if (g_chassis_debug.enabled_mask != 0x0FU ||
        (balance_requested &&
         (CHASSIS.wheel_motor[0].offline || CHASSIS.wheel_motor[1].offline)))
    {
        StopStand(STAND_MOTOR_OFFLINE);
        return;
    }
    if (!CHASSIS.lqr.wbr_lqr_valid)
    {
        StopStand(STAND_LQR_INVALID);
        return;
    }
		///////////////
    if (fixed_balance && entering_balance)
    {
        if (chassis_stand_mode != STAND_SUPPORT || stand_support_blend < 1.0f)
        {
            StopStand(STAND_FIXED_NOT_READY);
            return;
        }
        pose_phase = STAND_POSE_FIXED_ENTRY;
    }
    else if (request == STAND_SUPPORT || entering_balance || holding_support)
    {
        pose_phase = STAND_POSE_SUPPORTED;
    }
    pose_limits = &stand_pose_limits[pose_phase];
    pose_fault = StandPoseFaultMask(pose_limits);
    if (holding_support && !StandSupportHoldRateValid())
    {
        pose_fault |= STAND_POSE_HOLD_BODY_RATE;
    }
    if (pose_fault != 0U)
    {
        const uint16_t leg_angle_mask = STAND_POSE_LEFT_ANGLE | (STAND_POSE_LEFT_ANGLE << 1U);
        CaptureStandPoseFault(pose_fault, request, entering_balance, pose_limits);
        if (!fixed_balance || chassis_stand_mode != STAND_FIXED_BALANCE ||
            (pose_fault & ~leg_angle_mask) != 0U ||
            StandPoseFaultMask(&stand_pose_limits[STAND_POSE_SUPPORTED]) != 0U ||
            !StandSupportHoldRateValid())
        {
            StopStand(STAND_POSE_OUT_OF_RANGE);
            return;
        }

        /* Only a slow leg-angle excursion can fall back to wheel-free leg support. */
        HoldStandLegLengths();
        chassis_stand_fault = STAND_POSE_OUT_OF_RANGE;
        chassis_stand_balance_trace.exit_fault = STAND_POSE_OUT_OF_RANGE;
        next_mode = STAND_FIXED_SUPPORT_HOLD;
    }

    if (chassis_stand_mode == STAND_OFF)
    {
        HoldStandLegLengths();
        PID_clear(&CHASSIS.pid.roll_angle);
        chassis_stand_fault = STAND_OK;
    }
    if (fixed_balance && entering_balance)
    {
        HoldStandLegLengths();
        chassis_stand_fault = STAND_OK;
    }
    else if (entering_balance)
    {
        uint32_t now = osKernelGetTickCount();
        if (chassis_stand_mode != STAND_RISING)
        {
            BeginStandRise(now);
        }
        if (!UpdateStandRise(now))
        {
            next_mode = STAND_RISING;
            if ((uint32_t)(now - stand_rise_phase_tick) >= stand_rise_phase_timeout_ticks)
            {
                CaptureStandRiseTimeout(now);
                HoldStandLegLengths();
                chassis_stand_rise_timeout_phase = chassis_stand_rise_phase;
                chassis_stand_rise_phase = STAND_RISE_IDLE;
                chassis_stand_request = STAND_SUPPORT;
                chassis_stand_fault = STAND_RISE_TIMEOUT;
                next_mode = STAND_SUPPORT;
            }
        }
    }
    else if (chassis_stand_mode == STAND_RISING)
    {
        HoldStandLegLengths();
        chassis_stand_rise_phase = STAND_RISE_IDLE;
    }

    if (entering_balance && next_mode == request)
    {
        chassis_stand_rise_phase = STAND_RISE_IDLE;
        memset(CHASSIS.lqr.wbr_xref, 0, sizeof(CHASSIS.lqr.wbr_xref));
        CHASSIS.lqr.wbr_xref[0] = CHASSIS.fdb.state.s;
        CHASSIS.lqr.wbr_xref[2] = CHASSIS.fdb.state.yaw;
        CalcLQR(fixed_balance);
    }

    stand_support_blend = fminf(1.0f, stand_support_blend + 0.001f);//软启动斜坡
    stand_balance_blend = (next_mode == STAND_BALANCE || next_mode == STAND_FIXED_BALANCE) ?
                          fminf(1.0f, stand_balance_blend + 0.002f) : 0.0f;
    chassis_stand_mode = next_mode;
    LegTorqueController(support_force, joint_torque_limit);

    CHASSIS.joint_motor[3].set.tor = fp32_constrain(CHASSIS.cmd.leg[0].joint.T[0] * J1_DIRECTION,
                                                  -joint_torque_limit, joint_torque_limit);
    CHASSIS.joint_motor[2].set.tor = fp32_constrain(CHASSIS.cmd.leg[0].joint.T[1] * J0_DIRECTION,
                                                  -joint_torque_limit, joint_torque_limit);
    CHASSIS.joint_motor[0].set.tor = fp32_constrain(CHASSIS.cmd.leg[1].joint.T[0] * J3_DIRECTION,
                                                  -joint_torque_limit, joint_torque_limit);
    CHASSIS.joint_motor[1].set.tor = fp32_constrain(CHASSIS.cmd.leg[1].joint.T[1] * J2_DIRECTION,
                                                  -joint_torque_limit, joint_torque_limit);
    CHASSIS.wheel_motor[1].set.tor = stand_balance_blend * CHASSIS.lqr.wbr_u[0] * W1_DIRECTION;
    CHASSIS.wheel_motor[0].set.tor = stand_balance_blend * CHASSIS.lqr.wbr_u[1] * W0_DIRECTION;
}

void SendJointMotorCmd(void)
{
    DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
    DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
    DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
    DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
}

static int16_t StandWheelCurrent(const Motor_s *motor, float current_count_limit,
                                uint8_t wheel_bit)
{
    const float derate_speed_mps = 1.2f;//0.35
    const float cutoff_speed_mps = 2.0f;//0.65
    float current = fp32_constrain(motor->set.tor / stand_wheel_torque_per_count,
                                   -current_count_limit, current_count_limit);
    float rim_speed = WHEEL_RADIUS * motor->fdb.vel / motor->reduction_ratio;

    if (!isfinite(current) || !isfinite(rim_speed))
    {
        return 0;
    }

    /* Current and speed share the motor sign. Keep braking effort at high speed. */
    if (current * rim_speed > 0.0f && fabsf(rim_speed) > derate_speed_mps)
    {
        float scale = fp32_constrain((cutoff_speed_mps - fabsf(rim_speed)) /
                                     (cutoff_speed_mps - derate_speed_mps), 0.0f, 1.0f);
        current *= scale;
        chassis_stand_wheel_speed_limited_mask |= wheel_bit;
    }

    return (int16_t)current;
}

static void SampleStandBalance(StandBalanceSample_s *sample, uint32_t now, float current_limit)
{
    sample->tick = now;
    sample->blend = stand_balance_blend;
    sample->position_error_m = chassis_stand_position_error_m;
    sample->position_error_used_m = chassis_stand_position_error_used_m;
    sample->speed_mps = CHASSIS.lqr.wbr_x[1];
    sample->speed_error_used_mps = stand_speed_error_used_mps;
    sample->body_phi_rad = CHASSIS.lqr.wbr_x[8];
    sample->body_rate_rad_s = CHASSIS.lqr.wbr_x[9];
    sample->current_limit = current_limit;
    sample->force_limited_mask = chassis_stand_force_limited_mask;
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        const Leg_t *leg = &CHASSIS.fdb.leg[i];
        sample->leg_theta_rad[i] = leg->rod.Theta;
        sample->leg_rate_rad_s[i] = leg->rod.dTheta;
        sample->leg_length_m[i] = leg->rod.L0;
        sample->leg_length_ref_m[i] = CHASSIS.ref.rod_L0[i];
        sample->leg_swing_cmd_nm[i] = CHASSIS.cmd.leg[i].rod.Tp;
        sample->wheel_speed_mps[i] = WHEEL_RADIUS * leg->wheel.Velocity;
        sample->wheel_request_nm[i] = CHASSIS.lqr.wbr_u[i];
        sample->current_cmd[i] = CHASSIS.wheel_motor[1U - i].set.curr;
        sample->current_feedback[i] = CHASSIS.wheel_motor[1U - i].fdb.curr;
        sample->leg_swing_feedback_nm[i] = leg->rod.Tp;
        for (uint8_t pair = 0U; pair < 5U; ++pair)
        {
            uint8_t col = 2U * pair;
            float error = pair == 0U ? sample->position_error_used_m :
                          CHASSIS.lqr.wbr_x[col] - CHASSIS.lqr.wbr_xref[col];
            float rate_error = pair == 0U ? sample->speed_error_used_mps :
                               CHASSIS.lqr.wbr_x[col + 1U] - CHASSIS.lqr.wbr_xref[col + 1U];
            sample->wheel_terms_nm[i][pair] = -CHASSIS.lqr.wbr_K[i][col] * error
                                             -CHASSIS.lqr.wbr_K[i][col + 1U] * rate_error;
        }
    }
}

static void UpdateStandBalanceTrace(float current_limit)
{
    static uint8_t previous_mode = STAND_OFF;
    volatile StandBalanceTrace_s *trace = &chassis_stand_balance_trace;
    uint8_t mode = chassis_stand_mode;
    bool entering = mode != previous_mode;
    previous_mode = mode;
    if (mode != STAND_BALANCE && mode != STAND_FIXED_BALANCE)
    {
        return;
    }

    bool backward = CHASSIS.wheel_motor[1].set.curr * W1_DIRECTION <= -300.0f &&
                    CHASSIS.wheel_motor[0].set.curr * W0_DIRECTION <= -300.0f;
    float torque_limit = current_limit * stand_wheel_torque_per_count;
    uint8_t limited = 0U;
    if (fabsf(CHASSIS.wheel_motor[1].set.tor) > torque_limit)
    {
        limited |= 1U;
    }
    if (fabsf(CHASSIS.wheel_motor[0].set.tor) > torque_limit)
    {
        limited |= 2U;
    }

    uint32_t now = osKernelGetTickCount();
    bool first_backward = backward && (entering || trace->backward_valid == 0U);
    bool first_limit = limited != 0U && (entering || trace->current_limit_mask == 0U);
    /* Keep the last dispatched fixed-height command and its feedback before exit. */
    if (mode != STAND_FIXED_BALANCE && !entering && !first_backward && !first_limit &&
        (uint32_t)(now - trace->recent.tick) < 10U)
    {
        return;
    }

    StandBalanceSample_s sample;
    SampleStandBalance(&sample, now, current_limit);
    trace->recent = sample;
    if (entering)
    {
        trace->entry = sample;
        trace->mode = mode;
        trace->exit_fault = STAND_OK;
        trace->backward_valid = 0U;
        trace->current_limit_mask = 0U;
        trace->sequence++;
    }
    if (first_backward)
    {
        trace->first_backward = sample;
        trace->backward_valid = 1U;
    }
    if (first_limit)
    {
        trace->first_current_limit = sample;
        trace->current_limit_mask = limited;
    }
}
volatile uint8_t chassis_wheel_test_en = 0U;             /* 设为 1 开启轮毂单独测试（仅 STAND_OFF 生效） */
volatile float chassis_wheel_test_tor[2] = {0.27f, 0.72f};
void SendWheelMotorCmd(void)
{
    /* Ramp the actual CAN current ceiling, even if the LQR request is already large. */
    const float current_count_limit = stand_balance_blend * 14000.0f;
    int16_t right_current = 0;
    int16_t left_current = 0;

    chassis_stand_wheel_speed_limited_mask = 0U;
    const bool wheel_test_allowed = chassis_stand_request == STAND_OFF &&
                                    chassis_stand_mode == STAND_OFF &&
                                    chassis_stand_fault == STAND_OK &&
                                    !CHASSIS.wheel_motor[0].offline &&
                                    !CHASSIS.wheel_motor[1].offline;
    if (!wheel_test_allowed)
    {
        chassis_wheel_test_en = 0U;
    }
    if (wheel_test_allowed && chassis_wheel_test_en == 1U)
    {
        const float test_tor_limit = 0.8f; /* N m per wheel in standby. */
        const float test_curr_limit = 2000.0f; /* 单轮测试电流上限 2000 counts */
        float left_tor  = fp32_constrain(chassis_wheel_test_tor[0], -test_tor_limit, test_tor_limit);
        float right_tor = fp32_constrain(chassis_wheel_test_tor[1], -test_tor_limit, test_tor_limit);

        CHASSIS.wheel_motor[1].set.tor = left_tor * W1_DIRECTION;
        CHASSIS.wheel_motor[0].set.tor = right_tor * W0_DIRECTION;
        left_current  = StandWheelCurrent(&CHASSIS.wheel_motor[1], test_curr_limit, 1U << 0);
        right_current = StandWheelCurrent(&CHASSIS.wheel_motor[0], test_curr_limit, 1U << 1);
    }
    else if (g_imu_debug.ready && g_imu_debug.valid_attitude &&
             CHASSIS.lqr.wbr_lqr_valid)
    {
        right_current = StandWheelCurrent(&CHASSIS.wheel_motor[0], current_count_limit, 1U << 1);
        left_current = StandWheelCurrent(&CHASSIS.wheel_motor[1], current_count_limit, 1U << 0);
    }

    CHASSIS.wheel_motor[0].set.curr = right_current;
    CHASSIS.wheel_motor[1].set.curr = left_current;
    CanCmdDjiMotor(&hfdcan2, 0x200, right_current, left_current, 0, 0);
    UpdateStandBalanceTrace(current_count_limit);
}
void ChassisSendCmd(void) 
 {
	      SendJointMotorCmd();
        SendWheelMotorCmd();
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
