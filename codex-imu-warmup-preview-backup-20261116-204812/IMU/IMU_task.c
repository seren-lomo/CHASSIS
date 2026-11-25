#include "main.h"
#include "cmsis_os2.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "IMU_task.h"
#include "MahonyAHRS.h"
#include "pid.h"
#include "IMU_task.h"
#include "chassis_task.h"
/* Calibration requires the robot to remain still at its operating temperature. */
#define IMU_BIAS_SAMPLES 2000U
#define IMU_TEMP_STABLE_MS 3000U
#define IMU_TEMP_TOLERANCE 0.5f
#define IMU_CAL_MAX_GYRO 0.03f       /* rad/s: rejects obvious motion, not all slow turns. */
#define IMU_CAL_MAX_VARIANCE 9.0e-6f /* (rad/s)^2: reject noisy calibration windows. */
pid_type_def Temperature_PID={0};
float Temperature_PID_Para[3]={1600,50,40};
extern Chassis_s CHASSIS;
volatile ImuDebug_s g_imu_debug;
float gyro[3], accel[3], temp;
float gyro_correct[3]={0};
float RefTemp = 40;   //Destination
float roll,pitch,yaw=0;
uint8_t attitude_flag=0; /* 0=warmup, 1=stationary sampling, 2=calibrated. */
uint32_t correct_times=0;


void INS_Init(void)
{
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf初始化
		PID_init(&Temperature_PID, PID_POSITION,Temperature_PID_Para,2000,200); //加热pidlimit
		Mahony_Init(1000);  //mahony姿态解算初始化
    // imu heat init
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    do
    {
        g_imu_debug.init_attempts++;
        g_imu_debug.init_error = BMI088_init();
        if (g_imu_debug.init_error != BMI088_NO_ERROR)
            osDelay(100U);
    } while (g_imu_debug.init_error != BMI088_NO_ERROR);
    attitude_flag = 0U;
    correct_times = 0U;
    g_imu_debug.calibration_done = 0U;
    g_imu_debug.ready = 1U;
}

uint32_t temp_temperature=0;
void IMU_Temperature_Ctrl(){
	  PID_calc(&Temperature_PID, temp, RefTemp); //温度pid  //需要调一下pid使得温度在40°左右
		temp_temperature=(uint32_t)Temperature_PID.out; 
		if(Temperature_PID.out<0)
		{
			temp_temperature=0;
		}
    htim3.Instance->CCR4 = temp_temperature;
}
/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 任务参数
 * @retval: void
 * @details: IMU姿态控制任务函数
*/
/* The first valid attitude is published only after the startup calibration.
 * Raw sensor values/temperature remain visible while warming and sampling.
 */
void INS_Task(void)
{
    static uint32_t count;
    static uint32_t stable_since;
    static uint8_t temp_tracking;
    static float bias_sum[3];
    static float bias_square_sum[3];
    static uint8_t first_attitude = 1U;
    uint32_t now = HAL_GetTick();
    uint32_t spi_errors = g_bmi088_debug.spi_error_count;
    uint8_t i;
    uint8_t sample_valid;
    uint8_t temperature_ok;
    float accel_norm_sq;

    BMI088_read(gyro, accel, &temp);
    accel_norm_sq = accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2];
    sample_valid = (g_bmi088_debug.spi_error_count == spi_errors &&
                    accel_norm_sq > 1.0f && accel_norm_sq < 40000.0f &&
                    temp > -40.0f && temp < 85.0f);
    if (!sample_valid)
    {
        /* Do not collect a bias or integrate failed sensor transactions. */
        g_imu_debug.rejected_samples++;
        g_imu_debug.temperature_stable_ms = 0U;
        g_imu_debug.valid_attitude = 0U;
        temp_tracking = 0U;
        if (attitude_flag != 2U)
        {
            attitude_flag = 0U;
            correct_times = 0U;
        }
        htim3.Instance->CCR4 = 0U;
        g_imu_debug.heater_duty = 0.0f;
        return;
    }
    g_imu_debug.temperature = temp;
    if ((count++ % 10U) == 0U)
        IMU_Temperature_Ctrl();
    g_imu_debug.heater_duty = (float)htim3.Instance->CCR4 /
                              ((float)htim3.Instance->ARR + 1.0f);
    for (i = 0U; i < 3U; i++)
    {
        g_imu_debug.raw_gyro[i] = gyro[i];
        CHASSIS.imu->accel[i] = accel[i];
        CHASSIS.imu->gyro[i] = gyro[i] - gyro_correct[i];
    }

    temperature_ok = (fabsf(temp - RefTemp) <= IMU_TEMP_TOLERANCE);
    if (attitude_flag == 0U)
    {
        if (temperature_ok)
        {
            if (!temp_tracking)
            {
                stable_since = now;
                temp_tracking = 1U;
            }
            g_imu_debug.temperature_stable_ms = (uint32_t)(now - stable_since);
            if (g_imu_debug.temperature_stable_ms >= IMU_TEMP_STABLE_MS)
            {
                for (i = 0U; i < 3U; i++)
                    bias_sum[i] = bias_square_sum[i] = 0.0f;
                correct_times = 0U;
                attitude_flag = 1U;
            }
        }
        else
        {
            temp_tracking = 0U;
            g_imu_debug.temperature_stable_ms = 0U;
        }
    }
    else if (attitude_flag == 1U)
    {
        uint8_t stationary = (accel_norm_sq > 9.3f*9.3f &&
                              accel_norm_sq < 10.3f*10.3f);
        for (i = 0U; i < 3U; i++)
            if (fabsf(gyro[i]) > IMU_CAL_MAX_GYRO)
                stationary = 0U;
        if (!temperature_ok || !stationary)
        {
            for (i = 0U; i < 3U; i++)
                bias_sum[i] = bias_square_sum[i] = 0.0f;
            correct_times = 0U;
            g_imu_debug.calibration_restarts++;
            if (!temperature_ok)
            {
                attitude_flag = 0U;
                temp_tracking = 0U;
                g_imu_debug.temperature_stable_ms = 0U;
            }
        }
        else
        {
            for (i = 0U; i < 3U; i++)
            {
                bias_sum[i] += gyro[i];
                bias_square_sum[i] += gyro[i]*gyro[i];
            }
            correct_times++;
            if (correct_times >= IMU_BIAS_SAMPLES)
            {
                uint8_t accepted = 1U;
                for (i = 0U; i < 3U; i++)
                {
                    float mean = bias_sum[i] / (float)correct_times;
                    float variance = bias_square_sum[i] / (float)correct_times - mean*mean;
                    if (variance > IMU_CAL_MAX_VARIANCE)
                        accepted = 0U;
                }
                if (accepted)
                {
                    for (i = 0U; i < 3U; i++)
                    {
                        gyro_correct[i] = bias_sum[i] / (float)correct_times;
                        g_imu_debug.gyro_bias[i] = gyro_correct[i];
                    }
                    g_imu_debug.calibration_temperature = temp;
                    g_imu_debug.calibration_done = 1U;
                    attitude_flag = 2U;
                }
                else
                {
                    for (i = 0U; i < 3U; i++)
                        bias_sum[i] = bias_square_sum[i] = 0.0f;
                    correct_times = 0U;
                    g_imu_debug.calibration_restarts++;
                }
            }
        }
    }
    g_imu_debug.calibration_state = attitude_flag;
    g_imu_debug.calibration_samples = correct_times;
    if (attitude_flag != 2U)
        return;

    for (i = 0U; i < 3U; i++)
        gyro[i] -= gyro_correct[i];
    /* No artificial dead zone: retain real slow turns and residual noise. */
    if (first_attitude)
    {
        MahonyAHRSinit(accel[0], accel[1], accel[2], 0.0f, 0.0f, 0.0f);
        first_attitude = 0U;
    }
    IMU_QuaternionEKF_Update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
    Mahony_update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0, 0, 0);
    Mahony_computeAngles();
    pitch = Get_Pitch() / 57.29578f;
    roll = Get_Roll() / 57.29578f;
    yaw = Get_Yaw() / 57.29578f;
    CHASSIS.imu->angle[0] = roll;
    CHASSIS.imu->angle[1] = pitch;
    CHASSIS.imu->angle[2] = yaw;
    for (i = 0U; i < 3U; i++)
        CHASSIS.imu->gyro[i] = gyro[i];
    g_imu_debug.valid_attitude = 1U;
}


void imu_task(void *argument)
{
    osThreadId_t self = osThreadGetId();
    osPriority_t run_priority = osThreadGetPriority(self);
    uint32_t next_tick;
    (void)argument;
    /* Sensor reset/register checks must not block the chassis startup. */
    osThreadSetPriority(self, osPriorityNormal);
    INS_Init();
    osThreadSetPriority(self, run_priority);
    next_tick = osKernelGetTickCount();
    for (;;)
    {
        TaskTimingBegin(&g_imu_debug.timing);
        INS_Task();
        TaskTimingEnd(&g_imu_debug.timing);
        TaskWaitNext1ms(&next_tick, &g_imu_debug.timing);
    }
}


/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}
