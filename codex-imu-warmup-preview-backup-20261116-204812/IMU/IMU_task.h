#ifndef IMU_TASK_H
#define IMU_TASK_H
#include "main.h"
#include "bsp_task_timing.h"

typedef struct
{
    TaskTiming_s timing;
    uint32_t ready;
    uint32_t init_attempts;
    uint32_t init_error;        /* BMI088_init return code; zero means success. */
    uint32_t calibration_state; /* 0=warmup, 1=stationary sampling, 2=complete. */
    uint32_t calibration_done;
    uint32_t valid_attitude;
    uint32_t calibration_samples;
    uint32_t temperature_stable_ms;
    uint32_t calibration_restarts;
    uint32_t rejected_samples;
    float temperature;
    float calibration_temperature;
    float heater_duty;          /* 0..1, heater is capped at original 20% duty. */
    float raw_gyro[3];          /* rad/s, before startup bias correction. */
    float gyro_bias[3];         /* rad/s, measured while stationary. */
} ImuDebug_s;

extern volatile ImuDebug_s g_imu_debug;
void imu_task(void * argument);

#endif // IMU_TEMP_CTRL_H
