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
    uint32_t valid_attitude;    /* Fresh attitude available, including warmup preview. */
    uint32_t calibration_samples;
    uint32_t temperature_stable_ms;
    uint32_t calibration_restarts;
    uint32_t rejected_samples;
    float temperature;
    float calibration_temperature;
    uint32_t heater_pwm;        /* PWM compare counts (0..500 while tuning), not degrees C. */
    float heater_duty;          /* 0..1, heater is capped at 5% duty while tuning. */
    float raw_gyro[3];          /* rad/s, before startup bias correction. */
    float gyro_bias[3];         /* rad/s, measured while stationary. */
    uint32_t calibration_temp_rejects;
    uint32_t calibration_accel_rejects;
    uint32_t calibration_gyro_rejects;
    uint32_t calibration_noise_rejects;
    uint32_t calibration_windows; /* Completed 2000-sample windows, pass or fail. */
    uint32_t calibration_last_reject; /* Bit mask: 1=temp, 2=accel, 4=gyro,
                                      * 8=window noise, 16=invalid transaction. */
    float accel_norm;           /* m/s^2; sampling gate is 9.3..10.3. */
    float calibration_gyro_mean[3];   /* rad/s, last COMPLETE window. */
    float calibration_gyro_stddev[3]; /* rad/s, last COMPLETE window; limit=0.003. */
} ImuDebug_s;

extern volatile ImuDebug_s g_imu_debug;
void imu_task(void * argument);

#endif // IMU_TEMP_CTRL_H
