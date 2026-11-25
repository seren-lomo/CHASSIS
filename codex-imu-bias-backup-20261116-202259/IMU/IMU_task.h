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
} ImuDebug_s;

extern volatile ImuDebug_s g_imu_debug;
void imu_task(void * argument);

#endif // IMU_TEMP_CTRL_H
