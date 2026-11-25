#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "motor.h"

extern DmMeasure_s CAN_DM_MEASURE[DM_NUM];
extern DjiMotorMeasure_t CAN_DJI_MEASURE[5];

void DmFdbData(DmMeasure_s *dm_measure, uint8_t *rx_data);
void GetDmFdbData(Motor_s *motor, DmMeasure_s *dm_measure);
void DjiFdbData(DjiMotorMeasure_t *dji_measure, uint8_t *rx_data);
void GetDjiFdbData(Motor_s *p_motor, const DjiMotorMeasure_t *p_dji_motor_measure);
const DjiMotorMeasure_t *GetDjiMotorMeasurePoint(uint8_t can, uint8_t i);
void GetMotorMeasure(Motor_s *p_motor);

#endif /* CAN_RECEIVE_H */
