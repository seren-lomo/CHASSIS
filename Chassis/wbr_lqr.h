#ifndef WBR_LQR_H
#define WBR_LQR_H


/*
 * x_error (state minus reference, with any caller-side limits applied):
 * s ds yaw dyaw thetaL dthetaL thetaR dthetaR bodyPitch bodyPitchRate
 *
 * u:
 * TwL TwR TbL TbR
 *
 * leg_pitch_rate_damping is the added damping per leg in N m per rad/s.
 *
 * Returns 1 on success, 0 for invalid input.
 */

int WbrLqrStep(float Ll, float Lr, const float x_error[10],
               float K[4][10], float u[4],
               float common_leg_wheel_scale, float body_wheel_scale,
               float leg_pitch_rate_damping);

#endif
