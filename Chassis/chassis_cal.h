#ifndef CHASSIS_CAL_H
#define CHASSIS_CAL_H

//physical parameters ---------------------
#define LEG_L1 (0.210f)  // (m)腿1长度
#define LEG_L2 (0.250f)  // (m)腿2长度
#define LEG_L3 (LEG_L2)  // (m)腿3长度
#define LEG_L4 (LEG_L1)  // (m)腿4长度
#define LEG_L5 (0.0f)    // (m)关节间距


extern void GetL0AndPhi0(float phi1, float phi4, float L0_Phi0[2]);

extern void GetdL0AnddPhi0(float J[2][2], float d_phi1, float d_phi4, float dL0_dPhi0[2]);

extern void GetLegForce(float J[2][2], float T1, float T2, float F[2]);

extern void CalcJacobian(float phi1, float phi4, float J[2][2]);

extern void CalcVmc(float F0, float Tp, float J[2][2], float T[2]);

extern void CalcPhi1AndPhi4(float phi0, float l0, float phi1_phi4[2]);
extern float LegController_CalcSpringForce(float L0);

#endif  