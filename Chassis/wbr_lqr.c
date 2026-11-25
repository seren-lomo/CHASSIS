#include "wbr_lqr.h"
#include "wbr_lqr_table_1ms.h"
#include <math.h>

/* Find the interval and interpolation ratio. */
static int FindInterval(const float *grid, int n,
                        float value, float *ratio)
{
    int i;

    if (value <= grid[0]) {
        *ratio = 0.0f;
        return 0;
    }

    if (value >= grid[n - 1]) {
        *ratio = 1.0f;
        return n - 2;
    }

    for (i = 0; i < n - 1; i++) {
        if (value <= grid[i + 1]) {
            *ratio = (value - grid[i])
                   / (grid[i + 1] - grid[i]);
            return i;
        }
    }

    *ratio = 0.0f;
    return 0;
}

int WbrLqrStep(float Ll, float Lr, const float x_error[10],
               float K[4][10], float u[4],
               float common_leg_wheel_scale, float body_wheel_scale,
               float leg_pitch_rate_damping)
{
    int il, ir, row, col;
    float a, b;
    float w00, w10, w01, w11;
    float value, limit;

    for (row = 0; row < 4; row++)
        u[row] = 0.0f;

    if (!isfinite(Ll) || !isfinite(Lr))//检查  无穷大（!isfinite()
        return 0;

    for (col = 0; col < 10; col++) {
        if (!isfinite(x_error[col]))
            return 0;
    }

    /*
     * Reject lengths outside the gain table for this firmware interface.
     * MATLAB lookup clamps to the boundary instead.
     */
    if (Ll < wbr_lqr_left_grid[0] ||
        Ll > wbr_lqr_left_grid[WBR_LQR_NL - 1] ||
        Lr < wbr_lqr_right_grid[0] ||
        Lr > wbr_lqr_right_grid[WBR_LQR_NR - 1])
        return 0;

    il = FindInterval(wbr_lqr_left_grid,
                      WBR_LQR_NL, Ll, &a);//获取左腿索引/比例
    ir = FindInterval(wbr_lqr_right_grid,
                      WBR_LQR_NR, Lr, &b);//右腿索引/比例

    w00 = (1.0f - a) * (1.0f - b);
    w10 = a * (1.0f - b);
    w01 = (1.0f - a) * b;
    w11 = a * b;

 for (row = 0; row < 4; row++) {
    for (col = 0; col < 10; col++) {
        K[row][col] =
            w00 * wbr_lqr_k[il    ][ir    ][row][col] +
            w10 * wbr_lqr_k[il + 1][ir    ][row][col] +
            w01 * wbr_lqr_k[il    ][ir + 1][row][col] +
            w11 * wbr_lqr_k[il + 1][ir + 1][row][col];
    }
}

const float common_leg_gain =
    0.25f * (K[0][4] + K[0][6] + K[1][4] + K[1][6]);
const float common_leg_delta =
    (common_leg_wheel_scale - 1.0f) * common_leg_gain;

for (row = 0; row < 2; row++) {
    K[row][4] += common_leg_delta;  // 左腿角
    K[row][6] += common_leg_delta;  // 右腿角
    K[row][8] *= body_wheel_scale; // 车身俯仰角
    K[row][9] *= body_wheel_scale; // 车身俯仰角速度
}

/* Tp = -Tb, so this adds -damping * pitch_rate_error to each leg. */
K[2][9] -= leg_pitch_rate_damping;
K[3][9] -= leg_pitch_rate_damping;

for (row = 0; row < 4; row++) {
    value = 0.0f;
    for (col = 0; col < 10; col++) {
        value -= K[row][col] * x_error[col];
    }

    limit = wbr_lqr_torque_limit[row];
    if (value > limit)
        value = limit;
    else if (value < -limit)
        value = -limit;

    u[row] = value;
}

    return 1;
}

