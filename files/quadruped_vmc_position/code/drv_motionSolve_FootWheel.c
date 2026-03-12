/**
 * @brief 足轮正/逆运动学解算
 * @file drv_motionSolve_FootWheel.c
 * @note 摘自 applications/chassis/drv_motionSolve.c
 * @note LEG_LENGTH=180mm, FootWheelBasicPos 为髋关节相对车体中心坐标
 */

#define LEG_LENGTH 180.0f

// 正运动学: 关节角 → 足端坐标
void FootWheel_InvSolve(FootLeg_Angle_t input[4], m_Vector3_t output[4])
{
    for (int i = 0; i < 4; i++)
    {
        float sign1 = (i >= 2) ? -1 : 1;
        float diff_theta = input[i].theta2 - input[i].theta1;

        output[i].x = FootWheelBasicPos[i][0]
                      + sign1 * LEG_LENGTH * sinf((input[i].theta2 - input[i].theta1) * DAGTORAD)
                      - sign1 * LEG_LENGTH * sinf(input[i].theta1 * DAGTORAD);
        output[i].y = FootWheelBasicPos[i][1];
        output[i].z = FootWheelBasicPos[i][2]
                      - LEG_LENGTH * cosf(-(input[i].theta2 - input[i].theta1) * DAGTORAD)
                      - LEG_LENGTH * cosf(input[i].theta1 * DAGTORAD);
    }
}

// 逆运动学: 足端坐标 → 关节角
void FootWheel_Solve(m_Vector3_t *input, FootLeg_Angle_t *output)
{
    for (int i = 0; i < 4; i++)
    {
        float sign2 = (i >= 2) ? -1 : 1;
        float A = (powf(input[i].z - FootWheelBasicPos[i][2], 2) + powf(input[i].x - FootWheelBasicPos[i][0], 2));
        output[i].theta1 = acosf(sqrtf(A) / (2 * LEG_LENGTH)) * RADTODAG
                         + sign2 * atanf((input[i].x - FootWheelBasicPos[i][0]) / (input[i].z - FootWheelBasicPos[i][2])) * RADTODAG;
        output[i].theta2 = acosf((powf(input[i].z - FootWheelBasicPos[i][2], 2) + powf(input[i].x - FootWheelBasicPos[i][0], 2) - 2 * powf(LEG_LENGTH, 2)) / (2 * LEG_LENGTH * LEG_LENGTH)) * RADTODAG;
    }
}
