/**
 * @brief 通过雅可比矩阵将足端力映射为关节力矩
 * @file fun_Jacobi_get_torque.c
 * @note 摘自 applications/chassis/fun_Jacobi.c
 * @note L1=L2=0.18m, theta1=roll(0), theta2=大腿, theta3=小腿
 */
void get_torque(Torque_t Torque[4], Control_volume_t *Control_volume, Actual_physical_quantity_t *Actual_physical_quantity)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        theta_f1 = M_WHEEL_THIGH_ANGLE - Read_DM_wheel_data_Angle(FOOT_WHEEL_THIGH, i);
        theta_f2 = M_WHEEL_CALF_ANGLE - Read_DM_wheel_data_Angle(FOOT_WHEEL_CALF, i);

        float theta1 = 0 * DEG_TO_RAD;
        float theta2 = theta_f1 * DEG_TO_RAD;
        float theta3 = -theta_f2 * DEG_TO_RAD;

        float s1 = sinf(theta1), c1 = cosf(theta1);
        float s2 = sinf(theta2), c2 = cosf(theta2);
        float s3 = sinf(theta3), c3 = cosf(theta3);
        float c23 = cosf(theta2 + theta3);
        float s23 = sinf(theta2 + theta3);

        // 雅可比矩阵 J (3x3)
        Ji[0][0] = c1 * (L1 * c2 + L2 * c23);
        Ji[0][1] = -s1 * (L1 * s2 + L2 * s23);
        Ji[0][2] = -s1 * L2 * s23;
        Ji[1][0] = 0;
        Ji[1][1] = L1 * c2 + L2 * c23;
        Ji[1][2] = L2 * c23;
        Ji[2][0] = s1 * (L1 * c2 + L2 * c23);
        Ji[2][1] = c1 * (L1 * s2 + L2 * s23);
        Ji[2][2] = c1 * L2 * s23;

        // τ = -J^T * F
        Torque[i].T1 = -Ji[0][0] * Control_volume[i].Fx - Ji[1][0] * Control_volume[i].Fy - Ji[2][0] * Control_volume[i].Fz;
        Torque[i].T2 = -Ji[0][1] * Control_volume[i].Fx - Ji[1][1] * Control_volume[i].Fy - Ji[2][1] * Control_volume[i].Fz;
        Torque[i].T3 = -Ji[0][2] * Control_volume[i].Fx - Ji[1][2] * Control_volume[i].Fy - Ji[2][2] * Control_volume[i].Fz;

        // 更新足端速度 v = J * ω (用于下一拍力计算)
        int sign_thigh = (i == WHEEL_RF || i == WHEEL_LF) ? 1 : -1;
        int sign_calf  = (i == WHEEL_RF || i == WHEEL_LF) ? -1 : 1;
        Actual_physical_quantity[i].dx = Ji[0][1] * Read_DM_wheel_data_Speed(FOOT_WHEEL_THIGH, i) + Ji[0][2] * Read_DM_wheel_data_Speed(FOOT_WHEEL_CALF, i);
        Actual_physical_quantity[i].dy = Ji[1][1] * sign_thigh * Read_DM_wheel_data_Speed(FOOT_WHEEL_THIGH, i) + Ji[1][2] * sign_calf * Read_DM_wheel_data_Speed(FOOT_WHEEL_CALF, i);
        Actual_physical_quantity[i].dz = Ji[2][1] * Read_DM_wheel_data_Speed(FOOT_WHEEL_THIGH, i) + Ji[2][2] * Read_DM_wheel_data_Speed(FOOT_WHEEL_CALF, i);
    }
}
