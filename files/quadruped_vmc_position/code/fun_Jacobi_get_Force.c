/**
 * @brief 计算得到每条腿的 xyz 方向的力 (VMC 核心)
 * @file fun_Jacobi_get_Force.c
 * @note 摘自 applications/chassis/fun_Jacobi.c
 */
void get_Force(Control_volume_t *Control_volume, Actual_physical_quantity_t *Actual_physical_quantity, weighting_t *weighting)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        // pitch 修正高度
        if (i == WHEEL_RF || i == WHEEL_LF)
        {
            Actual_physical_quantity[i].z_set = Actual_physical_quantity[i].height_set
                                              + weighting->kpitch * sinf(Actual_physical_quantity[i].pitch_set - Actual_physical_quantity[i].pitch);
        }
        else if (i == WHEEL_RB || i == WHEEL_LB)
        {
            Actual_physical_quantity[i].z_set = Actual_physical_quantity[i].height_set
                                              - weighting->kpitch * sinf(Actual_physical_quantity[i].pitch_set - Actual_physical_quantity[i].pitch);
        }

        // roll 修正高度
        if (i == WHEEL_RF || i == WHEEL_RB)
            Actual_physical_quantity[i].z_set += weighting->kroll * sinf(Actual_physical_quantity[i].roll_set - Actual_physical_quantity[i].roll);
        if (i == WHEEL_LF || i == WHEEL_LB)
            Actual_physical_quantity[i].z_set -= weighting->kroll * sinf(Actual_physical_quantity[i].roll_set - Actual_physical_quantity[i].roll);

        // X 方向: kx*(x_set-x) + bx*(dx_set-dx)
        Control_volume[i].Fx = weighting->kx * (Actual_physical_quantity[i].x_set - Actual_physical_quantity[i].x)
                             + weighting->bx * (Actual_physical_quantity[i].dx_set - Actual_physical_quantity[i].dx);

        // Y 方向: 含 yaw 耦合
        Control_volume[i].Fy = weighting->ky * (Actual_physical_quantity[i].y_set - Actual_physical_quantity[i].y)
                             + weighting->by * (Actual_physical_quantity[i].dy_set - Actual_physical_quantity[i].dy)
                             + weighting->eta_yaw * (weighting->kyaw * (Actual_physical_quantity[i].yaw_set - Actual_physical_quantity[i].yaw)
                             + weighting->byaw * (Actual_physical_quantity[i].dyaw_set - Actual_physical_quantity[i].dyaw));

        if (i == WHEEL_RB || i == WHEEL_LB)
            Control_volume[i].Fy = -Control_volume[i].Fy;

        // Z 方向: 重力补偿 - kz*(z_set-z) - bz*(0-dz)
        Control_volume[i].Fz = weighting->gravity_compensation
                             - weighting->kz * (Actual_physical_quantity[i].z_set - Actual_physical_quantity[i].z)
                             - weighting->bz * (0 - Actual_physical_quantity[i].dz);
    }
}
