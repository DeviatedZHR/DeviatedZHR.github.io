/**
 * @brief 轮足预备模式 - 固定关节角度
 * @file mod_motion_FootPack_Present.c
 * @note 摘自 applications/chassis/mod_motion.c
 */
void FootPack_Present(FootLeg_Angle_t *output)
{
    FootLeg_Angle_t angle;
    angle.theta1 = M_WHEEL_THIGH_ANGLE02PRE + M_WHEEL_THIGH_ANGLE;  // 38.51°
    angle.theta2 = M_WHEEL_CALF_ANGLE02PRE + M_WHEEL_CALF_ANGLE;   // 134.89°

    for (int i = 0; i < WHEELS_NUM; i++)
    {
        output[i].theta1 = angle.theta1;
        output[i].theta2 = angle.theta2;
    }
}
