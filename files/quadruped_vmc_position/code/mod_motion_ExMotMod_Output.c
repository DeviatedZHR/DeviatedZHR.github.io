/**
 * @brief 统一运动输出接口 - 麦轮速度 + 足轮角度
 * @file mod_motion_ExMotMod_Output.c
 * @note 摘自 applications/chassis/mod_motion.c
 */
void ExMotMod_Output(const Ctrl_data_t *const ctrlData)
{
    Mot_base_t tarSpeed = ctrlData->xyw;
    FootLeg_Angle_t tarAngle[WHEELS_NUM];

    // 角度零位转换
    for (int i = 0; i < WHEELS_NUM; ++i)
    {
        tarAngle[i].theta1 = ctrlData->Angle_Data[i].theta1 - M_WHEEL_THIGH_ANGLE;
        tarAngle[i].theta2 = M_WHEEL_CALF_ANGLE - ctrlData->Angle_Data[i].theta2;
        tarAngle[i].theta1 = range_angle_180(tarAngle[i].theta1);
        tarAngle[i].theta2 = range_angle_180(tarAngle[i].theta2);
        if (tarAngle[i].theta2 <= 0.0f)
            tarAngle[i].theta2 = 0.0f;
    }

    // 速度平滑
    CtrlAccClampManage(&tarSpeed);

    // 麦轮逆解算
    MotModule_Resolve(tarSpeed, motorSetData);  // MecanOmni_Resolve

    // 足轮角度写入
    for (rt_uint8_t lo = 0; lo < WHEELS_NUM; lo++)
    {
        motorSetData2[FOOT_WHEEL_THIGH][lo] = -tarAngle[lo].theta1;
        motorSetData2[FOOT_WHEEL_CALF][lo] = tarAngle[lo].theta2;
    }

    // 输出到电机
    for (rt_uint8_t lo = 0; lo < WHEELS_NUM; lo++)
    {
        Wheel_Angle_Set(FOOT_WHEEL_THIGH, lo, motorSetData2[FOOT_WHEEL_THIGH][lo]);
        Wheel_Angle_Set(FOOT_WHEEL_CALF, lo, motorSetData2[FOOT_WHEEL_CALF][lo]);
        Wheel_Speed_Set(FOOT_WHEEL_RUN, lo, motorSetData[lo]);
    }
}
