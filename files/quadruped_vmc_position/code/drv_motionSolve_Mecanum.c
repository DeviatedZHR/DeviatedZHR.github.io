/**
 * @brief 麦轮底盘逆/正运动学
 * @file drv_motionSolve_Mecanum.c
 * @note 摘自 applications/chassis/drv_motionSolve.c
 * @note 坐标系: x 右手, y 正前, ω 逆时针为正
 */

#define ROOT2 1.41421f

// 逆运动学: (Vx,Vy,ω) → 四轮线速度 mm/s, 顺序 RF-LF-LB-RB
void MecanOmni_Resolve(Mot_base_t target_xyw, float output[])
{
    output[0] = ROOT2 * (target_xyw.vel.y - target_xyw.vel.x) + target_xyw.angvel * coefficient;
    output[1] = ROOT2 * (target_xyw.vel.y + target_xyw.vel.x) - target_xyw.angvel * coefficient;
    output[2] = ROOT2 * (target_xyw.vel.y - target_xyw.vel.x) - target_xyw.angvel * coefficient;
    output[3] = ROOT2 * (target_xyw.vel.y + target_xyw.vel.x) + target_xyw.angvel * coefficient;
}

// 正运动学: 四轮编码器 rpm → (vx, vy, ω)
void fromEncoderGetV(const float encoderData[4], Mot_base_t *real_xyw)
{
    float data_temp[4];
    for (int i = 0; i < 4; i++)
        data_temp[i] = encoderData[i] * RPMTORAD / REDUCT_RATIO * WCONVERT2V_k;

    real_xyw->vel.y = (data_temp[WHEEL_RF] + data_temp[WHEEL_LF] + data_temp[WHEEL_LB] + data_temp[WHEEL_RB]) / ROOT2 / 4.0f;
    real_xyw->vel.x = (-data_temp[WHEEL_RF] + data_temp[WHEEL_LF] - data_temp[WHEEL_LB] + data_temp[WHEEL_RB]) / ROOT2 / 4.0f;
    real_xyw->angvel = (data_temp[WHEEL_RF] - data_temp[WHEEL_LF] - data_temp[WHEEL_LB] + data_temp[WHEEL_RB]) / 4.0f / coefficient;
}
