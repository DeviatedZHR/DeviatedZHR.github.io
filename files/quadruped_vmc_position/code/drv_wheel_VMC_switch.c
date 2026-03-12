/**
 * @brief 关节电机 VMC(MIT) 与 位置(E-MIT) 模式切换与输出
 * @file drv_wheel_VMC_switch.c
 * @note 摘自 applications/chassis/foot/drv_wheel.c AnglePID_Cal_Thread
 */

// 当 DM_Ctrl_Type == DM_CtrlType_MIT 时执行 VMC 计算
if (DM_Ctrl_Type == DM_CtrlType_MIT)
{
    get_Actual_physical_quantity(Actual_physical_quantity);
    get_Force(Control_volume, Actual_physical_quantity, &VMC_Weighting);
    get_torque(Torque_VMC, Control_volume, Actual_physical_quantity);
}

// 大腿电机输出
switch (DM_Ctrl_Type)
{
case DM_CtrlType_MIT:
    SPI_CANSTransPIDOut_MIT(SPI_CAN1, Torque_VMC);
    break;
case DM_CtrlType_E_MIT:
    SPI_CANSTransPIDOut_E_MIT(SPI_CAN1, &wheel[FOOT_WHEEL_THIGH][0], Vel_Of_Pos, Torq_Of_Pos_Thigh);
    break;
case DM_CtrlType_Position:
    SPI_CANSTransPIDOut_Position(SPI_CAN1, &wheel[FOOT_WHEEL_THIGH][0], Vel_Of_Pos);
    break;
}

// 小腿电机输出 (同上，使用 SPI_CAN2, Torq_Of_Pos_Calf, Vel_Of_Pos*2)
