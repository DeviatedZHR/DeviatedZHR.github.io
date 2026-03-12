import mujoco
import mujoco.viewer
import time
import math

# 1. 加载模型
import os
xml_path = os.path.join(os.path.dirname(__file__), 'rotary_pendulum.xml')
# 也可以直接写绝对路径：
# xml_path = r"c:/Users/asus/Desktop/Mujoco/models/rotary_pendulum.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 2. 控制器参数（基线，保守调整）
# 能量起摆增益（略增快）
k_swing = 6.0

# 摆杆直立环（内环，主导）——强增益快速纠偏，匹配 RL 的持续稳定
Kp_pole = 280.0
Kd_pole = 24.0

# 旋臂位置环（外环）——尽早介入、足够强，防止臂漂移导致失稳
Kp_arm = 1.2
Kd_arm = 1.5
arm_gain_scale = 0.7
# 摆杆-臂耦合：摆杆倾时臂主动跟随，实现 RL 般的快速响应
k_pole_arm_coupling = 8.0   # 摆角对臂的耦合增益
k_pole_vel_coupling = 3.0   # 摆角速度对臂的耦合（预判倾倒趋势）

# 3. 初始化状态
mujoco.mj_resetData(model, data)
data.qpos[1] = math.pi  # 摆杆初始下垂

# 辅助函数：角度归一化到 [-pi, pi]，0为最高点
def wrap_pi(x):
    return (x + math.pi) % (2 * math.pi) - math.pi

# 4. 启动可视化界面
with mujoco.viewer.launch_passive(model, data) as viewer:
    step_start = time.time()

    # 模拟 C 代码中的全局/静态变量
    State = 0  # 0: 起摆状态, 1: 倒立平衡状态
    Count_Position = 0
    Flag_qb2 = 0
    Filtered_Arm_Pos = 0.0

    torque_limit = 20.0
    swing_timer = 0

    while viewer.is_running():
        # --- A. 读取状态 ---
        arm_pos = data.qpos[0]
        pole_pos = data.qpos[1]
        arm_vel = data.qvel[0]
        pole_vel = data.qvel[1]

        # 统一角度映射：0 表示竖直向上，±pi 表示下垂
        pole_angle_top = wrap_pi(pole_pos)

        target_torque = 0.0

        # --- B. 状态机 ---
        if State == 0 and abs(pole_angle_top) < 0.15:
            State = 1
        elif State == 1 and abs(pole_angle_top) > 1.5:
            # 放宽切换阈值：1.5 rad 才放弃平衡，给更多恢复机会
            State = 0
            Count_Position = 0
            Flag_qb2 = 0
            Filtered_Arm_Pos = arm_pos

        # --- C. 控制 ---
        if State == 0:
            # 起摆阶段：主要使用速度耦合能量注入，且在接近底部时周期性施加脉冲
            torque_limit = 20.0
            # 连续注能项（避免对小速度做持续小扭矩）
            if abs(pole_vel) > 0.05:
                target_torque = k_swing * pole_vel * math.cos(pole_angle_top)
            else:
                target_torque = 0.0

            # 周期性脉冲：当接近底部且速度很小的时候，施加短脉冲启动振动
            swing_timer += 1
            if swing_timer > 20:
                swing_timer = 0
                if abs(pole_angle_top) > 1.0 and abs(pole_vel) < 0.15:
                    # 脉冲方向基于 -cos(angle)，当在底部(-1)时为正，推动起振
                    pulse = 6.0 * math.copysign(1.0, -math.cos(pole_angle_top))
                    target_torque += pulse

        elif State == 1:
            torque_limit = 15.0  # 与 RL 相当，保证足够纠偏能力
            # 摆杆直立控制（内环）
            torque_from_pole = -(Kp_pole * pole_angle_top + Kd_pole * pole_vel)

            # === 摆杆-臂耦合（核心）：摆杆倾时臂立即跟随，实现 RL 般的快速响应 ===
            # 物理：摆向右倾 → 臂向右转 → 支点右移 → 帮助摆回正
            torque_coupling = k_pole_arm_coupling * pole_angle_top + k_pole_vel_coupling * pole_vel
            coupling_limit = 6.0  # 耦合项单独限幅，避免过大
            torque_coupling = max(-coupling_limit, min(coupling_limit, torque_coupling))

            # 臂环：极早开启，用于抑制臂漂移
            if Flag_qb2 == 0:
                if abs(pole_angle_top) < 0.06:
                    Count_Position += 1
                else:
                    Count_Position = max(0, Count_Position - 1)

                torque_from_arm = 0.0

                if Count_Position > 20 and abs(pole_vel) < 0.08 and abs(arm_vel) < 0.2:
                    Flag_qb2 = 1
                    Count_Position = 0

            # 旋臂位置环（外环）——摆角小时主导，拉回臂中心；摆角大时耦合主导
            torque_from_arm = 0.0
            if Flag_qb2 == 1:
                Filtered_Arm_Pos = Filtered_Arm_Pos * 0.97 + arm_pos * 0.03  # 更快响应

                pos_term = -(Kp_arm * arm_gain_scale * Filtered_Arm_Pos + Kd_arm * arm_gain_scale * arm_vel)
                # 摆角小时位置环强，摆角大时让位给耦合（反向缩放）
                pos_scale = max(0.0, 1.0 - abs(pole_angle_top) / 0.15)
                torque_from_arm = pos_term * pos_scale

                if abs(Filtered_Arm_Pos) < 0.02:
                    torque_from_arm = 0.0
                arm_torque_limit = 3.0
                torque_from_arm = max(-arm_torque_limit, min(arm_torque_limit, torque_from_arm))

            # 总臂贡献 = 耦合（摆倾时快速跟随）+ 位置环（摆稳时拉回中心）
            torque_from_arm_total = torque_coupling + torque_from_arm
            arm_total_limit = 8.0
            torque_from_arm_total = max(-arm_total_limit, min(arm_total_limit, torque_from_arm_total))

            target_torque = torque_from_pole + torque_from_arm_total

            # (已禁用早期 nudge/脉冲逻辑) 若需再用脉冲请告知

        # --- D. 限幅与下发 ---
        target_torque = max(-torque_limit, min(torque_limit, target_torque))
        data.ctrl[0] = target_torque

        # --- E. 步进 ---
        mujoco.mj_step(model, data)
        viewer.sync()

        # --- F. 时间同步 ---
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        step_start = time.time()