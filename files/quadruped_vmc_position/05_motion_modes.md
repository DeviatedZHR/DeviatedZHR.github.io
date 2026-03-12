# 05 运动模式与运动包

## 一、模式是什么？

### 1.1 两层模式

本机有**两层**模式，需要同时考虑：

| 层级 | 作用 | 例子 |
|------|------|------|
| **底盘模式 (motion_mode)** | 车身怎么动：跟不跟云台、是否小陀螺等 | 跟随、不跟随、小陀螺 |
| **足轮模式 (foot_mode)** | 腿怎么摆：高度、姿态、爬坡、下坡等 | 预备、正常、爬坡、下坡 |

两者组合决定最终行为，例如“跟随云台 + 爬坡模式”= 边跟云台边爬坡。

### 1.2 运动包 (FootPack)

**运动包**就是针对每种足轮模式写好的“动作函数”：给定当前状态，输出四腿的目标关节角（或目标足端坐标）。

---

## 二、底盘运动模式 (Motion_mode_e)

| 模式 | 含义 |
|------|------|
| NO_FOLLOW | 不跟随云台，小陀螺时可全向移动 |
| FOLLOW_GIMBAL | 跟随云台，控制方向以云台为参考 |
| SLOW_GYRO / FAST_GYRO | 慢/快陀螺 |
| MOVE_BACK / FOLLOWBACK_GIMBAL | 倒车 / 反向跟随 |
| CHASS_AUTO | 底盘自主（如视觉导航） |

---

## 三、足轮模式与运动包 (Foot_mode_e)

| 模式 | 运动包函数 | 作用 |
|------|------------|------|
| PRESENT_MODE | FootPack_Present | 预备姿态，四腿统一固定角度 |
| NORMAL_MODE | FootPack_Extend | 伸腿到最高，适合平跑 |
| FOOT_JUMP_MODE | FootPack_Jump | 跳跃（给固定力矩序列） |
| AUTO_MODE_1 | FootPack_Auto_1 | 自适应：用 IMU 维持机身水平 |
| AUTO_MODE_2 | FootPack_Auto2_Trot | Trot 对角小跑步态 |
| CLIMB_MODE | FootPack_Climb | 爬台阶（导轮已拆，现等同预备） |
| CLIMB_MODE_2 | FootPack_Climb_2 | 爬 43° 坡 |
| CLIME_MODE_DOWNHILL | FootPack_DownHill | 下坡 |
| RESET_MODE | FootPack_Reset | 关节电机复位（关再开） |
| SHAKE_MODE | FootPack_Shake | 晃动后腿（卡弹时用） |
| DEBUG_MODE | FootPack_Debug | 调试，可单独调每条腿 |
| DEPLOY_MODE | - | 部署，关闭关节电机 |

---

## 四、典型运动包说明（新手向）

### 4.1 FootPack_Present（预备）

四腿统一角度，不做高度、姿态调节：

```c
angle.theta1 = M_WHEEL_THIGH_ANGLE02PRE + M_WHEEL_THIGH_ANGLE;
angle.theta2 = M_WHEEL_CALF_ANGLE02PRE + M_WHEEL_CALF_ANGLE;
// 四腿相同
```

### 4.2 FootPack_Extend（伸腿）

足端坐标设为较高位置，再逆解得到关节角：

```c
vec[i].x = BasicPos[0] + (i<2 ? 50 : -50);  // 前后方向偏移
vec[i].z = -310;  // 高度
FootWheel_Solve(vec, output);
```

### 4.3 FootPack_Auto_1（自适应水平）

- 读 IMU 的 pitch、roll
- 用 PD 控制前后腿、左右腿的高度差
- 使机身尽量保持水平

### 4.4 FootPack_Auto2_Trot（Trot 步态）

- **前半周期**：RF、LB 抬腿；LF、RB 支撑
- **后半周期**：LF、RB 抬腿；RF、LB 支撑
- 抬腿轨迹用摆线，参数有周期 T、抬腿高度 h、占空比 φ 等

### 4.5 FootPack_Climb_2（爬坡）

状态机：准备 → 腿伸直 → 保持 → 回到预备。  
触发条件：姿态角（如 pitch）达到阈值（如 27°）。

### 4.6 FootPack_DownHill（下坡）

触发条件：pitch 达到阈值。  
根据车头朝向（Chassis_Dir）决定抬前腿还是抬后腿，避免前倾或后倾过多。

---

## 五、模式调度流程

```
1. 根据 motion_mode 选底盘速度包：
   - NO_FOLLOW/SLOW_GYRO/FAST_GYRO → MotPack_Small_Top
   - FOLLOW_GIMBAL/... → MotPack_Follow_Gim
   - CHASS_AUTO → MotPack_Only_Chass

2. 根据 foot_mode 选足轮包：
   - 调用对应 FootPack_*(output)
   - output 写入 ctrlData->Angle_Data

3. ExMotMod_Output(ctrlData)：
   - 麦轮：MecanOmni_Resolve → motorSetData
   - 足轮：Angle_Data → motorSetData2
   - 写入 Wheel_Angle_Set / Wheel_Speed_Set
```
