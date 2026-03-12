# Hero H7 四足轮腿机器人控制代码详解

> 基于 RT-Thread 操作系统和 STM32H7VIT6 单片机，使用 VMC（虚拟模型控制）与位置混合控制实现四足轮腿机器人的运动控制。

---

## 新手阅读建议

1. **先看 [01_architecture.md](01_architecture.md)**：了解整体结构、线程、控制流程。
2. **再看 [02_VMC.md](02_VMC.md)**：VMC 原理与实现，含论文参考与公式说明。
3. **然后看 [04_kinematics.md](04_kinematics.md)**：运动学基础，理解“速度/位置如何转换成电机指令”。
4. **最后看 [03](03_position_control.md)、[05](05_motion_modes.md)**：位置控制与各模式含义。

---

## 一、项目概述

### 1.1 硬件平台

- **主控芯片**: STM32H753VI (Cortex-M7)
- **操作系统**: RT-Thread
- **底盘类型**: 轮腿式（Foot Wheel），四足结构
- **电机配置**:
  - 关节电机: DM8009（大腿、小腿各4个，共8个）
  - 轮毂电机: 3508（4个麦轮）
- **通信**: FDCAN1/2、SPI-CAN（关节电机）、IMU

### 1.2 控制架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│                        控制数据源                                 │
│  (云台/遥控器/视觉/监视器) → Ctrl_data_t                         │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Chassis_Ctrl_Thread (2ms)                      │
│  ModeState_Scheduler: 运动模式调度 + 足轮模式调度                  │
│  - 底盘模式: NO_FOLLOW/FOLLOW_GIMBAL/SLOW_GYRO/FAST_GYRO/...       │
│  - 足轮模式: PRESENT/NORMAL/CLIMB/CLIMB_2/DOWNHILL/...             │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ExMotMod_Output                               │
│  - 麦轮速度解算: MecanOmni_Resolve(Vx,Vy,ω) → 4轮速度             │
│  - 足轮角度解算: FootPack_*() → 足端坐标 → FootWheel_Solve → 关节角 │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    电机控制层                                     │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ 关节电机(大腿/小腿): DM_CtrlType_E_MIT 或 DM_CtrlType_MIT    │ │
│  │  - E-MIT: 位置+速度环 PID → 力矩输出                         │ │
│  │  - MIT:   VMC 力矩直接输出 (Torque_VMC)                      │ │
│  └─────────────────────────────────────────────────────────────┘ │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ 轮毂电机: 速度 PID → 电流输出 (CAN)                           │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 核心控制策略

| 控制方式 | 适用对象 | 说明 |
|---------|---------|------|
| **位置控制 (E-MIT)** | 大腿、小腿关节 | 角度设定 → 关节角度 PID → 力矩输出，大部分模式使用 |
| **VMC 力矩控制 (MIT)** | 大腿、小腿关节 | 虚拟力/力矩 → 雅可比映射 → 关节力矩，可切换为 MIT 模式 |
| **速度控制** | 轮毂电机 | 麦轮线速度 mm/s → 速度 PID → 电流 |

## 二、文档结构

| 文档 | 内容 |
|------|------|
| [01_architecture.md](01_architecture.md) | 系统架构、线程、初始化流程 |
| [02_VMC.md](02_VMC.md) | VMC 虚拟模型控制原理与实现 |
| [03_position_control.md](03_position_control.md) | 位置控制与 E-MIT 模式 |
| [04_kinematics.md](04_kinematics.md) | 运动学解算（麦轮、足轮） |
| [05_motion_modes.md](05_motion_modes.md) | 模式与运动包（FootPack） |
| [code/](code/) | 关键代码片段 |

## 三、关键文件索引

| 模块 | 文件路径 |
|------|----------|
| 主入口 | `applications/Main/main.c` |
| 底盘控制 | `applications/chassis/app_ChassisCtrl.c` |
| 底盘驱动 | `applications/chassis/foot/drv_wheel.c` |
| VMC 控制 | `applications/chassis/fun_Jacobi.c` |
| 运动包 | `applications/chassis/mod_motion.c` |
| 运动学 | `applications/chassis/drv_motionSolve.c` |
| 控制消息 | `applications/chassis/controlMsg.h` |
| 参数配置 | `applications/robotData/hero_2025/Wheel_data.h` |
| SPI-CAN | `applications/Master_spi/drv_spiCanMaster.c` |

## 四、参考文献与链接

### VMC 理论参考

- **《基于虚拟模型的四足机器人对角小跑步态控制方法》**（Control of the Quadrupedal Trotting Based on Virtual Model），《机器人》期刊，2016，DOI: 10.13973/j.cnki.robot.2016.0064  
  相关表述也见于《四足机器人对角小跑步态虚拟模型直觉控制方法研究》等文献。

### 项目文档

- 轮腿 VMC 控制器设计: [语雀文档](https://hitwhlc.yuque.com/hero-rm/mox72g/gtg1viwzez5o3oka)
