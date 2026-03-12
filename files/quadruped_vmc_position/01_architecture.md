# 01 系统架构

## 一、前置知识：几个基本概念

### 1.1 RT-Thread 是什么？

**RT-Thread** 是一个实时操作系统（RTOS），类似“小型 Linux”：
- 提供**多任务（线程）**调度：多个任务轮流执行，实现“同时”做多件事
- 提供**信号量、互斥锁**等同步机制：让任务之间协调工作
- 提供**定时器**：每隔固定时间触发一次，常用于控制周期

本项目中，底盘控制每 **2ms** 执行一次，就是靠定时器 + 信号量实现的。

### 1.2 信号量（Semaphore）简单理解

- **作用**：让一个线程“等待”直到另一个线程“释放”
- **本项目中**：定时器每 2ms 触发一次，释放信号量；底盘控制线程一直在等这个信号量，一旦等到就执行一次控制逻辑，然后继续等下一次

### 1.3 本机的“四足轮腿”是什么？

- **四足**：四条腿，每条腿有大腿、小腿两个关节电机
- **轮腿**：每条腿末端装有**麦轮**，既能像腿一样抬落，又能像轮子一样滚动
- 因此需要控制：**8 个关节电机**（大腿×4 + 小腿×4）+ **4 个轮毂电机**

---

## 二、主程序初始化流程 (main.c)

程序上电后按顺序初始化各模块，任一步失败会触发 `Robot_Chassis_Reset` 复位。

```
1. 使能 Cache（提高访问速度）
   SCB_EnableICache(); SCB_EnableDCache();

2. 上电提示音
   set_buzzer(2000, 1); 延时 120ms

3. 监视器（监控系统健康）
   MONITOR_INIT(...)

4. CAN 总线
   Can1_Init(); Can2_Init();
   DrvSpiCanInit();  // 关节电机通过 SPI 转 CAN 与主控通信

5. 通信模块
   DJI_Init();        // 裁判系统（比赛用）
   Scpr_Com_Init();   // 超级电容
   Gimbal_Com_Init(); // 云台（上位机/遥控）

6. 底盘控制（核心）
   Chassis_Ctrl_Init();

7. 可选模块
   ChassisIMU_Init();   // 底盘 IMU
   Visualnit();        // 视觉
   OnlyChassis_Init(); // 遥控器
   UI_Init();          // 显示屏
   protct_ThreadInit();// 保护线程（过流、堵转等）
```

---

## 三、线程与定时器

| 名称 | 周期 | 栈大小 | 优先级 | 功能说明 |
|------|------|--------|--------|----------|
| Chassis_Thread | 2ms | 1024 | 10 | 底盘主逻辑：根据控制源数据做模式调度，输出目标角度/速度 |
| AnglePID_Cal_Thread | 2ms | 1024 | 5 | 关节电机：位置 PID 或 VMC 力矩计算，通过 SPI-CAN 发送 |
| RunPID_Cal_Thread | 2ms+1 | 1024 | 6 | 轮毂电机：速度 PID，通过 CAN 发送电流 |
| Cctrl_timer | 2ms | - | - | 软定时器，每 2ms 释放信号量，驱动 Chassis_Thread |

**优先级**：数字越小优先级越高。底盘控制(10) 高于关节(5)、轮毂(6)，保证控制指令优先执行。

---

## 四、底盘控制主循环（核心逻辑）

```
1. rt_sem_take(ChassCtrl_sem)     // 等待 2ms 定时器释放信号量
2. nowCtrlSrc = getNowCtrlSrc_s() // 获取当前控制源（云台/遥控/视觉等）
3. Ctrl_data = nowCtrlSrc->data   // 取出控制数据
4. ModeState_Scheduler(*Ctrl_data) // 模式调度：底盘模式 + 足轮模式
5. ExMotMod_Output(&fixCtrl)      // 把调度结果写入电机设定值
```

**控制源**：谁在“发指令”。例如云台发来速度、足轮模式，则 `Ctrl_data` 里就有 `xyw`（速度）、`foot_mode`（足轮模式）等。

---

## 五、控制数据结构 Ctrl_data_t

```
Ctrl_data_t
├── motion_mode    底盘运动模式：不跟随/跟随云台/小陀螺/快陀螺 等
├── foot_mode     足轮模式：预备/正常/爬坡/下坡/摇摆 等
├── xyw            速度矢量
│   ├── vel.x     左右速度 (mm/s)
│   ├── vel.y     前后速度 (mm/s)
│   └── angvel    自转角速度 (0.1°/s)
├── h_ave          平均高度 (mm)，部分模式用
└── Angle_Data[4] 四腿关节角度 (theta1 大腿, theta2 小腿)
```

---

## 六、电机分组与通信

| 组 | 电机 | 数量 | 控制方式 | 通信方式 |
|----|------|------|----------|----------|
| FOOT_WHEEL_THIGH | DM8009 大腿 | 4 | E-MIT 或 MIT | SPI-CAN1 |
| FOOT_WHEEL_CALF | DM8009 小腿 | 4 | E-MIT 或 MIT | SPI-CAN2 |
| FOOT_WHEEL_RUN | 3508 轮毂 | 4 | 速度 PID | CAN1 |

**SPI-CAN**：主控通过 SPI 连接一块“CAN 桥”芯片，再通过 CAN 与达妙电机通信。这样可以用少量 CAN 口控制多组电机。

---

## 七、DM 电机控制模式

```c
DM_CtrlType_E_MIT   // 扩展 MIT：位置环 + 速度环，输出力矩（常用）
DM_CtrlType_MIT     // 纯 MIT：直接给力矩，VMC 用
DM_CtrlType_Position// 纯位置：只做位置控制
```

- **E-MIT**：给定目标角度，电机内部做位置 PID + 速度环，输出力矩。适合大部分行走、爬坡等场景。
- **MIT**：主控直接计算好关节力矩，发给电机。用于 VMC 等力控场景。
