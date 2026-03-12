# 02 VMC 虚拟模型控制

## 一、什么是 VMC？——从直觉理解开始

### 1.1 核心思想（一句话）

**VMC（Virtual Model Control，虚拟模型控制）** 就是：在脑子里想象机器人足端/躯干上连着一些**虚拟的弹簧和阻尼器**，根据这些“虚拟元件”产生的力，反推出关节电机该输出多大的力矩。

> 类比：你用手推一个弹簧，弹簧会产生反作用力；VMC 就是先算出“理想情况下足端该受多大的力”，再通过数学关系转换成“关节电机该出多大力”。

### 1.2 为什么需要 VMC？

- **位置控制**：直接给定关节角度，适合静态、慢速、地面平整的场景。
- **力/力矩控制（VMC）**：给定足端力，适合**动态行走、不平地面、抗冲击**——因为力可以随地形和姿态自动调节，更“柔顺”。

本代码中的轮腿机器人同时支持两种方式，可根据模式切换。

---

## 二、论文参考与理论框架

### 2.1 参考论文

> **《基于虚拟模型的四足机器人对角小跑步态控制方法》**  
> （Control of the Quadrupedal Trotting Based on Virtual Model）  
> 刊于《机器人》期刊，DOI: 10.13973/j.cnki.robot.2016.0064

相关表述也常见于《四足机器人对角小跑步态虚拟模型直觉控制方法研究》等文献。

### 2.2 论文中的两大模块

| 模块 | 作用 | 本代码中的对应 |
|------|------|----------------|
| **支撑相控制** | 建立“躯干虚拟力”与“支撑腿关节力矩”的关系，控制躯干高度、姿态、前进速度、自转角速度 | `get_Force` + `get_torque`，通过虚拟力控制足端 |
| **摆动相控制** | 用虚拟弹簧-阻尼驱动摆动足沿给定轨迹运动 | 本代码中更多用位置控制实现摆动，VMC 侧重支撑相 |

### 2.3 虚拟模型包含什么？

- **虚拟腿**：把每条腿抽象成从躯干到足端的连线
- **虚拟弹簧-阻尼**：在 x、y、z 方向等效为弹簧（刚度 k）+ 阻尼（阻尼系数 b）
- **虚拟支撑**：支撑相时，虚拟力通过支撑腿传递到地面

---

## 三、本代码中的 VMC 实现流程（整体框图）

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Step 1: 获取实际物理量                                                    │
│  get_Actual_physical_quantity()                                          │
│  - 设定值: 高度 height_set, 姿态 pitch_set/roll_set/yaw_set              │
│  - 实际值: IMU 姿态, 关节角→足端位置(x,y,z), 关节速度→足端速度(dx,dy,dz)   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Step 2: 计算虚拟力（虚拟弹簧-阻尼模型）                                    │
│  get_Force()                                                            │
│  F = K*(位置误差) + B*(速度误差) + 重力补偿                                 │
│  得到每条腿的 Fx, Fy, Fz                                                 │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Step 3: 力→力矩映射（雅可比转置）                                         │
│  get_torque()                                                           │
│  τ = -J^T · F                                                           │
│  得到每条腿大腿、小腿的关节力矩 T2, T3                                     │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Step 4: 发送给电机                                                      │
│  SPI_CANSTransPIDOut_MIT() → 达妙电机 MIT 力矩模式                        │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 四、核心数据结构详解（新手必读）

### 4.1 Control_volume_t —— 虚拟力

```c
typedef struct _Control_volume_t {
    float Fx;  // 足端 x 方向虚拟力 (N)，左右方向
    float Fy;  // 足端 y 方向虚拟力 (N)，前后方向（本机 y 为前进方向）
    float Fz;  // 足端 z 方向虚拟力 (N)，竖直方向，向上为正
} Control_volume_t;
```

**含义**：每条腿足端在 x、y、z 三个方向上“应该”受到的力。这些力由虚拟弹簧-阻尼模型计算得到。

### 4.2 Torque_t —— 关节力矩

```c
typedef struct _Torque_t {
    float T1;  // roll 方向力矩（本机无此关节，恒为 0）
    float T2;  // 大腿关节力矩 (N·m)
    float T3;  // 小腿关节力矩 (N·m)
    float T1_max, T2_max, T3_max;  // 限幅（可选）
} Torque_t;
```

**含义**：通过雅可比矩阵，把足端力 F 转换成关节电机需要输出的力矩 τ。

### 4.3 Actual_physical_quantity_t —— 设定值与实际值

```c
typedef struct _Actual_physical_quantity_t {
    // 足端速度设定/实际 (m/s)
    float dx_set, dy_set, dz_set;
    float dx, dy, dz;

    // 足端位置设定/实际 (m)
    float x_set, y_set, z_set;
    float x, y, z;

    // 机身姿态设定/实际 (rad)
    float pitch, pitch_set;   // 俯仰
    float roll, roll_set;     // 横滚
    float yaw, yaw_set;       // 偏航
    float dyaw, dyaw_set;     // 偏航角速度

    float height_set;         // 期望机身高度 (m)，用于 z 方向
} Actual_physical_quantity_t;
```

**含义**：存放“我们希望机器人达到的状态”和“当前实际状态”，用于计算误差，进而得到虚拟力。

### 4.4 weighting_t —— 虚拟弹簧-阻尼参数

```c
typedef struct _weighting_t {
    float gravity_compensation;  // 重力补偿力，约等于 1/4 机身重量*g
    float bx, by, bz;            // 阻尼系数：抑制速度，防止振荡
    float kx, ky, kz;            // 刚度系数：位置误差产生恢复力
    float byaw, kyaw;            // 偏航方向的阻尼、刚度
    float eta_yaw;               // yaw 对 y 方向力的耦合权重
    float kpitch, kroll;         // 俯仰、横滚对高度设定的修正系数
    float khub;                  // 轮子（轮毂）位置控制权重
} weighting_t;
```

**新手理解**：
- **k 大**：位置偏差产生更大的力，响应快，但容易抖
- **b 大**：速度大时阻尼大，更稳，但可能显得“钝”
- 一般 z 方向（竖直）用较大的 k、b，保证高度和稳定性；x、y 方向可能只用阻尼（kx=0）做“软着陆”。

---

## 五、虚拟力计算 get_Force() —— 虚拟弹簧-阻尼模型

### 5.1 基本公式（论文中的弹簧-阻尼思想）

虚拟力可以写成：

$$F = K \cdot (x_{set} - x) + B \cdot (v_{set} - v) + F_{gravity}$$

- $x_{set}$, $x$：期望位置、实际位置
- $v_{set}$, $v$：期望速度、实际速度
- $K$：刚度（弹簧）
- $B$：阻尼
- $F_{gravity}$：重力补偿

### 5.2 高度设定修正（姿态耦合）

机身俯仰、横滚会影响各腿的“有效高度设定”：

```c
// 前腿 (RF, LF)：机身前倾时，前腿要抬得更高才能保持水平
z_set = height_set + kpitch * sin(pitch_set - pitch);

// 后腿 (RB, LB)：机身前倾时，后腿要放低
z_set = height_set - kpitch * sin(pitch_set - pitch);

// 右腿 (RF, RB)：机身右倾时，右腿要抬得更高
z_set += kroll * sin(roll_set - roll);

// 左腿 (LF, LB)：机身左倾时，左腿要抬得更高
z_set -= kroll * sin(roll_set - roll);
```

**直观理解**：机身歪了，就通过各腿的高度差把它“掰正”。

### 5.3 三个方向的虚拟力

| 方向 | 公式 | 说明 |
|------|------|------|
| **Fx** | $k_x(x_{set}-x) + b_x(v_{x,set}-v_x)$ | 本代码中 kx=0，相当于纯阻尼，主要抑制左右晃动 |
| **Fy** | $k_y(y_{set}-y) + b_y(v_{y,set}-v_y) + \eta_{yaw}(k_{yaw}\Delta\psi + b_{yaw}\Delta\dot\psi)$ | 含偏航耦合，后腿 Fy 取反（坐标系对称） |
| **Fz** | $F_{grav} - k_z(z_{set}-z) - b_z(0-v_z)$ | 重力补偿 + 高度刚度 + 阻尼，足端向下运动时阻尼起缓冲作用 |

**Fz 的符号说明**：$z_{set}-z$ 为正表示足端比设定位置低，需要向上的力把机身撑高，所以前面是负号；$0-v_z$ 表示期望竖直速度为 0，实际向下运动时 $v_z<0$，$b_z(0-v_z)>0$ 产生向上的阻尼力。

---

## 六、雅可比矩阵与力矩映射 get_torque()

### 6.1 为什么需要雅可比？

**问题**：我们算出了足端的力 F，但电机在关节上，只能输出关节力矩 τ。两者如何对应？

**答案**：通过**雅可比矩阵 J**。它描述的是“关节角速度 → 足端线速度”的线性关系：

$$\dot{p} = J \cdot \dot{q}$$

其中 $\dot{p}$ 是足端速度，$\dot{q}$ 是关节角速度。

根据虚功原理，**力与力矩的映射**为：

$$\tau = J^T \cdot F$$

本代码中采用 $\tau = -J^T \cdot F$，负号与坐标系和力矩正方向约定有关。

### 6.2 本机的腿结构（简化）

每条腿是**平面二连杆**（大腿 + 小腿），在 sagittal 平面内运动：

```
        髋关节
           ●
           │  θ1 (大腿与铅垂线夹角)
           │╲
           │ ╲  L1 = 180mm
           │  ╲
           │   ● 膝关节
           │  ╱
           │ ╱  L2 = 180mm
           │╱  θ2 (小腿与大腿夹角，外角)
           ●
         足端
```

- θ1（大腿）：相对于铅垂线的夹角
- θ2（小腿）：相对于大腿的夹角
- 本机没有 roll 关节，所以 T1 恒为 0

### 6.3 雅可比矩阵 J 的含义

J 是 3×3 矩阵（本机实际有效的是 2 列，对应大腿和小腿）：

- 第 j 列：第 j 个关节单位角速度时，足端在 x、y、z 方向的速度分量
- 例如 $J[1][1]$：大腿角速度对足端 y 方向速度的贡献

代码中通过几何关系直接写出 J 的解析形式（涉及 L1、L2、θ1、θ2、θ3 的 sin/cos），这里不展开推导，可参考机器人学教材中的平面二连杆雅可比。

### 6.4 力矩计算

```c
// τ = -J^T * F
Torque[i].T1 = -Ji[0][0]*Fx - Ji[1][0]*Fy - Ji[2][0]*Fz;  // 本机为 0
Torque[i].T2 = -Ji[0][1]*Fx - Ji[1][1]*Fy - Ji[2][1]*Fz;  // 大腿
Torque[i].T3 = -Ji[0][2]*Fx - Ji[1][2]*Fy - Ji[2][2]*Fz;  // 小腿
```

### 6.5 足端速度更新（用于下一拍力计算）

足端速度由关节角速度通过雅可比得到：$v = J \cdot \omega$。本代码中：

```c
Actual_physical_quantity[i].dx = Ji[0][1]*ω_thigh + Ji[0][2]*ω_calf;
Actual_physical_quantity[i].dy = Ji[1][1]*sign_thigh*ω_thigh + Ji[1][2]*sign_calf*ω_calf;
Actual_physical_quantity[i].dz = Ji[2][1]*ω_thigh + Ji[2][2]*ω_calf;
```

`sign_thigh`、`sign_calf` 用于处理前腿/后腿的坐标系差异。

---

## 七、实际物理量获取 get_Actual_physical_quantity()

### 7.1 设定值

- `height_set`：期望机身高度（质心到地面），由 `get_wolf_height()` 等得到，单位 m
- `dx_set = dy_set = dz_set = 0`：期望足端相对髋关节静止（支撑相简化）
- `pitch_set = yaw_set = 0`：期望机身水平、无偏航

### 7.2 实际值

- **姿态**：`pitch, roll, yaw` 来自底盘 IMU
- **足端高度 z**：由关节角通过正运动学计算（二连杆几何）
- **足端水平位置 y**：麦轮轮毂相对髋关节的水平距离，用于 y 方向力控制

---

## 八、参数表与调参建议

### 8.1 当前参数 (drv_wheel.h)

```c
#define BX      1.0f    // x 阻尼，较小
#define BY      70.0f   // y 阻尼
#define BZ      100.0f  // z 阻尼
#define KX      0.0f    // x 刚度，为 0 表示仅阻尼
#define KY      300.0f  // y 刚度
#define KZ      1000.0f // z 刚度，最大，保证高度控制
#define K_PITCH 0.280f  // 俯仰对高度修正
#define K_ROLL  0.210f  // 横滚对高度修正
#define QUARTER_UPSIDE_WEIGHT (32.0f * 9.8f / 4)  // 重力补偿 ≈ 78.4 N
```

### 8.2 调参直觉（新手向）

| 现象 | 可能原因 | 调整方向 |
|------|----------|----------|
| 机身高度不稳、上下抖 | kz 过大或 bz 过小 | 略降 kz，略增 bz |
| 反应慢、高度恢复慢 | kz 过小 | 增大 kz |
| 左右晃、前后晃 | by、bx 过小 | 增大 by、bx |
| 姿态修正不足 | kpitch、kroll 过小 | 增大 kpitch、kroll |

---

## 九、在代码中的调用位置

```c
// drv_wheel.c, AnglePID_Cal_Thread 中
if (DM_Ctrl_Type == DM_CtrlType_MIT) {
    get_Actual_physical_quantity(Actual_physical_quantity);
    get_Force(Control_volume, Actual_physical_quantity, &VMC_Weighting);
    get_torque(Torque_VMC, Control_volume, Actual_physical_quantity);
}
// ...
SPI_CANSTransPIDOut_MIT(SPI_CAN1, Torque_VMC);
SPI_CANSTransPIDOut_MIT(SPI_CAN2, Torque_VMC);
```

**注意**：当前工程中多数模式使用 E-MIT（位置控制），MIT（VMC）模式在部分场景下被注释，若需启用 VMC，需在对应 `Foot_mode` 下调用 `Type_Use_MIT()` 并保证上述逻辑执行。

---

## 十、延伸阅读

- 论文：*基于虚拟模型的四足机器人对角小跑步态控制方法*，机器人，2016
- 项目语雀： [轮腿VMC控制器设计](https://hitwhlc.yuque.com/hero-rm/mox72g/gtg1viwzez5o3oka)
