# 04 运动学解算

## 一、运动学在做什么？

### 1.1 逆运动学 vs 正运动学

| 类型 | 已知 | 求 | 本项目中用途 |
|------|------|-----|--------------|
| **逆运动学** | 期望的足端位置/速度 | 关节角度/角速度 | 给定“足端要到哪”，算“关节该转多少” |
| **正运动学** | 关节角度/角速度 | 足端位置/速度 | 给定“关节当前角度”，算“足端在哪、速度多少” |

### 1.2 本项目的两类运动学

1. **麦轮底盘**：车身速度 (Vx, Vy, ω) ↔ 四轮线速度
2. **足轮腿**：足端坐标 (x, y, z) ↔ 关节角 (θ1, θ2)

---

## 二、麦轮底盘运动学

### 2.1 坐标系约定

- **x 轴**：车体右侧为正
- **y 轴**：车体前方为正
- **ω**：绕竖直轴旋转，俯视逆时针为正

轮子顺序：**RF（右前）- LF（左前）- LB（左后）- RB（右后）**。

### 2.2 逆运动学：车身速度 → 轮速

已知车身速度 (Vx, Vy, ω)，求四轮线速度：

```c
output[0] = √2*(Vy - Vx) + ω * coefficient;  // RF
output[1] = √2*(Vy + Vx) - ω * coefficient;  // LF
output[2] = √2*(Vy - Vx) - ω * coefficient;  // LB
output[3] = √2*(Vy + Vx) + ω * coefficient;  // RB
```

**系数 coefficient**：与车体尺寸、麦轮布局有关，把角速度 ω 转换成轮子线速度的贡献。

### 2.3 正运动学：轮速 → 车身速度

已知四轮编码器转速 (rpm)，求车身 (vx, vy, ω)：

```c
// 先转为线速度 (mm/s)
data_temp[i] = encoderData[i] * RPMTORAD / REDUCT_RATIO * WCONVERT2V_k;

// 再按麦轮布局合成
real_xyw->vel.y = (d0+d1+d2+d3) / √2 / 4;
real_xyw->vel.x = (-d0+d1-d2+d3) / √2 / 4;
real_xyw->angvel = (d0-d1-d2+d3) / 4 / coefficient;
```

---

## 三、足轮运动学

### 3.1 腿的结构（平面二连杆）

每条腿在**矢状面**（侧面看）内运动，可简化为：

```
        髋关节 (相对于车体中心有固定偏移)
           ●
           │
           │  θ1：大腿与铅垂线夹角
           │╲
           │ ╲  L1 = 180 mm
           │  ╲
           │   ● 膝关节
           │  ╱
           │ ╱  L2 = 180 mm
           │╱  θ2：小腿与大腿夹角（外角）
           ●
         足端（装麦轮）
```

- **正运动学**：已知 θ1、θ2 → 求足端 (x, y, z)
- **逆运动学**：已知足端 (x, y, z) → 求 θ1、θ2

### 3.2 髋关节基础位置 (FootWheelBasicPos)

四腿髋关节相对车体中心的位置 (mm)：

```c
[WHEEL_RF] = {  VEHICLE_LONG/2, -VEHICLE_WIDTH/2, 0 };  // 右前
[WHEEL_LF] = {  VEHICLE_LONG/2,  VEHICLE_WIDTH/2, 0 };  // 左前
[WHEEL_LB] = { -VEHICLE_LONG/2,  VEHICLE_WIDTH/2, 0 };  // 左后
[WHEEL_RB] = { -VEHICLE_LONG/2, -VEHICLE_WIDTH/2, 0 };  // 右后
```

### 3.3 正运动学 FootWheel_InvSolve()

已知关节角 → 求足端坐标：

```c
diff_theta = theta2 - theta1;
sign1 = (i>=2) ? -1 : 1;  // 后腿符号相反

output[i].x = BasicPos[0] + sign1*L*sin(diff_theta) - sign1*L*sin(theta1);
output[i].y = BasicPos[1];  // 本结构 y 不变
output[i].z = BasicPos[2] - L*cos(-diff_theta) - L*cos(theta1);
```

几何上就是两段杆长的投影叠加。

### 3.4 逆运动学 FootWheel_Solve()

已知足端坐标 → 求关节角。通过几何关系反解：

```c
A = (z - z0)² + (x - x0)²;
theta1 = acos(√A / (2*L)) * RADTODAG + sign2 * atan((x-x0)/(z-z0)) * RADTODAG;
theta2 = acos((...) / (2*L²)) * RADTODAG;
```

---

## 四、角度零位与预备姿态

| 参数 | 值 | 含义 |
|------|-----|------|
| M_WHEEL_THIGH_ANGLE | 34.70° | 大腿电机零位对应的角度（与铅垂线夹角） |
| M_WHEEL_CALF_ANGLE | 135.99° | 小腿电机零位对应的角度（与大腿夹角） |
| M_WHEEL_THIGH_ANGLE02PRE | 3.81° | 从零位到预备姿态，大腿需再转过的角度 |
| M_WHEEL_CALF_ANGLE02PRE | -1.10° | 从零位到预备姿态，小腿需再转过的角度 |

---

## 五、车体参数 (Wheel_data.h)

```c
#define VEHICLE_LONG  438   // 前后轮间距 mm
#define VEHICLE_WIDTH 421   // 左右轮间距 mm
#define WHEEL_RADIUS  75    // 麦轮半径 mm
#define REDUCT_RATIO  (3591.0f/187.0f)  // 3508 减速比
```
