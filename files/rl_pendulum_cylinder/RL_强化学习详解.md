# 旋转倒立摆强化学习详解

> 本文档面向强化学习新手，从基础概念出发，详细讲解本项目中 RL 代码的每一部分。

---

## 目录

1. [强化学习基础概念](#一强化学习基础概念)
2. [马尔可夫决策过程（MDP）](#二马尔可夫决策过程mdp)
3. [项目结构与依赖](#三项目结构与依赖)
4. [物理模型简介](#四物理模型简介)
5. [环境类详解](#五环境类详解)
6. [奖励设计详解](#六奖励设计详解)
7. [PPO 算法简介](#七ppo-算法简介)
8. [训练与推理流程](#八训练与推理流程)
9. [演示脚本 rl_demo.py](#九演示脚本-rl_demopy)
10. [参数调优建议](#十参数调优建议)
11. [附录：关键术语表](#附录关键术语表)

---

## 一、强化学习基础概念

### 1.1 什么是强化学习？

**强化学习（Reinforcement Learning, RL）** 是机器学习的一个分支，其核心思想是：**智能体（Agent）通过与环境（Environment）的交互，根据获得的奖励（Reward）信号，学习如何做出最优决策**。

与监督学习不同，强化学习没有“标准答案”，只有“好”或“不好”的反馈。智能体通过大量试错，逐渐学会哪些动作在什么情况下能获得更多奖励。

### 1.2 生活中的类比

| 场景 | 智能体 | 环境 | 动作 | 奖励 |
|------|--------|------|------|------|
| 学骑自行车 | 你 | 自行车+路面 | 转向、刹车、蹬踏 | 保持平衡=正奖励，摔倒=负奖励 |
| 下棋 | 棋手 | 棋盘 | 落子 | 赢=正奖励，输=负奖励 |
| 倒立摆平衡 | 神经网络 | MuJoCo 仿真 | 电机力矩 | 竖直=正奖励，倒下=负奖励 |

### 1.3 强化学习的核心循环

```
     ┌──────────────────────────────────────────────────────────┐
     │                                                          │
     ▼                                                          │
┌─────────┐   观测(obs)   ┌─────────┐   动作(a)   ┌─────────┐   │
│  环境   │ ──────────→  │ 智能体  │ ──────────→ │  环境   │   │
│         │              │ (策略)  │              │         │   │
└─────────┘              └─────────┘              └─────────┘   │
     ▲                         │                        │        │
     │                         │                        │        │
     └─────────────────────────┴────────────────────────┘        │
                    奖励(r)、新状态(s')                          │
```

每一步：
1. 智能体根据当前**观测**选择**动作**
2. 环境执行动作，产生**新观测**和**奖励**
3. 智能体根据奖励更新策略，进入下一步

---

## 二、马尔可夫决策过程（MDP）

强化学习通常用 **马尔可夫决策过程（Markov Decision Process, MDP）** 来形式化描述。

### 2.1 MDP 五元组

| 符号 | 含义 | 本项目中 |
|------|------|----------|
| **S** | 状态空间（State） | 摆角、臂角、角速度等构成的连续空间 |
| **A** | 动作空间（Action） | 电机力矩，归一化到 [-1, 1] |
| **P** | 状态转移概率 P(s'\|s,a) | 由 MuJoCo 物理仿真决定 |
| **R** | 奖励函数 R(s,a,s') | 我们设计的 reward 公式 |
| **γ** | 折扣因子（0~1） | 0.995，表示对未来奖励的重视程度 |

### 2.2 马尔可夫性

**马尔可夫性**：下一状态只依赖于当前状态和动作，与更早的历史无关。即：

$$P(s_{t+1} | s_t, a_t, s_{t-1}, a_{t-1}, ...) = P(s_{t+1} | s_t, a_t)$$

在倒立摆中，只要知道当前摆角、臂角、角速度，就能预测下一步；不需要知道“10 秒前摆在哪”。

### 2.3 策略与目标

- **策略 π(a|s)**：在状态 s 下选择动作 a 的概率分布。本项目中由神经网络表示。
- **目标**：找到最优策略 π*，使得**累积折扣奖励**最大：

$$\max_\pi \mathbb{E}\left[ \sum_{t=0}^{\infty} \gamma^t r_t \right]$$

---

## 三、项目结构与依赖

### 3.1 文件结构

```
rotary_pendulum/
├── rotary_pendulum.xml   # MuJoCo 物理模型
├── rl_balance.py         # RL 环境定义 + 训练脚本
├── rl_demo.py            # 演示脚本（加载已训练模型）
├── pid_balance.py        # 传统 PID 控制（对比用）
└── RL_强化学习详解.md    # 本文档
```

### 3.2 主要依赖

| 库 | 用途 |
|----|------|
| **gymnasium** | RL 标准接口（原 OpenAI Gym） |
| **stable-baselines3** | PPO 等算法实现 |
| **mujoco** | 物理仿真引擎 |
| **numpy** | 数值计算 |

### 3.3 运行环境

```bash
conda activate mujoco_env
```

---

## 四、物理模型简介

### 4.1 旋转倒立摆结构

根据 `rotary_pendulum.xml`：

```
         [重球] ●
            │
            │ 杆 (pole)
            │
    ────────●────────  臂 (arm)，可绕基座旋转
            │
         [基座]
```

- **基座**：固定不动
- **臂**：绕 z 轴旋转，由电机驱动（`arm_joint`）
- **杆**：挂在臂末端，绕 x 轴旋转（`pole_joint`）
- **重球**：杆末端，质量 0.45 kg，是主要惯性来源

### 4.2 状态变量

| 变量 | 含义 | 单位 |
|------|------|------|
| `qpos[0]` | 臂角 θ_arm | rad |
| `qpos[1]` | 摆角 θ_pend | rad，0=竖直向上 |
| `qvel[0]` | 臂角速度 | rad/s |
| `qvel[1]` | 摆角速度 | rad/s |

### 4.3 控制输入

- 电机力矩施加在 `arm_joint` 上
- XML 中限制为 ±50 N·m，代码中实际使用 ±12 N·m

---

## 五、环境类详解

### 5.1 类定义与初始化

```python
class RotaryPendulumEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.max_torque = 12.0
        # ... 动作空间、观测空间
```

- 继承 `gym.Env`，符合 Gymnasium 标准接口
- `model`：物理模型结构（质量、惯性、关节等）
- `data`：仿真状态（位置、速度、控制输入等）
- `max_torque`：将 [-1, 1] 动作映射到 [-12, 12] N·m

### 5.2 动作空间 action_space

```python
self.action_space = spaces.Box(
    low=-1.0,
    high=1.0,
    shape=(1,),
    dtype=np.float32
)
```

- **类型**：连续空间 `Box`
- **维度**：1（标量）
- **范围**：[-1, 1]，表示归一化的力矩指令
- **实际力矩**：`torque = action * max_torque`，即 [-12, 12] N·m

### 5.3 观测空间 observation_space

```python
self.observation_space = spaces.Box(
    low=-np.inf,
    high=np.inf,
    shape=(6,),
    dtype=np.float32
)
```

- **维度**：6 维向量
- **无显式边界**：由物理量自然范围决定

### 5.4 观测内容 get_obs()

```python
def get_obs(self):
    theta_arm = self.data.qpos[0]
    theta_pend = self.data.qpos[1]
    dtheta_arm = self.data.qvel[0]
    dtheta_pend = self.data.qvel[1]
    theta_arm_wrapped = (theta_arm + np.pi) % (2 * np.pi) - np.pi
    return np.array([
        np.cos(theta_pend),      # [0]
        np.sin(theta_pend),      # [1]
        theta_arm_wrapped * 0.3, # [2]
        dtheta_arm,              # [3]
        dtheta_pend,             # [4]
        self.prev_action         # [5]
    ], dtype=np.float32)
```

| 索引 | 含义 | 说明 |
|------|------|------|
| 0 | cos(θ_pend) | 摆角余弦，竖直向上时=1，避免 ±π 跳变 |
| 1 | sin(θ_pend) | 摆角正弦，与 cos 一起唯一确定角度 |
| 2 | θ_arm × 0.3 | 臂角归一化到 [-π,π] 后缩放，与其他量级匹配 |
| 3 | dθ_arm | 臂角速度 |
| 4 | dθ_pend | 摆角速度 |
| 5 | prev_action | 上一步动作，便于策略输出平滑控制 |

**为何用 cos/sin 表示摆角？**

- 角度 θ 在 -π 和 π 处有 discontinuity（例如 179° 和 -179° 数值差很大但物理上接近）
- cos、sin 是连续的，网络更容易学习

### 5.5 重置 reset()

```python
def reset(self, seed=None, options=None):
    super().reset(seed=seed)
    mujoco.mj_resetData(self.model, self.data)
    if options and options.get("demo"):
        self.data.qpos[1] = np.random.uniform(-0.15, 0.15)
        self.data.qpos[0] = np.random.uniform(-0.1, 0.1)
    else:
        self.data.qpos[1] = np.random.uniform(-0.5, 0.5)
        self.data.qpos[0] = np.random.uniform(-0.3, 0.3)
    self.prev_action = 0
    return self.get_obs(), {}
```

- **调用时机**：每个 episode 开始时
- **demo 模式**：小扰动（摆 ±0.15 rad，臂 ±0.1 rad），便于演示
- **训练模式**：大扰动（摆 ±0.5 rad，臂 ±0.3 rad），增加数据多样性
- **返回值**：`(obs, info)`，info 为空字典

### 5.6 单步执行 step(action)

`step` 是 RL 交互的核心，完成“执行动作 → 仿真 → 计算奖励 → 返回结果”。

#### 5.6.1 动作平滑与限幅

```python
raw_action = float(action[0])
action_delta = np.clip(raw_action - self.prev_action, -0.24, 0.24)
action = 0.9 * self.prev_action + 0.1 * (self.prev_action + action_delta)
```

- **目的**：抑制网络输出的高频抖动和突变
- **单步变化限幅**：±0.24，防止相邻步动作差异过大
- **指数平滑**：90% 历史 + 10% 新值，使控制更平滑

#### 5.6.2 施加控制并仿真

```python
torque = float(action * self.max_torque)
self.data.ctrl[0] = torque
for _ in range(5):
    mujoco.mj_step(self.model, self.data)
```

- MuJoCo 默认 `timestep=0.01` s
- 每次 `step` 仿真 5 步，即 0.05 s 一个控制周期
- 相当于控制频率 20 Hz

#### 5.6.3 返回值

```python
return obs, reward, terminated, truncated, {}
```

| 返回值 | 含义 |
|--------|------|
| obs | 新观测 |
| reward | 本步奖励 |
| terminated | 是否自然结束（如摆倒下） |
| truncated | 是否被截断（如超时，本项目中未用） |
| {} | 额外信息 |

---

## 六、奖励设计详解

### 6.1 奖励公式

```python
reward = (
    8.0 * np.cos(theta)           # 主目标：保持竖直
    - 0.18 * dtheta**2             # 抑制摆角速度
    - 0.9 * theta_arm**2           # 臂贴近中心
    - 0.8 * dtheta_arm**2          # 抑制臂运动
    - 0.006 * torque**2            # 惩罚大扭矩
    - 1.5 * action_delta_for_reward**2  # 惩罚动作突变
)
```

### 6.2 各项含义

| 项 | 系数 | 作用 | 设计意图 |
|----|------|------|----------|
| cos(θ) | +8.0 | 竖直向上最大 | 主目标：保持平衡 |
| dθ² | -0.18 | 惩罚摆角速度 | 减少摆动、微振 |
| θ_arm² | -0.9 | 惩罚臂偏离中心 | 抑制滑移，臂尽量不动 |
| dθ_arm² | -0.8 | 惩罚臂速度 | 臂尽量静止 |
| torque² | -0.006 | 惩罚大扭矩 | 控制更平滑 |
| action_delta² | -1.5 | 惩罚动作突变 | 抑制抽搐 |

### 6.3 奖励设计原则

1. **主目标突出**：cos(θ) 权重最大，保证“平衡”优先
2. **多目标平衡**：在保持平衡的前提下，兼顾臂静止、平滑等
3. **量级匹配**：各项系数需调参，避免某一项完全主导
4. **稀疏 vs 稠密**：本奖励每步都有，属于稠密奖励，便于学习

### 6.4 终止条件

```python
terminated = bool(abs(theta) > (np.pi / 2 + 0.3))
```

- 摆角超过约 120° 视为倒下，episode 结束
- 给策略明确的失败信号，便于学习“不要倒”

---

## 七、PPO 算法简介

### 7.1 什么是 PPO？

**PPO（Proximal Policy Optimization）** 是一种策略梯度算法，由 OpenAI 提出，特点包括：

- 实现简单
- 样本效率较好
- 训练稳定
- 广泛应用于机器人、游戏等

### 7.2 策略梯度思想

1. 策略用神经网络表示：输入状态，输出动作分布
2. 与环境交互收集轨迹 (s, a, r, s', ...)
3. 计算每个动作的“优势”（比平均好多少）
4. 更新策略：增加好动作的概率，减少差动作的概率

### 7.3 PPO 的“Proximal”含义

- 限制每次策略更新的幅度，避免更新过大导致性能崩溃
- 通过裁剪目标函数实现，使新旧策略不会偏离太远

### 7.4 本项目的 PPO 参数

```python
model = PPO(
    "MlpPolicy",           # 多层感知机策略网络
    env,
    verbose=1,
    learning_rate=3e-4,    # 学习率
    n_steps=2048,         # 每次更新前收集的步数
    batch_size=256,       # 小批量大小
    gamma=0.995,          # 折扣因子
    gae_lambda=0.95,      # GAE 的 λ 参数
    n_epochs=10,          # 每批数据重复训练轮数
    policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128]))
)
```

| 参数 | 含义 | 调参建议 |
|------|------|----------|
| learning_rate | 梯度下降步长 | 过大易不稳定，过小收敛慢 |
| n_steps | 每次更新用的步数 | 越大方差越小，但更新频率低 |
| gamma | 未来奖励折扣 | 接近 1 更重视长期回报 |
| gae_lambda | 优势估计的 λ | 0.9~0.99 常用 |
| net_arch | 网络结构 | pi=策略，vf=价值函数 |

---

## 八、训练与推理流程

### 8.1 训练流程

```python
env = RotaryPendulumEnv()
check_env(env)  # 检查环境是否符合规范
model = PPO(...)
model.learn(total_timesteps=300000)
model.save("rotary_rl_model")
```

- `check_env`：验证 `reset`、`step` 返回值格式等
- `learn`：约 30 万步交互，自动完成采集数据、更新策略
- `save`：保存为 `rotary_rl_model.zip`（及可能的数据文件）

### 8.2 推理流程（训练脚本内嵌）

```python
obs, _ = env.reset(options={"demo": True})
with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
    while viewer.is_running():
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            obs, _ = env.reset(options={"demo": True})
        viewer.sync()
        time.sleep(0.01)
```

- `deterministic=True`：选概率最高的动作，而非采样，演示更稳定
- 倒下后自动 `reset`，开始新 episode

---

## 九、演示脚本 rl_demo.py

### 9.1 功能

- 加载已训练模型，无需重新训练
- 支持指定模型路径
- 支持 demo 模式 / 训练模式切换

### 9.2 使用方法

```bash
# 默认模型、demo 模式
python rl_demo.py

# 指定模型
python rl_demo.py -m path/to/model

# 使用训练时的随机初始状态
python rl_demo.py --no-demo-mode
```

### 9.3 核心逻辑

```python
model = PPO.load(args.model)
env = RotaryPendulumEnv()
obs, _ = env.reset(options={"demo": True})
# ... 与训练脚本相同的循环
```

---

## 十、参数调优建议

### 10.1 若平衡不稳

- 增加 `max_torque`（如 15）
- 增加训练步数（如 500000）
- 提高 cos(θ) 的奖励系数

### 10.2 若臂抖动大

- 增强 θ_arm²、dθ_arm² 惩罚
- 增强 action_delta² 惩罚
- 加强动作平滑（如 0.92/0.08）

### 10.3 若收敛慢

- 适当提高 learning_rate（如 5e-4）
- 增大 n_steps（如 4096）
- 检查奖励量级是否合理

### 10.4 若过拟合（训练好、演示差）

- 增加初始状态随机范围
- 训练时加入轻微噪声
- 适当增大网络或正则化

---

## 附录：关键术语表

| 术语 | 英文 | 含义 |
|------|------|------|
| 智能体 | Agent | 做决策的主体，本项目中为神经网络 |
| 环境 | Environment | 被控制的对象，本项目中为 MuJoCo 仿真 |
| 状态 | State | 环境的完整描述 |
| 观测 | Observation | 智能体实际能“看到”的状态（可能与状态不同） |
| 动作 | Action | 智能体可执行的操作 |
| 奖励 | Reward | 对当前状态/动作的标量评价 |
| 策略 | Policy | 状态→动作的映射，本项目中为神经网络 |
| 回合/片段 | Episode | 从 reset 到 terminated/truncated 的一次完整交互 |
| 折扣因子 | Gamma (γ) | 未来奖励的衰减系数 |
| 优势 | Advantage | 某动作相对于平均水平的优劣程度 |

---

## 参考资源

- [Gymnasium 文档](https://gymnasium.farama.org/)
- [Stable-Baselines3 文档](https://stable-baselines3.readthedocs.io/)
- [PPO 论文](https://arxiv.org/abs/1707.06347) (Schulman et al., 2017)
- [MuJoCo 文档](https://mujoco.readthedocs.io/)
