import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

import os
xml_path = os.path.join(os.path.dirname(__file__), 'rotary_pendulum.xml')

class RotaryPendulumEnv(gym.Env):

    def __init__(self):

        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # 最大力矩限制：需与 PID 相当，否则无法有效平衡（PID 平衡时约 10）
        self.max_torque = 12.0

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(1,),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(6,),
            dtype=np.float32
        )

        self.prev_action = 0

    def get_obs(self):
        theta_arm = self.data.qpos[0]
        theta_pend = self.data.qpos[1]
        dtheta_arm = self.data.qvel[0]
        dtheta_pend = self.data.qvel[1]
        # 臂角度归一化到 [-pi, pi]，避免无界输入影响网络
        theta_arm_wrapped = (theta_arm + np.pi) % (2 * np.pi) - np.pi
        return np.array([
            np.cos(theta_pend),
            np.sin(theta_pend),
            theta_arm_wrapped * 0.3,  # 缩放使量级与其它观测相近
            dtheta_arm,
            dtheta_pend,
            self.prev_action
        ], dtype=np.float32)

    def reset(self, seed=None, options=None):

        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        # options 含 demo=True 时使用小扰动，便于演示
        if options and options.get("demo"):
            self.data.qpos[1] = np.random.uniform(-0.15, 0.15)
            self.data.qpos[0] = np.random.uniform(-0.1, 0.1)
        else:
            self.data.qpos[1] = np.random.uniform(-0.5, 0.5)
            self.data.qpos[0] = np.random.uniform(-0.3, 0.3)

        self.prev_action = 0

        return self.get_obs(), {}

    def step(self, action):

        raw_action = float(action[0])
        # 适度增加平滑 + 单步限幅（±0.24），进一步减弱微小抽搐
        action_delta = np.clip(raw_action - self.prev_action, -0.24, 0.24)
        action = 0.9 * self.prev_action + 0.1 * (self.prev_action + action_delta)

        torque = float(action * self.max_torque)
        self.data.ctrl[0] = torque

        for _ in range(5):
            mujoco.mj_step(self.model, self.data)

        self.prev_action = action
        obs = self.get_obs()

        theta = self.data.qpos[1]
        dtheta = self.data.qvel[1]
        theta_arm = self.data.qpos[0]
        dtheta_arm = self.data.qvel[0]

        # 奖励设计：竖直向上 cos(theta)=1 最优；惩罚角速度、臂漂移、动作突变
        action_delta_for_reward = raw_action - self.prev_action
        reward = (
            8.0 * np.cos(theta)           # 主目标：保持竖直
            - 0.18 * dtheta**2             # 抑制摆角速度（减少抽搐式微振）
            - 0.9 * theta_arm**2           # 强位置惩罚：抑制滑移，臂贴近中心
            - 0.8 * dtheta_arm**2          # 强速度惩罚：抑制臂运动
            - 0.006 * torque**2            # 惩罚大扭矩（控制更平滑）
            - 1.5 * action_delta_for_reward**2  # 强惩罚动作突变，抑制抽搐
        )

        # 终止条件：摆杆倒下（超过约 90 度）
        terminated = bool(abs(theta) > (np.pi / 2 + 0.3))
        truncated = False

        return obs, reward, terminated, truncated, {}



if __name__ == "__main__":

    env = RotaryPendulumEnv()

    check_env(env)

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=256,
        gamma=0.995,
        gae_lambda=0.95,
        n_epochs=10,
        policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128]))
    )

    print("开始训练 (约 10–15 分钟)...")

    model.learn(
        total_timesteps=300000
    )

    model.save("rotary_rl_model")

    print("训练完成")



    print("开始演示...")

    obs, _ = env.reset(options={"demo": True})

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:

        while viewer.is_running():

            action, _ = model.predict(obs, deterministic=True)

            obs, _, terminated, truncated, _ = env.step(action)

            if terminated or truncated:
                obs, _ = env.reset()

            viewer.sync()

            time.sleep(0.01)