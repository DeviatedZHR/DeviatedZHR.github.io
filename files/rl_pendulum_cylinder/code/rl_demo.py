"""
RL 模型演示脚本：加载已训练的 PPO 模型，在 MuJoCo 中实时演示倒立摆平衡控制。
运行前需先执行 rl_balance.py 完成训练并生成 rotary_rl_model。
"""
import os
import time
import argparse

import mujoco
import mujoco.viewer

from stable_baselines3 import PPO

# 导入环境定义（与 rl_balance.py 共用）
from rl_balance import RotaryPendulumEnv

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL_PATH = os.path.join(SCRIPT_DIR, "rotary_rl_model")


def main():
    parser = argparse.ArgumentParser(description="RL 倒立摆模型演示")
    parser.add_argument(
        "--model",
        "-m",
        type=str,
        default=DEFAULT_MODEL_PATH,
        help="模型路径（默认: rotary_rl_model）",
    )
    parser.add_argument(
        "--no-demo-mode",
        action="store_true",
        help="关闭演示模式，使用训练时的随机初始状态",
    )
    args = parser.parse_args()

    if not os.path.exists(args.model + ".zip") and not os.path.isdir(args.model):
        print(f"错误：未找到模型文件 {args.model}")
        print("请先运行 python rl_balance.py 完成训练。")
        return 1

    print(f"加载模型: {args.model}")
    model = PPO.load(args.model)

    env = RotaryPendulumEnv()
    options = None if args.no_demo_mode else {"demo": True}
    obs, _ = env.reset(options=options)

    print("演示开始（关闭窗口退出）...")
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        while viewer.is_running():
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, _ = env.step(action)

            if terminated or truncated:
                obs, _ = env.reset(options=options)

            viewer.sync()
            time.sleep(0.01)

    print("演示结束")
    return 0


if __name__ == "__main__":
    exit(main())
