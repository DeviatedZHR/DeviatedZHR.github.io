# 关键代码片段

本目录存放从 Hero H7 四足轮腿控制代码中摘取的关键片段，便于快速查阅和移植。

| 文件 | 来源 | 说明 |
|------|------|------|
| fun_Jacobi_get_Force.c | fun_Jacobi.c | VMC 虚拟力计算 |
| fun_Jacobi_get_torque.c | fun_Jacobi.c | 雅可比力矩映射 |
| drv_motionSolve_FootWheel.c | drv_motionSolve.c | 足轮正/逆运动学 |
| drv_motionSolve_Mecanum.c | drv_motionSolve.c | 麦轮逆/正运动学 |
| drv_wheel_VMC_switch.c | drv_wheel.c | VMC 与位置模式切换 |
| mod_motion_FootPack_Present.c | mod_motion.c | 预备模式 |
| mod_motion_ExMotMod_Output.c | mod_motion.c | 统一运动输出接口 |

**注意**: 这些片段为示意代码，包含必要的宏和类型引用，直接编译需补充头文件和依赖。
