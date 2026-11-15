# PX4SIM 自研变构无人机仿真

自研 A 无人机已完成固定构形起飞、悬停和自动降落，已跑通 ROS 指令 X/H/V 变构，并完成 X/H10/V10 各 3 次固定构形飞行。下一步是 25° 构形和空中连续变构。

更新：2026年11月15日。

## 成果入口

- [A 无人机阶段成果](docs/A无人机阶段成果.md)：飞行结果、曲线与参数说明。
- [下载模型、参数包、录像和日志](https://github.com/RolyForden/PX4SIM/releases/tag/a-uav-baseline-2026-11-15)。
- [后续实施计划](docs/OpenCode后续执行计划.md)：关节模型、ROS接口及分阶段验证。

## 工作进度

| 工作 | 当前成果 | 下一步 |
|---|---|---|
| 标准 X500 | RTL返航着陆、Land、降落速度对照 | 保留为回归基线 |
| 自研 A 模型 | 最新STEP、123部件解析、独立SDF、PX4 airframe4999 | 运动部件分组及柔性关节近似 |
| A 固定构形飞行 | 约5m起飞、稳定悬停、Land、接地、自动上锁 | H10/V10 已通过；更大角度与重复性 |
| H/V 变构 | A近似台架X/H/X/V/X反馈；ROS指令已跑通 | 25°固定构形，再做空中连续变构 |
| 固定 H/V 飞行 | X/H10/V10 各 3 次起飞、悬停、Land、接地、自动上锁通过 | 25° 构形与空中变构 |
| 推力分配 | 6×4效能矩阵、A固定构形有界分配检查 | 随实际关节角更新矩阵 |

## A 无人机飞行结果

![固定构形飞行曲线](px4_rtl_land_validation/candidate_flight/analysis/demo_20261115_112750.png)

最大相对高度 **5.002m**，稳定悬停 **10.168仿真秒**，Land至接地 **9.032仿真秒**，接地至自动上锁 **2.012仿真秒**。Gazebo与QGC联合录像约37秒，采用脚本自动起降。

## 文件目录

| 内容 | 入口 |
|---|---|
| A模型代码与运行方法 | [candidate_flight](px4_rtl_land_validation/candidate_flight/) |
| 质量属性与参数假设 | [基线接入记录](px4_rtl_land_validation/candidate_flight/handoff_audit/report.md) |
| 交底书、构形图、关节图、绕线图 | [现实材料](px4_rtl_land_validation/现实材料/) |
| 模型资产及交接内容 | [资产清单](px4_rtl_land_validation/现实材料/模型资料交接清单.md) |
| A 近似变构台架 | [a_morphing](px4_rtl_land_validation/a_morphing/) · [第一轮结果](px4_rtl_land_validation/a_morphing/results/README.md) |
| 固定 H/V 小角飞行 | [fixed_flight](px4_rtl_land_validation/a_morphing/fixed_flight/) · [飞行结果](px4_rtl_land_validation/a_morphing/fixed_flight/results/README.md) · [ULog Release](https://github.com/RolyForden/PX4SIM/releases/tag/a-uav-fixed-hv-2026-11-15) |
| 标准X500测试 | [测试记录](docs/标准四旋翼验证.md) · [日志下载](https://github.com/RolyForden/PX4SIM/releases/tag/validation-2026-11-14) |

自研A整机及参数位于Release附件`A-UAV-20261115.zip`；`现实材料/UAV.STEP`用于早期参考台架。

单个文件可用 **Raw / Download raw file** 下载；Release附件位于 **Assets**；完整源码可用 **Code → Download ZIP** 获取。运行环境见[复现说明](docs/实验复现说明.md)。

早期参考台架与阶段记录见[归档](px4_rtl_land_validation/archive/)。
