# A 无人机固定构形阶段成果

2026-11-15 · [返回首页](../README.md)

## 模型与环境

最新A整机STEP包含123个部件实例，已生成独立Gazebo模型和PX4 airframe4999。环境为PX4 v1.17.0、Gazebo Harmonic 8.15.0、QGC v5.0.8，采用原PID增益。

## 飞行结果

两次测试均完成约5m起飞、稳定悬停、Land、接地和自动上锁。

| 日志 | 稳定悬停（仿真秒） | 最大高度 | Land至接地 | 接地至上锁 |
|---|---:|---:|---:|---:|
| rigid_20261115_110607.ulg | 8.208 | 5.008m | 8.896s | 2.012s |
| demo_20261115_112750.ulg | 10.168 | 5.002m | 9.032s | 2.012s |

两份ULog的dropout记录数均为0。[分析JSON与曲线](../px4_rtl_land_validation/candidate_flight/analysis/) · [日志及录像](https://github.com/RolyForden/PX4SIM/releases/tag/a-uav-baseline-2026-11-15)。

联合录像约37秒，显示Gazebo和QGC实时状态，起降指令通过MAVLink脚本执行。

## 测试配置

本轮采用固定构形刚体模型，柔性、舵机和夹爪关节锁定。质量属性来自BOM加权近似，动力系数采用现有基线值。完整配置见[参数接入记录](../px4_rtl_land_validation/candidate_flight/handoff_audit/report.md)。

## 后续工作

依据交底书和附图建立单舵机联动的H/V近似关节模型，接入ROS指令和关节反馈，再开展固定H/V构形飞行及动态分配验证。

## 后续：固定 H/V 10° 小角飞行（2026-11-15）

在近似变构台架之后，已生成固定 H 10°（airframe 4998）和固定 V 10°（airframe 4997）两个独立模型，并完成 X 回归、H10、V10 的起飞、约 5 m 悬停、Land、接地和自动上锁。三次 ULog dropout 均为 0。

- [固定 H/V 飞行说明](../px4_rtl_land_validation/a_morphing/fixed_flight/README.md)
- [飞行结果与曲线](../px4_rtl_land_validation/a_morphing/fixed_flight/results/README.md)
- [离线悬停检查](../px4_rtl_land_validation/a_morphing/fixed_flight/results/hover-feasibility.json)

这仍是 10° 固定小角构形，不是 25° 构形、空中连续变构或最终样机验证。
