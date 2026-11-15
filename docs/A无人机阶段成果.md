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

依据交底书和附图建立单舵机联动的 H/V 近似关节模型，接入 ROS 指令和关节反馈，并完成固定 H/V 10° 小角飞行。下一步开展动态分配验证。

## 后续：固定 H/V 10° 小角飞行（2026-11-15）

在近似变构台架之后，已生成固定 H 10°（airframe 4998）和固定 V 10°（airframe 4997）两个独立模型，并完成 X 回归、H10、V10 各 3 次飞行，共 9 次起飞、约 5 m 悬停、Land、接地和自动上锁。9 次 ULog dropout 均为 0。

- [固定 H/V 飞行说明](../px4_rtl_land_validation/a_morphing/fixed_flight/README.md)
- [飞行结果与曲线](../px4_rtl_land_validation/a_morphing/fixed_flight/results/README.md)
- [离线悬停检查](../px4_rtl_land_validation/a_morphing/fixed_flight/results/hover-feasibility.json)

下一阶段：25° 固定构形和空中连续变构。

## 动态变构链地面联调（2026-11-15）

已完成“实际关节反馈 → 当前重心/旋翼几何 → 6×4 推力效能矩阵 → 有界悬停分配”的地面联调。矩阵链以 20 Hz 运行，关节反馈约 5 Hz；单次计算中位 2.12 ms，关节消息到矩阵发布延迟中位 102 ms、P95 190 ms。X/H/V 构形下矩阵秩均为 4，V 构形同步更新旋翼位置和推力轴。

- [动态链接口与地面联调](../px4_rtl_land_validation/a_morphing/dynamic/README.md)
- [联调结果](../px4_rtl_land_validation/a_morphing/dynamic/results/README.md)

下一阶段：空中小角慢速变构测试。
