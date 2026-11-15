# A-UAV 参数接入记录

2026-11-15 · 来源：`A-UAV-20261115.zip`

## 数据核对

参数包内`cad/airframe.step`与项目A整机STEP的SHA256一致。项目已确认该CAD为最新版本。

使用`component_mass_properties.csv`独立重算质量、重心和惯量，并在临时副本执行参数脚本，结果与完整精度JSON一致。脚本会重写STL法向，重算工作使用隔离副本。[来源哈希与审计结果](audit.json)

| 项目 | 当前基线 |
|---|---|
| BOM总质量 | 1.15682kg |
| 已定位质量 | 0.76382kg |
| 待定位质量 | 0.393kg，暂置CAD原点 |
| 待补充质量的CAD实例 | 5个，包括雷达、夹爪及连接件 |
| BOM/CAD数量差异 | 电调4/2，M2螺母10/6 |

## 模型配置

- 固定构形刚体；柔性、舵机和夹爪关节锁定；碰撞体采用简化包络。
- 质量、重心和惯量采用BOM加权近似。
- 坐标约定：Gazebo FLU的X=-CAD Z、Y=-CAD X、Z=CAD Y。
- 同步变换网格、重心、电机位置及完整惯性张量：`c_flu = R c_cad`，`I_flu = R I_cad R^T`。
- 电机方向矩阵按列向量格式转置后再变换；当前有效推力轴设为模型+Z，旋向按仿真配置分配。
- 电机安装原点暂作力作用点，PX4力臂相对整机近似重心计算。
- 四个9g桨载体的质量、一次矩和惯量从基体扣除。独立汇总SDF后，整体质量、重心和张量与来源在1e-12绝对容差内一致。[核对结果](sdf_mass_check.json)
- 沿用整机原桨网格；独立HQ桨作为后续替换资产。
- 推力系数8.54858e-6 N/(rad/s)²、反扭矩比0.016m、最大转速1000rad/s，取自现有仿真基线；响应时间沿用标准模型配置。动力数据后续按规格或测试更新。

## 飞行回归

4ms物理步长、回环通信、原PID增益，执行约5m起飞、悬停、Land和自动上锁。

| 指标 | 结果 |
|---|---:|
| 最大相对高度 | 5.008m |
| 连续稳定悬停 | 8.208仿真秒 |
| 解锁期间最大水平偏移 | 0.060m |
| Land至接地 | 8.896仿真秒 |
| 接地至自动上锁 | 2.012仿真秒 |
| ULog dropout记录数 | 0 |

[原始ULog](https://github.com/RolyForden/PX4SIM/releases/download/a-uav-baseline-2026-11-15/rigid_20261115_110607.ulg) · [分析JSON](../analysis/rigid_20261115_110607_summary.json) · [曲线](../analysis/rigid_20261115_110607.png) · [参数核对](flight_parameter_audit.json)

ULog SHA256：`86041fc19802faa280ad37df3cb4ee94456d939bcc0633f6470c628411771430`。

输入JSON SHA256：`8b942c1af15e84174ff1f5c4e5f94bf70f5f981fb8b3fb950909a3e7d342bed1`。

参数从`settings.json`的`mass_properties_json`读取。复现顺序：审计 → 构建 → SDF质量核对 → 分配检查 → 飞行测试，命令见[运行说明](../README.md)。
