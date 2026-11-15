# A-UAV 交接基线接入记录

日期：2026-11-15。来源为用户交接的 `A-UAV-20261115.zip`，已解压，原始文件未改写。

## 来源与数据审计

已先读取包内README和完整精度JSON。包内 `cad/airframe.step` 与当前 `A无人机.STEP` SHA256一致，确认这是自研A整机，不是旧下载示例 `UAV.STEP`；是否对应最终加工版仍未独立确认。

独立使用 `component_mass_properties.csv` 重算质量、重心、惯量，并在隔离副本运行交接脚本，均与JSON一致。交接脚本的 `mesh_bounds` 会重写STL法向，因此没有在原始包上直接执行。原件SHA256清单和检查结果见 [audit.json](audit.json)。

JSON质量为 1.15682kg，已定位几何的质量0.76382kg；未定位0.393kg，占总质量约 33.97%，按原交接假设集中在CAD原点。5个CAD实例没有BOM质量，其中包含雷达、夹爪及连接件，不代表真实重量为零。电调BOM/CAD数量为4/2，M2螺母为10/6；这些没有擅自修补。

## 坐标与SDF接入

继续使用现有**临时软件调试坐标**：Gazebo FLU的 X=-CAD Z、Y=-CAD X、Z=CAD Y。此选择不确认实际机头或上方，记录在settings/config中；后续方向确认后须整体重算。

使用 `c_flu = R c_cad`、`I_flu = R I_cad R^T`，保留完整精度及带符号非对角项。电机位置同步转换；方向字段先转置形成通常的矩阵，再左乘R。没有把某个方向基向量自动判定为真实推力轴。当前有效推力向上与旋向仍为明确的仿真指定值。

电机安装原点暂作仿真力作用点，但标为未核实，不称作已知推力中心。PX4力臂按**整机近似重心**相对位置生成，而不是继续把CAD原点当重心。

没有粘贴URDF片段冒充SDF；SDF明确写出基体惯性坐标pose和六个张量分量。四个9g桨执行器载体使用简化自惯量，它们的质量、一次矩和惯量贡献从基体中扣除；从生成SDF独立重组后，整体质量、重心和完整惯量与来源的转换结果在1e-12绝对容差内一致。见 [sdf_mass_check.json](sdf_mass_check.json)。

原整机桨继续使用；未额外叠加独立HQ桨或额外桨质量。两份HQ模型仅登记为待替换资产，旋向和安装关系未确认。

## 实际仿真回归

新基线下已实际执行一次约5米起飞、悬停、Land、接地与自动上锁。保持4ms步长、回环Gazebo通信和原PID增益，使用无GUI服务，未连接真实硬件。

| 指标 | ULog结果 |
|---|---:|
| 最大相对高度 | 5.008m |
| 连续稳定悬停，仿真时间 | 8.208s |
| 解锁期间最大水平偏移 | 0.060m |
| Land指令至接地 | 8.896s |
| 接地至自动上锁 | 2.012s |
| ULog dropout记录数 | 0 |

- [原始ULog](https://github.com/RolyForden/PX4SIM/releases/download/a-uav-baseline-2026-11-15/rigid_20261115_110607.ulg)，SHA256 `86041fc19802faa280ad37df3cb4ee94456d939bcc0633f6470c628411771430`。
- [分析汇总](../analysis/rigid_20261115_110607_summary.json)、[曲线](../analysis/rigid_20261115_110607.png)、[参数核对](flight_parameter_audit.json)。
- 输入JSON SHA256：`8b942c1af15e84174ff1f5c4e5f94bf70f5f981fb8b3fb950909a3e7d342bed1`。每次测试归档输入配置、模型SDF、world、前后PX4参数及事件。

## 结论边界及复现

这证明交接的BOM近似质量属性已一致接入固定构形刚体仿真，并通过一次基础飞行回归；不证明真实整机性能、柔性关节响应、动态变构、实机推力标定或最终机型验证。

电机与桨型号仅作用户提供的信息登记。未用KV、500g/500N推算最大推力；推力/反扭矩/响应时间/最大转速仍沿用已标注的借用值，真实电池电压、旋向等保留未知。

质量属性优先由 `settings.json` 的 `mass_properties_json` 路径读取，无需手抄重心/惯量。原整机STEP哈希相同，因此沿用既有分部件网格，避免把含桨的整机STL与独立桨重复显示。

在candidate_flight目录对应脚本中，依次执行 `audit_handoff.py`（只在隔离副本复算）、`build_flight.py`、`verify_sdf_mass.py`、`check_allocation.py`，再按目录README启动并测试。旧占位值基线及旧通过日志已保留在diagnostic_variants和logs中。

本报告原记录对应单次无GUI回归。随后完成QGC联合显示和录像，见仓库阶段成果；交接包、两份ULog及录像已随2026-11-15成果整理发布。进程状态须使用时另行检查。
