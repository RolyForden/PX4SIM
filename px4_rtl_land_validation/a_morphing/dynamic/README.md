# 动态变构链接口与地面联调

本目录实现“实际关节反馈 → 当前重心/旋翼几何 → 推力效能矩阵 B → 有界悬停分配”的地面联调链。当前输出 JSON 数据，供后续 PX4 适配器消费。

## 接口

| 方向 | 话题 | 类型 | 内容 |
|---|---|---|---|
| 输入 | `/a_uav/joint_states` | `sensor_msgs/JointState` | Gazebo 实际关节角，顺序 `yaw_0,pitch_0,...,yaw_3,pitch_3` |
| 输入 | `/a_uav/morph/command` | `std_msgs/String` | `X`、`H`、`V` |
| 输出 | `/a_uav/allocation/matrix` | `std_msgs/String` JSON | 当前构形、关节角、重心、6×4 B 矩阵、秩、条件数 |
| 输出 | `/a_uav/allocation/motor_thrust` | `std_msgs/String` JSON | 有界悬停分配的四桨推力、残差、单桨上限 |
| 输出 | `/a_uav/allocation/status` | `std_msgs/String` JSON | 有效标志、构形、输入周期、矩阵周期、计算耗时、链路延迟 |

矩阵行顺序：`τx, τy, τz, Fx, Fy, Fz`；输入为单桨实际推力 N。

## 更新周期与延迟

地面联调实测（150 s 级序列，20 Hz 矩阵更新）：

| 指标 | 中位数 | P95 |
|---|---:|---:|
| 关节反馈周期 | 200.0 ms | 212.9 ms |
| 矩阵更新周期 | 50.0 ms | 50.1 ms |
| 单次矩阵+分配计算 | 2.12 ms | 2.65 ms |
| 关节消息到矩阵发布延迟 | 101.9 ms | 190.0 ms |

关节反馈来自当前 `docker exec gz topic` 临时适配，周期约 5 Hz；矩阵链以 20 Hz 运行，使用最新反馈并记录输入年龄。

## 状态与失效处理

- `X/H/V` 构形切换经 X 复位。
- 关节反馈超过 1.5 s 未更新时，`/a_uav/allocation/status` 标记 `valid=false`，不输出有效分配。
- 分配在 `0..max_thrust` 范围内求解，输出残差和单桨余量。
- 当前阶段只发布数据，不向 PX4 执行器写输出。

## 后续 PX4 适配

- 将 `/a_uav/allocation/motor_thrust` 映射到 PX4 执行器/控制分配入口（uXRCE-DDS 或 MAVLink）。
- 在正式 Harmonic ROS 桥下提升关节反馈到 50–100 Hz。
- 加入推力饱和、变化率限制、超时和构形切换状态机。
- 完成后再安排空中小角慢速变构测试。

## 运行

```sh
# 启动台架（见 ../README.md）
# 启动状态、指令和动态链
source /opt/ros/humble/setup.bash
ros2 launch px4_rtl_land_validation/a_morphing/dynamic/ros2/a_morph_dynamic.launch.py

# 另一个终端发送构形
ros2 topic pub --once /a_uav/morph/command std_msgs/msg/String "{data: H}"
```

动态链日志默认写入 `dynamic/results/ground-integration.jsonl`，可用 `PX4SIM_DYNAMIC_LOG` 覆盖。

## 目录

| 路径 | 内容 |
|---|---|
| `ros2/a_morph_dynamic.py` | 动态链节点 |
| `ros2/a_morph_dynamic.launch.py` | 状态、指令、动态链 launch |
| `results/ground-integration.jsonl.gz` | 本次地面联调原始更新日志 |
| `results/ground-integration-summary.json` | 周期、延迟和分构形统计 |
| `results/README.md` | 地面联调结果 |
