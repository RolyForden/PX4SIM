# A 无人机近似变构台架

**这是参考台架，不是最终样机，也不是飞行验证。** 它把 A 整机按四个机臂近似分组，用等效 yaw/pitch 关节表达 X/H/V 构形，用于验证运动学、关节反馈和 ROS 指令链路。

## 当前状态

- 已生成 A 近似变构 SDF、四机臂网格和集中配置。
- 台架固定、零重力、无碰撞、无电机推力。
- 已通过 Gazebo 实际读取关节角、旋翼位置和轴向。
- 已实现 ROS 2 指令与状态节点，但当前使用 `docker exec gz topic` 作为临时适配；不是 `ros_gz_bridge` 正式桥。
- 尚未接入 PX4，未做固定 H/V 飞行，未做空中连续变构。

## 目录

| 路径 | 内容 |
|---|---|
| `config/morph-config.json` | 机臂分组、质量、关节位置/轴/限位、X/H/V 目标、全部假设 |
| `scripts/build_bench.py` | 从 candidate 网格和 A 交接质量数据生成台架（重新生成需要本地 candidate_flight/meshes 和 A-UAV 参数包） |
| `scripts/verify_bench.py` | 执行 X→H→X→V→X，保存实际反馈 |
| `scripts/compute_allocation.py` | 根据实测位姿计算 X/H/V 推力效能矩阵 |
| `models/a_morph_bench/` | 生成的 SDF 和网格 |
| `worlds/a_morph_bench.sdf` | 固定台架世界 |
| `ros2/` | ROS 2 指令、状态节点和 launch |
| `results/` | 序列反馈、位姿快照、分配矩阵、ROS 指令日志 |

## 假设边界

- 机臂由 CAD 部件按最近旋翼和径向距离近似分组，不是实际铰链识别。
- 关节中心取距中心 0.075 m、z=0.0085 m；H/V 角均取 25°，都是可替换演示值。
- 每个机臂用 yaw+pitch 两个等效自由度协调，不代表实物有多个独立舵机。
- 实物是单舵机同轴双层反向绕线；H 与 V 是同一输入的不同分支，H↔V 第一版经 X 复位。
- 质量来自 BOM 近似；0.393 kg 未定位质量仍在机身；5 个 CAD 实例没有 BOM 质量。
- 机身/机臂惯量为近似对角值；不是完整实测张量，台架不用于飞行或动态变构结论。
- 第一轮只做主动位置控制；弹簧/阻尼被动回弹、绳索和 TPU 柔性响应尚未实现。
- 仓库内网格是低精度视觉版本，用于控制仓库体积；运动学结果不依赖网格分辨率，精确 CAD 网格仍在本地/Release。

## 运行

容器已存在，项目目录挂载到 `/work`。启动 Gazebo 台架：

```sh
docker exec -d -e GZ_IP=127.0.0.1 -e GZ_PARTITION=a_morph_bench \
  -e GZ_SIM_RESOURCE_PATH=/work/a_morphing/models \
  px4-rtl-land-harmonic bash -lc \
  'gz sim -s -r -v 3 /work/a_morphing/worlds/a_morph_bench.sdf >/tmp/a_morph_server.log 2>&1'
```

执行 X→H→X→V→X 并保存实际反馈：

```sh
px4_rtl_land_validation/.venv/bin/python \
  px4_rtl_land_validation/a_morphing/scripts/verify_bench.py
```

ROS 2 节点：

```sh
source /opt/ros/humble/setup.bash
ros2 launch px4_rtl_land_validation/a_morphing/ros2/a_morph.launch.py

# 另一个终端
ros2 topic pub --once /a_uav/morph/command std_msgs/msg/String "{data: H}"
ros2 topic echo /a_uav/joint_states
ros2 topic echo /a_uav/morph/status
```

当前主机 `ros_gz_bridge` 是 Fortress 版本，不能直接桥接容器 Harmonic，因此 ROS 节点通过 `docker exec gz topic` 读取/发送。该适配明确标注为临时方案，后续应换成匹配 Harmonic 的 ROS-Gazebo 桥。

## 结果

- [结果说明](results/README.md)：X/H/V 序列、截图和离线分配摘要。
- `results/sequence.json`：X/H/X/V/X 的目标、实际角、误差、旋翼位置/轴向。
- `results/pose-*.json`：每一步的原始 Gazebo 位姿。
- `results/allocation.json`：X/H/V 的离线 6×4 推力效能矩阵、秩、悬停推力与残差。

离线分配不是 PX4 实时分配器，不能据此声称任意六自由度可控或真实动力足够。
