# A 无人机悬停中的机械臂运动

从锁臂基线恢复原生 Delta 机构运动。在 3 m 悬停中执行完整末端轨迹，先空载，再做 50 g 负载对照。

[实际结果、问题与改动](results/README.md) · [对照数据](results/comparison.json) · [ULog 清单](results/ulog-manifest.json)

## 代码与实际执行路径

```text
build_model.py → world.sdf：自由机身、四旋翼、可动机械臂
ROS 轨迹 → CAD 逆解 → 三个目标关节角 → 插件关节力矩
                                         ↓
                       DART 球铰／闭链约束 → 机身反作用
                                         ↓
                            仿真传感器 → PX4 → 电机推力
```

- `build_model.py`：从已验证的集成快照恢复运动，移除世界固定点，恢复四个旋翼关节和电机插件；独立 airframe 为 4994。质量和初始完整惯量与锁臂基线逐项比较。
- `controller.cpp`：沿用集成台架的关节力矩驱动，保留十二个球铰和六个运行时闭链连接。读取实际关节角、机身／平台位姿、安装接口六维力、总重心及完整惯量。飞行版本去掉固定台架连接的初始化依赖。
- `ros_control.py`：沿用 CAD 逆解和梯形速度规划，增加左右往返、`hold` 保持指令，使用独立 `/a_uav/air_arm/` 命名空间。Gazebo 插件与 ROS 的本机 UDP 为 14860/14861。
- `launch.py`：同一世界预加载机身和独立末端平台，PX4 通过 `PX4_GZ_MODEL_NAME=a_arm` 接入已有模型，避免平台先自由落下、机身后生成。启动原生 PX4 二进制与 ROS 节点。
- `test_flight.py`：MAVLink 起降、10 秒静止悬停、ROS `demo`、10 秒恢复悬停、Land 和自动上锁。检查关节反馈、闭链和飞行状态，异常时保持机械臂并请求降落。
- `analyze.py`：同时读取原生 ULog 与 Gazebo 实际反馈，按运动前／中／后三段验收。`audit.py` 核验模型、来源和参数；`summarize.py`、`diagnose_sampling.py` 生成对照图和采样诊断。

本轮关节驱动保持原台架增益和 2 N·m 力矩限幅；所有 MC/MPC 参数保持锁臂基线值。分配器使用初始重心对应的旋翼几何，不在线写入 CA 参数。

## 本机执行

依赖现有容器 `px4-rtl-land-harmonic`、PX4 编译产物、集成模型快照／网格、ROS 2 Humble 及 `.venv`。从工作空间根目录执行，使用未占用的新试验名：

```bash
python3 px4_rtl_land_validation/a_arm_hover/launch.py --name empty_new --payload 0
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_arm_hover/test_flight.py empty_new
python3 px4_rtl_land_validation/a_arm_hover/stop.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_arm_hover/analyze.py empty_new
```

测试程序会实际起飞。飞行期间不要执行停止命令。50 g 对照使用 `--payload 0.05` 和新名称。结果目录保存本次世界、控制源代码、参数和日志；不会覆盖已有试验。当前生成的世界对应最后一次负载工况。

手动演示的 ROS 指令如下；自动测试期间不要另发指令：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /a_uav/air_arm/status --once
ros2 topic pub --once /a_uav/air_arm/command std_msgs/msg/String '{data: "demo"}'
ros2 topic pub --once /a_uav/air_arm/command std_msgs/msg/String '{data: "hold"}'
```

`demo` 依次执行回位、下降 25 mm、回位、X 正向 15 mm 往返、下降平面半径 15 mm 整圆、回位、Y 正向 15 mm→负向 15 mm→回位；速度 15 mm/s、加速度 30 mm/s²。等待 `busy: false` 且 `completed_id` 更新后再继续。`hold` 保持实际关节角，不触发突然回位。
