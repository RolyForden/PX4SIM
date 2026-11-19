# A 无人机：变构与机械臂耦合悬停

5 m 悬停中，依次执行固定 H25／V25 下的机械臂完整轨迹，再执行 X→H25→X 与机械臂左右往返、X→V25→X 与机械臂连续画圆。先验收空载，再增加 50 g 复现相同序列。

[实际结果与问题记录](results/README.md) · [原生 ULog 清单](results/ulog-manifest.json) · [原生矩阵读回](results/native-matrices.json)

## 集成路径

```text
ROS 变构命令 → 仿真时间插补 → Gazebo 8 个变构关节
ROS 机械臂命令 → CAD 逆解 → 3 个主动关节力矩 → 原生闭链
                                      ↓
             Gazebo 实际链接位姿 → 整机重心／惯量、旋翼位置／方向
                                      ↓
               ROS 动态分配几何 → 专用 PX4 SITL 分配接口
                                      ↓
                          PX4 控制 → 四旋翼推力
```

机械臂通过十二个球铰与六处运行时固定连接闭合。机身无世界固定点，运动反作用通过独立安装接口传递。完整物理矩阵 `B` 保留三轴力矩和三轴力六行；实际控制矩阵 `B_control` 选择三轴力矩及机体 Z 推力，Fx/Fy 控制行置零。倾斜旋翼的横向推力仍在 Gazebo 中作用，由位置／姿态闭环处理，不作为两个额外独立控制自由度。

新接口的原因和首轮故障回放见结果报告。PID 与机械臂增益沿用前轮，位置／姿态控制器和原生分配求解器沿用；本轮使用独立 `px4_coupled` SITL 可执行程序，构建时临时应用适配补丁，结束后恢复基线源码和二进制。

## 文件职责

- `build_model.py`：从已验证的 A 集成快照生成可变构、可操作机械臂的 `world.sdf`，保留修正后的质量与惯量。IMU 5000 Hz，物理步长 0.2 ms，独立 airframe 4993。
- `controller.cpp`：原生机械臂闭链、关节力矩驱动、安装接口六维力，逐链接汇总实际整机重心和张量，读取实际旋翼位姿。
- `ros_control.py`：机械臂轨迹、逆解与反馈；`demo` 完整轨迹，`sweep` 四次左右往返，`circles` 三次整圆，`hold` 保持当前位置。
- `gz_link.cpp`：容器内 Gazebo 原生关节反馈与位置目标通信；不是主机 Fortress 的 ros_gz_bridge。
- `morph_control.py`：按仿真时间以峰值 5°/s 平滑变构；根据实际整机重心及旋翼位姿形成动态矩阵，接收 PX4 已应用几何的序号回执。
- `build_px4.py`、`px4-morph-geometry.patch`：构建本轮专用分配接口，保存补丁及二进制哈希，恢复基线。
- `check_geometry.py`：H/V 0–25° 的秩、悬停推力及余量检查；`replay_inverse.cpp`：首轮问题的实际几何回放。
- `test_flight.py`：起降、分阶段命令、状态与中止监测；`capture_live.py`：两机构同时运动时读回原生矩阵。
- `analyze.py`、`crosscheck_mass.py`、`audit.py`：原生 ULog／真实位姿验收、独立重心与张量重构、源文件与参数核验。
- `finalize_log.py`：PX4 退出、缓冲写完后复制原始 ULog；`summarize.py`：曲线、逐阶段指标、日志清单。

## 本机运行

复用已有 Docker Harmonic、ROS 2 Humble、PX4 编译环境及模型网格。`build_px4.py` 仅在需要重新构建专用程序时运行。以下从工作空间根目录执行，使用新试验名：

```bash
python3 px4_rtl_land_validation/a_coupled/launch.py --name empty_new --payload 0
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_coupled/test_flight.py empty_new
```

同一试验还需并行运行原生动态矩阵采集：

```bash
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_coupled/capture_live.py empty_new
```

完整飞行结束、已落地上锁后：

```bash
python3 px4_rtl_land_validation/a_coupled/stop.py
python3 px4_rtl_land_validation/a_coupled/finalize_log.py empty_new
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_coupled/analyze.py empty_new
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_coupled/crosscheck_mass.py empty_new
```

`launch.py` 会打开当前世界对应的 Gazebo GUI，使用 NVIDIA 图形环境。测试期间由脚本独占控制，不能同时从 QGC 或其他终端发飞行／机构命令。记录读取只消费完整 JSON 行，未写完的行保留到下一次读取。

## ROS 接口

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /a_uav/coupled/morph/status --once
ros2 topic echo /a_uav/coupled/arm/status --once
ros2 topic pub --once /a_uav/coupled/morph/command std_msgs/msg/String '{data: "H:25"}'
ros2 topic pub --once /a_uav/coupled/morph/command std_msgs/msg/String '{data: "X"}'
ros2 topic pub --once /a_uav/coupled/morph/command std_msgs/msg/String '{data: "V:25"}'
ros2 topic pub --once /a_uav/coupled/arm/command std_msgs/msg/String '{data: "demo"}'
```

仅供独立人工演示使用，自动试验时不另发。H/V 切换须先回 X，并等待 `ready=true`、`transition=false`；机械臂等待 `busy=false` 和完成序号更新。中止时分别发送 morph `HOLD` 与 arm `hold`，保持当前关节角，再由飞控处理降落，不突然强制回 X。

当前世界文件是最后生成的工况；每次 `runs/<名称>/` 保存独立世界、源代码、参数、原始反馈和 ULog。原始运行目录与二进制保留本地。
