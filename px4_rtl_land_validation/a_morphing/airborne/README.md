# A 无人机空中变构

当前模型：`a_air`；PX4 airframe：`4996`。[成果与配置](../../../docs/当前成果.md)

## 环境与资产

本机环境为Ubuntu22.04、Docker容器`px4-rtl-land-harmonic`、ROS2 Humble和QGC v5.0.8。容器将`px4_rtl_land_validation/`挂载到`/work`，使用NVIDIA CDI。

PX4源码位于`px4_rtl_land_validation/PX4-Autopilot`，版本v1.17.0、提交`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，已具备SITL构建产物。

从[Release](https://github.com/RolyForden/PX4SIM/releases/tag/a-uav-airborne-2026-11-15)下载：

| 附件 | 用途与位置 |
|---|---|
| a-air-runtime-assets.zip | 解压到本目录，提供models/a_air/meshes/完整网格 |
| a-air-flight-evidence.zip | 解压到本目录，提供最新ULog、事件、参数及关节记录 |
| A-UAV-20261115.zip | 原始CAD与BOM参数包，可按需下载 |
| A-UAV-flight-demo.webm | 人工操作演示 |
| SHA256SUMS.txt | 附件校验和 |

Python工具使用根目录requirements。ROS节点使用宿主`/usr/bin/python3`，依赖rclpy和numpy。原生传输辅助进程在Harmonic容器内编译，依赖gz-transport13、gz-msgs10和g++。

## 首次准备

在仓库根目录执行，复用本机已有环境：

```sh
docker start px4-rtl-land-harmonic
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_morphing/airborne/build_px4.py
```

该脚本生成专用`px4_morph`及源补丁，随后恢复PX4原源码和基线程序。

## 启动

```sh
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_morphing/airborne/start_demo.py
```

启动Gazebo、PX4、原生反馈进程、ROS节点及QGC。QGC地址使用UDP14550，测试脚本使用14650/14651，Gazebo分区为`a_air`。

## 操作

1. QGC点击起飞，设为5m，稳定悬停。
2. 在ROS终端执行H25，约10秒后到位。
3. 回X并等待到位，再执行V25。
4. 回X，QGC选择Land，接地后确认自动上锁。

ROS终端初始化：

```sh
source /opt/ros/humble/setup.bash
```

H25：

```sh
ros2 topic pub --once /a_uav/morph/command std_msgs/msg/String '{data: "H:25"}'
```

回X：

```sh
ros2 topic pub --once /a_uav/morph/command std_msgs/msg/String '{data: "X"}'
```

V25：

```sh
ros2 topic pub --once /a_uav/morph/command std_msgs/msg/String '{data: "V:25"}'
```

查看状态：

```sh
ros2 topic echo --once /a_uav/morph/status
```

下一步操作前确认`ready=true`、`transition=false`且关节到位。也支持`H:10`、`V:10`等0～25°目标。

## 自动回归与分析

自动测试独占操作时执行：

```sh
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_morphing/airborne/test_hover.py --sequence H:10 X V:10 X H:25 X V:25 X
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_morphing/airborne/analyze_tests.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_morphing/airborne/plot_regression.py
```

测试发送起降指令和20Hz中立摇杆输入，并记录关节跟踪、姿态及位置。数据位于`logs/`、`parameters/`和`sessions/`，原生ULog位于`runtime/log/`。日常手动飞行由QGC提供输入。

## 停止

```sh
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_morphing/airborne/stop_demo.py
```

## 代码

| 文件 | 内容 |
|---|---|
| prepare_runtime.py / launch.py | 当前模型与PX4运行目录 |
| build_px4.py / px4-morph-geometry.patch | 专用SITL分配接口 |
| gz_link.cpp | 原生Gazebo关节反馈与目标通信 |
| ros_control.py / geometry.py | ROS指令、平滑过渡、重心及旋翼几何 |
| mavlink_flight.py / test_hover.py | 自动飞行与运行边界检查 |
| analyze_tests.py / plot_regression.py | 最新ULog及关节记录分析 |
| verify_allocator.py | V25状态下的PX4矩阵读回核对 |
| ../config/morph-config.json | 机臂分组、质量、关节与目标角 |
| results/ | 当前验证结果 |
