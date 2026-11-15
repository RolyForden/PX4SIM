# A 无人机固定构形仿真

最新A整机的Gazebo模型、PX4配置及测试工具。[阶段成果](../../docs/A无人机阶段成果.md) · [质量属性配置](handoff_audit/report.md)

## 资产准备

从[Release](https://github.com/RolyForden/PX4SIM/releases/tag/a-uav-baseline-2026-11-15)下载：

1. `A-UAV-20261115.zip`解压到`px4_rtl_land_validation/A-UAV-20261115/`。
2. `A-flight-meshes-20261115.zip`解压到`px4_rtl_land_validation/candidate_flight/`，恢复分部件和模型网格。

也可用OCP运行`import_candidate.py --export`重新导出网格。导入脚本优先读取`../现实材料/A无人机.STEP`，其次读取参数包内`cad/airframe.step`。

运行依赖本机既有Docker容器、PX4源码和编译产物。Python依赖见根目录requirements；CAD导入额外依赖OCP，质量重算需要openpyxl。

## 工具

| 文件 | 用途 |
|---|---|
| settings.json | 质量参数路径、坐标变换、动力系数 |
| audit_handoff.py | 临时副本中的参数重算与来源核对 |
| integrate_handoff.py | 质量分配与惯性张量转换 |
| verify_sdf_mass.py | SDF整体质量、重心、惯量核对 |
| build_flight.py | 生成模型和airframe4999 |
| launch.py / stop.py | 启停专用仿真进程 |
| test_flight.py | 解锁、起飞、悬停、Land、接地及上锁验证 |
| analyze_flight.py | ULog分析与曲线输出 |
| check_allocation.py | 四电机有界最小二乘推力分配，输出单位N |
| diagnose_actuators.py | 零重力条件下的电机力矩方向检查 |
| record_demo.py | 本机X11/GStreamer窗口录制，MP4转换依赖宿主cv2 |

## 本机运行

容器`px4-rtl-land-harmonic`将项目目录挂载到`/work`。先生成并检查模型，再执行飞行测试：

```sh
docker start px4-rtl-land-harmonic
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/build_flight.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/verify_sdf_mass.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/check_allocation.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/launch.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/test_flight.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/analyze_flight.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/stop.py
docker stop px4-rtl-land-harmonic
```

通信采用`GZ_IP=127.0.0.1`、`GZ_PARTITION=px4_candidate_flight`。GUI使用同一分区，并设置`LIBGL_ALWAYS_SOFTWARE=0`、`__GLX_VENDOR_LIBRARY_NAME=nvidia`。

ULog原件位于`runtime/log/`，测试归档位于`logs/`，参数位于`parameters/`。每次测试使用时间戳命名。姿态或位置越界时停止测试；严重发散时停止仿真进程。
