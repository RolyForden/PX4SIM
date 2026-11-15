# A 无人机固定构形仿真代码

当前已通过的结果见[阶段成果](../../docs/A无人机阶段成果.md)。该目录名字candidate沿用历史命名，几何是自研A整机，2026-11-15已确认为最新版本，不是旧UAV.STEP。

## 获取依赖资产

从[本轮Release](https://github.com/RolyForden/PX4SIM/releases/tag/a-uav-baseline-2026-11-15)下载：

1. `A-UAV-20261115.zip`解压到`px4_rtl_land_validation/A-UAV-20261115/`，提供原始CAD、质量参数及重算脚本。
2. `A-flight-meshes-20261115.zip`解压到`px4_rtl_land_validation/candidate_flight/`，恢复`meshes/`和`models/candidate_rigid/meshes/`；或使用OCP执行`import_candidate.py --export`后再构建。
3. 两份ULog与MP4是测试证据，运行模型无需下载。

质量重算审计需要numpy及openpyxl（见根requirements）；当前本机飞行venv未装openpyxl，不能直接用它跑audit_handoff。已在现有带依赖的Python运行审计通过。

本机已有环境的运行说明如下。Docker镜像、PX4源码及构建产物不随Git发布，换电脑需要自行准备。`record_demo.py`依赖本机X11/GStreamer、宿主cv2及打开的窗口，只是历史录制工具，不能作为通用一键录制入口。


最新数据接入见 [A-UAV交接基线接入记录](handoff_audit/report.md)；此前占位惯量测试保留在本地历史记录中。这不是最终样机或飞行中变构验证。

## 文件入口

- `settings.json`：通过 `mass_properties_json` 指向交接JSON，使用其完整精度质量属性；同时记录临时坐标变换及仍未标定的动力假设。
- `audit_handoff.py` / `integrate_handoff.py` / `verify_sdf_mass.py`：隔离复算、完整张量转换、质量分配和独立SDF汇总检查。
- `import_candidate.py`：优先读取 `../现实材料/A无人机.STEP`，否则读取交接包 `cad/airframe.step`。运行时加 `--export` 导出123个部件网格，依赖参考模型目录所列OCP包。
- `build_flight.py`：生成独立Gazebo模型和airframe4999，不改上游PX4源码；读取现有PX4构建产物。`models/candidate_rigid/`为实际生成模型。
- `launch.py` / `stop.py`：专用回环接口和 `GZ_PARTITION=px4_candidate_flight`，其中 `GZ_IP=127.0.0.1` 为已验证配置。
- `test_flight.py`：核对候选进程和airframe，执行实际解锁/起飞/悬停/Land，记录事件、参数和ULog，超出姿态/位置范围时报错。越界后的自动恢复未验证，严重发散时需停止整个仿真。
- `analyze_flight.py`：读取真实ULog，检查离地/悬停/接地/上锁并绘图，拒绝把标准模型诊断或不完整测试标成候选飞行通过。
- `check_allocation.py`：离线、4电机的有界最小二乘，输出实际推力(N)，不是PX4实时分配器。
- `diagnose_actuators.py`：无PX4零重力电机方向诊断，不是飞行实验。

## 在当前已安装环境运行

以下依赖本机已有Docker容器，将 `px4_rtl_land_validation/` 挂载到 `/work`，并已有PX4编译产物。不是从零安装教程。

```sh
docker start px4-rtl-land-harmonic
# 已有网格时不必重跑CAD导入
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/build_flight.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/verify_sdf_mass.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/check_allocation.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/launch.py
# 下面实际发送仿真解锁和飞行指令
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/test_flight.py
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/analyze_flight.py
# 完成或失败后停止本项目仿真
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/candidate_flight/stop.py
docker stop px4-rtl-land-harmonic
```

启动默认无GUI，以减少桌面负担；不应把容器默认存在的 `LIBGL_ALWAYS_SOFTWARE=1` 当作实际GUI渲染结论。启动器显式覆盖NVIDIA相关设置。本轮实际硬件渲染证据见 `gpu_renderer.log`，后续GUI必须使用同一Gazebo分区。

原生ULog在 `runtime/log/`；测试脚本归档在 `logs/`。失败或中断期间复制的文件可能早于原件最终写入，完整取证以停止后的原件为准。已有数据不删除，复测自动使用新时间戳。
