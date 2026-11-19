# A 无人机锁臂悬停基线

空载和末端 50 g 各 3 次，均通过 2 m 起飞、20 个仿真秒悬停、Land 和自动上锁。机械臂与变构关节锁住，保留完整质量和惯量，使用原 PX4 SITL 与既有控制参数。

[实际结果、模型改动和问题处理](results/README.md) · [对照数据](results/comparison.json) · [ULog 清单](results/ulog-manifest.json)

## 执行路径

`build_model.py` 从已验证的集成模型快照生成锁臂 SDF、世界和独立 airframe 4995；`launch.py` 启动 Gazebo 与原生 PX4；`test_flight.py` 通过本地 MAVLink 发送指令并记录状态；`analyze.py` 从原生 ULog 验收；`summarize.py` 绘制汇总曲线；`audit.py` 核验模型和参数。

本机依赖：现有 Docker 容器 `px4-rtl-land-harmonic`，PX4 编译产物、集成机械臂模型资源、`a_with_arm_fixed/results/{empty,payload_50g}/world.sdf` 和现有 `.venv`。所有路径由脚本位置解析。

在工作空间根目录运行一个新试验（名称须未使用）：

```bash
python3 px4_rtl_land_validation/a_locked_hover/launch.py --name empty_new --payload 0
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_locked_hover/test_flight.py empty_new
px4_rtl_land_validation/.venv/bin/python px4_rtl_land_validation/a_locked_hover/analyze.py empty_new
python3 px4_rtl_land_validation/a_locked_hover/stop.py
```

50 g 对照将 `--payload` 改成 `0.05` 并使用新名称。测试脚本会实际解锁起飞，应按顺序运行；飞行期间不要执行停止命令。`run_suite.py` 是本轮三空载后再三负载的执行记录入口，已存在的结果用于保留证据，不覆盖。

当前生成的 `models/a_locked/model.sdf` 对应最后一轮 50 g；每次运行目录都有独立模型、参数、日志快照。机械臂运动飞行另立测试，不在本轮执行。
