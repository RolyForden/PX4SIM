# PX4SIM

PX4 与 Gazebo 仿真协作仓库，用于保存可共享的测试代码、环境版本和进度说明。

当前已完成官方标准 X500 的基础飞行、Return 和 Land 验证。自研可变构形机型的模型与控制分配工作尚未完成，不能用 X500 的通过结果替代自研机型验证。

## 当前环境

| 组件 | 固定版本或状态 |
|---|---|
| 操作系统基线 | Ubuntu 22.04 |
| PX4 | v1.17.0，提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Gazebo | Harmonic，Gazebo Sim 8.15.0 |
| 标准机型 | 官方 `gz_x500`，`SYS_AUTOSTART=4001` |
| QGroundControl | 本地验证使用 v5.0.8 |
| 执行方式 | 本地独立 Docker 环境；Python 经专用回环 MAVLink 端口操作 SITL |

PX4 上游源码、系统依赖和 Docker 镜像不包含在本仓库中。只有本仓库文件并不足以启动仿真，必须先准备相应运行环境。

## 仓库结构

```text
README.md
requirements.txt
docs/
  BASELINE.md             # 标准 X500 验证摘要与结论边界
  COLLABORATION.md        # 资料交接、共享范围和协作方法
px4_rtl_land_validation/
  validate.py             # 已有本地 SITL 自动测试脚本
  analyze.py              # PX4 原生 ULog 分析脚本
```

这两个脚本是已有本地测试工具，仍约定固定的容器名和目录布局，尚未整理成跨机器的一键安装方案。`validate.py` 会发送解锁与飞行指令，仅在明确安排的本机 SITL 测试中运行。

## 标准基线脚本

在已经配置好的本地环境中，控制端口为 `127.0.0.1:14650/14651`，容器名为 `px4-rtl-land-harmonic`。脚本先核对本机 PX4 进程、PX4 心跳和 X500 机型编号。

```sh
# 确认环境、模型和专用端口都已准备好后，才执行飞行脚本。
python px4_rtl_land_validation/validate.py rtl --name rtl_default
python px4_rtl_land_validation/validate.py land --name land_default
```

日志输出位于脚本所在目录下的 `logs/` 和 `parameters/`，这些目录不会提交。将待分析的真实 `.ulg` 放入本地 `px4_rtl_land_validation/logs/` 后，可以运行：

```sh
python px4_rtl_land_validation/analyze.py
```

## 下一阶段

- 确认构形定义、关节运动方式和旋翼编号。
- 记录建模参数的来源及未知项。
- 为自研机型建立独立 Gazebo 模型与执行器映射，保留 X500 基线。
- 计算并核对不同构形的力/力矩分配关系。
- 区分机构动作展示与 PX4 闭环飞行验证，视频中明确标注所展示的内容。

详细交接信息见 [协作说明](docs/COLLABORATION.md)，标准机型结果见 [基线记录](docs/BASELINE.md)。

## 共享范围

本仓库当前公开。按项目负责人的要求，只上传可共享的通用代码与说明；技术交底书、原始 CAD/STEP、STAR-CCM+ 工程、设计参数及其衍生图示、原始飞行日志和本机诊断资料先留在本地。

根目录 `.gitignore` 使用允许清单；新增文件不会自动进入共享范围。需要公开新文件时，应先确认其来源、内容和授权，再更新清单。不要用 `git add -f` 绕过这一约束。
