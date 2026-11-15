# A 无人机 ROS 变构实施计划

更新：2026-11-15。

进展：A 近似台架 X/H/V、ROS 指令、X/H10/V10 各 3 次固定构形飞行和动态变构链地面联调已完成。下一步是空中小角慢速变构。

## 目标

实现A模型的ROS指令变构、实际关节反馈和分构形推力分配。先完成地面X/H/V切换，再验证固定H/V飞行，最后接入空中连续变构。

## 当前基线

- A整机CAD已确认为最新版本，固定构形起飞、悬停、Land和自动上锁已通过。
- 舵机选型为NANGU 20KG碳刷款。
- 机构采用单舵机双层反向绕线、双TPU柔性件和滑块限位。
- 质量、柔性和动力参数集中配置，随资料更新。
- 演示由用户操作录制，自动化工作先完成模型、接口和验证证据。

## 工作环境

项目根目录：`/home/ray/文档/ChatGPT/PX4`。

| 项目 | 配置 |
|---|---|
| PX4 | px4_rtl_land_validation/PX4-Autopilot，v1.17.0 |
| 容器 | px4-rtl-land-harmonic |
| 镜像 | px4-rtl-land-harmonic:gpu-base-20261113 |
| Gazebo | Harmonic 8.15.0，项目目录挂载/work |
| ROS | 宿主/opt/ros/humble；接入前检查桥接包版本 |
| Python | px4_rtl_land_validation/.venv/bin/python |
| QGC | /home/ray/下载/QGroundControl-v5.0.8-x86_64.AppImage |
| 通信 | GZ_IP=127.0.0.1，GZ_PARTITION=px4_candidate_flight |
| MAVLink | GCS UDP14550，脚本14650/14651 |
| 物理步长 | 4ms |
| GPU | NVIDIA CDI；GUI设置LIBGL_ALWAYS_SOFTWARE=0、__GLX_VENDOR_LIBRARY_NAME=nvidia |

沿用现有容器和GPU配置。启动前检查进程、容器和端口，测试使用独立仿真实例。质量重算依赖numpy/openpyxl；现有飞行venv需补齐openpyxl或使用已有兼容解释器。参数脚本在临时副本执行。cv2位于宿主/usr/bin/python3。

## 输入文件

1. 根README及[阶段成果](A无人机阶段成果.md)。
2. [资产清单](../px4_rtl_land_validation/现实材料/模型资料交接清单.md)、交底书及构形/爆炸/绕线图。
3. 本地A-UAV-20261115中的README、uav_parameters.json、部件质量CSV及装配变换。
4. candidate_flight中的settings.json、flight_config.json、cad_inventory.json和handoff_audit/report.md。

## 第一阶段：A模型关节与ROS接口

在独立`px4_rtl_land_validation/a_morphing/`目录实现，保留candidate_rigid作为回归基线。

### 模型

- 按CAD索引划分机身和四个机臂，将部件分组、关节中心、转轴、限位、质量及假设写入集中配置。
- 围绕柔性件设置等效弯曲自由度、弹性回复和必要的滑动限位。
- 单个构形输入协调四臂：X为中立，H和V分别为两个分支，H/V切换经X复位。
- 首版使用平滑目标角映射驱动关节；柔性响应以弹簧阻尼近似，绳索以传动映射表达。
- 主动位置控制与被动回弹分别测试。
- 沿用当前坐标约定，同步变换网格、关节、旋翼位置、重心及完整惯性张量。

### ROS接口

检查Humble与容器Harmonic的桥接兼容性。优先采用匹配的ros_gz与JointPositionController；必要时在隔离工作空间编译桥，或以ROS节点调用容器gz transport建立临时适配。

建议接口：

| 接口 | 内容 |
|---|---|
| /a_uav/morph/command | X/H/V或限定范围的单变量构形指令 |
| /a_uav/joint_states | Gazebo实际位置和速度 |
| /a_uav/morph/status | 当前构形、转换状态、实际角度和超时信息 |

设置输入范围、角度限位和变化率，依据实际反馈判断到位。

### 验收

在固定机身台架执行X→H→X→V→X，读回关节角、旋翼位置和推力轴，与解析几何比较。预先设置误差阈值，保存时间戳、指令、反馈及截图，给出简短启动和切换命令。

交付：SDF/网格、ROS节点及launch、集中配置、验证数据和一页结果说明。

## 第二阶段：分构形质量与推力分配

根据实际关节角，计算旋翼位置r、推力方向d、当前重心c。在FRD系中构建：

```text
B_i = [ (r_i-c) × d_i - k_i d_i ; d_i ]
```

输出顺序为τx、τy、τz、Fx、Fy、Fz；输入为单桨实际推力N。k按Gazebo与PX4约定统一旋向符号；PX4归一化执行器映射单独处理。

- V构形同时更新旋翼位置和推力轴。
- 按部件质量分组重算重心和惯量；待定位的0.393kg暂保留于机身，五个缺质量实例登记待补。
- 各构形保持总质量一致，保留惯性非对角项符号。
- 对X/H/V计算有界悬停解、残差、执行器余量和矩阵条件。

## 第三阶段：固定H/V飞行

在安排飞行测试后，生成独立模型和airframe，锁定构形，更新SDF质量属性及PX4旋翼位置、轴向和旋向。

顺序：X回归 → 小角H → 小角V。各次完成起飞、稳定悬停、Land、接地和自动上锁，归档ULog、事件、模型及前后参数。分析高度、姿态、速度和模式；越界中止测试，严重发散停止仿真。

## 第四阶段：空中连续变构

前述阶段通过后，建立实际关节状态→几何及重心→效能矩阵更新链。检查PX4矩阵更新入口、推力饱和、变化率和反馈时延，先做小角低速试验，记录关节角、旋翼输出、姿态及分配残差。

## 执行与归档

范围为PX4 SITL、Gazebo和ROS。保留现有PID、容器及基线数据，新增模型独立存放。代码和结论更新到GitHub，大型CAD、网格、日志及录像通过Release提供。每阶段报告完成内容、测试结果和下一步。
