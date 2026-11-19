# 原生约束机械臂：固定机身测试

当前后续入口。沿用原 CAD 拆分、关节坐标、质量假设和控制轨迹，用 Gazebo/DART 原生约束替代手写弹簧阻尼闭合。全程固定机身，不启动 PX4。

## 实际实现路径

1. `build_model.py` 生成树状 SDF：三个主动转动关节、六个肘部球铰、六个杆端球铰。每个杆端球铰的另一侧是一个很小的连接体 `tip_i_k`。
2. `controller.cpp` 在首次初始化时创建六个 `DetachableJoint` 组件，将这些连接体固定连接到末端平台。
3. Gazebo 将这些组件交给 DART 的固定连接/闭链约束求解。球铰允许杆件转动，连接体与平台的固定连接将回路闭合。
4. 插件仅向三个主动关节施加控制力矩，读取实际角度、位姿和关节传递力。不施加杆端弹簧力，不重置被动位置。

“原生”指由物理引擎的关节约束求解传力；仍需要插件在运行时创建闭合连接，不能只靠在 SDF 中直接添加重复父连接。

每个连接体质量为 0.01 g，六个合计 0.06 g，属于求解辅助体，CAD 无新增实体。这部分质量已计入支撑力验算。主动臂、连杆、平台及负载仍使用本轮近似质量/惯量，尚未表示实测样机。

## 启动与控制

```bash
cd /home/ray/文档/ChatGPT/PX4/px4_rtl_land_validation/delta_dynamics/native
python3 run.py --gui --payload 0.05
```

新终端加载 ROS：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /a_uav/arm_native/command std_msgs/msg/String '{data: "demo"}'
```

轨迹为下降 25 mm、回位、横移 15 mm、回位、下降平面画半径 15 mm 圆、回位。

```bash
ros2 topic echo /a_uav/arm_native/status --once
ros2 topic pub --once /a_uav/arm_native/command std_msgs/msg/String '{data: "home"}'
python3 stop.py
```

ROS 话题使用 `/a_uav/arm_native/`，Gazebo 分区为 `a_delta_native`，本机 UDP 14840/14841。

## 本轮实测（2026-11-19）

| 工况 | 完整轨迹 | 最大末端误差 | 最大闭合误差 | 回位误差 |
|---|---|---:|---:|---:|
| 重力空载 | 通过 | 0.576 mm | 1.40 μm | 0.0153 mm |
| 额外 50 g | 通过 | 0.572 mm | 1.40 μm | 0.0148 mm |
| 额外 50 g，固定机身姿态为 roll=0.12、pitch=-0.08、yaw=0.37 rad | 通过 | 0.581 mm | 1.42 μm | 0.0148 mm |

三次机身位移均为零。第三次改变的是固定台架的初始朝向，未进行空中飞行或运动基座测试。

球铰传递力之和包括平台、负载和连接体重量。空载静态竖直力为 0.491088 N，加 50 g 后为 0.981589 N，与对应重量吻合。

原始证据保存在 `results/{工况}/feedback.jsonl`、`summary.json` 和 `mount_wrench.jsonl`。后者读取的是 `fixed_body`——机身到世界的固定点——关节六维反力，包含整机测试质量，不是独立机械臂安装法兰传感器。比较运动扰动时减去了本轮静态均值。

完整比较见 `../study/comparison.json`、`../study/comparison.png`，最小兼容性试验见 `../native_probe/results.json`。

## 环境与边界

本机 Gazebo Sim 8.15.0、gz-physics 7.9.0、DART 6.13.2，未升级或重装。直接 SDF 双球铰试验中，第二个连接被拒绝；运行时固定连接方案已通过最小模型和完整机构测试。

数值约束仍存在求解容差。尚未验证机身自由运动、碰撞抓取、真实舵机响应、柔性材料和实物参数；这些与闭链能否在当前环境运行是不同验证项。
