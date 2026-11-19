"""Write the Chinese report from accepted results and preserved failure records."""
from pathlib import Path
import json
R=Path(__file__).resolve().parent;O=R/'results';data=json.loads((O/'comparison.json').read_text());cases=data['accepted'];inverse=json.loads((O/'inverse-diagnosis.json').read_text())
s='''# H/V 25°变构与机械臂耦合悬停

日期：2026-11-19。空载完整序列通过后，执行同条件 50 g 负载对照。两条主试验均通过；保留两条此前失败／中断的原生记录。测试期间打开对应 Gazebo GUI，图形环境为 NVIDIA RTX 4060；本轮未录屏。

## 本轮实际改动

1. 新建 `a_coupled`，从已修正质量、重心及完整惯量的 A 集成快照生成自由飞行模型。恢复八个等效变构关节，保留三个机械臂主动关节、十二个球铰及六处运行时闭链连接。空载 1.46888 kg，额外 50 g 后 1.51888 kg。
2. Gazebo 插件依据实际链接位姿汇总整机重心、惯量，并读取各旋翼位置及推力方向。动态力臂为旋翼位置减去当前整机重心，包含变构机臂、机械臂和负载。
3. ROS 变构节点直接采用上述实际几何，沿用已有 PX4 本机 UDP 几何接口和应用序号回执。机械臂、变构、分配记录使用独立 `/a_uav/coupled/` 命名空间，不使用主机 Fortress 桥接 Harmonic。
4. 变构插补改用仿真时间，峰值速度 5°/s、25°单程约 9.375 s；机械臂速度 15 mm/s、加速度 30 mm/s²。物理步长 0.2 ms、IMU 5000 Hz，保留上一轮已验证的接地采样设置。
5. 使用专用 `px4_coupled` SITL 适配程序：控制矩阵选择滚转、俯仰、偏航、机体 Z 推力四项。完整物理矩阵保留六行，倾斜旋翼横向推力仍实际作用于机身。修正原因见下文。

PID、机械臂关节增益及动力系数保持前轮值。CA 参数快照保持初始几何，运行时矩阵通过专用接口更新，而非反复写参数。构建适配程序时临时修改的 PX4 基线源码及二进制已恢复，专用程序及补丁独立保存。

## 实际序列与验收

5 m 起飞 → X 静止 10 s → H25 保持并完成机械臂完整序列 → 回 X → V25 保持并完成完整序列 → 回 X → X→H25→X 同时左右往返 → X→V25→X 同时连续画圆 → X 恢复悬停 10 s → Land → 接地 → 自动上锁。

机械臂完整序列包括下降 25 mm、X 横移 15 mm、下降平面半径 15 mm 整圆、Y 正负 15 mm 往返和回位。H 同时段左右往返四次，V 同时段画圆三次。所有持续时间均为仿真时间。

每阶段均以原生 ULog 和 Gazebo 实际反馈验收：真实及估计高度误差、水平偏移 <0.5 m，滚转／俯仰峰值 <5°，无电机上限饱和，机械臂跟踪 <3 mm、闭链误差 <10 μm，变构跟踪 <5°，矩阵秩 4、悬停推力余量为正。固定 H/V 实际角度达到 25°；同时动作累计重叠 >15 s，机械臂四条命令全部完成。最终核对自动上锁、地面真实位姿、无 failsafe／ULog dropout，以及降落估计与真值偏差 <0.3 m。

## 实际结果

下表为全体正式悬停阶段最差值，位置来自 Gazebo 真实位姿，姿态来自 PX4 ULog。

| 工况 | 真实高度最大误差 cm | 真实水平最大偏移 cm | 滚转／俯仰最大绝对值 ° | 机械臂最大误差 mm | 变构最大跟踪误差 ° |
|---|---:|---:|---:|---:|---:|
'''
for c in cases:
 p=list(c['phases'].values());s+=f"| {'空载' if c['payload_kg']==0 else '50 g'} | {max(v['truth_height_max_error_m'] for v in p)*100:.2f} | {max(v['truth_horizontal_max_offset_m'] for v in p)*100:.2f} | {max(max(v['roll_pitch_peak_deg']) for v in p):.3f} | {max(v['arm_max_tracking_error_m'] for v in p)*1000:.3f} | {max(v['max_morph_tracking_error_deg'] for v in p):.3f} |\n"
s+='''
### 同时运动阶段

| 工况 | 组合 | 阶段 s | 实际同时动作 s | 真实高度误差 cm | 真实水平偏移 cm | 姿态峰值 ° |
|---|---|---:|---:|---:|---:|---:|
'''
for c in cases:
 for key,label in [('coupled_H','H25＋左右往返'),('coupled_V','V25＋连续画圆')]:
  v=c['phases'][key];s+=f"| {'空载' if c['payload_kg']==0 else '50 g'} | {label} | {v['sim_s']:.3f} | {v['overlap_sim_s']:.3f} | {v['truth_height_max_error_m']*100:.2f} | {v['truth_horizontal_max_offset_m']*100:.2f} | {max(v['roll_pitch_peak_deg']):.3f} |\n"
s+='''
两轮均完成全部阶段、回 X、机械臂回位、降落和自动上锁。电机上限饱和、飞行 failsafe、ULog dropout 均为零，所有记录的惯量张量正定，整机质量恒定。

![空载和50g完整序列](comparison.png)

绿色区域为固定构形下的机械臂运动，橙色区域为变构与机械臂同时动作。

### 动态分配与质量核验

每轮六次原生矩阵读回：地面 X、固定 H25、固定 V25、H 同时运动中、V 同时运动中、最终 X。与实测几何对应的控制矩阵比较，容差 0.02；运动中读回与反馈存在短时采样差，保存读回前后记录以便复核。

'''
for c in cases:
 mass=json.loads((R/'runs'/c['name']/'mass-crosscheck.json').read_text());s+=f"- {'空载' if c['payload_kg']==0 else '50 g'}：六次矩阵最大差 {max(v['max_abs_error'] for v in c['native_matrix_readbacks']):.6f}；独立重构质心最大差 {mass['max_com_error_m']*1000:.4f} mm、惯量元素最大差 {mass['max_tensor_error_kg_m2']:.8f} kg·m²。\n"
s+='''
质量核验独立使用 CAD 正解、实际主动关节角、平台位姿和插值后的变构角，再通过旋转张量与平行轴公式合成。动态采样核对容差为重心 0.1 mm、惯量元素 1e-5 kg·m²；这是软件与坐标一致性核对，不是实物测量精度。

## 遇到的问题与处理

### 首轮 V25 分配失稳

`empty_01` 完成 H25 机械臂序列，在 V25 机械臂阶段发生电机输出突变和俯仰失稳。触发中止后保持两套机构并请求 Land，之后检测到接地和自动上锁；该轮不是正常通过的飞行，日志完整保留。

物理 6×4 矩阵仍为秩 4、悬停推力可行，但回放同一批实际几何到 PX4 单精度广义逆函数时，出现数值异常：

'''
s+=f"- 回放 {inverse['full']['samples']} 帧，原完整六行控制矩阵有 {inverse['full']['rank_counts'].get('5.0',0)} 帧被判为秩 5，最大重构残差 {inverse['full']['max_reconstruction_error']:.0f}。\n- 选择四个独立控制轴后，同一批数据全部为秩 4，最大重构残差 {inverse['four_axes']['max_reconstruction_error']:.8f}。\n"
s+='''
四个电机不能独立实现六个任意力／力矩目标。本轮将控制矩阵 Fx/Fy 行置零，保留实际转矩和 Fz；完整物理 B 及倾斜旋翼的横向推力没有删除，仍由 Gazebo 和位置／姿态闭环体现。未用 PID 调参掩盖该问题。修正后重新跑完整空载，验收通过才进入负载对照。

### 重测日志读取中断

`empty_02` 在稳定的 H25 机械臂运动期间，监测程序读到尚未写完的 JSON 行，解析异常；中止降落流程又碰到同一半行的后半段，因此暂停仿真并保存现场。该轮未完成正常降落，不计通过。关闭程序后的全部保存行逐条 JSON 检查均有效，确认是并发文件读取问题。

读取器改为只消费换行完整的记录，遇半行回退到原位置等待下一次读取；没有跳过完整的坏记录或改变运动误差界限。随后空载和负载均重新执行完整序列。负载轮的独立矩阵采集器还在空文件创建瞬间启动，已补充等待首条完整记录并重新启动采集；飞行未中断，两段运动中矩阵均已实际读回。初次采集启动错误单独保存。

### 运行方式和日志完整性

起初仅启动物理后端，随后打开连接到同一试验的 GUI；之后各轮启动都包含 GUI。变构按仿真时间规划，图形及计算负载影响墙钟速度，不改变规定的仿真运动速率。图形渲染使用 GPU，物理求解与 PX4 计算在 CPU 上执行。

ULog 使用 PX4 退出并写完缓冲后的原件重新校验，保留之前复制快照的大小与哈希元数据，不修改任何飞行数据。

负载轮参数核对发现，默认 `SENS_BAR_AUTOCAL=1` 在解锁前将 `CAL_BARO0_OFF` 从 0 自动保存为 -2.44140625 Pa。原生 ULog 起始参数已经是该值，飞行中没有再次更新；本地 VehicleAirData 自动 GNSS／气压高度对齐代码及 Barometer 参数保存路径与此相符。因此核验区分自动校准、累计计数和控制参数变化；MC/MPC 与 CA 参数始终不变。原始参数快照完整保留。

### 接口反力矩的尖峰

原始安装接口 Mx/My 数据保留了短时尖峰，空载最大约 0.818 N·m、50 g 约 0.553 N·m，曲线没有滤除。同期机身响应仍满足本轮飞行判据。若后续要用这些峰值计算结构或舵机载荷，需要单独检查约束求解、时间步长和采样收敛；本轮不把它们直接解释成实物载荷。

## 证据入口

- `comparison.json`：全部十个阶段的估计／真实位置、姿态、机构跟踪、接口反力、矩阵秩及推力余量。
- `native-matrices.json`：两轮共十二次 PX4 原生矩阵读回，包含运动中采样。
- `audit.json`：独立质量核验、原生闭链与关节结构、参数一致性、基线恢复及 ULog 校验。
- `inverse-diagnosis.json`、`inverse-replay-full.txt`、`inverse-replay-four_axes.txt`：数值问题回放。
- `failure-empty_01.json`、`failure-empty_02.json`：失败与中断记录。
- `ulog-manifest.json`、`SHA256SUMS.txt`：两份通过、两份失败／中断，共四份真实 ULog 路径、大小和 SHA256。
- 本地 `../runs/<名称>/`：模型、源快照、起飞前后参数、原始机械臂／变构反馈、ROS 命令及原生 ULog。原始运行目录和二进制未加入 Git。

## 结论

完成当前近似模型下的 H/V 25°变构与机械臂操作耦合悬停验证，空载和 50 g 均通过。本轮没有调整 PID，实际验证了从关节反馈、整机重心和旋翼几何，到 PX4 动态分配、电机输出、机身响应的整条链路。

两种负载各完整通过一次，尚未做重复性统计，不能把两轮细小差异全部归因为负载。质量与动力仍是当前近似设定，变构关节为主动位置控制的等效模型；本轮没有证明真实柔性材料／舵机传动、实机参数、机身平移时同时操作或更大速度／行程下的性能。
'''
(O/'README.md').write_text(s);print('Chinese coupled-test report written.')
