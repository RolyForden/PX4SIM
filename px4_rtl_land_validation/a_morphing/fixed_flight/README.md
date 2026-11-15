# 固定 H/V 小角构形飞行准备

**这是基于 A 近似质量和几何的固定构形演示，不是最终样机，也不是实机参数。** 本轮锁定 10° 小角 H/V，先完成离线悬停可行性检查，再依次做 X 回归、固定 H 10°、固定 V 10° 的起飞/悬停/Land 测试。

## 当前状态

- 已生成两个独立固定构形模型：`a_fixed_h10`（airframe 4998）、`a_fixed_v10`（airframe 4997）。
- 已重算固定构形的重心、完整近似惯量、旋翼位置和推力轴。
- 已生成对应 PX4 `CA_ROTOR*` 分配配置。
- 已通过离线有界悬停可行性检查：秩 4、条件数 62.5、单桨余量约 5.7 N。
- 已实际完成 X 回归、H 10°、V 10° 的起飞、约 5 m 悬停、Land、接地和自动上锁。
- 三次飞行 ULog dropout 均为 0。

## 离线悬停检查

| 构形 | 矩阵秩 | 条件数 | 悬停推力 N | 单桨余量 N | 加权残差 | 结论 |
|---|---:|---:|---|---:|---:|---|
| X | 4 | 62.5 | 2.829 / 2.839 / 2.828 / 2.841 | 5.708 | 0.0 | 可行 |
| H 10° | 4 | 62.5 | 2.830 / 2.839 / 2.828 / 2.840 | 5.709 | 0.0 | 可行 |
| V 10° | 4 | 62.5 | 2.873 / 2.883 / 2.872 / 2.884 | 5.664 | 0.002837 | 可行 |

这是借用动力系数下的离线检查；秩 4 只是一项，不能说明真实电机/桨或全姿态可控。

## 实际飞行结果

| 构形 | 最大相对高度 | 稳定悬停 | Land 至接地 | 接地至上锁 | 最大滚转/俯仰 |
|---|---:|---:|---:|---:|---:|
| X 回归 | 4.998 m | 8.144 s | 8.816 s | 2.016 s | 0.34° / 0.29° |
| H 10° | 4.986 m | 8.224 s | 8.832 s | 2.008 s | 0.35° / 0.20° |
| V 10° | 4.999 m | 8.280 s | 8.928 s | 2.008 s | 0.24° / 0.36° |

汇总与曲线见 [results](results/README.md)；原始 ULog 见 [Release](https://github.com/RolyForden/PX4SIM/releases/tag/a-uav-fixed-hv-2026-11-15)。

## 文件

| 路径 | 内容 |
|---|---|
| `scripts/build_fixed_models.py` | 从 A 近似台架生成固定 H/V 模型、网格和 airframe |
| `scripts/check_hover.py` | 离线有界悬停可行性检查 |
| `models/a_fixed_h10/`、`models/a_fixed_v10/` | 固定构形 SDF 和网格 |
| `airframes/` | PX4 airframe 配置（4998 / 4997） |
| `config/` | 固定构形的质量、重心、惯量、旋翼位置/轴向和假设 |
| `launch.py` | 启动独立固定构形 SITL |
| `test_fixed.py` | 起飞/悬停/Land 测试并保存 ULog、事件和参数 |
| `analyze_fixed.py` | 读取真实 ULog 并生成曲线/汇总 |
| `results/` | 离线检查、飞行汇总和曲线 |
| `logs/`、`parameters/` | 本地原始 ULog、事件和参数快照，默认不提交 |

## 运行

```sh
# 离线检查
px4_rtl_land_validation/.venv/bin/python \
  px4_rtl_land_validation/a_morphing/fixed_flight/scripts/check_hover.py

# 启动固定 H 10°
px4_rtl_land_validation/.venv/bin/python \
  px4_rtl_land_validation/a_morphing/fixed_flight/launch.py \
  --model a_fixed_h10 --airframe 4998 --partition a_fixed_h10

# 测试（实际发送仿真解锁/起飞/Land）
px4_rtl_land_validation/.venv/bin/python \
  px4_rtl_land_validation/a_morphing/fixed_flight/test_fixed.py \
  --model a_fixed_h10 --airframe 4998 --name-prefix fixed_h10

# 分析
px4_rtl_land_validation/.venv/bin/python \
  px4_rtl_land_validation/a_morphing/fixed_flight/analyze_fixed.py \
  --name fixed_h10_20261115_135506

# 停止
px4_rtl_land_validation/.venv/bin/python \
  px4_rtl_land_validation/candidate_flight/stop.py
```

V 10° 把 `a_fixed_h10/4998` 换成 `a_fixed_v10/4997`。容器和运行目录沿用现有 `px4-rtl-land-harmonic` / `candidate_flight/runtime`。

## 边界

- 固定构形角为 10°，只用于首轮小角飞行；25° 构形仍只在运动学台架验证。
- 旋翼动力系数、反扭矩、旋向和响应时间仍为借用/示意值。
- 质量、重心、惯量来自 BOM 近似；0.393 kg 未定位质量仍在机身，5 个 CAD 实例无 BOM 质量。
- 未实现空中连续变构、实际关节角驱动的 PX4 实时分配、真实 TPU/绳索柔性或实机验证。
- 本轮只证明近似固定构形在 SITL 中能完成基本起飞/悬停/Land 闭环。
