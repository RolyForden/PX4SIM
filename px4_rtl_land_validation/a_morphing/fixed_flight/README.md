# 固定 H/V 10° 小角构形飞行

本轮使用 A 近似质量和几何，锁定 10° 小角 H/V，完成离线悬停检查，以及 X 回归、固定 H 10°、固定 V 10° 各 3 次 SITL 飞行。

## 当前状态

- 已生成两个独立固定构形模型：`a_fixed_h10`（airframe 4998）、`a_fixed_v10`（airframe 4997）。
- 已重算固定构形的重心、完整近似惯量、旋翼位置和推力轴。
- 已生成对应 PX4 `CA_ROTOR*` 分配配置。
- 离线有界悬停检查：秩 4、条件数 62.5、单桨余量约 5.7 N。
- X、H10、V10 各完成 3 次起飞、约 5 m 悬停、Land、接地和自动上锁。
- 9 次飞行 ULog dropout 均为 0。

## 离线悬停检查

| 构形 | 矩阵秩 | 条件数 | 悬停推力 N | 单桨余量 N | 加权残差 |
|---|---:|---:|---|---:|---:|
| X | 4 | 62.5 | 2.829 / 2.839 / 2.828 / 2.841 | 5.708 | 0.0 |
| H 10° | 4 | 62.5 | 2.830 / 2.839 / 2.828 / 2.840 | 5.709 | 0.0 |
| V 10° | 4 | 62.5 | 2.873 / 2.883 / 2.872 / 2.884 | 5.664 | 0.002837 |

动力系数沿用现有基线值。

## 实际飞行结果

每种构形 3 次，数值为均值 ± 标准差：

| 构形 | 最大相对高度 | 稳定悬停 | Land 至接地 | 接地至上锁 |
|---|---:|---:|---:|---:|
| X 回归 | 5.001 ± 0.002 m | 8.131 ± 0.019 s | 8.837 ± 0.048 s | 2.013 ± 0.004 s |
| H 10° | 4.995 ± 0.006 m | 8.211 ± 0.037 s | 8.875 ± 0.060 s | 2.012 ± 0.003 s |
| V 10° | 5.001 ± 0.004 m | 8.203 ± 0.059 s | 8.891 ± 0.059 s | 2.012 ± 0.003 s |

详细逐次数据和曲线见 [results](results/README.md)；原始 ULog 见 [Release](https://github.com/RolyForden/PX4/releases/tag/a-uav-fixed-hv-2026-11-15)。

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
| `logs/`、`parameters/` | 本地原始 ULog、事件和参数快照 |

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

## 测试条件

- 构形角：10° 固定 H/V；25° 构形目前在运动学台架验证。
- 模型：A 近似质量/几何，SITL。
- 质量、重心、惯量：BOM 近似；0.393 kg 未定位质量在机身，5 个 CAD 实例无 BOM 质量。
- 动力：沿用基线电机系数、反扭矩和响应参数。
- 下一阶段：25° 构形、重复性扩展、空中连续变构。
