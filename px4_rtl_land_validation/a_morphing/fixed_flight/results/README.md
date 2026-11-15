# 固定 H/V 10° 小角飞行结果

日期：2026-11-15。PX4 SITL + Gazebo Harmonic，A 近似模型，固定构形角 10°。

## 结论

X 回归、固定 H 10°、固定 V 10° 各完成 3 次，共 9 次飞行。每次都完成起飞、约 5 m 悬停、Land、接地和自动上锁；9 次 ULog dropout 均为 0。

## 汇总

| 构形 | 次数 | 最大高度 | 稳定悬停 | Land 至接地 | 接地至上锁 |
|---|---:|---|---:|---:|---:|
| X 回归 | 3 | 5.001 ± 0.002 m | 8.131 ± 0.019 s | 8.837 ± 0.048 s | 2.013 ± 0.004 s |
| H 10° | 3 | 4.995 ± 0.006 m | 8.211 ± 0.037 s | 8.875 ± 0.060 s | 2.012 ± 0.003 s |
| V 10° | 3 | 5.001 ± 0.004 m | 8.203 ± 0.059 s | 8.891 ± 0.059 s | 2.012 ± 0.003 s |

## 逐次结果

| 构形 | 日志 | 最大高度 | 稳定悬停 | Land 至接地 | 接地至上锁 | 最大滚转/俯仰 |
|---|---|---:|---:|---:|---:|---:|
| X | `rigid_20261115_135319` | 4.998 m | 8.144 s | 8.816 s | 2.016 s | 0.34° / 0.29° |
| X | `rigid_20261115_141203` | 5.003 m | 8.104 s | 8.904 s | 2.008 s | 0.50° / 0.39° |
| X | `rigid_20261115_141524` | 5.003 m | 8.144 s | 8.792 s | 2.016 s | 0.28° / 0.31° |
| H 10° | `fixed_h10_20261115_135506` | 4.986 m | 8.224 s | 8.832 s | 2.008 s | 0.35° / 0.20° |
| H 10° | `fixed_h_20261115_141310` | 4.999 m | 8.160 s | 8.960 s | 2.012 s | 0.23° / 0.43° |
| H 10° | `fixed_h_20261115_141631` | 4.999 m | 8.248 s | 8.832 s | 2.016 s | 0.50° / 0.40° |
| V 10° | `fixed_v10_20261115_135641` | 4.999 m | 8.280 s | 8.928 s | 2.008 s | 0.24° / 0.36° |
| V 10° | `fixed_v_20261115_141417` | 5.006 m | 8.136 s | 8.808 s | 2.012 s | 0.39° / 0.33° |
| V 10° | `fixed_v_20261115_141737` | 4.997 m | 8.192 s | 8.936 s | 2.016 s | 0.39° / 0.29° |

## 证据

- [flight-summary.json](flight-summary.json)：全部 9 次飞行的逐次数据和统计。
- [hover-feasibility.json](hover-feasibility.json)：X/H10/V10 离线有界悬停检查。
- [x-regression-summary.json](x-regression-summary.json) / [x-regression.png](x-regression.png)：X 回归首条曲线。
- [h10-flight-summary.json](h10-flight-summary.json) / [h10-flight.png](h10-flight.png)：H 10° 首条曲线。
- [v10-flight-summary.json](v10-flight-summary.json) / [v10-flight.png](v10-flight.png)：V 10° 首条曲线。
- 其余逐次曲线：`results/*_summary.json` 和 `results/*.png`。
- 原始 ULog：[Release](https://github.com/RolyForden/PX4/releases/tag/a-uav-fixed-hv-2026-11-15)。
- ULog 文件名、大小和 SHA256：[ulog-manifest.json](ulog-manifest.json)。

## 测试条件

- 构形：10° 固定 H/V；25° 构形目前在运动学台架验证。
- 质量/重心/惯量：BOM 近似；0.393 kg 未定位质量在机身，5 个 CAD 实例无 BOM 质量。
- 动力：沿用基线电机系数、反扭矩、旋向和响应参数。
- 下一阶段：25° 构形、重复性扩展、空中连续变构。
