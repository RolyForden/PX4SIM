# 固定 H/V 小角飞行结果

日期：2026-11-15。固定构形角 10°，PX4 SITL + Gazebo Harmonic，近似 A 模型。

## 结论

X 回归、固定 H 10°、固定 V 10° 三次都完成起飞、约 5 m 悬停、Land、接地和自动上锁；三次 ULog dropout 均为 0。最终状态为未解锁、已接地。

## 飞行汇总

| 构形 | 日志 | 最大高度 | 稳定悬停 | Land 至接地 | 接地至上锁 | 最大滚转/俯仰 |
|---|---|---|---:|---:|---:|---:|
| X 回归 | `rigid_20261115_135319.ulg` | 4.998 m | 8.144 s | 8.816 s | 2.016 s | 0.34° / 0.29° |
| H 10° | `fixed_h10_20261115_135506.ulg` | 4.986 m | 8.224 s | 8.832 s | 2.008 s | 0.35° / 0.20° |
| V 10° | `fixed_v10_20261115_135641.ulg` | 4.999 m | 8.280 s | 8.928 s | 2.008 s | 0.24° / 0.36° |

## 证据文件

- [flight-summary.json](flight-summary.json)：三次飞行的数值汇总。
- [x-regression-summary.json](x-regression-summary.json) / [x-regression.png](x-regression.png)：X 回归。
- [h10-flight-summary.json](h10-flight-summary.json) / [h10-flight.png](h10-flight.png)：H 10°。
- [v10-flight-summary.json](v10-flight-summary.json) / [v10-flight.png](v10-flight.png)：V 10°。
- [hover-feasibility.json](hover-feasibility.json)：X/H10/V10 离线有界悬停检查。

原始 ULog 和参数快照在本地：

- X：`candidate_flight/logs/rigid_20261115_135319.ulg`，参数在 `candidate_flight/parameters/`。
- H10 / V10：`a_morphing/fixed_flight/logs/`，参数在 `a_morphing/fixed_flight/parameters/`。

ULog 不提交进 Git，后续通过 Release 发布；文件名、大小和 SHA256 见 [ulog-manifest.json](ulog-manifest.json)。

## 边界

- 这是 10° 固定小角构形，不是 25° 构形，也不是空中连续变构。
- 动力系数、反扭矩、旋向和响应时间仍是借用/示意值。
- 质量/重心/惯量为 BOM 近似；0.393 kg 未定位质量仍在机身，5 个 CAD 实例无 BOM 质量。
- 坐标、质量分组和固定构形角都可替换；不等同于最终加工版或实机参数。
- 结果只证明当前 SITL 近似模型的基本闭环可用，不能证明真实整机性能。
