# 参考模型与推力矩阵工具

使用`UAV.STEP`建立X/H/V运动学台架。[阶段成果](../../docs/参考模型阶段成果.md) · [矩阵说明](分配矩阵说明.md) · [配置](bench_assumptions.json)

## 工具

| 文件 | 用途 |
|---|---|
| prepare_cad.py | 读取../现实材料/UAV.STEP，导出网格和几何检查 |
| kinematics.py | 构形运动学与效能矩阵 |
| build_bench.py | 生成固定台架bench.sdf |
| command_shape.py | 发送构形指令、读回旋翼位姿、生成验证JSON |
| gazebo_V_reference.png | Gazebo台架截图 |
| cad_preview.png | 几何预览 |

## 构建

使用根目录Python依赖；重新导出CAD网格另需`requirements-cad.txt`。

```sh
python px4_rtl_land_validation/morphing_prototype/kinematics.py
python px4_rtl_land_validation/morphing_prototype/prepare_cad.py
python px4_rtl_land_validation/morphing_prototype/build_bench.py
```

## 运行

容器`px4-rtl-land-harmonic`将项目目录挂载到`/work`，台架分区为`px4_morph_bench`。服务器与GUI分别占用一个终端：

```sh
docker exec -e GZ_PARTITION=px4_morph_bench px4-rtl-land-harmonic gz sim -s -r /work/morphing_prototype/bench.sdf
docker exec -e GZ_PARTITION=px4_morph_bench px4-rtl-land-harmonic gz sim -g
python px4_rtl_land_validation/morphing_prototype/command_shape.py X
python px4_rtl_land_validation/morphing_prototype/command_shape.py H
python px4_rtl_land_validation/morphing_prototype/command_shape.py V
```

切换脚本先回中立态，再到目标构形，输出`verification_*.json`。需要归档的旧结果先保存。使用其他容器或挂载路径时，同步调整SDF资源路径和`command_shape.py`中的`PREFIX`。
