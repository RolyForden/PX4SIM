# 参考模型与推力矩阵工具

**参考模型，非最终样机；这是机构/计算工具验证，不是飞行验证。**

先读[阶段结论](../../docs/参考模型阶段成果.md)、[矩阵说明](分配矩阵说明.md)和[台架假设](bench_assumptions.json)。

## 主要文件

- `prepare_cad.py`：读取上级目录 `UAV.STEP`，输出网格、几何检查和静态预览。
- `kinematics.py`：示例构形与通用效能矩阵计算，运行后生成 `allocation_demo.json`。
- `build_bench.py`：生成固定台架 `bench.sdf`，网格位于 `meshes/`。
- `command_shape.py`：发送 X/H/V 关节指令，读取 Gazebo 位姿并核对，生成 `verification_*.json`。
- `gazebo_V_reference.png`：实际 Gazebo 界面截图。`cad_preview.png` 是静态几何绘图，不是飞行截图。

## 计算与网格生成

已有 Python 环境中安装仓库根目录依赖。重新从 STEP 生成网格还需本目录的 `requirements-cad.txt`；直接使用已导出的网格无需安装 CAD 库。

```sh
python px4_rtl_land_validation/morphing_prototype/kinematics.py
# 仅在需要重新生成网格时执行
python px4_rtl_land_validation/morphing_prototype/prepare_cad.py
python px4_rtl_land_validation/morphing_prototype/build_bench.py
```

## 已验证的 Gazebo 运行布局

当前脚本使用本机已有容器 `px4-rtl-land-harmonic`（Gazebo Sim 8.15.0），将仓库 `px4_rtl_land_validation/` 挂载到容器 `/work`，并设置 `GZ_PARTITION=px4_morph_bench`。SDF 网格路径为 `/work/morphing_prototype/meshes/`。这不是跨机器一键安装包；使用其他目录/容器时需相应调整路径及 `command_shape.py` 的 `PREFIX`。

在已有容器和图形显示配置下，分别运行服务器、GUI，再在宿主机执行切换命令：

```sh
docker exec -e GZ_PARTITION=px4_morph_bench px4-rtl-land-harmonic gz sim -s -r /work/morphing_prototype/bench.sdf
docker exec -e GZ_PARTITION=px4_morph_bench px4-rtl-land-harmonic gz sim -g
python px4_rtl_land_validation/morphing_prototype/command_shape.py X
python px4_rtl_land_validation/morphing_prototype/command_shape.py H
python px4_rtl_land_validation/morphing_prototype/command_shape.py V
```

服务器与 GUI 各占一个终端，图形驱动设置依本机环境决定。切换脚本会先回中立态，覆盖对应验证 JSON；首次运行前保留需归档的旧结果。这里只控制独立仿真关节，不启动 PX4 或发送真实无人机指令。
