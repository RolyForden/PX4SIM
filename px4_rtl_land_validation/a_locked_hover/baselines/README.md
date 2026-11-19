# 锁臂基线输入

`empty`、`load50g` 分别提取自已通过的 `empty_01`、`load50g_01` 锁臂飞行。保存起飞时使用的质量属性、独立 airframe 和 MC/MPC 参数，供机械臂运动与耦合模型构建使用。

这些文件与本地原试验输入逐字节一致。原生 ULog 路径和 SHA256 见上级 results/ulog-manifest.json。运行记录保留在本地 runs，模型构建使用本目录，不依赖本地飞行目录。
