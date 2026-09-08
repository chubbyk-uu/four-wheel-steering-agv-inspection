# 共享静态场景与几何一致性

> 历史实验记录：本文参数、性能和“当前/默认”描述属于当时版本，不作为新版验收。现行550 kg、20 mm/4K/1.5 m、1 m轨迹间距、11 kHz目标见[当前模型基线](LARGE_AGV_REBUILD.md)。历史命令须按现行配置调整，旧16 mm标定不可用于新版；大体积实验资产可能已清理，复现需重新生成。

当前默认生成水平网格路面（`--profile flat`），不带波浪、深坑、缓坡或挡路屏障。下述既有验证数据来自显式 `--profile optical_stress` 光学夹具；它不属于当前行驶验收范围，旧资产不会因默认值改变而自动变平。

2026-09-07：完成静态几何共享及跨后端验证。不是完整场景真值同步；本记录认证静态几何；后续 AGV 动态几何及正式采集接入见 [OptiX 集成](STAGE2_OPTIX_INTEGRATION.md)，材质/光度一致性仍待实现。

## 单一几何来源

`agv_linescan/shared_scene.py` 根据一组参数生成 100×10 m 场景：约 10 cm 剖分的起伏地面、凹坑、缓坡、直立屏障和架空板，共 200264 个三角形。世界坐标米制，变换烘焙到顶点，SDF 不再额外移动或缩放模型。导出显式逐面法线，满足本机 DART 碰撞网格导入要求；无显式法线的早期样例曾在物理导入时失败，已修复。

- `manifest.json`：模式版本、单位、范围、资产名称、三角形计数及 SHA256。
- 每个对象一份 OBJ，GZ 的 visual 和 collision 引用同一个文件；OptiX 用独立 OBJ 读取器读取这些文件，删除了旧基准内置的第二份地形生成逻辑。
- `world.sdf`：没有隐藏的默认平面，实际碰撞地面就是这份网格。
- `grid.png`、`grid.mtl`：GZ 网格展示。OptiX 暂时仍使用程序网格反射率；两者不保证相同灰度、纹理插值或照明，因此本阶段只认证几何一致。

加载前核对网格、展示材质和 SDF 的校验值，并检查 visual/collision 的 URI、单位缩放、模型集合和零变换。资产目录使用导出时的绝对 URI，移动目录需重新生成，不能直接搬迁后忽略检查。

## 已验证

- Ogre2 从 SDF 中导入视觉网格并实际渲染预览；随后使用 CPU RayQuery 查询几何。OptiX 从 manifest 独立导入 OBJ，查询相同原点、方向和距离上限。
- 246 条射线的命中/未命中及对象身份全部一致；命中距离最大差 **1.788×10⁻⁷ m（约 0.00018 mm）**，低于预设 50 µm 门限。这是选定射线的后端一致性，不代表场景建模的绝对精度或全部边界均经过验证。
- 查询包含 49 条相机视线、196 条灯光可见性路径和 1 条背景射线。灯光路径中 22 条被挡、174 条畅通；相机视线覆盖地面、屏障及架空板。没有比较两套光度模型或最终灰度。
- GZ 实际加载共享世界并执行 20 步，无物理导入错误；另用正常 `sim.launch.py` 加载 AGV，发送带仿真时间戳的零速度，静置 3 秒后 HOLD，车体位置约 `(2.00009, -0.000006, 0.36805)` m。尚未做斜坡行驶、坑洼通过性或完整接触动力学验收。
- OptiX 读取新场景后的短回归：16 个 CPU 像素核对最大差 0 DN；有/无噪声的分批结果逐像素一致，阴影有效。仅为功能回归，未重新作 30 秒持续性能验收。
- 44 项包测试通过，包括修改资产、碰撞偏移/缩放后的拒绝检查和网格法线输出检查。
- 启动入口拒绝共享三维场景配旧平面/解析成像后端，避免视觉与采样静默分叉。当前正式 GZ 渲染后端可以配共享场景，但仍是低速参考路线。

报告 `results/stage2_shared_scene.json`；预览 `docs/shared_scene_gz.png`。测试数据 `/tmp/agv_shared_scene_v2`，交叉验证原始记录 `/tmp/agv_shared_validation_final`，AGV 启动日志 `/tmp/agv_shared_agv_sim.log`，包测试 `/tmp/agv_shared_tests_final.log`。

## 复现

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/generate_shared_scene.py --profile optical_stress --output /tmp/agv_shared_scene
bash tools/optix_scan/build.sh /tmp/agv_shared_optix_build > /tmp/shared_build.log 2>&1
python3 tools/validate_shared_scene.py \
  --manifest /tmp/agv_shared_scene/manifest.json \
  --output /tmp/agv_shared_check > /tmp/shared_check.log 2>&1
```

输出目录需尚不存在。OptiX 使用既有实验性隔离运行库，不修改系统驱动；Ogre2 默认 D3D12。

在 AGV 场景中打开：

```bash
ros2 launch agv_bringup sim.launch.py \
  scene_manifest:=/tmp/agv_shared_scene/manifest.json spawn_x:=2
```

默认仍跟随 AGV。若需要低速 GZ 参考成像可额外传 `linescan:=true linescan_backend:=render`；不要传 `cuda_tiles` 来拍摄该三维场景。`optix` 后端的隔离运行与采集步骤见 [OptiX 集成](STAGE2_OPTIX_INTEGRATION.md)。

后续已复用该资产协议接入 AGV 九组运动几何、实际灯具附件和正式采集接口。静态资产协议本身仍不能保证曝光时刻的动态一致，需结合逐曝光姿态输入与测试。
