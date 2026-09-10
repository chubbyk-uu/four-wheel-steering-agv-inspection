# 当前道路资产与恢复

当前使用20×10 m全宽Concrete047A道路，默认普通启动仍为网格。道路含5×5 m板块、中央双黄实线y=±0.15 m、白边线y=±4.5 m，细长分叉裂缝名义宽0.8–2 mm。颜色＋法线贴图，统一粗糙度0.60；精细沟槽用于成像，轮胎使用受校验的浅凹陷碰撞代理。

从仓库根目录、已加载ROS和工作区环境运行：

```bash
python3 tools/fetch_ambient_concrete.py
python3 tools/generate_streaming_road.py \
  --output assets/road/baked_fullwidth_20m_v1 --length 20 --full-width
ros2 launch agv_bringup sim.launch.py rviz:=true \
  scene_manifest:=assets/road/baked_fullwidth_20m_v1/manifest.json spawn_x:=6
```

下载代理按本机配置显式传给下载工具。输出目录必须不存在；已有可用资产不需要重新生成。OptiX采图还需[对应运行环境](OPTIX_SETUP.md)；任务采集命令见[采集指南](RECTANGLE_CAPTURE.md)。

高清包含边界余量，范围x=[−1.024,21.504]、y=[−6.144,6.144] m。0.25 mm/纹素，共1056块颜色R8＋法线RG8，未压缩纹素约13.34 GB，另需源图、几何、临时数据和采图空间。32槽纹素负载约385.5 MiB，不含BVH、输出、驱动和GUI；不能将其当作进程总显存。

GZ约4 mm/纹素显示地图，OptiX按交点从高清瓦片采样，两者使用相同世界坐标。旧资产曾有显示UV镜像：

```bash
python3 tools/fix_road_display_uv.py assets/road/baked_fullwidth_20m_v1/manifest.json
```

该迁移不重烘焙高清纹素；完成后重启Gazebo。新生成器已修正。错误原因及实际渲染验证见[UV问题记录](issues/ROAD_DISPLAY_ALIGNMENT.md)。裂缝碰撞边界见[碰撞说明](issues/COLLISION_PROXY.md)。

下一步先预算并生成100×10 m全宽采集区域及额外加减速/转场缓冲区，再开展新版整车完整采集验收；它是正式拼接前的必经阶段，顺序和门槛见[实施计划第6节](MISSION_IMPLEMENTATION_PLAN.md)。历史100 m中央窄走廊与20 m全宽测试不是同一个范围。原始生成预算、旧性能和复现记录保留在[全宽实验](archive/stage2/STAGE2_FULLWIDTH_ROAD.md)、[100 m走廊实验](archive/stage2/STAGE2_STREAMED_ROAD.md)中。


2026-09-09维护：旧小试片生成器也采用显式浅凹陷碰撞代理，与全宽生成器统一；不自动修改既有资产。参见[碰撞约束](issues/COLLISION_PROXY.md)。OptiX无灯具连杆的台架回退现在从led_length_m计算发光段，并保留两端各20 mm非发光区；正式整车仍从URDF导出的发光点计算。0.60/1.20 m台架与显式灯具坐标的真实GPU阴影对照已通过。回归数据见[场景护栏](../results/review_scene_guards.json)。

## 分块加载预算与校验复用（2026-09-09）

保持32个GPU槽位、单加载线程、50 ms热等待期限；不静默降清晰度，不跳过缺失纹理。新增read/sha256/upload的总耗时与最长单块耗时、sha256_bytes、verification_reuse_files统计。文件首次读取校验完整SHA256；本次采样器生命周期内，只在预期哈希以及文件dev/inode/size/纳秒mtime/ctime全部一致时复用校验结论。读取前后核对同一文件描述符，文件被替换、截断、改写或manifest哈希改变会重新校验/拒绝。该机制要求采集期间源资产不可变，不监控已驻留GPU的文件变化，也不作为可变网络文件系统的完整性保证。不会增加GPU纹理槽位或复制整个路面到内存。

20 m全宽道路上的台架以4096×4096、三曝光样本、16个LED阴影样本、原图fsync及独立ROS接收校验运行96块，约35.1秒；每10块反向一次（相机X=2..17 m，约15 m一程），包含大量跨块和逐出重载。优化前/后1211/1210次加载、1187/1186次逐出，SHA256总耗时6.254/0.765秒，加载总耗时9.615/5.227秒。两次均约11.20 kHz、预热后零必需瓦片缺失，96张PGM逐字节一致。操作系统缓存未清空，加载次数相差一块来自异步预取；这些是观察值，不是磁盘抖动最坏保证，也不是11.20 kHz整车物理闭环验证。

复现当前独立台架（工作区已source，OptiX运行库按安装指南设置）：

```bash
python3 tools/benchmark_optix_stream.py \
  --scene assets/road/baked_fullwidth_20m_v1/manifest.json \
  --output local_data/cache_audit --blocks 96 --pass-blocks 10 \
  --start-x 2 --track-y 0 --rate 11200
```

10 km/h、20 m、GUI/RViz整车另测：14块/54,605行、ROS与归档完全一致、零必需缓存缺失、最终HOLD；最大缓存等待0.050 ms。不过当前环境实时率约0.51，隔离构建修改前版本同条件约0.52，均未复现历史0.99。后续已定位并修复采集位姿插值的重复YAML解析，实时率恢复0.996，见[修复记录](issues/SCAN_STABILITY.md)。仍不能把采集完整性passed解释为实时验收passed。详见[完整对照](../results/review_material_cache.json)。

## 100×10 m完整采集区（生成接入中，2026-09-10）

空间/资源依据见[全区域预算](FULL_ROAD_ACCEPTANCE.md)。用户提出磁盘成本后，全量烘焙已停止；下面命令保留作未压缩基准，不是当前推荐继续执行的步骤。先验证该文档中的材质复用/按需生成方案；20 m快速回归资产保留：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/generate_streaming_road.py \
  --output assets/road/baked_fullwidth_100m_v1 --length 100 --full-width \
  --end-buffer 8 --side-buffer 1.5 --workers 4
python3 tools/check_full_road.py assets/road/baked_fullwidth_100m_v1/manifest.json \
  --output results/full_road_asset_integrity.json
```

100×10 m仍是采集区；实体缓冲区扩展至116×13 m，高清光学域再大一圈。缓冲区同样烘焙颜色/法线；它没有延长采集ROI，也不允许规划器把所有光学裙边当成可行驶范围。`--workers`限定1～4，各线程写独立图块、共享只读材质布局，像素结果不随并行数改变。最长显示分区约5×7.68 m，GZ保持约4 mm显示图，RViz单块最长边1024像素。默认不更换已有20 m巡检入口。

进度写在生成目录`bake_progress.json`，完整成功后才产生通过共享场景验证的manifest。中断后只可在代码、源图和recipe一致时使用`--reuse-tiles`；新旧烘焙器哈希不匹配时使用新目录，不绕过校验。高清文件采用临时文件完成后重命名，结束时逐邻块检查颜色及法线gutter完全一致。独立检查工具再次核对全部原始瓦片哈希、分区覆盖、显示尺寸及显式碰撞代理。


## 实验性按需GPU材质（2026-09-10）

20 m对照试片已验证，默认入口暂不切换。无需新增SDK；现有CUDA/OptiX构建会包含`agv_runtime_material`。需要先按上文恢复可对照的20 m资产和Concrete047A源图，然后运行：

```bash
python3 tools/create_recipe_scene.py \
  --scene assets/road/baked_fullwidth_20m_v1/manifest.json \
  --output local_data/runtime_recipe_scene
ros2 launch agv_bringup sim.launch.py rviz:=true \
  scene_manifest:=local_data/runtime_recipe_scene/manifest.json spawn_x:=6
```

工具复用原几何/显示资产并导出源材质配方；输出目录必须不存在，当前使用本地硬链接，因此输出与参考资产须在同一文件系统。逻辑场景文件完整，不依赖原目录的高清瓦片；运行期间不得改写共享源文件。移到其他机器时应复制全部输出文件。`local_data`、源图和生成资产不提交Git。

独立全瓦片复现（先建立输出目录供编译，配方由上面的场景生成工具提供）：

```bash
mkdir -p local_data/recipe_check
nvcc -shared -Xcompiler=-fPIC --fmad=false -O3 -std=c++17 -arch=sm_75 \
  src/agv_linescan/src/optix/runtime_material.cu -lcrypto \
  -o local_data/recipe_check/libruntime_material.so
python3 tools/probe_runtime_material.py \
  --output local_data/recipe_compare \
  --library local_data/recipe_check/libruntime_material.so --all --repeats 1
```

比较工具会另导出一份配方，检查所有颜色/法线字节，差异非零即失败。当前配方特化于既有Concrete047A、标线与AI裂缝规则，并非通用材质编辑器；更改底材/标线/裂缝算法后必须重新导出并复核像素。浮点融合关闭以复现CPU参考量化，不宜自行开启fast-math。完整性能、资源边界及下一步见[试片结果](FULL_ROAD_ACCEPTANCE.md#gpu按需材质试片结果2026-09-10)。
