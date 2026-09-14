# 展开瓦片、缓存校验与GPU配方演进

历史实验及旧资产恢复命令；当前使用[道路指南](../../ROAD_ASSETS.md)中的紧凑生成器，不必先生成展开瓦片。

2026-09-09维护：旧小试片生成器也采用显式浅凹陷碰撞代理，与全宽生成器统一；不自动修改既有资产。参见[碰撞约束](../../issues/COLLISION_PROXY.md)。OptiX无灯具连杆的台架回退现在从led_length_m计算发光段，并保留两端各20 mm非发光区；正式整车仍从URDF导出的发光点计算。0.60/1.20 m台架与显式灯具坐标的真实GPU阴影对照已通过。回归数据见[场景护栏](../../../results/review_scene_guards.json)。

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

10 km/h、20 m、GUI/RViz整车另测：14块/54,605行、ROS与归档完全一致、零必需缓存缺失、最终HOLD；最大缓存等待0.050 ms。不过当前环境实时率约0.51，隔离构建修改前版本同条件约0.52，均未复现历史0.99。后续已定位并修复采集位姿插值的重复YAML解析，实时率恢复0.996，见[修复记录](../../issues/SCAN_STABILITY.md)。仍不能把采集完整性passed解释为实时验收passed。详见[完整对照](../../../results/review_material_cache.json)。

## 100×10 m完整采集区（生成接入中，2026-09-10）

空间/资源依据见[全区域预算](../../FULL_ROAD_ACCEPTANCE.md)。用户提出磁盘成本后，全量烘焙已停止；下面命令保留作未压缩基准，不是当前推荐继续执行的步骤。先验证该文档中的材质复用/按需生成方案；20 m快速回归资产保留：

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

20 m对照试片已验证，巡检默认入口已切换到正式资产目录的紧凑版。下面的转换命令用于已有旧烘焙场景；新恢复请优先用本文开头的`--runtime-material`直接生成，无须先展开旧高清瓦片。无需新增SDK；现有CUDA/OptiX构建会包含`agv_runtime_material`。要重做逐字节对照，可显式去掉`--runtime-material`并输出到`assets/road/baked_fullwidth_20m_v1`恢复旧瓦片基准，再运行：

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

比较工具会另导出一份配方，检查所有颜色/法线字节，差异非零即失败。当前配方特化于既有Concrete047A、标线与AI裂缝规则，并非通用材质编辑器；更改底材/标线/裂缝算法后必须重新导出并复核像素。浮点融合关闭以复现CPU参考量化，不宜自行开启fast-math。完整性能、资源边界及下一步见[试片结果](FULL_ROAD_BUDGET_HISTORY.md#gpu按需材质试片结果2026-09-10)。
