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

100×10 m全宽场景仍需后续预算及新版整车验收；历史100 m中央窄走廊与20 m全宽测试不是同一个范围。原始生成预算、旧性能和复现记录保留在[全宽实验](archive/stage2/STAGE2_FULLWIDTH_ROAD.md)、[100 m走廊实验](archive/stage2/STAGE2_STREAMED_ROAD.md)中。


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
