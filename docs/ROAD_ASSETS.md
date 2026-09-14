# 当前道路资产与恢复

巡检默认使用20×10 m全宽Concrete047A道路，100×10 m含缓冲区紧凑资产也已生成并完成全区采集验收；普通sim启动的网格场景与巡检入口须区分。道路含5×5 m板块、中央双黄实线y=±0.15 m、白边线y=±4.5 m，细长分叉裂缝名义宽0.8–2 mm。颜色＋法线贴图，统一粗糙度0.60；精细沟槽用于成像，轮胎使用受校验的浅凹陷碰撞代理。

从仓库根目录、已加载ROS和工作区环境运行：

```bash
python3 tools/fetch_ambient_concrete.py
python3 tools/generate_streaming_road.py \
  --output assets/road/runtime_fullwidth_20m_v1 --length 20 --full-width --runtime-material
ros2 launch agv_bringup sim.launch.py rviz:=true \
  scene_manifest:=assets/road/runtime_fullwidth_20m_v1/manifest.json spawn_x:=6
```

下载代理按本机配置显式传给下载工具。输出目录必须不存在；已有可用资产不需要重新生成。OptiX采图还需[对应运行环境](OPTIX_SETUP.md)；任务采集命令见[采集指南](RECTANGLE_CAPTURE.md)。

高清包含边界余量，范围x=[−1.024,21.504]、y=[−6.144,6.144] m。0.25 mm/纹素，逻辑上1056块颜色R8＋法线RG8。现行紧凑版保存源素材与布局，由GPU按需生成高清瓦片，不再预存约13.34 GB展开纹素。20 m场景约0.93 GB（含保留的布局/对照清单），仍另需采图空间。32槽纹素约385.5 MiB，配方源素材/工作缓冲另约642 MB；两者均不含BVH、输出、驱动和GUI。

GZ约4 mm/纹素显示地图，OptiX按交点从高清瓦片采样，两者使用相同世界坐标。旧资产曾有显示UV镜像：

```bash
python3 tools/fix_road_display_uv.py assets/road/baked_fullwidth_20m_v1/manifest.json
```

该迁移不重烘焙高清纹素；完成后重启Gazebo。新生成器已修正。错误原因及实际渲染验证见[UV问题记录](issues/ROAD_DISPLAY_ALIGNMENT.md)。裂缝碰撞边界见[碰撞说明](issues/COLLISION_PROXY.md)。

100×10 m全区稳定采集已通过，现行结果见[全区验收](FULL_ROAD_ACCEPTANCE.md)。历史100 m窄走廊、20 m全宽、展开瓦片与紧凑配方是不同阶段，不混用其资源数字或恢复命令；本页带日期的旧台架命令只用于对应历史资产回归。


## 配方与校验边界

当前GPU配方特化于Concrete047A、标线和裂缝布局；不是通用材质编辑器。修改材质规则后须重新生成并与独立参考比较，不能自行开启fast-math改变像素。固定槽位、预热和有界等待仍保留，缺块不得用低清占位维持行频。组件显存不等于进程或整卡占用。

旧展开瓦片、不可变文件哈希复用对照与配方转换命令见[路线历史](archive/maintenance/ROAD_MATERIAL_EVOLUTION.md)，不作为新机器默认恢复步骤。

若需要运行下文CPU/GPU像素复核，可在已构建agv_linescan后包装现有静态库（CUDA安装目录按本机环境调整）：

```bash
mkdir -p local_data/recipe_check
g++ -shared -o local_data/recipe_check/libruntime_material.so \
  -Wl,--whole-archive build/agv_linescan/libagv_runtime_material.a \
  -Wl,--no-whole-archive -L/usr/local/cuda/lib64 -lcudart -lcrypto
```

## 直接生成100 m紧凑场景

```bash
python3 tools/generate_streaming_road.py \
  --output assets/road/runtime_fullwidth_100m_v1 --length 100 --full-width \
  --end-buffer 8 --side-buffer 1.5 --runtime-material
python3 tools/check_full_road.py assets/road/runtime_fullwidth_100m_v1/manifest.json \
  --output local_data/compact100_asset_check.json
ros2 launch agv_bringup inspection.launch.py \
  scene_manifest:=assets/road/runtime_fullwidth_100m_v1/manifest.json
```

`--runtime-material`只支持全宽道路和新输出目录，不与旧瓦片续烘焙混用。100×10 m为ROI，可行驶区为x=[−8,108]、y=[−6.5,6.5] m；高清域更大。首轮完整资产2.45 GB、约3分30秒生成、生成RSS峰值3.48 GB；逻辑高清6930块，但磁盘不保存展开瓦片。48个显示分区、7712280个视觉/OptiX三角形、96个碰撞代理三角形。静态检查不能代替运动或图像验收。

要复核GPU与CPU原配方，先按前文编译独立库，再运行：

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/check_runtime_recipe.py \
  assets/road/runtime_fullwidth_100m_v1/manifest.json \
  --library local_data/recipe_check/libruntime_material.so \
  --output local_data/compact100_pixel_check.json
```

它只临时生成代表瓦片，不将整条道路展开。当前首尾/外侧缓冲区、黄白标线、两种翻转裂缝的所有对比字节一致。旧`baked_*`目录不是紧凑版运行依赖；旧压缩/全瓦片对照工具仍需要显式恢复旧基准，不能把配方目录冒充旧瓦片目录。

## 箭头和禁停网格标识试片

在恢复20 m紧凑资产后生成独立试片：

```bash
python3 tools/generate_marked_road.py --output local_data/road_markings_probe_v1
```

输出目录必须不存在。复用原资产不可变文件的硬链接，修改配方、SDF及显示PNG前先断开链接，因此原道路保持不变，避免复制近GB源纹理。不要手动覆写试片内共享的`.raw`、网格或quilt文件。删除试片不影响原道路。跨文件系统不支持硬链接，需在同一文件系统生成。

试片manifest可传给已有`--scene`采集工具。标识几何与用途见[SURFACE_HEADING_STRIP_DESIGN.md](SURFACE_HEADING_STRIP_DESIGN.md)。显示4 mm纹素、采样0.25 mm纹素仍沿用原配置。新增标识仅在缓存烘焙时计算，不在每根扫描线的射线命中路径中计算；不能把单次烘焙计时当成整车11 kHz验收。

实际显示检查：`tools/render_road_alignment.py --manifest local_data/road_markings_probe_v1/manifest.json --output local_data/road_markings_ogre_v1`（使用当前Mesa/ROS运行环境）。除原双黄线坐标外，新增实际Ogre2禁停框像素与独立俯视投影的重合率检查。

高清烘焙检查工具`tools/validate_road_test_markings.py`接受`--source`原recipe.json、`--trial`试片recipe.json、`--library`暴露recipe C接口的共享库、`--output`结果JSON。可将构建产物包装为测试共享库：

```bash
g++ -shared -o /tmp/libagv_recipe_probe.so -Wl,--whole-archive build/agv_linescan/libagv_runtime_material.a -Wl,--no-whole-archive -L/usr/local/cuda/lib64 -lcudart -lcrypto
LD_LIBRARY_PATH=/usr/local/cuda/lib64 python3 tools/validate_road_test_markings.py --source assets/road/runtime_fullwidth_20m_v1/recipe.json --trial local_data/road_markings_probe_v1/recipe.json --library /tmp/libagv_recipe_probe.so --output local_data/road_markings_probe_v1/cuda_validation.json
```
