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
