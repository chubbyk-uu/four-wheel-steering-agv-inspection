# Gazebo 默认锁定跟车与滚轮缩放

默认 `follow_camera:=true`。启动时相机在AGV后方偏侧，朝道路+X延伸方向；跟随车辆位置并始终看向车辆，掉头后车辆仍保持在视野中心。

- 跟车时锁定拖动朝向和平移，防止手动偏转后车辆离开视野。
- 滚轮向前/向后：拉近/拉远跟车距离；松手后及行驶中保持缩放结果。
- `follow_camera:=false`：关闭自动跟车，恢复普通自由相机。
- `gui_config:=/path/to/gui.config`：自定义初始相机位姿和跟随偏移。

使用CameraTracking的`FOLLOW_LOOK_AT`模式，follow_target和track_target均为agv。辅助脚本要求连续3秒确认，成功后退出；跟车位置增益0.2、看向目标增益1.0。默认`camera_pose`为`-4 -3 3 0 0.35 0.25`，`follow_offset`为`-4 -3 2.6`，偏移随车辆局部坐标旋转。

项目`FollowCameraZoom` GUI插件在同时跟随和看向同一目标时拦截拖动；滚轮按原逻辑更新持久跟随偏移，距离限制0.8～80 m。关闭跟车后恢复原生拖动和缩放。当前验证使用`tools/validate_gui_camera.py`，检查拖动不能改变朝向、滚轮距离能保持以及原地转向后车辆仍在画面中心。

当前锁定模式GUI＋RViz回归通过：中键拖动朝向变化0，滚轮距离5.6356→3.5934→4.1750 m，直行后保持4.1750 m；原地转向后相机光轴与车体方向夹角约1.3×10⁻⁷ rad，结束HOLD。这里测的是相机是否看向车辆，不再要求车辆掉头时相机世界朝向不变。记录见[锁定跟车结果](../results/gui_locked_follow.json)。本项不开线阵采集，不构成采图吞吐测试。

## 历史自由转头验证（已被锁定模式替代）

2026-09-08在本机Gazebo GUI/D3D12中验证：模拟鼠标中键拖动改变朝向0.4503 rad（约25.8°）；松手后及AGV以0.5 m/s继续行驶后，读取的朝向均保持不变。车辆实际位移2.409995 m，相机位移2.410018 m，跟随正常。结束确认HOLD。记录见[gui_free_look.json](../results/gui_free_look.json)。

同日增加滚轮回归：三次向前滚动使距离从5.6356 m变为3.5934 m，两次向后滚动变为4.8506 m；松手及车辆再行驶2.37 m后距离仍为4.8506 m，相机位移与车辆一致。鼠标自由转头继续有效，结束确认HOLD。默认无参数启动已自动确认FOLLOW_FREE_LOOK。见[gui_follow_zoom.json](../results/gui_follow_zoom.json)。

## 使用水泥贴图道路

普通启动仍使用网格世界；水泥贴图资产并未删除。已按[全宽道路记录](archive/stage2/STAGE2_FULLWIDTH_ROAD.md)生成当前20 m全宽资产后，从仓库根目录启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch agv_bringup sim.launch.py \
  scene_manifest:=assets/road/runtime_fullwidth_20m_v1/manifest.json spawn_x:=2
```

此命令默认启用锁定跟车和滚轮缩放，暂不启用线阵采集。交互回归可在GUI加载完成后执行（会模拟鼠标输入并以0.5 m/s短距离行驶，结束停车）：

```bash
python3 tools/validate_gui_camera.py --output /tmp/agv-camera-check.json
```

水泥贴图场景复测通过：朝向改变约25.8°后，松手及行驶期间朝向漂移为0；跟随距离5.6356→3.5934→4.8506 m，车辆继续行驶约2.375 m后保持4.8506 m。相机与车辆位移差约0.0064 mm。使用归一化四元数计算角差；本轮未开启线阵采集，验证范围仅为GUI交互。见[贴图场景复测记录](../results/gui_textured_follow_zoom.json)。

## RViz显示

默认Fixed Frame为`base_link`，Grid Reference Frame为`world`，通过`/visualization/tf`同时间戳轮组快照显示悬挂及转向。世界网格相对跟车视角移动，车体与固定附件保持刚性连接。该显示参考系不改变控制、定位或归档坐标；自行切回world时，逐部件latest-time查询仍可能混入异步世界位姿。旧同时间戳计数只能证明消息一致性，不能证明屏幕无抖动。最新检查见[维护记录](archive/integration/MAINTENANCE_AUDIT.md)。
