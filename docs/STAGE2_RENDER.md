# C++ Gazebo 线阵场景采样后端

> 历史实验记录：本文参数、性能和“当前/默认”描述属于当时版本，不作为新版验收。现行550 kg、20 mm/4K/1.5 m、1 m轨迹间距、11 kHz目标见[当前模型基线](LARGE_AGV_REBUILD.md)。历史命令须按现行配置调整，旧16 mm标定不可用于新版；大体积实验资产可能已清理，复现需重新生成。

> 当前运行参数已迁移为 **4K / 1.2 m、行间距 0.29296875 mm**。高行频实现与验收记录见 [CUDA 批量采样说明](STAGE2_CUDA.md)；11 kHz / 22 kHz 的完整成像链路验收仍未完成。


## 实现与运行

`agv_gz_linescan` 是加载在 GZ 服务器内的自定义系统插件。通过 `RenderUtil` 从实体组件同步真实场景，在专用线程使用 Ogre2 渲染。不使用标准相机传感器的定时发布，不逐行经过 ROS，也不再以解析网格值代替渲染结果。GUI 仍在独立进程，采用项目默认 D3D12 / NVIDIA 和跟随视角。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release > /tmp/agv_build.log 2>&1
source install/setup.bash
ros2 launch agv_bringup sim.launch.py linescan:=true capture_dir:=/tmp/agv_rendered_linescan
```

`linescan_backend:=render` 为开启线阵后的默认后端；`linescan_backend:=analytic` 可回到 Python 解析测试基线。不开启线阵时，原有控制启动方式不变。`headless:=true` 关闭 GUI，但服务器仍需图形上下文；本机使用 WSLg / D3D12，不自动退回软件渲染。

开启和关闭采集仍使用 `/linescan/set_enabled` 的 `std_srvs/srv/SetBool` 服务，运动通过 `/cmd_vel` 控制器。关闭采集的成功响应表示已完成已有图块的写盘；先关闭采集再退出仿真。输出到独立 `session_cpp_*` 目录，包含未经校正的 Mono8 PGM、JSON 标签、参数快照和中断事件。PGM 为无损未压缩格式，避免 PNG 压缩占用采集线程；现有 `rectify_linescan.py` 可以读取。

## 时序、投影与吞吐设计

- 从物理步的四轮关节位置生成编码器触发，不依赖 ROS `/joint_states` 或 50 Hz 真值里程计更新。车体真值姿态仅在传感器内部生成观察射线，未接入控制或定位算法。
- 单个物理步可以产生多次触发，逐个保存独立时间；曝光跨步时等下一步提供完整插值区间。相机位置线性插值，姿态 SLERP。1 ms 步长在最高车速下约有 18～19 次触发，不重复使用同一时刻图像代替这些行。
- 渲染的是实际横向窄视场：内部 5120×1，以 4096×7 μm、20 mm 焦距及一维像素射线多项式推导视野。额外横向范围用于畸变映射与插值，不压缩普通面阵的完整纵向视野。
- 每行曝光用三个时间中点近似。三个渲染相机在场景开始渲染前分别设置到各自位姿，共用一次场景 `PreRender` / `PostRender`，启用 GZ 的 GPU 提交批处理（最多 6 个 pass 后刷新），之后分别读回并累计为一行。这是曝光视图批处理，**尚未实现多行 GPU 图块累计或异步批量读回**。
- 渲染与像素处理在 C++ 内完成，4096 行累计为一个图块。图像 `header.stamp` 仍为末行曝光中点，标签包含首末行及少量中间行的三维外参、时间和编码器距离。全局行号连续，反向或停采重新分段，不补造缺失行。
- 图块写盘与 ROS 发布移到有界后台任务，最多两个在途图块；超出容量等待，不无声丢弃。`publish_call_seconds` 只代表发布调用时间，不能当作端到端传输延迟。验收工具实际订阅完整 ROS 图像，并核对尺寸、字节数和末行时间戳。
- 物理线程等待本步需要的渲染完成，保证不积累无限采样队列。负载过高会降低仿真实时率，不会自动降低仿真车速；需要根据测量结果主动降低 `/cmd_vel` 的扫描速度。`scan_speed_limit` 是采集准入上限，超限时分段停采，不是自动车速控制。

## 模型与网格修正

真实渲染发现原先相机前方的两根支架遮住扫描平面，因此光学中心前向位置改为车体 `x=0.70 m`，高约 `0.83705 m`；LED 机械位置随相机保持前置偏移。相机参数与 URDF 使用同一配置，质量总额仍按原模型预算。

10 cm 网格、2 mm 线宽合并为一个 OBJ 网格，避免约 400 个独立视觉模型的绘制提交开销。网格具有明确法线和材质，作为地面涂画标记，不投射细条几何阴影。它不是水泥路面贴图。

`scan_probe:=true` 可加入位于 `(1.2, 0.5)`、顶面高 6 cm 的红色测试板，验证真实材质、视差和遮挡。该标记仅用于传感器诊断，无碰撞模型，不能作为障碍避让测试。

## 验证

```bash
colcon test --packages-select agv_linescan agv_control > /tmp/agv_linescan_tests.log 2>&1
colcon test-result
python3 tools/validate_rendered_linescan.py --probe > /tmp/agv_render_test.log 2>&1
```

工具使用独立 ROS 域（默认 81）和 GZ 分区，发送真实控制命令，结束后停车并关闭实例。检查完整 4K 图块与尾块、行号和编码器距离连续性、网格中心投影误差、测试板遮挡、ROS 图块送达，以及分阶段耗时。与解析测试台相比，此后端多了真实场景、光照、遮挡和 GPU 渲染工作，两者的行频不能直接当作同工作负载的性能对比。

## 当前边界

- 当前验收对象是静态网格与静态遮挡物。场景物体状态按当前物理步更新，相机按每个曝光采样时刻插值；其他运动物体尚未按子步插值。标签记录场景快照时间，不能宣称高速动态遮挡物的时序已准确复现。
- 当前保留 GZ 场景光照、材质和阴影，但灰度来自已渲染 RGB 的亮度组合；曝光时长参与几何采样，尚未实现线性辐射能量、短曝光亮度、噪声、饱和及主动 LED 阴影抑制模型。
- 镜头像素到射线映射、横向反采样校正沿用解析原型，自动实测标定与运动几何校正未实现。不能将已有标定参数注入和校正称为完整标定验收。
- 渲染上下文由本插件独占一个服务器渲染线程。当前世界没有另一个 GZ Sensors 渲染系统；后续接入其他 GPU 传感器时，应统一渲染线程和生命周期，不能直接在第二个线程共享 Ogre2 引擎。

接口依据：本机 Gazebo Harmonic 的 `RenderUtil.hh`、`Camera.hh`、`Scene.hh`；对应官方 [RenderUtil 源码](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/rendering/RenderUtil.cc) 和 [Scene 渲染批处理接口](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8/include/gz/rendering/Scene.hh)。
