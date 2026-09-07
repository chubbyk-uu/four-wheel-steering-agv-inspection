# Four-Wheel Steering AGV Inspection

四轮独立驱动、独立转向（4WIDS）的 ROS 2 / Gazebo 巡检仿真项目。八电机底盘支持横移、斜行和原地旋转；控制器根据实际轮组状态处理驱动反转、转向限位、制动和停稳对轮。

![AGV](docs/agv_stage1.png)

## 当前范围

- 65 kg 参数化底盘，车体约 0.90 × 0.65 m；仿真限速 **10 km/h**。
- 线阵相机 **4096 像素 / 1.2 m**，约 **0.293 mm/像素**；16 mm 镜头，名义光心高度约 0.670 m。
- 编码器等距触发，沿同一路径连续生成无重叠图块；标签参考末行，支持稀疏位姿记录。
- 可选 CUDA 平面后端与实验性 OptiX 三角网格后端，含相对条形 LED、遮挡、曝光和横向畸变模型。
- 已选 **Concrete047A**：10 × 10 m 离线样片、5 × 5 m 板缝、标线、AI 图像来源的 0.8–2 mm 名义宽裂缝和小破损。**真实混凝土纹理尚未接入 OptiX，当前相机仍拍网格。**
- 覆盖规划、避障、RTK/IMU/里程计融合与完整巡检拼接尚未完成。11/22 kHz 是专项采样目标；10 km/h 的正常等距供给约 9.48 kHz，不等于完整链路已达到 22 kHz。

规范：[PROJECT_SPEC.md](PROJECT_SPEC.md)。当前结果与历史实验见 [docs](docs)，其中早期参数和性能结论只适用于对应实验。

## 环境与安装

基础环境为 **Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、Python 3.12、C++17/CMake**。支持原生 Linux；已有 WSL2/WSLg 实测，GUI 需要可用硬件图形驱动。ROS/Gazebo 版本配对见[官方说明](https://gazebosim.org/docs/harmonic/ros_installation/)。

先按 [ROS 2 Jazzy 安装文档](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 配置 ROS 软件源，再安装：

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-forward-command-controller ros-jazzy-joint-state-broadcaster \
  python3-colcon-common-extensions python3-rosdep build-essential cmake git curl \
  python3-pytest python3-numpy python3-scipy python3-pil python3-yaml \
  python3-matplotlib python3-opencv libopencv-contrib-dev libssl-dev
# 仅首次初始化 rosdep 时执行；若已初始化则跳过：
sudo rosdep init
rosdep update
```

ROS 使用系统 Python，避免用 Conda 环境替换。烘焙需要 OpenCV contrib 的 `cv2.ximgproc.thinning`：

```bash
python3 -c 'import cv2; assert hasattr(cv2.ximgproc, "thinning")'
```

## 从 GitHub 恢复与构建

仓库本身就是 colcon 工作区，直接在仓库根目录构建：

```bash
git clone https://github.com/chubbyk-uu/four-wheel-steering-agv-inspection.git
cd four-wheel-steering-agv-inspection
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
# 基础构建不要求 NVIDIA GPU、CUDA 或 OptiX：
colcon build --symlink-install --cmake-args -DAGV_ENABLE_CUDA=OFF
source install/setup.bash
```

基础构建可以控制底盘、运行解析网格/慢速 Ogre2 参考采样，不承诺高行频性能。恢复不依赖其他机器人项目、个人目录或历史 /tmp 文件。

## 快速运行

原生 Linux：

```bash
ros2 launch agv_bringup sim.launch.py gpu_backend:=native rviz:=true
```

WSL2 + WSLg + NVIDIA：

```bash
ros2 launch agv_bringup sim.launch.py rviz:=true
```

WSL 默认 Mesa D3D12/NVIDIA；其他显卡可指定 `gpu_adapter:=AMD` 或 `Intel`。无 GUI 用 `headless:=true`。默认自由视角，`follow_camera:=true` 开启跟车，`gui_config:=/path/to/gui.config` 自定义。

另开终端，进入同一仓库并加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/command_velocity.py --vx 0.5 --seconds 5
python3 tools/command_velocity.py --vy 0.5 --seconds 5
python3 tools/command_velocity.py --vx 0.3 --vy 0.3 --seconds 5
python3 tools/command_velocity.py --wz 0.3 --seconds 5
```

工具结束时发送零速度。`/cmd_vel` 是车体坐标系 `geometry_msgs/msg/TwistStamped`，使用 `linear.x/y`、`angular.z` 及当前仿真时间。只运行一个指令发布者，不绕过控制器直接操作关节；切换运动可能先进入 BRAKE、ALIGN，再 DRIVE。

低速网格相机示例（替换上面的启动命令，不同时启动两个实例）：

```bash
ros2 launch agv_bringup sim.launch.py gpu_backend:=native \
  linescan:=true linescan_backend:=analytic
# 另一个已加载环境的终端：
ros2 service call /linescan/set_enabled std_srvs/srv/SetBool '{data: true}'
python3 tools/command_velocity.py --vx 0.1 --seconds 5
ros2 service call /linescan/set_enabled std_srvs/srv/SetBool '{data: false}'
```

采集目录默认 `/tmp/agv_linescan`，可用 `capture_dir:=/path/to/capture` 修改。原图 `/linescan/image_raw`，显示用 `/linescan/image_preview`。正式 GZ 场景参考后端为 `linescan_backend:=render`。

## 可选 CUDA / OptiX

完整分平台步骤见 [OptiX：原生 Linux 与 WSL2 安装指南](docs/OPTIX_SETUP.md)。

高行频后端需要 NVIDIA GPU 和 CUDA toolkit；已有测试组合为 CUDA 12.8、OptiX SDK 9.1.0、RTX 5080。该组合不是最低硬件规格，也不保证其他硬件相同性能。

CUDA 安装按 [NVIDIA 指南](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)，OptiX SDK 从 [NVIDIA 官方仓库](https://github.com/NVIDIA/optix-sdk/releases/tag/v9.1.0) 获取。SDK、运行库和驱动不随本仓库分发。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args \
  -DAGV_ENABLE_CUDA=ON -DAGV_ENABLE_OPTIX=ON \
  -DAGV_OPTIX_SDK="$HOME/opt/optix-sdk-9.1.0"
source install/setup.bash
python3 tools/generate_shared_scene.py --output /tmp/agv_flat_scene
# 原生 Linux，需驱动提供可用 libnvoptix：
ros2 launch agv_bringup sim.launch.py gpu_backend:=native \
  linescan:=true linescan_backend:=optix \
  scene_manifest:=/tmp/agv_flat_scene/manifest.json spawn_x:=2
```

WSL OptiX 是**实验性单独配置**，基础 CUDA 可用不表示 OptiX 可用。详见 [WSL 实验记录](docs/STAGE2_OPTIX_WSL.md)。已有相容隔离运行库时，配置 `AGV_OPTIX_RUNTIME`，通过 `bash tools/with_optix_runtime.sh bash` 进入子进程，再加载 ROS 和工作区并启动。脚本不安装驱动或替换系统库；新机器需要重新验证，不能仅靠克隆复现驱动兼容性。

## 恢复混凝土样片

Git 包含两张 AI 缺陷原图、烘焙脚本、下载元数据和校验摘要；不包含 8K 源颜色图、下载 ZIP 或高分辨率图块。基础仿真不需要下载纹理。

```bash
python3 tools/fetch_ambient_concrete.py
# 需要代理时自行填写：--proxy http://127.0.0.1:7890
python3 tools/bake_concrete_road.py --output assets/road/baked_concrete047a_v1
python3 tools/check_baked_road.py assets/road/baked_concrete047a_v1
python3 tools/preview_baked_road.py assets/road/baked_concrete047a_v1
```

输出目录必须不存在，防止覆盖已有资产。下载约 1 GB，样片约 1.6 GiB，建议为此过程预留至少 4 GiB 磁盘空间；100 × 10 m 全场资产需要更多。大场景纹理缓存尚待实现，不能直接把全部图块装进显存。

Concrete047A 为 [ambientCG CC0 素材](https://ambientcg.com/view?id=Concrete047A)。官方物理尺寸未给出，暂按整张 2.1 m 映射。烘焙约 0.25 mm/纹素；裂缝尖端、分叉和最终相机像素宽度仍需独立验证。[当前样片](docs/concrete047a_comparison.png)。

## 检查与项目结构

```bash
colcon test > /tmp/agv_tests.log 2>&1
colcon test-result --verbose
python3 tools/check_model.py
python3 -m pytest -q tools/test_concrete_quilt.py
```

OptiX/GPU 测试需要对应硬件和运行库；纯 CPU 构建不注册这些可选 GPU 测试。历史结果见 `results/`，不是新机器的测试保证。

| 路径 | 内容 |
| --- | --- |
| `src/agv_description` | 几何、URDF、机械与相机参数 |
| `src/agv_control` | 四轮运动分配、转向限位与状态机 |
| `src/agv_linescan` | 触发、采样、辐射、标定及可选 GPU 后端 |
| `src/agv_bringup` | 启动、GUI/RViz、控制配置 |
| `tools` | 资产生成、验证和操作工具 |
| `assets/road` | AI 原图、来源摘要及恢复说明 |
| `docs` / `results` | 技术记录、样片和历史验证摘要 |

源码按 Apache-2.0 分发；外部素材与生成资产说明见 [THIRD_PARTY.md](THIRD_PARTY.md)。公开仓库不含旧本地 Git 历史、机器工作约定、原始进程日志、个人代理配置、SDK/驱动、构建目录和完整采集数据。详见[发布范围](docs/REPOSITORY.md)。
