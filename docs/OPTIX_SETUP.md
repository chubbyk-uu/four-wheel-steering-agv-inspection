# OptiX 安装：原生 Linux 与 WSL2

SDK 提供头文件、示例和 API；CUDA toolkit 提供编译器及 CUDA 用户库；真正执行光线追踪还需要驱动侧 OptiX 运行库。**只安装 SDK 不够，nvidia-smi 成功也不代表 optixInit 成功。**

| 项目 | 原生 Ubuntu Linux | Windows + WSL2 Ubuntu |
| --- | --- | --- |
| GPU 驱动 | Linux 主机安装相容 NVIDIA 驱动 | Windows 主机安装 NVIDIA Windows 驱动 |
| CUDA 驱动接口 | Linux 驱动的 libcuda | WSL 映射的 /usr/lib/wsl/lib/libcuda.so.1 |
| WSL 内安装 Linux 显示驱动 | 不适用 | 不安装 |
| CUDA toolkit | NVIDIA Ubuntu toolkit | 选 WSL-Ubuntu toolkit，避免附带 Linux 驱动的元包 |
| OptiX SDK | Linux SDK，9.1.0 为本项目测试版本 | 同一 Linux SDK |
| OptiX 运行库 | 通常随相容 Linux 驱动提供 | 先测系统接口；本项目使用额外隔离库的实验路线 |
| GZ 图形后端 | gpu_backend:=native | 默认 Mesa D3D12，需 WSLg |

## 1. 原生 Linux

按 [NVIDIA Linux CUDA 安装指南](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) 安装适配 GPU、CUDA 和 OptiX 版本的驱动/toolkit；不要把某台机器的驱动版本当通用要求。重启并确认 `nvidia-smi`、`/usr/local/cuda/bin/nvcc --version` 正常。确认系统加载器能找到 `libnvoptix.so.1`，不从 WSL 示例目录复制库来替换本机驱动。

SDK 从 [官方 9.1.0 release](https://github.com/NVIDIA/optix-sdk/releases/tag/v9.1.0) 获取，例如：

```bash
mkdir -p "$HOME/opt"
git clone --branch v9.1.0 --depth 1 https://github.com/NVIDIA/optix-sdk.git "$HOME/opt/optix-sdk-9.1.0"
export AGV_OPTIX_SDK="$HOME/opt/optix-sdk-9.1.0"
```

从本项目根目录编译并运行最小初始化探针：

```bash
c++ -std=c++17 -I"$AGV_OPTIX_SDK/include" -I/usr/local/cuda/include \
  results/optix_wsl_probe/init.cpp -L/usr/local/cuda/lib64/stubs \
  -lcuda -ldl -o /tmp/agv_optix_init
/tmp/agv_optix_init
```

这里 stubs 仅用于**链接**，不得加入运行时 LD_LIBRARY_PATH；运行时必须加载真实驱动。cuInit、optixInit、设备上下文创建都应返回 0。

再按 SDK 自带构建说明构建 `optixTriangle` 示例，并运行一次 `--dim=64x48 --file /tmp/optix_triangle.ppm`，确认真实 optixLaunch 和输出，不止初始化。示例构建依赖以 SDK 的 README/CMake 为准。

## 2. WSL2：先装 CUDA，再单独验证 OptiX

在 Windows 安装适配 GPU 的 NVIDIA 驱动，并在 PowerShell 执行 `wsl --update`。WSL 内按 [NVIDIA WSL 指南](https://docs.nvidia.com/cuda/wsl-user-guide/index.html) 安装 toolkit：选择 WSL-Ubuntu 包，或配置官方源后安装 `cuda-toolkit-12-8` 等 toolkit-only 包。**不要安装 cuda、cuda-12-x、cuda-drivers 等会拉入 Linux 驱动的元包，也不要执行 Linux .run 驱动安装。**

确认 `/usr/lib/wsl/lib/nvidia-smi`、nvcc 正常；SDK 获取和探针编译与上节相同。先在默认环境运行探针。如果三阶段成功并且 Triangle 成功，不必使用额外兼容库。

若默认 OptiX 不可用，本项目参考 [NVIDIA 论坛实验方案](https://forums.developer.nvidia.com/t/running-optix-on-wsl-2026-version/382414)。这是实验路线，不是所有 WSL/驱动组合的保证。已测试组合：RTX 5080、Windows 驱动 616.64、CUDA 12.8、SDK 9.1.0、Linux 用户态组件 610.57.04；Windows/Linux 版本不同，本项目没有据此宣布正式兼容。

### 隔离准备（不运行驱动安装器）

从 [NVIDIA 官方 Linux 610.57.04 目录](https://download.nvidia.com/XFree86/Linux-x86_64/610.57.04/) 下载对应 x86_64 `.run` 文件到临时工作目录。需要代理时为 curl 添加自己的 `--proxy` 参数。

```bash
mkdir -p /tmp/agv-optix-extract
cd /tmp/agv-optix-extract
curl --fail --location --retry 2 \
  https://download.nvidia.com/XFree86/Linux-x86_64/610.57.04/NVIDIA-Linux-x86_64-610.57.04.run \
  -o NVIDIA-Linux-x86_64-610.57.04.run
# 普通用户，只解包；不加 sudo，不运行安装流程：
sh NVIDIA-Linux-x86_64-610.57.04.run --extract-only
export AGV_OPTIX_RUNTIME="$HOME/opt/optix-runtime-610.57.04"
mkdir -p "$AGV_OPTIX_RUNTIME"
cp NVIDIA-Linux-x86_64-610.57.04/libnvoptix.so.610.57.04 \
  NVIDIA-Linux-x86_64-610.57.04/libnvidia-rtcore.so.610.57.04 \
  NVIDIA-Linux-x86_64-610.57.04/libnvidia-gpucomp.so.610.57.04 \
  NVIDIA-Linux-x86_64-610.57.04/nvoptix.bin "$AGV_OPTIX_RUNTIME/"
ln -s libnvoptix.so.610.57.04 "$AGV_OPTIX_RUNTIME/libnvoptix.so.1"
```

以上目录应是新建隔离目录，已有文件先核对，不能混合来自不同 Linux 驱动包的组件。不覆盖 `/usr/lib/wsl/lib` 或 Windows DriverStore，不将隔离路径写入全局 shell 配置。本仓库 wrapper 的组件检查明确针对 610.57.04，换版本需要重新核对脚本和兼容性。

回到项目根目录，运行：

```bash
bash tools/with_optix_runtime.sh /tmp/agv_optix_init
bash tools/with_optix_runtime.sh \
  "$AGV_OPTIX_SDK/build/bin/optixTriangle" --dim=64x48 --file /tmp/optix_triangle.ppm
```

示例可执行文件位置以实际 SDK 构建目录为准。记录测试前后 nvidia-smi、系统库摘要，必要时用 `LD_DEBUG=libs` 检查 OptiX 来自隔离目录、CUDA 来自 WSL 接口。原始机器日志不要提交。驱动升级后重跑验证；进程隔离并不能消除不兼容组件带来的 GPU 异常风险。

## 3. 项目构建和启动

两种系统都使用：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args \
  -DAGV_ENABLE_CUDA=ON -DAGV_ENABLE_OPTIX=ON \
  -DAGV_OPTIX_SDK="$AGV_OPTIX_SDK"
source install/setup.bash
python3 tools/generate_shared_scene.py --output /tmp/agv_flat_scene
```

WSL 隔离运行库测试可显式加 CMake 参数 `-DAGV_OPTIX_TEST_RUNTIME="$AGV_OPTIX_RUNTIME"`；原生系统不要传入 WSL 路径。SDK 不存在时当前 CMake 不构建 OptiX，不能把构建成功当作后端已经可用。

原生启动见 README 的 `gpu_backend:=native` 命令。WSL：

```bash
bash tools/with_optix_runtime.sh bash -c '
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch agv_bringup sim.launch.py linescan:=true linescan_backend:=optix \
    scene_manifest:=/tmp/agv_flat_scene/manifest.json spawn_x:=2 rviz:=true
'
```

必须在 wrapper 子进程内重新加载 ROS 和工作区，恢复 ROS 动态库路径。启动后按 README 启用采集和发送低速命令；初始化验证、静态射线测试、整车 GUI 采集是三个不同验收层次。常见 7804/7805 错误应先查运行库/入口点，不先修改相机算法。不能启用 OptiX 时可重新构建 `-DAGV_ENABLE_OPTIX=OFF`，但参考后端不提供同等高行频能力。
