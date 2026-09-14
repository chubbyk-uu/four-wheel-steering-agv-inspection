# WSL 私有 Mesa 的恢复与切换

本配方针对 **Ubuntu 24.04 x86_64 / WSL2 + WSLg、D3D12**。修复源码版本固定为 Ubuntu `25.2.8-0ubuntu0.24.04.2`，保留发行版补丁。原生 Linux 使用其正常图形驱动，不默认启用本配方；CUDA/OptiX、ROS 和道路资产仍需按各自安装说明恢复。

## 新机器构建

先安装基础开发环境（这些是开发依赖，不是替换系统 Mesa）：

```sh
sudo apt-get update
sudo apt-get install build-essential ninja-build pkg-config dpkg-dev patch python3 \
  libglvnd-dev zlib1g-dev libzstd-dev libexpat1-dev libdrm-dev libudev-dev \
  libelf-dev libunwind-dev libwayland-dev libx11-dev libxext-dev libx11-xcb-dev \
  libxxf86vm-dev libxrandr-dev x11proto-dev spirv-tools \
  libxcb-glx0 libxcb-shm0 libxcb-shape0 libxcb-dri2-0 libxcb-dri3-0 \
  libxcb-randr0 libxcb-present0 libxcb-sync1 libxcb-xfixes0 libxcb-render0 libxshmfence1
```

仓库根目录执行：

```sh
python3 tools/build_private_mesa.py
```

脚本读取版本锁定文件 `tools/patches/mesa-build-lock.json`，下载 Ubuntu 源码与指定版本的私有依赖，校验 SHA256，解包发行版补丁，再应用项目补丁，编译安装到 `local_data/mesa-source-build/install`。不调用 sudo、不安装系统包、不修改系统库或全局环境。完整输出在构建目录 `build.log`，成功时生成 `build-result.json`。源码和私有依赖均固定并校验；基础系统开发包仍由 Ubuntu 仓库管理，因此不承诺跨机器二进制逐字节相同。

下载遵循标准 `http_proxy` / `https_proxy` 环境变量。代理由使用者在当前终端设置，不写入仓库。APT 需要可用的软件包索引；固定版本若被镜像移除，脚本明确失败，不自动换版。可以通过 `--cache` 提供以前的 `downloads/` 和 `deps/`，仍逐文件校验。源码签名文件被保留并校验哈希，但脚本不声称验证维护者 PGP 签名。

已有构建目录不会覆盖或删除。要重建，指定新目录：

```sh
python3 tools/build_private_mesa.py --root local_data/mesa-rebuild --jobs 8
# 可选复用已下载的包：追加 --cache local_data/mesa-source-build
```

## 启动与回退

```sh
python3 tools/with_mesa_runtime.py ros2 launch agv_bringup inspection.launch.py
python3 tools/with_mesa_runtime.py --system ros2 launch agv_bringup inspection.launch.py
# 自定义构建目录需要指定对应安装前缀：
python3 tools/with_mesa_runtime.py --prefix local_data/mesa-rebuild/install COMMAND
```

普通 `ros2 launch` 不会自动选择私有版。`--system` 恢复包装前的环境；推荐在新终端命令中选择版本。嵌套回退恢复的是最初保存的变量值，不保留中间其他包装器对同一变量的修改。

**没有软件渲染回退。** 构建仅启用 `d3d12`、禁用 LLVM，不包含 llvmpipe/swrast；`LIBGL_DRIVERS_PATH` 指向私有目录。D3D12 无法初始化时会失败，不会自动退到软件渲染。适配器选择偏好本身不等于严格验证 NVIDIA，运行后仍应核对渲染器实际名称；其他硬件能否被 D3D12 选中由后端决定。

**预加载会传给所有子进程，包括不渲染的 Python 节点。** 私有 Gallium 的 SONAME 为 `libgallium-25.2.8.so`，系统版含 Ubuntu 发行版后缀，仅设置 `LD_LIBRARY_PATH` 不能替换不同名字的依赖。启动器同时绝对路径预加载配套 Gallium、GLX、EGL、GBM，避免系统前端重新带入旧 Gallium；也能应对 OptiX 包装器重设库搜索路径。不能未经进程映射验证就简化为仅设置搜索路径。此作用域仅限包装命令及其后代，不影响其他终端或整个系统。

## 验证与分发

运行 `python3 tools/test_mesa_runtime.py`，再按[最小复现步骤](issues/CAPTURE_MEMORY_GROWTH.md#源码版最小复现第3项)对比系统版和修复版的 RSS、实际加载库及全图校验值。构建成功不代表 GUI、雷达或整车长任务验收完成。

Git 保存：构建脚本、版本/哈希锁、源码补丁、启动器、测试、恢复说明和精简验证结果。不要上传 `local_data/` 中的源码包、依赖包、构建产物、共享库或机器日志；代理配置和凭据也不上传。重建需要网络或经过校验的缓存，Git 仓库本身不是完整离线安装包。

2026-09-14验证：现有WSL主机在独立空目录完整重建成功，使用校验缓存并另测HTTPS源码清单、APT固定包下载；24,000次间接绘制稳态RSS持平、全图校验一致，系统库未变。[精简结果](../results/mesa_portable_build.json)。这不是全新操作系统安装测试。
