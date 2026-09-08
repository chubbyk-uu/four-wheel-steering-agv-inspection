> 已归档：保留原始设计、问题和实验数据；文中的“当前/下一步”属于当时阶段，现行入口见 [当前状态](../../CURRENT_STATUS.md)。历史命令仍从仓库根目录运行。

# OptiX WSL 最小可行性验证

> 历史实验记录：本文参数、性能和“当前/默认”描述属于当时版本，不作为新版验收。现行550 kg、20 mm/4K/1.5 m、1 m轨迹间距、11 kHz目标见[当前模型基线](../../LARGE_AGV_REBUILD.md)。历史命令须按现行配置调整，旧16 mm标定不可用于新版；大体积实验资产可能已清理，复现需重新生成。

2026-09-07：**环境可行性通过，性能与完整场景尚未验证。**

本机 RTX 5080、Windows 驱动 616.64、WSL 内核 6.18.33.2、CUDA 12.8、OptiX SDK 9.1.0。SDK 位于 `$HOME/opt/optix-sdk-9.1.0`。

默认 WSL `libnvoptix.so.1` 为加载辅助库，没有 `optixQueryFunctionTable`。默认最小程序返回 `OPTIX_ERROR_ENTRY_SYMBOL_NOT_FOUND (7805)`；这不能据此判断 WSL 无法运行 OptiX。

参考 [NVIDIA 工程师的实验性方案](https://forums.developer.nvidia.com/t/running-optix-on-wsl-2026-version/382414)，通过已配置代理下载 [Linux 610.57.04 驱动包](https://download.nvidia.com/XFree86/Linux-x86_64/610.57.04/)，仅执行 `--extract-only`，未执行驱动安装。公开目录未找到与本机精确匹配的 615/616 包，610.57.04 不是已获官方保证的兼容组合。

从同一包提取 `libnvoptix.so.610.57.04`、`libnvidia-rtcore.so.610.57.04`、`libnvidia-gpucomp.so.610.57.04` 和 `nvoptix.bin`，放在 `$HOME/opt/optix-runtime-610.57.04`，其中 `libnvoptix.so.1` 为目录内软链接。使用进程级加载路径，无需覆盖 `/usr/lib/wsl/lib`、Windows DriverStore 或修改全局 shell 配置。隔离文件不能消除混用驱动组件导致的 GPU/显示驱动异常风险；本轮仅做小规模测试。

验证结果：

- CUDA 初始化、OptiX 初始化、OptiX 设备上下文创建均返回成功。
- 官方 `optixTriangle` 编译成功；64×48 图像的三角形求交、着色、`optixLaunch`、读回及 PPM 输出成功，PNG 预览见 `optix_wsl_triangle.png`。
- `LD_DEBUG=libs` 确认 OptiX/RTCore/GPU compiler 来自上述独立目录，CUDA 驱动来自原 WSL 路径。
- 测试前后系统 `libnvoptix.so.1`、`libcuda.so.1`、`libnvidia-gpucomp.so` 的 SHA256 相同；测试后 `nvidia-smi` 正常。
- 不涉及降噪器（虽然提取了权重），没有验证复杂遮挡、条灯阴影、持续行频或长期稳定性。

复现（不启动 GUI）：

```bash
bash tools/with_optix_runtime.sh \
  $HOME/opt/optix-sdk-9.1.0/build/bin/optixTriangle \
  --dim=64x48 --file /tmp/optix_triangle.ppm
```

退出该命令即结束专用环境；正常 ROS/GZ 启动不会使用此加载路径。Windows 驱动更新后需要重新验证，不能假设该组合永久兼容。

公开仓库只保留探针源码，原始机器日志不上传；探针位于 `results/optix_wsl_probe/`；构建日志为 `/tmp/agv_optix_triangle_build.log`，下载、解包日志分别为 `/tmp/agv_optix_runtime_download.log`、`/tmp/agv_optix_extract.log`。下一步是在共享场景资产上测试 4K、三次曝光采样、非平面地形与有限长度 LED 遮挡的正确性和持续性能。
