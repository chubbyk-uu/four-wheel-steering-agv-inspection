# 采集进程内存持续增长（根因已确认，正式修复待部署）

**状态**：2026-09-14已确认 **Mesa 25.2.8 D3D12后端的命令签名缓存查找参数错误**。对应Ubuntu源码已完成独立构建，项目启动选择/回退检查及源码版最小渲染复现均通过。系统库未修改，源码版GUI/整车采集与完整长任务验收尚未完成。
**影响**：阻断实施计划第6节验收——长任务会耗尽主机内存并导致任务故障。
**首次记录**：2026-09-14。

---

## 已确认的源码根因

错误位于Mesa的`src/gallium/drivers/d3d12/d3d12_cmd_signature.cpp:68`，不是本项目源码，也不是已经证明的NVIDIA闭源驱动错误：

```diff
- _mesa_hash_table_search(ctx->cmd_signature_cache, &key)
+ _mesa_hash_table_search(ctx->cmd_signature_cache, key)
```

`key`本身已经指向签名结构体，哈希/比较按该结构体的16字节内容进行。多一层`&`使查找读取指针变量及其邻接栈字节，
而插入使用正确的结构体内容。相同间接绘制无法正常命中缓存，反复调用`CreateCommandSignature`；
重复键插入替换旧entry的数据指针，但不会释放旧的签名包装对象和COM引用，因此旧对象失去回收路径。
详见[Mesa 25.2.8源码](https://gitlab.freedesktop.org/mesa/mesa/-/blob/mesa-25.2.8/src/gallium/drivers/d3d12/d3d12_cmd_signature.cpp#L68)。

匹配本机ELF Build ID的Ubuntu调试符号将主要栈解码为：

```text
Ogre GL3PlusVaoManager::_update → glFenceSync
  → _mesa_fence_sync → tc_flush → _tc_sync → tc_batch_execute
  → d3d12_draw_vbo → d3d12_get_cmd_signature
  → ID3D12Device::CreateCommandSignature
```

同步调用在这里触发此前排队的绘制，不能把分配归咎于fence本身。实际Gazebo计数中创建2000个GL同步对象时已删除1995个，
存活量只有5个；正确配对的独立fence测试也保持稳定。

验证使用相同系统驱动、同一块GPU：

| 验证 | 原版 | 只修缓存查找参数 |
|---|---|---|
| 独立三角形，间接绘制2001→10001次 | RSS增加23,600 KiB，约2.95 KiB/次 | RSS不增长 |
| 相同三角形改为直接绘制 | RSS不增长 | 无需修改 |
| 100 m道路，雷达＋OptiX均启用，静止40–90 s | 4764.36→4883.88 MiB，拟合约2.380 MiB/s | 4746.51→4746.25 MiB，基本持平（少量回收） |

独立程序不依赖ROS、Gazebo、OptiX或道路资产，使用固定间接命令，逐次`glFinish`排除无限排队；三种三角形测试均无GL错误，
最终像素检查`rendered=true`。末尾首次读回另有一次性分配，上表采用读回前相同绘制次数窗口，不把它当泄漏。
额外使用原版库预加载的控制试验仍增长，不能仅用加载顺序解释修正版持平。
最终Gazebo对照使用独立ROS域和Gazebo分区，无分析器；40 s内原版收到左右雷达399/398帧，修正版398/397帧，
每帧1000×8点。修正版的实际映射已确认只有临时Gallium副本，避免误把未加载修正或未运行雷达当成通过。

**临时修正的性质**：没有安装新驱动或覆盖系统文件。对经过SHA256/指令字节核验的库副本，将查找参数的`lea`改为`mov`，
等价于上述源码单点修改，仅通过诊断子进程的`LD_PRELOAD`加载。它用于验证因果，不是正式部署方式；正式方案应从源码补丁构建独立Mesa或采用已核实包含修复的发行包。
首次只设置`LIBGL_DRIVERS_PATH`未替换实际加载的Gallium，已识别并排除为修正版；该轮仍可作为原库观测。

已备好[英文上游报告](MESA_D3D12_UPSTREAM_REPORT.md)、[独立复现程序](../../tools/probe_mesa_indirect_memory.cpp)、
[源码补丁](../../tools/patches/mesa-d3d12-command-signature-key.patch)和[实测结果](../../results/mesa_d3d12_signature_root_cause.json)。用户已在上游[#14802](https://gitlab.freedesktop.org/mesa/mesa/-/work_items/14802)发布简短英文根因说明（用户截图确认）；完整代码与补丁尚未发布。
根因确认不等于完整验收：仍须部署正式修正版并重跑GUI/RViz、行驶采图和长任务，检查是否另有独立增长。

## 独立源码构建（第1项）

新机器请使用[可移植构建与切换指南](../MESA_SETUP.md)，下文保留最初构建记录。新增脚本已在独立目录完整重建并通过24,000次渲染复现，见[恢复验证](../../results/mesa_portable_build.json)。

使用Ubuntu源码包`25.2.8-0ubuntu0.24.04.2`，保留其发行版补丁，再应用项目的一行修复；不是直接用上游裸源码替代Ubuntu版本，也没有升级整个系统Mesa。
下载来自`https://archive.ubuntu.com/ubuntu/pool/main/m/mesa/`，通过HTTPS取得`.dsc`，逐个核对其中的SHA256与文件大小；未声称完成维护者PGP签名验证。
源码包、依赖包、构建选项和安装库的哈希见[构建记录](../../results/mesa_d3d12_source_build.json)。

所有本机产物在已被Git忽略的`local_data/mesa-source-build/`：

| 子目录 | 用途/本次约占空间 |
|---|---|
| `downloads/` | Ubuntu `.dsc`、原始源码和Debian补丁包，约43 MB |
| `source/` | `dpkg-source -x`展开并应用Ubuntu补丁的源码，约311 MB |
| `deps/`、`sysroot/` | 下载的构建依赖包及私有解包目录；sysroot约19 MB |
| `build/` | Meson/Ninja编译产物，约123 MB |
| `install/` | 本次独立安装前缀，约22 MB |

缺少的工具/头文件通过`apt-get download`下载、`dpkg-deb -x`解包进`sysroot`，**没有安装系统包**。依赖包完整版本见构建记录。
包括Meson 1.7、Mako/MarkupSafe、DirectX-Headers 1.614.1、Flex/Bison以及XCB/Wayland开发文件。
私有`.pc`文件的`prefix=/usr`重定位到`sysroot/usr`；构建进程单独设置`PATH`、`PYTHONPATH`、`PKG_CONFIG_PATH`、头文件路径和`BISON_PKGDATADIR`，未写入用户全局环境。

构建选择D3D12图形后端，保留X11/Wayland、EGL/GLX、GBM、GLVND及GLES2；不构建其它Gallium/Vulkan驱动、视频编解码、LLVM或Microsoft CLC。
这是面向本项目的独立图形库，**不具有Ubuntu完整Mesa包的全部驱动功能**。运行时库名为`libgallium-25.2.8.so`，与Ubuntu库的发行版后缀不同，后续加载步骤必须核验实际映射，不能直接照搬之前二进制副本的路径。

Meson配置如下（`MESA_BUILD_ROOT`表示本次独立目录，环境准备见前段）：

```sh
meson setup "$MESA_BUILD_ROOT/build" "$MESA_BUILD_ROOT/source" \
  --prefix="$MESA_BUILD_ROOT/install" --libdir=lib --buildtype=release --wrap-mode=nofallback \
  -Dgallium-drivers=d3d12 -Dvulkan-drivers= -Dplatforms=x11,wayland \
  -Dglx=dri -Degl=enabled -Dgbm=enabled -Dglvnd=enabled \
  -Dgles1=disabled -Dgles2=enabled -Dllvm=disabled \
  -Dgallium-va=disabled -Dgallium-vdpau=disabled -Dgallium-d3d12-video=disabled \
  -Dmicrosoft-clc=disabled -Dbuild-tests=false
ninja -C "$MESA_BUILD_ROOT/build" -j8
meson install -C "$MESA_BUILD_ROOT/build" --no-rebuild
```

本机环境脚本保留为`configure.sh`和`compile.sh`；完整日志保留为`prepare.log`、`configure.log`、`build-install.log`，均在该独立目录。
编译和安装成功；已核对目标源码仅有这一行变化、安装ELF的链接依赖可解析、安装符号链接完整、系统Gallium哈希与诊断前一致。
第1项结束时**未运行源码构建版的三角形复现、GUI或采集测试**，也未修改启动脚本。前文内存改善数据仍仅属于早期二进制副本实验，不能当成本次源码构建版的测试结果。

## 项目启动选择（第2项）

`python3 tools/with_mesa_runtime.py COMMAND ...`为子进程选择独立Mesa，增加`--system`回到原环境；`--prefix`可指定另一处同结构安装。只修改子进程环境，不改系统库或用户全局配置。直接执行普通`ros2 launch`仍使用原环境。

脚本同时选择Gallium、GBM、GLX/EGL库和对应DRI/EGL路径，避免旧loader加载另一套Gallium。绝对路径预加载可兼容OptiX脚本重设`LD_LIBRARY_PATH`；修复版缺失时明确失败。嵌套调用通过环境快照恢复脚本首次介入前的图形加载环境；不要把嵌套回退理解为保留中途第三方脚本对这些变量的修改。

4项环境回归通过；`/proc/self/maps`实际检查修复版、系统版、两种OptiX组合顺序及嵌套切回，四个相关库均来自预期来源。见[加载检查](../../results/mesa_runtime_selection.json)。这一步只检查动态加载，不声称GUI画面或泄漏验收通过。

## 源码版最小复现（第3项）

2026-09-14使用上述启动脚本，各组实际绘制24,000次；前景/背景像素检查通过，无GL错误，三组32×32 RGBA图像的FNV-1a校验值均为`23b38e4559434925`。

| 组别 | 第6001次RSS (KiB) | 第22001次RSS (KiB) | 拟合增长 (KiB/次) |
|---|---:|---:|---:|
| 系统Mesa，间接绘制 | 195564 | 242772 | 2.950333 |
| 源码修复版，间接绘制 | 119772 | 119772 | 0 |
| 系统Mesa，直接绘制 | 175824 | 175824 | 0 |

末尾首次像素读回仍有一次性分配，不纳入持续增长窗口。源码版仅构建D3D12，初始RSS不能与完整Ubuntu包直接比较收益；此处检验的是预热后的增长趋势。
实际进程映射确认三组均只加载预期的Gallium。系统版GLX最小程序只需GLX/Gallium；修复版由脚本预加载四个库，因此数量不同属预期。GL版本分别报告Ubuntu版`25.2.8-0ubuntu0.24.04.2`和源码版`25.2.8`，渲染器均为D3D12/NVIDIA RTX 5080。

逐次样本和运行库哈希见[源码版对照](../../results/mesa_source_runtime_repro.json)。复现命令（项目根目录）：

```sh
g++ -O2 -Wall -Wextra tools/probe_mesa_indirect_memory.cpp -o /tmp/mesa_probe -lGL -lX11
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
python3 tools/with_mesa_runtime.py --system /tmp/mesa_probe indirect 24000
python3 tools/with_mesa_runtime.py /tmp/mesa_probe indirect 24000
python3 tools/with_mesa_runtime.py --system /tmp/mesa_probe direct 24000
```

这次验证的是源码构建版，区别于前面的二进制副本诊断；**仍未做源码版Gazebo GUI、RViz、雷达或整车采集验收**。本轮按要求完成第2、3项后停止。

---

## 分配栈与同场景复核（2026-09-14）

原记录中的RSS增长是真实观测，但不能据此排除Gazebo渲染，也不能将匿名映射大小当成一次图像分配的大小。
本轮在初始化完成后注入临时目录中的heaptrack，未重编Gazebo，未修改系统驱动或全局ptrace设置。
下面的对照均使用相同车体、同源贴图道路、D3D12、无GUI/无RViz；通过修改**启动生成的临时SDF**，只移除指定系统插件。

| 20 m道路对照 | 稳态RSS变化 | 斜率 |
|---|---|---|
| 保留OptiX线阵与GPU雷达，静止不采图 | 1823.14 → 1848.70 MiB / 25.04 s | 约1.02 MiB/s |
| 只移除线阵系统，保留GPU雷达，同一场景 | 1427.08 → 1509.22 MiB / 80.21 s | 约1.02 MiB/s |
| 保留OptiX线阵，只移除Sensors渲染系统，静止 | 858.57 → 858.60 MiB / 80.15 s | 小于0.001 MiB/s |

100 m道路在**不加载线阵系统、车辆静止**时，同样出现持续增长：未接入调试器的35–45 s窗口约2.38 MiB/s，
与原记录的静止增长量级一致。该轮后续接入GDB会干扰运行速度，不能把整个窗口当成未插桩的性能结果。

动态对照保留OptiX、关闭雷达渲染，用带仿真时间戳的`base_link`速度指令以±1 m/s往返8次，
每次行驶4 s、零速3 s并关闭相机，再开始下一次。保存23块、80,482行原图（约329.65 MB），
预热后的45–155 s观测窗RSS范围904.76–905.05 MiB，没有随这些图像持续累积。
此为短程内存诊断，不是完整任务/GUI验收；最终零速请求已发送，独立HOLD订阅晚于关机窗口未取得消息，所有诊断进程已退出。

heaptrack的初始化后观测窗报告约31.48 MB未释放malloc/new分配，其中约23.07 MB来自`libd3d12.so`，
约7.90 MB来自NVIDIA WSL用户态图形驱动`libnvwgf2umx.so`。主要调用栈为：

```text
gz::sim::systems::SensorsPrivate::RenderThread / RunOnce
  → gz::sensors::GpuLidarSensor::Update
  → gz::sensors::RenderingSensor::Render
  → gz::rendering::Ogre2Scene::FlushGpuCommandsAndStartNewFrame
  → Ogre::CompositorManager2::_updateImplementation
  → Ogre::GL3PlusVaoManager::_update
  → libgallium (Mesa D3D12)
  → libd3d12core.so / libd3d12.so / libnvwgf2umx.so
```

**该轮结论边界（后续已由上节进一步定位）**：找到持续增长的渲染调用链，并通过有无线阵、是否启用雷达渲染的同场景对照排除“线阵插件是必要条件”。
这不是已经证明某个闭源驱动函数缺少一次`free`：heaptrack的未释放量不等于全部RSS，驱动直接分配/映射及分配器保留页也须区分。
尚不能把历史行驶任务的全部增长速率都归给同一处，更不能据此宣称长任务已经修好。
后续应对这条图形资源创建/回收链做最小复现与后端对照，再决定修复或隔离方案；不能以禁用避障雷达作为正式验收通过。

数值、实验窗口和验证边界见[本轮根因证据](../../results/capture_memory_root_cause.json)。下面保留最初排查记录，已被本节纠正的推断不作为当前结论。

---

## 一句话

带线扫相机插件的 Gazebo 仿真进程，内存按**时间**持续增长（约 2–10 MiB/s），任务跑到约
15 分钟时越过 20 GiB 预算，31 分钟时达到 30 GiB 并耗尽主机内存，任务随之故障。
当时观测到增长主要落在**大块匿名映射**上，`[heap]`变化较小；这不等于malloc总量不涨。

---

## 环境

| 项 | 值 |
|---|---|
| 主机 | i7-13700F（8 P核 + 8 E核，24 线程），31 GiB RAM，RTX 5080 16 GB |
| 系统 | WSL2，内核 6.18.33.2-microsoft-standard-WSL2，WSL 2.7.13.0，Windows 10.0.26200.9445 |
| 驱动 | NVIDIA 616.92 |
| 仿真 | ROS 2 Jazzy + Gazebo Harmonic（`gz sim --force-version 8`），headless 或 GUI |
| 渲染 | OptiX 运行时 610.57.04 / SDK 9.1.0（WSL 上 OptiX 非官方支持） |

## 相关系统结构

自研 Gazebo 系统插件 `agv_linescan`（C++，`src/agv_linescan/src/gz_linescan.cpp`），
加载进 `gz sim` 服务端进程，负责：

- 按**空间**触发的线扫相机：车辆每前进 `line_spacing_m = 0.3662 mm` 触发一行，
  每行宽 4096 像素，每行用 3 次曝光（`exposure_s = 20 µs`）在 OptiX 里采样
- 每 `block_rows = 4096` 行组成一块，**一块像素 = 4096×4096 mono8 = 16.0 MiB**
- 每块写一个 `.pgm` + 一个 `.json` 到磁盘，并**从 gz 进程内发布一条
  `sensor_msgs/Image`（16 MiB）到 ROS**
- 地面材质按瓦片在 GPU 上按需生成，缓存 32 槽（约 385 MiB 显存），窗口式预取

---

## 症状

同一配置（100×10 m 十道全区采集）下，进程树 RSS 峰值随**任务时长**增长：

| 扫描速度 | 任务时长 | 进程树 RSS 峰值 | 预算 20 GiB |
|---|---|---|---|
| 2.778 m/s | 374 s | 14.7 GiB | 内 |
| 2.778 m/s | 835 s | 19.7 GiB | 勉强内 |
| 1.0 m/s | 1398 s | **25.4 GiB** | **超** |
| 0.5 m/s | 1862 s | **30.07 GiB** | **超**，主机仅 31 GiB |

0.5 m/s 那次触发主机内存压力，任务在第 22/29 步以 `CAMERA_STATE_TIMEOUT` 故障
（相机服务调用超过 0.5 s 未返回）。

**关键一点：增长不跟着像素产出走。** 四档任务归档的总行数相同（面积相同、行间距是空间量），
额定速度每秒产生的行数是 0.5 m/s 的 6 倍，但它涨得**更慢**。

---

## 已做的测量

### 一、按进程归因

3×2 m 小任务，每 2 秒采一次各进程 RSS：

| 进程 | 稳态斜率 |
|---|---|
| **`gz sim`** | **+9.7 MiB/s**（3290 MiB@24s → 3873 MiB@84s） |
| RViz | 持平 324 MiB |
| mission_operator | 涨到 98 MiB 后持平 |
| ekf / 测量适配器 / ros_gz 桥 | 全程合计几十 MiB |

同一任务、**不加载线扫插件**时，`gz sim` 为 +0.5 MiB/s。

### 二、静止分离（车全程不动 → 编码器不前进 → 一行都不产生）

同一次启动分两段，各 90 秒：

| 阶段 | 斜率 |
|---|---|
| 插件已加载、未开采集 | **+2.23 MiB/s** |
| 开启采集（仍静止） | **+2.39 MiB/s** |

> 说明：该段前 36 秒是场景载入（665 → 4728 MiB），不计入斜率。

**当时推断，现已撤回排他归因**：加载插件时增长、开启但不产行时无额外增长，并不能证明增长由插件造成。
跨场景/运行条件的9.7与2.3 MiB/s也不能直接相减，作为“行驶产行部分”的独立测量。

### 三、按映射种类分离

每 5 秒读 `/proc/<gz sim>/maps` 与 `smaps_rollup`，按映射种类分组，穿越一次 3×2 m 采集任务：

| 分组 | 变化 |
|---|---|
| `[heap]`（brk） | 220 → 245 MiB，**持平** |
| 匿名小映射（<8 MiB） | 97 → 108 MiB，**持平** |
| 文件映射 | 持平 |
| **匿名大映射（≥8 MiB）** | 85 秒内**新增 20 个**，约每 4.3 秒一个 |
| RSS | +4.73 MiB/s，即**每个新映射约 19.8 MiB** |

**观测边界**：匿名映射驻留页增长显著。`maps`中的区间可以合并，不能据此反推出每次分配大小；`[heap]`也不包含全部malloc arena。

---

## 已排除

| 怀疑对象 | 排除依据 |
|---|---|
| C++ 容器里的小对象累积 | 原排除理由撤回：小对象也可能位于mmap支持的malloc arena；需查分配栈与存活量 |
| 传感器位姿历史 `history_` | 每次 push 后裁到 8 条 |
| C++ 归档写队列 `writes_` | 硬限 2 个 future 在飞（各持一块像素），上界 32 MiB |
| 地面瓦片缓存 | 固定 32 槽，且在**显存**上；实测显存峰值三次持平 9.72–9.78 GiB |
| RViz / ros_gz 桥 / 定位节点 | 按进程测过，都不构成量级 |
| gz 本身 | 原排除撤回：本轮同一贴图场景、不加载线阵插件仍增长；雷达渲染栈已实测定位 |
| "每归档一块漏一块" | 观测窗内新增 20 个映射，但任务只归档 6 块，**对不上** |

---

## 最初的未解决问题（保留排查历史）

**采集路径里，什么东西每约 4 秒分配一次约 16–20 MiB 的匿名内存且不被复用？**

两个方向的线索互相矛盾，这正是难点：

- **按尺寸**：19.8 MiB 最接近一块图像的 16.0 MiB（4096×4096 mono8），指向像素缓冲或
  从 gz 进程内发布的 16 MiB `sensor_msgs/Image`（DDS 侧的样本池是否只增不减？）
- **按节奏**：每 4.3 秒一个，与归档块速率对不上（该任务 85 秒只归档 6 块），
  而且**静止不产行时也在涨** 2.3 MiB/s，说明至少有一部分与图像完全无关

## 最初的工具限制（已解除）

最初未安装`heaptrack`、`valgrind`。本轮已在临时目录提取heaptrack/GDB，通过初始化后注入抓取分配栈；
因此“不重编gz就只能看/proc”不是实际限制。

## 最初的下一步候选（优先级已由新证据取代）

1. 在插件自己的大块分配点（像素缓冲、曝光批、ROS 消息）打计数点并随块元数据上报。
   便宜；若打点总量对得上而进程仍在涨，就**直接证明分配不在我们的代码里**。
2. 装 heaptrack、重编带符号的 gz 与插件，按分配栈定位。彻底但要拉依赖、重编，
   且泄漏可能在库内（OptiX/CUDA/DDS）而非自研代码。
3. 单独验证 DDS 侧：把 16 MiB 图像发布关掉再跑同样任务，看 7.4 MiB/s 那部分是否消失。
   这一刀最便宜，且能直接证伪"按尺寸"那条线索。

## 复现方式

```bash
# 带采集（约 3 分钟，会看到 gz sim 持续增长）
bash tools/with_optix_runtime.sh bash -c '
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/validate_operator_session.py --output local_data/leak --no-cancel-probe'

# 观察：找出 RSS 最大的那个匹配 "gz sim" 的进程（第一个匹配的是 ruby 包装，只有 2 MiB），
# 读它的 /proc/<pid>/maps 与 smaps_rollup，按映射种类分组即可复现上表。
```

数据：[采集内存增长](../../results/capture_memory_growth.json)、
[收尾尝试](../../results/speed_tier_closure_attempt.json)、
[预算对账](../../results/resource_budget_reconciliation.json)。
