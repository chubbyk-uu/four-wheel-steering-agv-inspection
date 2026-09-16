# 100 m 场景载入时间

更新：2026-09-16。相关：[道路资产](../ROAD_ASSETS.md)、[当前状态](../CURRENT_STATUS.md)。

## 实测预算（干净实例，`inspection.launch.py` 默认 100 m 道路，逐进程 RSS 采样）

改动前，从启动到内存平台约 **63.7 s**、峰值 **9.75 GiB**：

| 阶段 | 耗时 | 内容 |
|---|---|---|
| 启动前 | **16 s** | 仿真进程尚未启动。`shared_scene.validate` 8.6 s + `build_road_display` 5.9 s，单线程串行 |
| gz **server** 加载 | 25 s | ruby 进程 0 → **4.70 GiB** |
| rviz2 | 6 s | 276 MiB（低清代理，已是正确做法） |
| gz **GUI** 加载 | 17 s | 第二个 ruby 进程 0 → **4.25 GiB** |

根因是**同一份路面被重复加载**。`world.sdf` 里 48 个 `terrain_N` 模型的 `<visual>`
指向的就是 OptiX 成像用的那批 OBJ（**7.71 M 三角面、0.4 面每 cm²**，另加 96 张
1054×1920 PNG）。gz server（其 `Sensors` 系统要为两个 `gpu_lidar` 建渲染场景）和
gz GUI **各自独立建一遍 Ogre2 场景**，合计 8.95 GiB，占峰值内存 90%。
launch 进程在两者之前又把 2.0 GB OBJ 解析一遍做校验。

## 已做：复用启动前的派生结果（`agv_linescan/derived_cache.py`）

两项都是资产字节的纯函数，而资产在多次运行之间不变：

| | 冷 | 热 | 省 |
|---|---|---|---|
| `shared_scene.validate` | 8.62 s | **2.09 s** | 6.5 s |
| `build_road_display` | 5.61 s | **0.16 s** | 5.5 s |

端到端（同一台机器、干净实例、逐进程采样）：

| | gz server 起 | gz GUI 起 | 到达平台 | 峰值 |
|---|---|---|---|---|
| 改动前 | 18.1 s | 49.3 s | **63.7 s** | 9.75 GiB |
| 改动后（热缓存） | **4.0 s** | **35.4 s** | **51.8 s** | 9.69 GiB |

整条链前移约 14 s，平台提前 **11.9 s**。GUI 不是按墙钟定时，而是挂在 `spawner`
退出后 `gui_start_delay`（默认 6 s，`sim.launch.py:316`），所以前端省下的时间会
如实传导到后面，不会被固定等待吃掉。峰值内存不变——这条改的是时间，不是内存。

> 测量注意：第一次热测量读数为"无改善"，是因为上一轮的 gz 进程尚未退出、两个采样
> 脚本又写同一个文件。每次测量前必须确认无残留 `ruby`/`rviz2` 进程，采样文件按运行
> 区分。全区验收一贯要求的"干净实例"前提，同样适用于计时测量。

`validate` 的 8.6 s 里 `digest` 只占 1.59 s（2.43 GB），`read_obj` 1.27 s，
**`validate_proxy` 占 5.49 s**——它把 7.7 M 三角面读回来、对代理高度场逐顶点和
逐面重采样。缓存的正是这一段推导。

### 缓存键的性质（这是关键，不是实现细节）

- 键**只由已验证的内容构成**：资产条目（内含其网格与各代理文件的 sha256）、
  它所对照的 `ground_material` 与 `display_uv_projection`，以及**检查代码自身的
  源码指纹**。改了检查逻辑，所有条目自动失效，不依赖手工改版本号。
- **绝不使用路径、大小或 mtime。** 会因就地改写而失效的缓存，不值这几秒。
- **命中只跳过推导，不跳过任何完整性检查。** 命中路径照样 digest 全部代理侧文件
  （`proxy_files()` 枚举，1.09 GB、约 0.7 s），就是未命中路径本来会验的那些；
  `build_road_display` 在查缓存**之前**就校验全部源纹理哈希。
- 目录条目的 `.complete` 标记最后写入，崩溃或磁盘满留下的半份拷贝读作未命中。

存储默认在 `$XDG_CACHE_HOME/agv-4wids`（100 m 道路约 68 MB，其中 road_display 代理
68 MB、48 个 scene_geometry 条目各不足 1 KB）。`AGV_DERIVED_CACHE_DIR` 可改位置，
`AGV_DERIVED_CACHE=off` 强制每次重做。测试由两个包各自的 `test/conftest.py` 隔离到
`tmp_path`——否则上一轮留下的条目会让一个已经不再触发的检查看起来仍是绿的。

回归见 `src/agv_linescan/test/test_derived_cache.py`：命中后篡改代理文件与篡改光学
网格都仍被拒；`proxy_files()` 对两种代理方法都覆盖全部带 sha256 的条目；改动检查
代码会改变指纹；半份目录条目与损坏的 JSON 都读作未命中。

## 未做（已评估）

- **给显示单独出一套降采样网格**：真正的大头（~42 s、~7 GiB）。±2 mm 起伏的路
  在显示上用不着 1.6 cm 网格。做法是生成 `display_terrain_N.obj` 并让 `world.sdf`
  的 `<visual>` 指向它们，**`manifest['assets']` 一个字节不动**——线阵相机走 OptiX 的
  `LoadScene`，读的是 `manifest['assets']`，不经过 Gazebo 的 Ogre2 视觉，所以改不动
  任何采集像素，且该性质可由一次短程采集逐像素比对验证。
  副作用需如实记录：两个 `gpu_lidar` 对渲染场景求交，回波会改变（量级 <2 mm，
  远低于雷达噪声，但确实变了）。
- **server 与 GUI 并行**：看似能省 17 s，**不要做**。错开是故意的，见
  [控制回路停顿](CONTROL_LOOP_STALL.md) 中 GUI 的 D3D12 崩溃放大器。
- `headless:=true` 直接省掉 GUI 那段与 4.25 GiB，适用于不需要人盯着的批处理。

## 未优化的次要项（已量，保留记录）

`optix/scene_io.h` 的 `LoadScene` 每行构造一个 `std::istringstream` 解析 OBJ，
`Sha256()` 先把整个文件读进 `std::string` 再哈希。单个 47 MB 文件实测：哈希
0.116 s vs 流式 0.023 s，解析 0.213 s vs `from_chars` 0.128 s。全部 48 个约省 7 s，
但这发生在 server 加载期内、与上面的 42 s 重叠，且该读取器是**刻意独立实现**的
交叉校验，改它要单独权衡。
