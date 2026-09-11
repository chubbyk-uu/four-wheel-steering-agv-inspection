# 控制回路被全系统I/O停顿击穿，并把自己的停摆报成定位失效

2026-09-11。一次21分钟的100×10 m任务在第2道x=78.9 m处以`STALE_OR_UNREADY_LOCALIZATION`中止。
同一条记录里定位健康是`READY`，轮速/IMU/GNSS滞后分别只有0.010/0.010/0.130 s，局部与全局滤波
滞后均0.008 s。被判超时的不是定位，是执行器自己测到的"消息到达墙钟年龄"：mode 0.388、odom
0.380、health 0.432 s，全部超过`wall_timeout_s`0.35。数据见[结果](../../results/control_loop_stall.json)。

## 定位过程与三次判断修正

回调计时显示`Executor.tick`单拍墙钟0.379 s、线程CPU仅0.67 ms，即阻塞而非计算。但当时诊断字段
只在`state=='FAULT'`那一拍附加，拿不到阻塞发生在拍内哪一步，用了整整一次任务才还原。因此现在
**任何超过`control_stall_threshold_s`的控制拍都归档**，不再依赖它是否故障。

先后排除：物理步进9.6 ms、OptiX采样批次22.1 ms、采样队列21.1 ms、块写盘6.1 ms；Fast DDS共享
内存段不足（本项目已按128 MiB配置，且0.38 s不符合其3 s心跳量级）；时钟源（控制周期已用steady
clock，与仿真时间分离）。

一条曾被当作根因的推断是错的：`navigation.jsonl`里`/odometry/global`无间隔，被我读成"定位进程
未停顿"。该文件按**仿真时间戳**去重写入，墙钟冻结0.38 s不会在其中留下痕迹。真正排除整机冻结的
是另一条证据：适配器上报的`measurement_age`是仿真时间，轮速滞后能长到0.392 s说明仿真时钟仍在
推进，Gazebo没有停。

## 根因：三层

1. **触发层（环境）**：采集以约6.5 MB/s（16 MiB/块）向WSL2虚拟磁盘写脏页，回写突发使整个客户机
   出现约0.35 s的I/O停顿。`/proc/pressure/io`实测单窗口`io_full`104 ms、`io_some`149 ms，20 Hz的
   采样器自身也被拖到0.345 s。这一层在WSL环境内无法消除。
2. **放大层（缺陷）**：三个实时节点都在控制/健康回路里做同步写盘和同步发布——执行器50 Hz的
   `write+flush`与`/mission/status`、`tracking_node`同样、定位适配器20 Hz的`archive.flush()`与
   `/localization/status`。停顿一来，那一拍里**任何会睡眠的系统调用**都被挡住。
   最硬的证据是把遥测发布移出控制线程后，同样的停顿**换了落点**：`status_publish 0.328 s`变成
   `log_flush 0.349 s`。所以根因不是某个调用。
3. **误判层（缺陷）**：被阻塞的线程随后用自己的墙钟去测"消息到达年龄"，必然全部超限，于是把
   自身停摆归咎于定位。

`qos_profile_sensor_data`只去掉重传，不会让`publish()`变成非阻塞——传输层的缓冲区分配仍会等待
消费者。原注释"A slow display must not back-pressure the control loop"预判了危险，但选的手段无效。

## 修复

- 新增`offload.py`（`agv_mission`与`agv_localization`各一份，由防漂移测试守住，两包之间没有依赖
  关系，让底层依赖高层是反的）：
  - `TelemetryPublisher`：有界队列、丢最旧、计数；遥测可丢。
  - `ArchiveWriter`：不丢，归档是证据；**拥有文件句柄的生命周期**（先排空再flush再close，杜绝
    "句柄先关、写线程还在写"的次序隐患），写失败计数并上报。
- 执行器、`tracking_node`、定位适配器的遥测发布与全部归档写入（含采集事件、块元数据、采集区间、
  暂停事件）移出控制/健康线程。控制拍现在不做任何文件系统调用。
- 新故障`ARCHIVE_WRITE_FAILED`：归档线程写失败不得静默。
- 新故障`CONTROL_LOOP_STALLED`：用执行器自己的steady clock测拍间隙判定，**排在定位新鲜度检查
  之前**——被阻塞的线程无权判断它没能读到的消息是否新鲜。`max_control_step_s`取代两处硬编码0.1。
- 常驻诊断`PhaseTrace`＋`StallWatch`：分段计时、`getrusage(RUSAGE_THREAD)`的主动等待/被抢占/缺页
  计数（区分"被调度器抢走CPU"与"阻塞在系统调用"）、内核等待点`wchan`采样；峰值每100拍归档。

## 验证

完整10道100×10 m任务，66,845拍、1378 s：**0次控制停顿**，控制拍最差分段3.5 ms
（`status_publish`峰值0.000194 s，修复前0.328 s）。同期`/proc/pressure/io`记录到18次系统I/O停顿，
含连续6个窗口、每52 ms阻塞25–35 ms，控制回路未受影响。

## 未解决：WSLg图形上下文与gz-transport发现线程的偶发启动失败

同期5次整区跑里有3次在启动阶段失败，两种表现：

- `[QT] Failed to create OpenGL context for format QSurfaceFormat(version 2.0, ...)`，GUI退出后
  `on_exit_shutdown`连带杀掉服务器；
- `gz::transport::v13::Discovery<ServicePublisher>::RecvMessages() → env() → getenv`段错误。

环境为Mesa 25.2.8、Gazebo Sim 8.11.0、`GALLIUM_DRIVER=d3d12`、NVIDIA适配器。**没有找到与本现象匹配
的上游报告**，检索到的相近条目逐条核对后都不是同一回事，记录于此以免重复检索：

| 条目 | 状态 | 为何不匹配 |
|---|---|---|
| [gz-sim#2614](https://github.com/gazebosim/gz-sim/issues/2614) | 开启 | 仅错误串相同；环境为Docker、未提WSL、可稳定复现、无任何诊断 |
| [gz-rendering#852](https://github.com/gazebosim/gz-rendering/issues/852) | **已关闭** | 是d3d12下的**着色器编译**失败（`TerraShadowGenerator failed to compile`），不是上下文创建 |
| [wslg#321](https://github.com/microsoft/wslg/issues/321) | 开启 | 双GPU＋雷电坞环境下**所有**OpenGL程序全部失败；我们是偶发，多数情况正常 |
| [wslg#1045](https://github.com/microsoft/wslg/issues/1045) | 开启 | AMD显卡，每次必现，崩在d3d12的`create_gfx_pipeline_state` |
| [gz-sim#2952](https://github.com/gazebosim/gz-sim/issues/2952) | 开启 | 栈同为`Discovery::RecvMessages`，但崩在protobuf解析、平台为macOS M2 |

能站得住的只有两点。其一，失败发生在我们的节点开始工作之前，不在本项目代码内。其二，`getenv`那条
有明确机理：C库的`getenv`对并发`setenv`不安全——`setenv`扩容时会重新分配`environ`数组，另一线程
`getenv`可能读到已释放的指针。gz-transport在发现线程里调用`env()`，而插件加载在主线程改环境变量，
两者构成竞态。这条解释不依赖任何issue，但也**尚未在本项目上验证**。OpenGL上下文那条目前无法解释。

`sim.launch.py`已把RViz推迟到控制器spawn之后，注释写明不要让两个WSLg图形客户端竞争启动时的上下文
创建；但Gazebo GUI自身仍与传感器服务器同时创建上下文，未被错开。5次中2次成功的前面都刚好有一段
显式等待，样本太小，不作为结论。可选缓解尚未采用，需要先定：错开GUI启动、只让GUI走软件GL（与
"不得静默回退软件渲染"的约定冲突，需显式选择）、或让GUI失败降级为无头而不是终止任务。
