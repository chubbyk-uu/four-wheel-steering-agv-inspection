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

## 部分缓解：WSLg图形上下文与gz-transport发现线程的偶发启动失败

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

`sim.launch.py`原先把服务器和GUI合成一个`gz sim -r --gui-config <world>`进程：GUI一退出，gz sim自己
就杀服务器（日志里的`Escalating to SIGKILL on [Gazebo Sim Server]`），`on_exit_shutdown`再连带停掉整个
launch。现改为两次独立调用——`gz sim -s -r <world>`，以及在控制器spawn结束后才启动的
`gz sim -g --gui-config <cfg>`，即RViz已经在用的同一个错开点，注释写明不要让两个WSLg图形客户端竞争
启动时的上下文创建。**两者都保留`on_exit_shutdown: true`**：GUI失败仍然终止任务，因为验收条件是
GUI＋RViz＋OptiX联合跑，静默降级为无头会产出看起来合格、实际不满足条件的结果。

启动专项对照（启动→等待真实HOLD→再观察40 s→拆除，20 m场景，d3d12/NVIDIA）见
[结果](../../results/startup_reliability.json)：

| | 试次 | 失败 | 启动耗时 |
|---|---|---|---|
| 改前 | 12 | 2（各一次`QT_OPENGL`与gz-transport段错误） | 51.5–55.5 s |
| 改后 | 12 | 0 | 51.4–52.1 s |

**这个差异在统计上不显著**（Fisher精确检验双侧`p=0.478`）：12次/组分辨不了17%与0%。而且有混杂
——改前的两次失败都落在前4次，紧接在一次长时间GPU重载任务之后；改后一组是在机器安静下来之后
才开始的。能确定的只有拆分本身无害且不增加启动时间。保留该改动的理由是它代价为零、并与RViz已经
需要的错开一致，**不是**因为改善已被证明。要做出显著结论，每组约需20次以上。

**2026-09-14复发一次**：第6节验收连跑中，一次GUI启动在`D3D12: Removing Device`后`QT_OPENGL`失败，任务未开始；静置60 s重试通过（n=1）。该次启动距上一个13分钟全区GPU重载任务结束仅20秒，**正是上面记下的那个混杂条件**。**冷却时间A/B已由用户否决**（2026-09-14）：每组15–20次启动、约1.5小时一组，为了验一个已知不显著的量去人为制造故障，代价与收益不成比例。

### 2026-09-15：查清了结局，没查清触发

09-15上午又连着两次（`rough_closed_loop_v1/10mm`、`zero`，相隔6分钟，手工重试均通过）。连同09-14那次共三次，**形状完全一致**：

| | 日期 | `Removing Device` | GUI pid | RViz pid |
|---|---|---|---|---|
| `accept_rated_6_gui_abort` | 09-14 | 有 | 14455 | 14456 |
| `rough_closed_loop_v1/10mm` | 09-15 | **无** | 32529 | 32530 |
| `rough_closed_loop_v1/zero` | 09-15 | 有 | 34794 | 34795 |

三次都是**GUI与RViz同一瞬间启动、pid连号、输的总是GUI**；三次里有一次根本没有设备移除那行，**所以触发机制不止一种，但结局是同一条**：

```
[可选] D3D12: Removing Device.     ← /usr/lib/wsl/lib/libd3d12core.so，微软D3D12运行时
glx: failed to create drisw screen ← Mesa src/glx/drisw_glx.c:688，回退软件驱动
[QT] Failed to create OpenGL context ...
gz sim -g 退出 134（SIGABRT），on_exit_shutdown 连带终止整个任务
```

**确证：私有Mesa的回退路径是确定失败的。** `LIBGL_ALWAYS_SOFTWARE=1`对照（探针为[probe_gl_context.cpp](../../tools/probe_gl_context.cpp)，用的正是GUI那个表面格式：窗口、双缓冲、深度24、模板8）：

| | 探针 | 真实`gz sim -g` |
|---|---|---|
| 系统Mesa | `llvmpipe`，退出0 | 跑满40 s无异常 |
| 私有Mesa | **与三次故障日志逐字相同**的`glx: failed to create drisw screen`，退出4 | — |

原因是结构性的：所有`*_dri.so`都是`libdril_dri.so`的符号链接，真正的驱动代码在`libgallium-*.so`里；私有构建是`-Dgallium-drivers=d3d12 -Dllvm=disabled`，里面根本没有软件驱动。**把系统`dri`目录加进`LIBGL_DRIVERS_PATH`也救不回来**——绝对`LD_PRELOAD`钉死了私有`libgallium`，已实测确认。

**更正上一版的措辞。** 原文写"当时环境为私有Mesa……但**这不是成因**"，把触发和放大混为一谈了。准确的说法是：**触发在两种Mesa下都会发生，私有Mesa拿掉的是恢复路径**——把"可降级存活"变成"必死"。这正是"换了本地Mesa之后更频繁"的机制。至于09-11系统Mesa那次`drisw`是否真的尝试过、是否救回来了，**无从判断**：那份日志已不存在。

**排除的两条**（均在空闲机器上测，而真实故障恰恰不在空闲时发生，所以这只缩小范围、不定位机制）：并发创建上下文1/2/4路共175次，0失败；占住4.0/8.2/11.5 GiB显存（空闲降到3.4 GiB）后各16次，0失败。

**`Removing Device`不是Mesa打的**：这个串在Mesa 25.2.8整棵源码树里不存在，它在`libd3d12core.so`里，旁边还有`Removing device due to driver error`和一批out-of-memory消息。触发在Mesa以下、本项目之外。

**那个一行Mesa补丁不是嫌疑**：`d3d12_cmd_signature.cpp`按内容哈希与比较（`_mesa_hash_data`/`memcmp`整个结构体），插入的是条目自己拥有的`&data->key`副本，用`key`代替`&key`查找不引入任何生命周期或别名问题；原代码才是越界读栈。

**注释与代码原本不符**：RViz上方那句"不要让两个WSLg图形客户端竞争启动时的上下文创建"从未被实现——RViz和GUI都挂在同一个`OnProcessExit(spawner)`上，所以一起启动。三次故障的连号pid就是这件事。

**已实施并收严的修复**（[结果](../../results/gui_startup_abort.json)）：

1. **真正错开**：`gui_start_delay`（默认6.0 s）让GUI在RViz那个事件之后再等一段。已验证RViz（pid 70881）报出`OpenGl 4.6`之后GUI（pid 70919）才启动，相隔38个pid而非连号。
2. **重试仅限D3D12启动阶段**：GUI改为自己的`ExecuteProcess`（不再是第三个`gz_sim.launch.py`include，那个include在本工作区算出的模型/插件路径都是空串，唯一让出的只是它无条件的`Shutdown`）。只有D3D12、GUI尚未确认就绪、退出码134且仍有预算时才重试；GUI一旦就绪，之后任何退出都立即`Shutdown`。原实现只判断134，会把任务运行中的GUI abort也伪装成可恢复启动故障，已纠正。原生Linux不启用该重试，6 s错开也只用于D3D12＋RViz组合。
3. **GUI就绪是运动许可条件**：`follow_camera.py`随首个GUI启动并跨重试等待，要求`/gui/currently_tracked`连续稳定3 s，再原子写入就绪证据；默认模式同时确认`FOLLOW_LOOK_AT`及两个AGV目标，`follow_camera:=false`只确认GUI插件存在，不改变自由视角。`swerve_controller`在证据出现前不启动，因此启动失败、错开窗口或重试等待期间车辆都不可能先行运动。探针超时或异常同样终止整个launch。
4. **就绪超时与重试共用预算**：探针原固定90 s，而校验曾允许5次、每次间隔120 s的组合，导致合法配置会由探针先错误地报就绪失败。现按“全部重试等待＋每次GUI启动15 s＋15 s传输余量”推导探针超时，最低90 s、最高300 s；超出300 s的组合在启动时拒绝。`gui_start_delay`不计入，因为探针和首个GUI都在该延迟结束后才启动。

**为什么不把软件驱动加回去**：它确实能救——系统Mesa那一栏就是证明——但救回来的是一个跑在`llvmpipe`上的GUI，正是本项目明确拒绝的静默降级（验收要求GUI＋RViz＋OptiX真机渲染），而且软件渲染还会去抢物理步需要的CPU。有了重试之后，一次abort的代价是十秒而不是一整跑，回退能买到的东西已经没有价值。

实际回归确认两种正常路径：D3D12＋RViz默认跟车时，RViz先报告OpenGL 4.6，随后GUI启动，跟车连续稳定3 s后才出现`swerve_controller`进程；`rviz:=false follow_camera:=false`时，自由视角就绪后才启动控制器。GUI就绪后手工关闭，launch立即停止且没有重试。GUI策略、边界参数、ROS附加参数、两种就绪语义和原子证据另有5个注册测试保护；全工作区433项测试通过。

**不声称**：没有测量修复前后的失败率。要分辨10%上下的两个率，每组约需20次带GUI启动、每次几分钟，那正是用户否决过的那个A/B。这些改动的依据是确定性证据和一致的故障形状，不是显著性。

**仍未实施**：`D3D12: Removing Device`是设备移除事件，WSL的`dxg`驱动会在`dmesg`里记录原因。在运行脚本里顺带保存启动窗口的`dmesg`，下次自然发生即可拿到移除原因（挂起／驱动内部错误／复位），那才是能向上游提交的材料。**制造故障不如等故障**。

见[第6节收尾](../../results/section6_closure.json)。
