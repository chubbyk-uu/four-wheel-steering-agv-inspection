# 文档入口

日常只需先读[当前状态](CURRENT_STATUS.md)，再按任务查下表。安装、构建和基础运行从[项目README](../README.md)进入；已确认的完整目标与约束以[项目规范](../PROJECT_SPEC.md)为准。历史目录不作为默认阅读入口。

| 要做的事 | 现行文档 |
|---|---|
| 查车体、相机、雷达和安装参数 | [550 kg模型基线](LARGE_AGV_REBUILD.md) |
| 规划下一步与确认尚未实现的功能 | [实施计划](MISSION_IMPLEMENTATION_PLAN.md) |
| 使用RViz任务面板、载入区域和采图 | [规划器](../src/agv_mission/README.md) → [任务执行](RECTANGLE_EXECUTION.md) → [采集与标签](RECTANGLE_CAPTURE.md) |
| 平地加减速俯仰与悬挂是否调软 | [现行调参](SUSPENSION_TUNING.md) → [旧刚度对照](ACCEL_PITCH_RESPONSE.md) |
| 匀速轻微振动、真实碰撞成本与水泥实拍变形 | [毫米级试片及图像对照](ROUGH_ROAD_PROBE.md)、[闭环与横向死区](ROUGH_CLOSED_LOOP_PROBE.md) |
| 相机支架刚度、被动弯曲与短道图像对照 | [支架柔性实验](CAMERA_MOUNT_FLEX.md) |
| 编码器、实际轮径与原图尺度误差 | [编码器模型](ENCODER_MODEL.md) |
| 定位、噪声、延时与外参 | [定位包](../src/agv_localization/README.md)、[误差基线](LOCALIZATION_NOISE_BASELINE.md) |
| GUI跟车和缩放 | [GUI操作](GUI_CAMERA.md) |
| 100×10 m场景预算及完整采集验收 | [全区域验收](FULL_ROAD_ACCEPTANCE.md) |
| 恢复道路与处理旧贴图 | [道路资产](ROAD_ASSETS.md) |
| 离线校正、灰度和后续TIFF | [离线处理](OFFLINE_PROCESSING.md) |
| 原生Linux / WSL的OptiX安装区别 | [OptiX安装](OPTIX_SETUP.md) |
| 核对回归覆盖及人工检查范围 | [验收约定](VISUAL_ACCEPTANCE.md) |
| 三维地表、双天线和未来条带拼接 | [后续设计](SURFACE_HEADING_STRIP_DESIGN.md)，其中研究方案不等于已实现 |
| WSL私有Mesa源码构建与回退 | [恢复指南](MESA_SETUP.md) |
| GitHub恢复和分发边界 | [仓库说明](REPOSITORY.md) |
| 查关键技术问题、根因与解决方案 | [问题与解决方案](LESSONS.md) |

## 重要问题保留入口

- [起伏道路RTF与采集残差](issues/ROUGH_ROAD_RUNTIME.md)：负端RTF已修复；残差故障被飞行记录仪捕获为单个物理步的接触链不连续，ODE/Bullet短道A/B已否决直接切换Bullet，三角网接触机理尚未关闭。

- [采集进程内存增长](issues/CAPTURE_MEMORY_GROWTH.md)：Mesa D3D12命令签名缓存的`&key`错误已确认，附独立复现、源码补丁和上游报告；私有源码修复版已投入运行并据此关闭第6节，系统Mesa与上游均未修复。
- [仿真关闭段错误与C盘转储](issues/GZ_SHUTDOWN_CRASH_DUMPS.md)：Gazebo退出时卸载库与存活线程竞争导致段错误，WSL把约等于RSS的整份core写进C盘；`ulimit -c 0`与`maxCrashDumpCount=0`实测均无效，源码确认只有负值才关闭采集。
- [转向恢复直行、8°机械余量](issues/STEERING_SPIN_RECOVERY.md)：保留旧126°动作的原因、8°配置实测，以及静止起步另加20°段余量的取舍。
- [控制回路停顿](issues/CONTROL_LOOP_STALL.md)：全系统I/O停顿击穿实时回路的三层根因、把自身停摆误报成定位失效的判据缺陷，以及WSLg图形上下文偶发失败的现状。
- [扫描道终点过冲](issues/PASS_TERMINAL_OVERSHOOT.md)：执行延迟与舵轮限位分支两个成因、停止距离钳制与段余量。
- [覆盖缺口：末端与起点](issues/COVERAGE_TAIL_GAP.md)：采集在ROI边界关闭而传感器会丢掉收尾短图，过扫距离改由规划器按上界导出（与道长无关）；以及起步对轮在快门已开时作废首块，改为对轮完成后再开采集。
- [路面UV镜像](issues/ROAD_DISPLAY_ALIGNMENT.md)：错误公式、实际渲染证据、迁移和回归。
- [掉头对轮后航向保护](issues/POST_TURN_HEADING.md)：对轮导致车身偏转、DRIVE不能作为唯一开采条件，前导段闭环恢复与原保护保留。
- [扫描摆动、后退调整、预览与条光](issues/SCAN_STABILITY.md)：保留调参前后原图和定位误差对照。
- [沟槽碰撞简化](issues/COLLISION_PROXY.md)：视觉/碰撞边界、成本与不适用工况。
- [道末采集关闭握手把整车刹停](issues/CAPTURE_CLOSE_HANDSHAKE_BRAKE.md)：关闭握手期间发零速度导致摆臂锁存Brake，30个道末中5次，代价约2–3 s；不影响采集数据，未修复。

## 历史资料

[归档索引](archive/README.md)保留早期底盘、CUDA/Ogre/OptiX路线比较、选材、标定、性能和阶段交付记录，以及[状态流水存档](archive/status/STATUS_LOG_2026-09.md)。旧模型数据不能作为550 kg版本的性能承诺。图片统一位于`images/`，数值报告保留在仓库`results/`。

维护时优先更新现行指南与当前状态；阶段实验放入archive，影响现行行为的重要问题放入issues。保留证据和来源，不在README末尾不断追加相互矛盾的阶段状态。


## 文档维护分工

- PROJECT_SPEC只定义目标/约束；CURRENT_STATUS只记录当前状态和下一步。
- 操作指南保留当前恢复/运行命令；issues保留关键故障的根因、修复和证据，页首先给最新状态。
- 数值细节以results为据，日期流水和已替代方案放archive，不覆盖旧实验参数。
- 本次整理移出的[规范进展](archive/maintenance/SPEC_PROGRESS_SNAPSHOT_2026-09-14.md)、[全区预算历史](archive/maintenance/FULL_ROAD_BUDGET_HISTORY.md)、[条带试验](archive/maintenance/STRIP_EXPERIMENTS_2026-09-14.md)、[复核流水](archive/maintenance/REVIEW_FIXES_2026-09-14.md)、[材质路线](archive/maintenance/ROAD_MATERIAL_EVOLUTION.md)和[旧版LESSONS](archive/maintenance/LESSONS_BEFORE_REWRITE_2026-09-14.md)均保留追溯，不作为默认阅读入口。
