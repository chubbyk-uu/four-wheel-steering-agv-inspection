# 文档入口

日常只需先读[当前状态](CURRENT_STATUS.md)，再按任务查下表。安装、构建和基础运行从[项目README](../README.md)进入；已确认的完整目标与约束以[项目规范](../PROJECT_SPEC.md)为准。历史目录不作为默认阅读入口。

| 要做的事 | 现行文档 |
|---|---|
| 查车体、相机、雷达和安装参数 | [550 kg模型基线](LARGE_AGV_REBUILD.md) |
| 规划下一步与确认尚未实现的功能 | [实施计划](MISSION_IMPLEMENTATION_PLAN.md) |
| 使用RViz任务面板、载入区域和采图 | [规划器](../src/agv_mission/README.md) → [任务执行](RECTANGLE_EXECUTION.md) → [采集与标签](RECTANGLE_CAPTURE.md) |
| 定位、噪声、延时与外参 | [定位包](../src/agv_localization/README.md)、[误差基线](LOCALIZATION_NOISE_BASELINE.md) |
| GUI跟车和缩放 | [GUI操作](GUI_CAMERA.md) |
| 100×10 m场景预算及完整采集验收 | [全区域验收](FULL_ROAD_ACCEPTANCE.md) |
| 恢复道路与处理旧贴图 | [道路资产](ROAD_ASSETS.md) |
| 离线校正、灰度和后续TIFF | [离线处理](OFFLINE_PROCESSING.md) |
| 原生Linux / WSL的OptiX安装区别 | [OptiX安装](OPTIX_SETUP.md) |
| 核对回归覆盖及人工检查范围 | [验收约定](VISUAL_ACCEPTANCE.md) |
| 三维地表、双天线和未来条带拼接 | [后续设计](SURFACE_HEADING_STRIP_DESIGN.md)，其中研究方案不等于已实现 |
| GitHub恢复和分发边界 | [仓库说明](REPOSITORY.md) |

## 重要问题保留入口

- [转向恢复直行、8°机械余量](issues/STEERING_SPIN_RECOVERY.md)：保留旧126°动作的原因和新配置实测。
- [路面UV镜像](issues/ROAD_DISPLAY_ALIGNMENT.md)：错误公式、实际渲染证据、迁移和回归。
- [扫描摆动、后退调整、预览与条光](issues/SCAN_STABILITY.md)：保留调参前后原图和定位误差对照。
- [沟槽碰撞简化](issues/COLLISION_PROXY.md)：视觉/碰撞边界、成本与不适用工况。

## 历史资料

[归档索引](archive/README.md)保留早期底盘、CUDA/Ogre/OptiX路线比较、选材、标定、性能和阶段交付记录。旧模型数据不能作为550 kg版本的性能承诺。图片统一位于`images/`，数值报告保留在仓库`results/`。

维护时优先更新现行指南与当前状态；阶段实验放入archive，影响现行行为的重要问题放入issues。保留证据和来源，不在README末尾不断追加相互矛盾的阶段状态。
