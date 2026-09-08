# 历史资料索引

这些文档保留原始问题、设计取舍、来源、图片和数值报告。旧参数与“下一步”描述属于当时阶段，不作为现行配置；请先读[当前状态](../CURRENT_STATUS.md)。实验命令仍从仓库根目录运行。

## 早期底盘与八电机控制

- [第一阶段：AGV 模型与基本控制](stage1/STAGE1.md)

## 线阵路线、材质、标定与性能实验

- [第二阶段：线阵网格采集原型](stage2/STAGE2.md)
- [分叉裂缝局部试片](stage2/STAGE2_BRANCH_CRACK.md)
- [线阵暗场、平场和横向标定](stage2/STAGE2_CALIBRATION.md)
- [Concrete047A 与 Gravel 同布局比较](stage2/STAGE2_CONCRETE047A_COMPARE.md)
- [Brushed Concrete 03 与AI裂缝烘焙样片（历史，已停用）](stage2/STAGE2_CONCRETE_BAKE.md)
- [水泥路面标定复核与离线校正](stage2/STAGE2_CONCRETE_CORRECTION.md)
- [第二阶段：CUDA 批量线阵网格原型](stage2/STAGE2_CUDA.md)
- [10 km/h 平地采集与 GUI / RViz 抖动修复](stage2/STAGE2_FLAT_GUI_10KMH.md)
- [20×10 m 全宽道路与相邻扫描试验](stage2/STAGE2_FULLWIDTH_ROAD.md)
- [Gravel Concrete 03：同源底纹与一致预览](stage2/STAGE2_GRAVEL_BAKE.md)
- [局部裂缝、板缝沟槽成本实验](stage2/STAGE2_GROOVE_COST.md)
- [Gazebo / Ogre2 多行批量成像实验](stage2/STAGE2_OGRE_BATCH.md)
- [16 mm OptiX 标定与在线校正](stage2/STAGE2_OPTIX_CALIBRATION.md)
- [OptiX 正式采集接口与逐曝光动态几何](stage2/STAGE2_OPTIX_INTEGRATION.md)
- [OptiX 三维线阵成像微基准](stage2/STAGE2_OPTIX_SCAN.md)
- [OptiX WSL 最小可行性验证](stage2/STAGE2_OPTIX_WSL.md)
- [Concrete047A 颜色、法线与粗糙度成本](stage2/STAGE2_PBR_COST.md)
- [第二阶段：条形 LED 与线阵响应](stage2/STAGE2_RADIOMETRY.md)
- [C++ Gazebo 线阵场景采样后端](stage2/STAGE2_RENDER.md)
- [Review 第一批修复：采集正确性、队列与故障恢复](stage2/STAGE2_REVIEW_FIXES.md)
- [测试环境、平地边缘与灯条收敛](stage2/STAGE2_REVIEW_FOUNDATION.md)
- [双向两车道路面与小范围整车采集](stage2/STAGE2_SHARED_ROAD.md)
- [共享静态场景与几何一致性](stage2/STAGE2_SHARED_SCENE.md)
- [100 m 道路颜色＋法线流式采样](stage2/STAGE2_STREAMED_ROAD.md)
- [平地纹理与大场景加载计划](stage2/STAGE2_TEXTURE_PLAN.md)
- [20 mm 镜头与跑道分块纹理原型](stage2/STAGE2_TILES.md)

## 定位、闭环与维护阶段报告

- [三维定位接入与验证](integration/LOCALIZATION_IMPLEMENTATION.md)
- [文档、RViz与数据维护审计](integration/MAINTENANCE_AUDIT.md)
- [矩形巡检任务、时间轨迹与融合定位设计](integration/RECTANGLE_MISSION_PLAN.md)
- [单段梯形/三角形时间闭环](integration/TRACKING_IMPLEMENTATION.md)
