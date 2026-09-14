# 原图、离线校正与后续拼接

当前4096像素/1.5 m、默认4096行/块，可配置纵向行数；普通暂停保留缓存；结束/取消/故障时默认丢弃不足1000行尾图并记录缺口，达到阈值才保存实际行数。同轨沿程连续无重叠，重叠来自不同轨迹。每图标签参考末行，保留少量中间位姿，不能套用面阵图像中心位姿。

正式流程只在线采集Mono8线性原图，不设置`correction_profile`。RViz预览默认面积缩小至512×512并做sRGB显示转换；预览提亮不改变原图曝光或归档像素。

**阶段门槛已于2026-09-14满足**：100×10 m全区域稳定采集通过[实施计划第6节](MISSION_IMPLEMENTATION_PLAN.md)（[收尾记录](../results/section6_closure.json)），正式离线标定、条带优化和拼接/TIFF开发可以开始。下文的诊断工具用法保留不变。

有与当前光学/安装/照明相匹配的标定后，已有工具可逐块离线处理：

```bash
python3 tools/correct_linescan_session.py \
  --input /path/to/capture/session_cpp_TIMESTAMP \
  --profile /path/to/calibration_matching_current_capture.json \
  --output /path/to/new_corrected_directory
```

不需要ROS/Gazebo/GPU。输出目录必须新建，保留原图及行数、时间和稀疏标签；缺块、尺寸或标定不匹配会拒绝处理。当前20 mm/v7及0.30 m/15°条光已有[固定平面实测标定](../results/camera_20mm_measured_profile.json)，独立平场列均值CV为0.000498、移位验证靶最大误差0.720像素，见[记录](../results/camera_20mm_calibration.json)。该模型标定的是固定高度下横向映射，不独立解算焦距、相机高度及安装姿态，也不代表运动几何已校正；旧16 mm profile不能用于当前原图。均匀标定板用于平场，不能拿水泥纹理当仪器响应消除。

重新采集标定参考：在WSL先运行`bash tools/with_optix_runtime.sh bash`，在该子shell内加载ROS和工作区环境，再执行`python3 tools/with_mesa_runtime.py python3 tools/validate_optix_calibration.py --output 新目录 --grid-scene assets/road/runtime_fullwidth_20m_v1/manifest.json`。工具依次直行采集暗场、平场、条纹靶、移位验证靶和路面，并确认停车；先前环境若只加载Mesa而没有OptiX运行库，会在采集前启动失败。标定场宽4 m满足大车出生包络。

短尾丢弃造成的块号跳跃，仅在原始`events.jsonl`有合法`tail_discarded`事件时允许。离线工具核验事件行数、首末行号及有限时间戳，并将事件与保存图块一起检查块号、相邻行范围、时间和段号顺序；最终尾图同样检查。输出`summary.json`的`discarded_tail_blocks`保留完整原始尾图事件（含`first`、`last`及标签），不能只凭`block_id_gaps`判断末端缺口，该字段只列保存图块之间的跳号。其它原始事件仍留在源目录，不复制为校正后的事件日志。

后续顺序：暗场/平场及横向光学校正→基于连续条带的运动几何补偿和跨轨配准→拼接→统一亮度编码的查看版TIFF。不得把同轨无重叠文件当独立面阵图任意平移；不能逐块自动拉伸形成亮度接缝。保留线性数据、无效与饱和信息，长路段采用分块处理/BigTIFF预算。

## 短程双道重投影诊断（已实现）

```bash
python3 tools/project_linescan_strips.py \
  --mission /path/to/mission --navigation /path/to/navigation \
  --profile results/camera_20mm_measured_profile.json --output /path/to/new_projection
```

输入原始图块和融合导航，仅读原始标签的行号与时间，忽略其中的真值位置/姿态。横向用测得的多项式射线和暗场/平场响应；每行按稀疏时间锚点、融合位置/姿态及估计相机外参投影，最后一次双线性前向散射到地面栅格。各道分别输出图像和有效掩膜，不做特征匹配、接缝融合或填洞。输出行对应道路+X、列对应+Y，分辨率默认0.3662109375 mm。当前仅支持与世界坐标一致的z=0平面，限制1亿输出像素，供短程验证；不是100 m成品输出器。

同一批数据加`--camera-x-residual-mm 3`可做受控重放：仅给估计相机杆臂增加车体X方向3 mm，不修改原图/导航文件，隔离两次驾驶噪声的影响。这是额外误差，不会清除导航档案中已有的标定误差。它不等于重新采了一次残差工况。

2026-09-14：GUI/RViz下3×2 m、0.5 m/s采得6块20762行，逐轮记录保留，最终HOLD。重放像素相关测得两道沿程偏移+2.987/-3.006 mm；图像实际查看确认方向统一，但标线仍有估计轨迹引入的波动，尚未进行跨道匹配。GUI窗口存在及数据链路通过，窗口截图获取未成功，不追加跟车交互/车轮外观人工验收结论。详情见[诊断记录](../results/strip_pair_reprojection.json)。

限制：射线模型从固定高度平面标定换算，使用名义高度，未独立识别焦距/高度/倾角；稀疏行时间插值不能恢复未观测的高速变化；前向重采样没有超分辨率能力。这一阶段证明外参残差传播和数据接口可用，不证明跨道配准精度或裂缝尺寸测量精度。

完整条带优化、拼接和TIFF导出尚未实现，设计见[连续条带方案](SURFACE_HEADING_STRIP_DESIGN.md)。旧标定的测量方法、独立验证及离线工具测试证据完整保存在[历史校正记录](archive/stage2/STAGE2_CONCRETE_CORRECTION.md)，不作为新版标定已通过的证明。
