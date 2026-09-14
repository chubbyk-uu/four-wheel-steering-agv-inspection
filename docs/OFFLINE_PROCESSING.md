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

不需要ROS/Gazebo/GPU。输出目录必须新建，保留原图及行数、时间和稀疏标签；缺块、尺寸或标定不匹配会拒绝处理。当前20 mm/v7及0.30 m/15°条光的标定仍待重做，旧16 mm profile不能用于当前原图。均匀标定板用于平场，不能拿水泥纹理当仪器响应消除。

后续顺序：暗场/平场及横向光学校正→基于连续条带的运动几何补偿和跨轨配准→拼接→统一亮度编码的查看版TIFF。不得把同轨无重叠文件当独立面阵图任意平移；不能逐块自动拉伸形成亮度接缝。保留线性数据、无效与饱和信息，长路段采用分块处理/BigTIFF预算。

完整条带优化、拼接和TIFF导出尚未实现，设计见[连续条带方案](SURFACE_HEADING_STRIP_DESIGN.md)。旧标定的测量方法、独立验证及离线工具测试证据完整保存在[历史校正记录](archive/stage2/STAGE2_CONCRETE_CORRECTION.md)，不作为新版标定已通过的证明。
