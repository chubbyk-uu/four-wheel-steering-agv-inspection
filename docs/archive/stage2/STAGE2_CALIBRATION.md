> 已归档：保留原始设计、问题和实验数据；文中的“当前/下一步”属于当时阶段，现行入口见 [当前状态](../../CURRENT_STATUS.md)。历史命令仍从仓库根目录运行。

# 线阵暗场、平场和横向标定

> 历史实验记录：本文参数、性能和“当前/默认”描述属于当时版本，不作为新版验收。现行550 kg、20 mm/4K/1.5 m、1 m轨迹间距、11 kHz目标见[当前模型基线](../../LARGE_AGV_REBUILD.md)。历史命令须按现行配置调整，旧16 mm标定不可用于新版；大体积实验资产可能已清理，复现需重新生成。

2026-09-06。本文保留 CUDA 平面成像的历史标定。当前 16 mm 安装与常驻校正节点见 [OptiX 标定与在线校正](STAGE2_OPTIX_CALIBRATION.md)，不要将本文旧 profile 用于新后端。

## 算法与边界

- 暗场平均得到逐列偏置 D，均匀亮场平均得到响应 F−D；以有效列响应的中位数 S 为目标，校正为 `(raw−D)·S/(F−D)`。每份参考至少 256 行，本次使用 4096 行。该做法与工业相机的逐列 DSNU/PRNU 标定一致，见 [Basler 官方说明](https://docs.baslerweb.com/flat-field-correction)。
- 先校正原始列的响应，再进行横向插值；实现合并为两个源像素的加权和，避免中间量量化或裁剪。校正后暗场对应零，绝对灰度与原图黑电平不同；没有逐张自动亮度归一化。
- 横向标定板使用 50 mm 间隔的 2 mm 黑线。输入为实际图像中检测的线中心及人工已知的标定板坐标，拟合三次单调多项式，再建立反向重采样表。标定程序不读取仿真畸变系数或逐行真值位姿。
- 这是**固定高度、垂直安装、已对齐平面标定板的横向度量标定**。单张板不能独立分解焦距、安装高度、主点和横向外参；没有把这些量伪装成已估计的完整内外参。车身姿态变化和运动畸变仍需后续位姿融合处理。
- 参考板必须均匀、无阴影，且坐标顺序和相机方向正确。算法检查特征数量、单调性和拟合残差；不负责自动识别任意标定板或消除错误的标定板原点。
- 不向最外侧已测量特征以外外推；无效列、涉及原始 ADC 饱和的重采样像素填零。记录有效列掩码、原始饱和数和校正裁剪数。坏列不自动补洞，过曝不能恢复。
- 光学、LED、曝光和传感器配置生成摘要，使用校正文件时检查一致性；改变补光、增益、安装或曝光需要重新标定。该检查只能核对配置，不能发现真机灯具老化、未记录的机械位移或动态阴影。
- 行数、行顺序、块间关系保持不变。位置标签仍对应末行曝光中点，保留原始 first/last/pose_tags，另写校正文件标识。沿同一轨迹不人为增加重叠行。

## 实际采集验证

`tools/validate_measured_calibration.py` 通过现有 CUDA 纹理传感器拍摄暗场、亮场、标定板、错开半格的独立验证板和原有网格，使用相同的噪声、补光、晕影和 PRNU 模型。参考材质为线性 Mono8 均匀反射率。周期标定板的相同瓦片用硬链接节省磁盘，成像仍经过原有瓦片采样链。

亮场第一块用于标定，第二块用于验证，噪声实现不同；几何用相差 25 mm 的另一块板验证。未使用生成时的多项式来替代拟合或计算验证板像素位置。

本次结果：

| 检查 | 校正前 | 校正后 |
|---|---:|---:|
| 独立亮场列均值的标准差/均值 | 7.3198% | 0.06025% |
| 独立板横向位置最大误差 | 65.167 px | 0.3547 px |

25 个标定特征，拟合最大残差 0.4226 px；本次 4096 列均有效。两块连续网格分别校正与合并后校正逐像素相同，末行标签保持不变。这证明校正运算不制造分块接缝，不代表已经完成世界坐标多轨迹拼接。

## 使用

已构建工作区、加载 ROS 与项目环境后：

```bash
python3 tools/validate_measured_calibration.py \
  --terrain /tmp/agv_runway_tiles_100x10/manifest.json \
  --output /tmp/agv_measured_calibration_new > /tmp/agv_measured_calibration_new.log 2>&1

python3 tools/rectify_linescan.py \
  --calibration /tmp/agv_measured_calibration_new/measured.json \
  --capture-config src/agv_description/config/linescan.yaml \
  --input /tmp/agv_measured_calibration_new/grid/block_0.pgm \
  --output /tmp/agv_measured_calibration_new/corrected.pgm
```

`tools/calibrate_linescan.py` 可独立处理自己的暗场、亮场和条纹板图像；要求目标坐标 JSON 和采集条件 JSON。验证器输出这些文件作为示例。图像校正工具继续兼容原有 nominal YAML，但只有 measured JSON 走图像估计的暗场/平场/几何链。

## 性能范围

离线 Python 校正微基准约 4.1 万行/秒，未计采集、ROS 或磁盘，不能作为 22 kHz 完整验收。

`tools/benchmark_cuda_linescan.py --calibration PROFILE` 在独立 ROS 接收进程中执行校正，并将校正图像 PGM 写盘及 fsync 后才确认该块。计时包含 CUDA 采集、原图写盘/fsync、ROS 原图传输、逐字节核对、校正及校正图写盘/fsync。校正图未二次通过 ROS 发布，元数据写盘没有 fsync；没有 GZ 物理与实时控制负载。全场景验收标志仍为 false。

持续测试已通过（`/tmp/agv_corrected_23khz_v1`）：100×10 m 瓦片场景，三个扫描段各 80 块，累计 240 块 / 983040 行；42.7559 秒，平均 **22991.94 行/秒**。240 块均经独立 ROS 接收与逐字节核对。496 次瓦片加载，必需瓦片缺失 0；采样批次最长 7.89 ms，块确认间隔最长 196.57 ms，均在本测试预算内。校正加校正图落盘/fsync 每块平均 112.48 ms、最长 123.09 ms。

结果：[校正验证及持续测试](../../../results/stage2_linescan_calibration.json)，[本次网格环境标定文件](../../../results/linescan_calibration_grid.json)，[前后对比图](../../images/linescan_calibration_comparison.png)。保存的标定文件仅适用于记录的仿真安装与光照条件；来源图像路径及 SHA256 保留于文件中，原始大图位于 `/tmp/agv_measured_calibration_v2`，可按上述命令重新采集。

尚未完成：将校正作为 GZ 正常运行时的常驻 ROS 节点并与控制、采集同时进行负载验收；完整内外参分离、倾斜/颠簸运动补偿、真实水泥纹理及多轨迹拼接。当前没有改变正常 GZ 启动默认的 raw 图像输出。
