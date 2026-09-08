# 矩形任务采集联动

在矩形时间闭环执行器上新增可选`capture:=true`。每轨开始前等待相机准备确认；整条PASS包含加速、扫描和驶出，融合位置推导的扫描中心越过区域终点0.10 m后关闭采集，车辆继续前向驶出，换道前等待尾块落盘确认；STOPPING也是兜底关门条件。横移、掉头和终点微调不启用采集。零指令仍交给底层制动，不操纵关节。

## 采集边界与成功语义

每条原始档案包含少量区域外加速/驶出余量，后续按地面坐标裁切。门控不试图通过50 Hz ROS服务准确截出亚毫米边界，也不以参考路程触发拍照。行触发仍来自实际轮编码器，完整图块4096×4096、尾块保持实际行数，Mono8，未在线平场或畸变校正。

`COMPLETED`用于纯运动模式。开启采集时末态为`ACQUIRED`：表示运动完成、采集已关闭且写入确认，**不自动等于区域完整性验收通过**。验证工具随后检查原图、连续传感器段、采集端点和融合标签，成功写出独立报告；失败会保留全部证据。真实场景存在定位不确定度，自动覆盖认证、暂停补扫和异常恢复仍需后续实现。

启动时先明确关闭相机，防止继承未知采集状态。服务异步调用，等待期间保持停车；超时/渲染故障锁存FAULT。取消会制动并请求关采集，FIX恢复不自动恢复运动。用户主动终止整个进程不等于完成归档握手，应优先调用取消服务并等到停车/关闭确认。

## 四驱四转编码器模型

旧模式只接受近零轮角和很小的四轮速度差，无法直接承受闭环纠偏。新模式显式启用`projected_encoder`：

- 将每轮速度按实际转角转换为车体系二维向量，按轮组位置拟合vx/vy/wz。
- 累计每轮编码器增量乘相邻转角中点的cos，再四轮平均，得到车体X方向距离。对称轮组的旋转项相抵；驱动反号＋轮角增加π仍等价。
- 运动有效性改为检查拟合残差、横向速度、偏航速度和速度上限。初始试验阈值分别0.03 m/s、0.10 m/s、0.08 rad/s，明确归档；它们是本轮低速联合测试档位，不是任意曲线采图或最高速承诺。
- BRAKE期间只要运动仍有效，就继续编码器触发。纯横移/转身由上层门控禁止；不把正转/反转轮角分支误当扫描方向变化。

既有直线模式默认不变。新距离是车体纵向编码器投影，不是世界轨迹弧长，也没有假装消除轮胎滑移、车体俯仰引起的实际地面采样变化；后续离线几何校正仍依赖位姿与地表模型。

## 导航标签与档案

`capture_intervals.json`记录轨迹ID及开关确认，`capture_events.jsonl`保留全部传感器分段/异常，`capture_blocks.jsonl`保存收到的原始元数据。

`tools/label_mission_capture.py`按采集区间关联原图，读取50 Hz融合导航和相机标定外参，为每块已有的少量标签生成map中相机位姿；标签仍以最后一根扫描线曝光中点作为图块参考，不将其改到图片中心。位置线性插值、姿态SLERP，要求双侧导航时间括号不超过60 ms，禁止外推、跨标定或混用frame。导航真值不参与生成这些标签。

输出`capture_manifest.json`为独立生产侧标签；GZ原始元数据中的成像真值保留原有明确命名，供独立评价。原始像素不修改，未开始拼接。

## 运行

先复制矩形请求，将`road.frame_id`明确设为`map`，确认当前无障碍可行驶边界。准备新的相机配置文件（仅覆盖运动采样模型，保留当前20 mm/1.5 m光学参数）：

```bash
python3 tools/prepare_mission_camera.py --output /tmp/agv_mission_camera.yaml
ros2 launch agv_bringup sim.launch.py localization:=true rviz:=true \
  linescan:=true linescan_backend:=optix \
  scene_manifest:="$PWD/assets/road/baked_fullwidth_20m_v1/manifest.json" \
  camera_config:=/tmp/agv_mission_camera.yaml scan_speed_limit:=0.8 \
  spawn_x:=3 capture_dir:=/tmp/agv_mission_raw \
  localization_output_dir:=/tmp/agv_mission_navigation
```

```bash
ros2 run agv_mission execute_rectangle --ros-args \
  -p use_sim_time:=true -p request:=/path/to/map_request.yaml \
  -p output_dir:=local_data/mission_capture_01 -p autostart:=true -p capture:=true
python3 tools/label_mission_capture.py \
  --mission local_data/mission_capture_01 --navigation /tmp/agv_mission_navigation
```

执行器输出目录、导航目录和准备配置文件使用新名字。WSL OptiX包装和D3D12环境沿用根README；普通Linux使用本机驱动/SDK路径。公开仓库不包含大型道路烘焙文件，恢复道路资产后再运行真实纹理场景。

## 验证边界

聚合结果见[四轨采集报告](../results/stage3_rectangle_capture.json)，修订前后比较见[扫描稳定性](SCAN_STABILITY.md)。ROS原图与磁盘像素逐块SHA256校验；每块行号跨度与实际行数一致；采集区域由独立真值评价纵向端点，要求同一个未中断传感器段跨过每轨整个ROI。轨迹间距1 m、幅宽1.5 m。未把少量稀疏标签当成逐像素覆盖证明，未做往返图像特征拼接，也未宣称暂停/补扫已完成。

运行优化见[扫描稳定性](SCAN_STABILITY.md)：全局融合反馈保留，前向驶出替代后退，预览512×512，条光降至0.30 m/偏竖直15°。
