# 控制与成像修改的观察验收

此前验收过度依赖任务完成、消息连续和清单校验，遗漏了用户实际看到的行为。限位的两级余量累计到15°，对轮效率没有被单独量化；分块贴图的生成和校验使用相同错误UV公式，数值检查通过但实际画面镜像；自由转头跟车也没有用掉头后目标仍在视野内作为判据。

后续修改对应组件时，按以下已实现的检查验收，而不是只报告“单元测试通过”。这些是本机GUI/资产相关验证，不代表已接入无GPU的自动CI。

| 修改内容 | 必查行为 | 现有检查 |
|---|---|---|
| 运动分配、转向限位或换道策略 | 直行与横移后正反原地旋转、恢复直行；每轮实际起止角、驱动反向与最短合法分支；限位拒绝原因 | `tools/validate_spin_recovery.py`，C++转向回归；[限位记录](issues/STEERING_SPIN_RECOVERY.md) |
| 跟车视角或GUI交互 | 实际鼠标拖动被锁定、滚轮缩放保持、车辆移动和掉头后仍在视野内、最终HOLD | `tools/validate_gui_camera.py`；[视角记录](GUI_CAMERA.md) |
| 地面烘焙、分块或UV映射 | 实际Ogre2渲染的标线位于已知世界坐标；检查双黄线、白边线、板缝及采图对应区域 | `tools/render_road_alignment.py`，共享场景回归；[映射记录](issues/ROAD_DISPLAY_ALIGNMENT.md) |
| 闭环扫描或预览链路 | GUI＋RViz＋原图归档；实际运动是否弯曲图像、预览分辨率、连续ROI、原图与ROS像素一致 | `tools/validate_rectangle_execution.py --gui --scene ...`，稳定性及覆盖审计；[扫描记录](issues/SCAN_STABILITY.md) |

所有会移动车辆的工具须独占速度指令来源，不能与任务执行器同时运行；使用实际仿真时间戳，结束确认停车。演示前明确正在测试哪种动作，避免把停车检查或短暂对轮当作完整运动演示。

配置一致只证明文件符合约定，实际渲染一致才证明约定在渲染器中成立；单一模块的检查不能代替端到端观察。报告应区分实际测量、历史结果和未验证范围，不能将观察验收留给用户完成。

联合实时性能验收必须给`tools/validate_rendered_linescan.py`加`--require-realtime`，在GUI＋RViz＋采图条件下测量稳速段。实时率低于0.95时写出报告并失败；不带此选项仍可做慢速渲染诊断，但不能据其passed宣称实时通过。记录车速、距离、场景、预览及补光配置；10 km/h时约7.59 kHz，与独立11 kHz采样台架验收分开。

暂停协议修改后，用`tools/validate_rectangle_execution.py --gui --profile normal --scene ... --pause-once`验证原图跨停车时间而不分帧、相邻行号/标签、无PASS后退及正常区域连续性。再分别用`--stop-after-pause cancel`和`--stop-after-pause localization_timeout`验证未满帧明确关闭，后者只挂起本次启动的定位适配进程并在验证/清理时恢复，不影响其他实例。故障恢复后仍应保持FAULT；测试的passed表示预期故障处理通过，不表示采集区域完成。


运动故障回归使用`tools/validate_rectangle_execution.py --gui --profile normal --scene ... --moving-fault localization_timeout`或`camera_disabled`，记录故障时速度、零指令延时、真值停车距离、实际逐轮转角、尾图与HOLD；不得将仿真时间延时当作墙钟响应。普通暂停需同时复测，防止心跳误报。结束后用`tools/audit_mission_capture.py`检查归档并生成补扫预览；用验证器`--request`执行选定候选，独立审计新会话，不将局部补扫当作全区域完成。


RViz面板改动后运行`tools/validate_operator_session.py --output 新目录 --inspect-seconds 25`，检查真实Gazebo/RViz联合流程、请求保存重载、越界拒绝、运行中锁定、暂停保留帧、审计与新任务取消；记录实际逐轮转角、ROS/原图哈希及最终HOLD。另需实际点击面板控件并观察状态改变，检查预览和覆盖标记显示正常、车辆移动时仍固定在道路坐标，而不是只检查ROS话题存在。启动时TF未就绪必须能恢复，不能留下红色预览状态。普通colcon包含Qt控件状态与后端护栏测试，不会自动启动完整GUI。WSLg下X11抓屏可能全黑，不能把黑色抓屏当作真实窗口；必要时从宿主窗口采集证据，公开图裁去本机标题/路径。

巡检入口速度、尾图与道路显示改动后，另运行以下GUI场景（每次使用新输出目录）：

- `tools/validate_operator_session.py --output local_data/check_08 --speed .8 --length 5 --width 2`：运行中改参数拒绝、暂停后满帧、重新准备与取消。
- `tools/validate_operator_session.py --output local_data/check_10kmh --speed 2.777777777777778 --length 5 --width 1 --start-x 8 --spawn-x 1.93 --no-pause --no-cancel-probe`：有足够加减速空间的高速单道，检查终点、实际速度与实时率；不代表高速多道换道验收。
- `tools/validate_operator_session.py --output local_data/check_tail --speed .2 --length .05 --width 1 --short-tail-probe`：不足1000行不输出图像，保留丢弃事件，覆盖报告不冒充完整。

面板实际观察需包含同源路面、双黄线、白边线、青色道路边界及移动时坐标固定；小窗口是缩略图，原图尺寸另外显示。保存的满帧必须4096×4096，尾图至少1000行，ROS像素与PGM逐字节核对。验证器另保存逐轮转角与真值速度/时钟供诊断。

同一工具加`--fault-probe`时，正常采集后另开新任务，在运动中注入`unsupported_scan_motion`状态事件，要求FAULT、HOLD、采集关闭；它验证任务层保护，不伪称真实渲染器故障。测试主流程通过但后续探针未完成时，结果`passed`保持false。


2026-09-10按需材质试片：以`--scene local_data/runtime_recipe_scene/manifest.json --start-x 11 --start-y 1.5 --spawn-x 9.5 --length 3 --width 2 --speed .8 --no-cancel-probe`运行上述GUI验证器，包含暂停恢复。实际查看RViz窗口，确认路面、双黄线/白边线、边界和机器人显示；运行中显示4096×1670尾图，宽矩形预览符合实际行数。6张19718行原图与ROS一致，最终HOLD。GZ参与联合运行但本次未取得独立GZ窗口截图，未重新验收鼠标交互或全部动态部件抖动；几何/显示贴图未修改。完整20 m材质与旧高清1056块逐字节相同，96张独立OptiX采图亦逐字节相同，见[报告](../results/runtime_material_recipe_probe.json)。此项不代替100 m、10 km/h或重复异常验收。
