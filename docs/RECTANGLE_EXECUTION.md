# 矩形任务执行

已接入绝对map目标、当前位置直线接入、停车航向对齐、连续PASS、横移、180°转身与前向驶出补偿。PASS将规划的ACCELERATE/SCAN/RUNOUT_BRAKE合为一条静止到静止时间曲线，扫描边界处不停。行距固定1 m，加速度0.8/减速度1.0 m/s²，当前试验0.5 m/s。

执行前须连续READY＋HOLD 5秒；输入道路frame必须显式map。车辆当前位置和规划路径都检查1.5 m扫掠半径；可行驶矩形的内缩区域是凸集，因此合法端点间的直线接入也不越界。该检查依赖用户声明的无障碍矩形，不是避障。每段终点是绝对规划坐标，不能把上一段误差累计到新目标。

```bash
# 将rectangle_demo.yaml复制到自己的任务文件，明确road.frame_id: map。
ros2 run agv_mission execute_rectangle --ros-args \
  -p use_sim_time:=true -p request:=/path/to/request.yaml \
  -p output_dir:=local_data/rectangle_run_01 -p autostart:=true
```

需先启动localization:=true的仿真；输出目录必须不存在。默认autostart=false，调用`/mission/start`（Trigger）开始，`/mission/cancel`取消并等待HOLD。取消不自动恢复；不同时运行其他cmd_vel发布者。每实例一项任务，归档plan.json、steps.json和execution.jsonl。

当前尚无动态障碍、RViz任务面板和最高速区域闭环验收。采集联动独立验证，不能把此运动报告称为完成了矩形图像覆盖。

后续已接可选capture模式，握手和归档语义见[矩形采集](RECTANGLE_CAPTURE.md)；本文件中的运动报告保留为独立基线。

当前换道采用前向驶出，已删除ENTRY后退段；0.5 m/s时中间轨迹的扫描后行程2.55625 m，掉头后直接对接下一道加速起点。调参、耗时与原图几何对照见[扫描稳定性优化](issues/SCAN_STABILITY.md)。

`tools/validate_rectangle_execution.py --gui` 默认启用锁定跟随（FOLLOW_LOOK_AT），始终看向车辆、保留滚轮缩放；需要固定场景视角时显式添加 `--no-follow-camera`。此前测试脚本强制关闭跟车，现已与正常启动的默认行为统一。

跟车启动需连续3秒确认目标为agv，避免仅凭场景加载期间一次短暂确认就退出；成功后不持续重发设置，不覆盖用户后续操作。旧版自由转头复查已保留在历史记录；当前锁定跟车验证见[视角说明](GUI_CAMERA.md)。


## 历史运动基线

以下14步结果早于前向换道修订；当前四道为11步，现行表现见扫描稳定性记录。

零噪声3×2 m两轨和带GUI＋RViz、正常噪声的8×4 m四轨均全部完成并停在HOLD。四轨共14个任务步骤，正常噪声各PASS参考/融合位置误差峰值约6.2–7.9 cm。它不是独立真值定位精度，也不是采集覆盖证明。聚合结果见[运动验收](../results/stage3_rectangle_execution.json)。


## 暂停与恢复

运行中调用`/mission/pause`（Trigger），进入PAUSING并持续发零速度，由底层按既定减速度制动；融合反馈有效、底层HOLD、速度低于阈值且停稳0.5 s后进入PAUSED。初始对轮结束前尚未开始采样；已有采集段暂停时保持相机启用，不关闭未满帧、不清零编码器触发余量。

调用`/mission/resume`显式恢复：当前位置到原计划绝对终点重新生成静止到静止的梯形/三角形时间曲线，不追赶暂停前的旧时间表，不重新跑整道；若已越过本道扫描终点，恢复只走剩余驶出段，不重新开启采集。仅在健康反馈、HOLD、相机握手完成时接受；停止后位置变化超过0.20 m、姿态变化超过0.10 rad、PASS终点已在后方，或保留图块的扫描段需要原地转向时拒绝，保留暂停状态并要求取消/重规划。这些是恢复护栏，不是亚毫米覆盖认证。

暂停期间定位过期或时钟停滞仍锁存FAULT，健康恢复不自动续跑。取消/故障走关闭采集的独立路径；取消等HOLD和相机关闭确认后成为CANCELED，故障保持FAULT。暂停记录位于`pause_events.json`，关闭原因归入`capture_intervals.json`。每实例仍只执行一项任务；不支持重启进程后自动续接内存中的未满帧。

```bash
ros2 service call /mission/pause std_srvs/srv/Trigger '{}'
ros2 service call /mission/resume std_srvs/srv/Trigger '{}'
ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'
```
