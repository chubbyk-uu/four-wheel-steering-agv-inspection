# 矩形任务执行

已接入绝对map目标、当前位置直线接入、停车航向对齐、连续PASS、横移、180°转身与前向驶出补偿。PASS将规划的ACCELERATE/SCAN/RUNOUT_BRAKE合为一条静止到静止时间曲线，扫描边界处不停。行距固定1 m，加速度0.8/减速度1.0 m/s²，当前试验0.5 m/s。

执行前须连续READY＋HOLD 5秒；输入道路frame必须显式map。车辆当前位置和规划路径都检查1.5 m扫掠半径；可行驶矩形的内缩区域是凸集，因此合法端点间的直线接入也不越界。该检查依赖用户声明的无障碍矩形，不是避障。每段终点是绝对规划坐标，不能把上一段误差累计到新目标。

零噪声3×2 m两轨和带GUI＋RViz、正常噪声的8×4 m四轨均全部完成并停在HOLD。四轨共14个任务步骤，正常噪声各PASS参考/融合位置误差峰值约6.2–7.9 cm。它不是独立真值定位精度，也不是采集覆盖证明。聚合结果见[运动验收](../results/stage3_rectangle_execution.json)。

```bash
# 将rectangle_demo.yaml复制到自己的任务文件，明确road.frame_id: map。
ros2 run agv_mission execute_rectangle --ros-args \
  -p use_sim_time:=true -p request:=/path/to/request.yaml \
  -p output_dir:=local_data/rectangle_run_01 -p autostart:=true
```

需先启动localization:=true的仿真；输出目录必须不存在。默认autostart=false，调用`/mission/start`（Trigger）开始，`/mission/cancel`取消并等待HOLD。取消不自动恢复；不同时运行其他cmd_vel发布者。每实例一项任务，归档plan.json、steps.json和execution.jsonl。

当前尚无暂停/恢复、动态障碍、RViz任务面板和最高速区域闭环验收。采集联动独立验证，不能把此运动报告称为完成了矩形图像覆盖。

后续已接可选capture模式，握手和归档语义见[矩形采集](RECTANGLE_CAPTURE.md)；本文件中的运动报告保留为独立基线。

当前换道采用前向驶出，已删除ENTRY后退段；0.5 m/s时中间轨迹的扫描后行程2.55625 m，掉头后直接对接下一道加速起点。调参、耗时与原图几何对照见[扫描稳定性优化](SCAN_STABILITY.md)。

`tools/validate_rectangle_execution.py --gui` 默认启用位置跟随（FOLLOW_FREE_LOOK），可手动旋转视角和滚轮缩放；需要固定场景视角时显式添加 `--no-follow-camera`。此前测试脚本强制关闭跟车，现已与正常启动的默认行为统一。

跟车启动需连续3秒确认目标为agv，避免仅凭场景加载期间一次短暂确认就退出；成功后不持续重发设置，不覆盖用户后续操作。GUI重启复查中，两次间隔8秒的读数均为FOLLOW_FREE_LOOK/agv，视角位置随车移动约4.08 m、朝向保持不变。
