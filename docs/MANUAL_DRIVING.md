# 手动驾驶与低速相机试运行

从 README 移出，内容未改。这些命令用于不跑巡检任务、只手动驱动底盘或试跑低速相机的场合。

## 车体速度指令

启动仿真后另开终端，进入同一仓库并加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 tools/command_velocity.py --vx 0.5 --seconds 5
python3 tools/command_velocity.py --vy 0.5 --seconds 5
python3 tools/command_velocity.py --vx 0.3 --vy 0.3 --seconds 5
python3 tools/command_velocity.py --wz 0.3 --seconds 5
```

四条分别是前进、横移、斜行和原地旋转，覆盖 4WIDS 底盘的运动能力。

工具结束时发送零速度。`/cmd_vel` 是车体坐标系 `geometry_msgs/msg/TwistStamped`，使用
`linear.x/y`、`angular.z` 及当前仿真时间。**只运行一个指令发布者**，不绕过控制器直接操作
关节；切换运动可能先进入 BRAKE、ALIGN，再 DRIVE。

## 低速网格相机示例

替换启动命令使用，不要同时启动两个实例：

```bash
ros2 launch agv_bringup sim.launch.py gpu_backend:=native \
  linescan:=true linescan_backend:=analytic
# 另一个已加载环境的终端：
ros2 service call /linescan/set_enabled std_srvs/srv/SetBool '{data: true}'
python3 tools/command_velocity.py --vx 0.1 --seconds 5
ros2 service call /linescan/set_enabled std_srvs/srv/SetBool '{data: false}'
```

采集目录默认 `/tmp/agv_linescan`，可用 `capture_dir:=/path/to/capture` 修改。
原图 `/linescan/image_raw`，显示用 `/linescan/image_preview`。
正式 GZ 场景参考后端为 `linescan_backend:=render`。

相关：[视角说明](GUI_CAMERA.md)、[矩形任务执行](RECTANGLE_EXECUTION.md)、
[离线处理](OFFLINE_PROCESSING.md)。
