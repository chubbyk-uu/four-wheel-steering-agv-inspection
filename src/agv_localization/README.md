# 三维融合定位与测量接口

已接入四驱四转轮里程计、Gazebo六轴IMU、双天线RTK位置模拟、安装/标定残差、局部/全局EKF及导航归档。当前是在已知ENU地图上的低速平地验证；不是区域闭环执行器，也没有用GNSS代替图像配准。

## 启动

依赖`ros-jazzy-robot-localization`、NumPy/SciPy和已有ROS桥接包，`rosdep install --from-paths src --ignore-src --rosdistro jazzy -y`可解析依赖。本机已有这些依赖，本轮无需新安装。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select agv_localization agv_description agv_bringup
source install/local_setup.bash
ros2 launch agv_bringup sim.launch.py \
  localization:=true localization_profile:=normal rviz:=true \
  localization_output_dir:=local_data/navigation_demo_01
```

每次使用新的归档目录，已有`navigation.jsonl`会拒绝覆盖。不传目录时创建独立`/tmp/agv_navigation_*`目录。零噪声/零延时对照用`localization_profile:=zero`，它不清除配置文件中的固定外参残差。原有无定位演示保持可用，`localization`默认关闭；定位不依赖CUDA/OptiX。

`spawn_yaw:=0.7`可验证非零初始朝向；它只设置物理出生朝向，不输入滤波器初始航向。等待`/localization/status`的READY再执行后续任务。

## 数据与坐标

| 话题 | 内容 | 频率 |
|---|---|---|
| `/joint_states` | 四个累计驱动角及转向角 | 100 Hz |
| `/sensors/imu/raw` | GZ真实动力学产生的IMU原始数据 | 100 Hz |
| `/sensors/gnss/fixed/left`、`right` | 同测量时刻的两个有效FIX天线位置，PoseWithCovarianceStamped | 10 Hz |
| `/localization/wheel_odom` | 编码器量化/几何标定后的vx、vy、wz；不融合它的pose | 100 Hz |
| `/localization/imu` | 角速度与加速度；orientation_covariance[0]=−1 | 100 Hz |
| `/localization/tilt` | 重力方向，仅roll/pitch；停稳窗口或行驶中扣除运动加速度后 | 停稳最多1 Hz；行驶中最多10 Hz |
| `/localization/contact_velocity` | 持续接地的车体法向速度约束，明确不是编码器直接测量 | 100 Hz |
| `/localization/gnss_pose` | 杆臂补偿后的XYZ/yaw联合观测及交叉协方差 | 10 Hz |
| `/odometry/local`、`/odometry/global` | robot_localization的局部/全局估计 | 100 / 50 Hz |

GNSS节点仅模拟已转换成局部ENU的天线位置；当前地图原点和坐标轴已知，`world→map`固定恒等，不做一次人为经纬度往返转换。尚未实现真实接收机驱动、WGS84/UTM与`navsat_transform_node`联调；接真机时需提供测地基准和FIX/相对基线协议适配。Pose消息的orientation并非天线姿态观测。

生产TF由两个EKF唯一发布`map→odom→base_link`。原有RViz使用独立`/visualization/tf`显示真实机械几何，保持防抖配置；不能将显示真值TF作为控制输入。测量适配器和两个EKF均不订阅`/ground_truth/*`，只有模拟GNSS源及独立测试器读取它。

## 噪声、航向与初始化

参数来自[已确认基线](../../docs/LOCALIZATION_NOISE_BASELINE.md)。每根天线每水平轴σ=2.5 cm，垂直σ=4 cm，10 Hz；50±5 ms传输延迟，抖动截断±3σ，保留原始测量时间戳。IMU和轮里程计分别5±1 ms、10±2 ms；队列有界512条，乱序测量由迟到数据配置处理。

四轮使用累计角度量化，避免每步独立取整丢失小位移；最小二乘解算vx/vy/wz，保留横移。轮径固定残差、转角白噪声与轮组几何/零位残差分别处理。位置和速度不重复融合为独立观测。

双天线基线1.10 m，航向来自两点之差并扣除标定基线方向。默认尚未知真机载波相位相对解精度，采用两个独立位置解，理论航向σ约1.84°；不能把它包装成双天线设备的最终规格。配置`gnss_correlation`允许表达已知的共模相关性，改变它必须有依据。基线和后置杆臂同时传播到XYZ/yaw的完整4×4协方差，保留位置—航向相关项。

IMU不融合Gazebo提供的理想绝对姿态。停稳时roll/pitch由连续静止加速度样本获得（`stationary_tilt_samples`，默认60，须短于任务转场dwell）；同时要求实测轮速、角速度和重力模长满足门限。

行驶中按AHRS常规做法提供连续重力参考：加速度计测的是比力，必须先由独立速度源扣除运动加速度才能读作重力。运动项`a = dv/dt + omega x v`来自四轮转向编码器解出的车体twist最小二乘拟合，**不使用加速度计自身积分，也不使用真值**。拟合斜率代表窗口中心时刻，因此加速度计在同一跨度上平均并按该时刻发布；若与最新样本配对，pitch会被jerk乘以半个窗口的量偏置。不确定度包含斜率标准误、半窗斜率差给出的jerk界、与补偿量成正比的滑移/悬挂项，以及残余重力模长偏差。`motion_tilt_max_accel_m_s2`是加速度计可信度门：运动加速度超过该值时轮速模型不再可靠（滑移、悬挂俯仰），姿态改由陀螺推算。整条通路可用`motion_tilt_enabled`关闭。健康消息中`motion_tilt_rejects`统计的是拒收期内的IMU样本（100 Hz），而`motion_tilt_updates`受`motion_tilt_interval_s`限频，两者之比不是接受率。

首版加速度发布但不融合；固定零偏未被宣称自动消除。单条GNSS基线不能观测全部三轴姿态。四轮独立转向可侧移，因此不施加NHC横向零速假设——车体横向速度由轮速直接测量，测量优于假设。

两个滤波器均`two_d_mode=false`，局部不融合GNSS。全局加入XYZ/yaw；迟到数据历史1 s，输出预测到当前时刻。过程噪声是初始工程配置，后续要结合任务速度和创新统计继续评价。配置依据[robot_localization官方状态估计说明](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/state_estimation_nodes.rst)及[坐标/测量说明](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/preparing_sensor_data.rst)。

### 接地约束的适用范围

首次2 s GNSS失锁试验发现未观测的垂直速度把GNSS高度噪声外推成约47 cm最大位置误差。当前车辆持续接地，新增单独的模型约束：**base_link坐标中vz≈0，σ=0.02 m/s**，不将其称为编码器测量，也不锁死世界坐标Z。沿坡面前进时，车体vx经姿态变换仍产生世界高度变化。

开关为`assume_continuous_ground_contact`。当前平地、持续接地工况默认开启；离地/严重悬挂跳动不适用，届时必须检测接触并撤销或放宽约束。坡面仅有数学接口测试，未完成真实坡面仿真验收。

## 外参与归档

本轮采用**物理模型T_true不变，算法T_hat带固定标定残差**的实验方式。不是每帧抖动，不改变相机实际射线和光照；固定残差作用在算法使用的坐标变换上。若以后改T_true，仍必须同步修改实际物理/光学几何。

- GNSS共同支架的局部平移、旋转向量及两根天线各自相位中心残差，影响航向与杆臂补偿。
- IMU安装旋转影响角速度与重力初始化；平移记录为标定外参，当前不融合加速度，未实施加速度杆臂补偿。
- 相机及两台C8输出独立`*_calibrated`静态坐标系；旋转采用SO(3)旋转向量右乘，平移在原传感器局部轴表达，不直接相加欧拉角。
- 轮径、轮组XY位置和转向零位一起纳入标定档案及ID。

`navigation.jsonl`保存50 Hz全局估计的XYZ、四元数、速度、协方差和标定ID；`calibration.json`保存算法外参。独立`evaluation/`放模拟配置/真实天线安装和测试器记录的真值轨迹。输入图块的融合标签和相机标定ID仍待任务采集阶段接入；当前旧采集档案不能据此宣称已换成融合标签。C8标定坐标已预留，点云误差融合/避障验收后置。

`extrinsic_probe.yaml`是3 mm/0.1°量级的确定性敏感性工况，包含GNSS、相机、IMU和转向零位，不代表制造公差。`outage_probe.yaml`为正常噪声下第13秒开始的2秒FIX失锁。

## 状态与验证

READY同时要求轮/IMU、局部/全局滤波输出新鲜、重力初始化完成、有效GNSS测量年龄小于0.5 s。缺测或失锁发布NOT_READY及`stop_required=true`，恢复后重新评价。当前只输出状态，尚未接任务控制器停车；独立墙钟通信看门狗由下一阶段执行器落实，不将ROS时钟暂停误当作已完成安全停机。

```bash
colcon test --packages-select agv_localization
colcon test-result --test-result-base build/agv_localization
python3 tools/validate_localization.py --profile zero \
  --output local_data/localization/check_zero_01.json
python3 tools/validate_localization.py --profile normal --gui \
  --config src/agv_localization/config/outage_probe.yaml --expect-outage --spawn-yaw 0.7 \
  --output local_data/localization/check_outage_01.json
```

测试器开启并关闭自己的仿真，发送车体速度指令，经原有对轮/制动状态机完成前进、横移、旋转和倒退，最终检查HOLD。真值仅用于评估。回归门限是故障检测门限，不是真机精度承诺；最新实测与残余边界见[定位验收记录](../../docs/archive/integration/LOCALIZATION_IMPLEMENTATION.md)。
