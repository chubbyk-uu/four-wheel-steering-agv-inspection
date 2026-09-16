# 道末采集关闭握手把整车刹停

2026-09-16。**未修复**，已定位到具体代码行与发生率。数值见
[握手刹车记录](../../results/capture_close_handshake_brake.json)。

操作员在GUI里看到的现象是：跑到一道末尾时车提前刹车、几乎停住，又往前挪一小段，再停一次。
不是每道都有，隔几道出现一次。

## 机理

执行器在相机越过区域加过扫距离后请求关闭采集（`execution_node.py:316`）。**请求在途期间，
执行器直接发零速度并提前返回**：

```python
# execution_node.py:295
if self.capture.future is not None and (self.core is None or not self.capture.target):
    self.command([0,0,0]);return
```

该分支在`core.update()`之前返回，所以跟踪器这一拍不更新，`tracker_state`停在RUNNING、
`goal_error_m`冻结——归档里看到的指令冻结就是这么来的。

握手若在一个控制拍（20 ms）内完成，什么都不会发生。**超过一拍**，摆臂控制器就在DRIVE态
看到零轮速请求，锁存`Mode::Brake`、原因`STOP_REQUEST`（`swerve.hpp:143`）；而Brake只有在
**车轮真正停住**之后才释放（`swerve.hpp:157`）。从2.78 m/s停下要约2.8 s、3.9 m。

恢复后`was_running`为真，跟踪器按剩余距离**重建一条静止到静止的曲线**
（`tracking.py:210`），于是二次加速到约1.5 m/s、走完剩余2.6 m、再停一次。

## 这件事代码里已经知道，但只修了一半

`execution_node.py:292`的注释原话：

> An open now happens while a pass is already driving its lead-in, and must not brake the
> vehicle for the duration of a handshake.

"开"的方向用条件里的`not self.capture.target`豁免掉了，**"关"的方向仍然照旧叫停整段**。

## 发生率

统计口径为PASS段内既非终点停车、也非验证脚本暂停探针、也非故障刹车的刹停：

| 运行 | 道末数 | 事件 | 发生在 |
|---|---:|---:|---|
| `rough100_v2_full_rated_1`（09-16） | 10 | 3 | 第1、2、6道 |
| `rough100_full_rated_4`（09-15） | 10 | 2 | 第5、9道 |
| `accept_rated_6`（09-14） | 10 | 0 | — |

合计30个道末中5次，约17%。是服务往返时延的**竞态**，与具体哪一道无关。每次代价：
滑行3.877–3.941 m、2.76–2.78 s，整道多花约2–3 s。

不计入统计的两类：每轮第0道的`PASSING`/`PAUSED`是
`tools/validate_operator_session.py`故意做的暂停保帧探针（滑行约2.2 m）；
`rough100_v2_full_rated_1`第9道754.676 s那次是采集运动故障本身导致的停车。

## 影响与边界

**未观察到对采集数据的影响**：刹车那一拍`capture_active`已经是false，车已越过ROI进入
缓冲区。代价是时间，以及一次轨迹完全没有预算的全权限制动。

**不是新引入的**：09-14和09-15的全区跑里都在。

## 两个修法方向（均未实施）

1. 把"开"方向已有的豁免延伸到"关"——过了ROI本来就在减速段，不需要为握手再叫停。
2. 保留"关必须落定才进下一段"的约束，但把它挪到**段切换处**，而不是压在PASS段中间。
