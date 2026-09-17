# 道末采集关闭握手把整车刹停

2026-09-16 定位，2026-09-17 列为**要修**。已定位到具体代码行与发生率，改法见下，尚未实施。数值见
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

## 决定修（2026-09-17）

操作员在全区跑的第6道末再次看到，确认与09-14/09-15同形。列为要修项。

## 具体改法（一处条件，已核对上下文，尚未实施）

`execution_node.py:295`：

```python
if self.capture.future is not None and (self.core is None or not self.capture.target):
```
改为
```python
if self.capture.future is not None and self.core is None:
```

即把"开"方向已有的豁免对称地给"关"。**"关必须落定才进下一段"这条约束不会丢**，因为：

- 新段只能从`if self.core is None:`分支里建（`SegmentTracker(...)`），而该分支在
  future未决时仍然halt——保证成立。
- PASS段中途（`self.core`活着）请求关闭时不再halt，跟踪器按既定减速曲线跑到终点；
  段完成时`self.core=None`（346行），若此时future仍未决，下一拍在**段边界**halt，
  而那时车已经停稳，不会触发`Mode::Brake`。
- `track_end`每拍重复请求是无害的：`capture.py:56`在future未决时直接返回False。

不涉及`swerve.hpp`的Brake锁存——**不要**去放宽那个，`STOP_REQUEST`必须及时。
暂停/恢复路径（136行同样置`core=None`）不受影响。

## 怎么验证（这一条比改法更要紧）

这是服务往返时延与一个20 ms控制拍的**竞态**，发生率约17%。

**全区跑不是合适的验证手段**：一轮只有10个道末，按17%期望约1.7次，修好与运气好
区分不开。而且本仓库任务级不可逐字节复现（见
[场景载入时间](SCENE_LOAD_TIME.md)），不能做运行间逐项对照，只能比**发生率**。

建议两条一起做：

1. **换个能高频触发的靶子**：`--length 3 --width 10`即10道×3 m，两分钟就能收10个道末，
   跑几轮即可攒够30–50个样本，比全区快一个数量级。
2. **直接量竞态本身，而不是数症状**：记录每次关闭握手的往返时长，以及PASS段内是否
   锁存过`Mode::Brake`/`STOP_REQUEST`。有了这两个量，改动前后比的是"握手超过一拍的
   比例"和"超过一拍时是否仍然刹停"，不必靠速度曲线反推。

判据建议：改动后，握手超过一拍的比例可以不变（那是服务时延，本来就不归这条改动管），
但**超过一拍时不再出现PASS段内的Brake锁存**。
