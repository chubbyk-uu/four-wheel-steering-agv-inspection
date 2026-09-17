# 道末采集关闭握手把整车刹停

2026-09-16 定位，2026-09-17 **已修并验证**：改前改后各 4 轮高频靶子、各 40 个道末，关闭握手
超过一拍的 10 次→22 次，其中锁存 Brake 的 **10 次→0 次**。定位数值见
[握手刹车记录](../../results/capture_close_handshake_brake.json)，修复与验证见
[修复结果](../../results/capture_close_handshake_fix.json)。

操作员在GUI里看到的现象是：跑到一道末尾时车提前刹车、几乎停住，又往前挪一小段，再停一次。
不是每道都有，隔几道出现一次。（下文"机理""发生率"两节记录的是修复前的状态。）

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

## 已修（2026-09-17）

`execution_node.py` 那一处条件：

```python
if self.capture.future is not None and (self.core is None or not self.capture.target):
```
改为
```python
if self.capture.future is not None and self.core is None:
```

即把"开"方向已有的豁免对称地给"关"。**"关必须落定才进下一段"这条约束没有丢**：新段只能从
`if self.core is None:` 分支里建（`SegmentTracker(...)`），而该分支在 future 未决时仍然 halt。
PASS 段中途（`self.core` 活着）请求关闭时不再 halt，跟踪器按既定减速曲线跑到终点；段完成时
`self.core=None`，若此时 future 仍未决，下一拍在**段边界** halt，而那时车已经停稳，不触发
`Mode::Brake`。`track_end` 每拍重复请求无害：`capture.py:56` 在 future 未决时直接返回 False。

没有动 `swerve.hpp` 的 Brake 锁存——`STOP_REQUEST` 必须及时。暂停/恢复路径不受影响
（`resume()` 本来就要求 `capture.future is None`）。

单元测试 `src/agv_mission/test/test_capture_handshake_halt.py` 共 5 例：**旧条件下挂 2 例**，
另外 3 例新旧都过——那三例正是这次改动不许破坏的不变量（开不许刹住行进中的 PASS、无握手的
一拍原样通过、握手未决时段边界仍然 halt）。

## 怎么量的：两个已在归档里的量

这是服务往返时延与一个 20 ms 控制拍的**竞态**，数"刹了几次"量到的一半是当天的调度。直接量
竞态本身的两个量 `execution_node` **每拍本来就记**，不需要加任何控制路径代码：

- `capture_pending` 连续为真的拍数 = 握手时长；
- `kind=='PASS'` 且 `motion_state=='BRAKE'` 且 `motion_reason=='STOP_REQUEST'` = 刹车锁存。

握手落在一拍之内不可能刹住任何东西：future 是在该拍指令发出之后才置上的，所以**拍数 −1
就是执行器停在 halt 分支的拍数**。工具见 `tools/analyze_capture_handshake.py`，
`--expect-no-close-latch` 给判据；样本里若一次都没超过一拍，它同样报错退出——那种样本区分
不了"修好了"和"运气好"。

## 靶子为什么用 3 m

`planner.py` 的 `lead = v²/2a + margin`、`runout = max(v²/2d, overrun) + margin` **都不含区域
长度**，而一个 PASS 步是 ACCELERATE+SCAN+RUNOUT_BRAKE 合成的单条停-停梯形曲线，总行程
lead + length + runout。2.7778 m/s 下 lead 4.9225 m、中间道 runout 7.2225 m（末道 4.9225 m）、
加减速斜坡 9.6451 m，总行程 12.85–15.15 m：**满速一定跑到**，关闭时的状态（中间道剩
6.6568 m、速度 2.7778 m/s；末道剩 4.3568 m、2.6403 m/s）与 100 m 任务**逐位相同**。
差别只在巡航段 1.08 s 而不是 36 s，十个道末四分钟收齐而不是十三分钟。实测峰值速度
2.944–2.950 m/s，确认跑到全速。

## 验证结果

`tools/validate_operator_session.py --start-x 6 --start-y -5 --spawn-x 3 --length 3 --width 10
--speed 2.777777777777778 --rounds 4 --no-cancel-probe`，改前改后各 4 轮、各 40 个道末：

| | 关闭握手 | 超过一拍 | 其中锁存 Brake | PASS 段持续 ≥0.4 s 的刹车 |
|---|---:|---:|---:|---:|
| 改前 | 40 | 10 | **10** | **10**（每次 140–142 拍 ≈ 2.8 s） |
| 改后 | 40 | 22 | **0** | **0** |

打开方向两组都是 40/40 超过一拍、0 次刹停——豁免本来就在那一侧生效。改前那 10 次每次
140–142 拍，正是 2.78 m/s 滑行到停的 2.8 s，与机理预测一致。

五次历史全区跑（50 个道末）用同一工具复算：关闭超过一拍 6 次、**6 次全刹**，道号与本文
上面手工数出来的完全一致（第 5、9 道和第 1、2、6 道）。

数据质量两组一致：每一轮都 ESTIMATED_COMPLETE、无质量标志、无未验证区间；稳态 RTF
改前 0.9920、改后 0.9905。

**两组不是"超过一拍比例"的受控对照**：改前那一组是带着两个上一次录 GIF 会话遗留的孤儿节点
跑的（`mission_operator` 与 `execution_node`，各占 12–15% CPU），两组之间才发现并清掉。
比例本来就不归这条改动管。值得注意的是改后一组比例**更高**（22/40 对 10/40）——一种可能是
修好之后车在握手期间继续行驶，相机保持 7.59 kHz 采样，而改前的 halt 减轻了这份负载；这是
猜测不是测量。无论如何这让结论偏保守：改后暴露量是改前的两倍多，仍然零次锁存。

操作员在改前那一组运行时于 GUI 里独立看到其中一次刹停，与归档记录的是同一件事。

完整数值见[修复结果](../../results/capture_close_handshake_fix.json)。
