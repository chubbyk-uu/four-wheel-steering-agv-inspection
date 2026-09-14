# 仿真关闭时段错误，WSL把整份core写进C盘

**状态**：2026-09-14确认。关闭时的段错误来自Gazebo自身的退出流程，不是本项目插件；
真正的危害是WSL默认把整个地址空间转储到Windows C盘，**转储大小约等于进程RSS**。
转储抑制已验证：`maxCrashDumpCount=-1`并重启WSL；`0`无效。Gazebo退出段错误本身仍未修复，不能把停止写转储当成修复崩溃。
**历史影响**：曾威胁第6节长任务验收并消耗C盘空间。转储抑制和Mesa修复后第6节已通过，见[收尾记录](../../results/section6_closure.json)；下文保留原高内存工况。

---

## 现象

用户发现Windows C盘持续减少，来源是`/usr/bin/ruby3.2`不断向C盘临时目录写崩溃转储。

`ruby3.2`**就是`gz sim`的服务端进程**。`gz`是ruby脚本，进程映像是ruby，脚本改写了`$0`，
所以`ps`显示`gz sim -s -r ...`而`%E`记录的可执行文件是`/usr/bin/ruby3.2`。不是ruby本身在崩。

转储路径由WSL的内核core管道决定：

```
/proc/sys/kernel/core_pattern = |/wsl-capture-crash %t %E %p %s
```

文件落在`%LOCALAPPDATA%\Temp\wsl-crashes\`，命名为`wsl-crash-<时间>-<pid>-<可执行文件>-<信号>.dmp`，
末尾的`-11`是SIGSEGV。文件本身是标准ELF core（首字节`7f 45 4c 46`）。

## 崩溃点

从2.95 GB的core里直接解析NT_PRSTATUS与NT_FILE（本机无gdb）：崩溃线程RIP落在
`libgcc_s.so.1`的异常展开器内（`_Unwind_GetTextRelBase`与`_Unwind_RaiseException`之间），
80个线程，崩溃线程的栈VMA为8 MiB而RSP距栈顶仅8 KiB——**是线程入口附近，不是栈溢出**。

复现时拿到Gazebo自带的回溯，指向同一处：

```text
Stack trace (most recent call last) in thread 56727:
 #2  /lib/x86_64-linux-gnu/libc.so.6        (clone3)
 #1  /lib/x86_64-linux-gnu/libc.so.6        (start_thread)
 #0  Object "[0x7eaab8432b10]", at 0x7eaab8432b10
Segmentation fault (Address not mapped to object [0x7eaab8432b10])
```

栈底两帧是libc的线程启动，最上面一帧**不属于任何已映射对象**。即关闭过程中某个库被卸载，
而仍有线程在其代码里运行，下一条指令落到已经unmap的页上——dlclose与存活线程的竞争。
core里的映射表是崩溃时刻的快照，那个库已经不在表内，因此**无法从中指认是哪一个库**。

## 触发范围（两项对照）

| 对照 | 结果 |
|---|---|
| 手工`pkill`（SIGTERM直发服务端） | 段错误，写出2.95 GB转储 |
| 进程组SIGINT，即验收脚本的正常关闭路径（`tools/validate_rendered_linescan.py:392`的`os.killpg(sim.pid, SIGINT)`） | **同样段错误**，SIGINT后5秒未退被launch升级为SIGTERM |
| `linescan:=false linescan_backend:=render`，不加载本项目线阵插件 | **同样段错误**，同一"Address not mapped to object"签名 |

所以这不是`agv_linescan`的问题，也不是某种粗暴关闭方式独有；**正常验收流程每次关闭都会触发**。

## 体积与后果

转储大小约为进程RSS的0.7–0.8倍，实测：

| 服务端RSS | 转储大小 |
|---:|---:|
| 4.81 GB（静止100 m场景） | 2.93 GB |
| 4.19 GB（同场景，无线阵插件） | 3.04 GB |

按此外推，**全区任务峰值20–30 GiB会产生约16–24 GB的单次转储**。WSL默认`maxCrashDumpCount=10`，
即最多保留10份。2026-09-14当天C盘余量一度只剩31 GB，而第6节待跑的是额定档×3加0.5 m/s档共4次长任务。

## 抑制手段：0是最坏的取值，-1才关闭

**`ulimit -c 0`无效**。用一个必定段错误的最小程序测试：软限制为0的shell里运行，内核仍报
`(core dumped)`并生成转储。core_pattern是管道时不按RLIMIT_CORE抑制。

**`.wslconfig`的`maxCrashDumpCount=0`无效，而且是所有取值里最坏的一个**。
设置后`wsl --shutdown`重启，再连续制造4次段错误，`wsl-crashes`目录**逐次累积到4份**，无删除也无上限。

原因在[WSL源码](https://github.com/microsoft/WSL)里是确定的，不是推测：

- `WslCoreVm.cpp`启动采集线程和注入内核参数的条件都是`MaxCrashDumpCount >= 0`
  （`kernelCmdLine += " WSL_ENABLE_CRASH_DUMP=1"`）。本机`/proc/cmdline`里确实带着`WSL_ENABLE_CRASH_DUMP=1`。
- `linux/init/main.cpp`只在看到该环境变量时调用`EnableCrashDumpCollection()`，也就是设置那条管道`core_pattern`。
- `wslutil.cpp`的`EnforceFileLimit`开头是`if (Limit <= 0) return;`——**0不是"保留0份"，而是跳过清理**。
  它每次只删一个最旧文件，所以正数N把目录大致维持在N份。
- 相关修复[PR #13755](https://github.com/microsoft/WSL/pull/13755)正是"`maxCrashDumpCount`设为0时服务崩溃"。

于是三种取值的实际语义是：

| 取值 | 采集 | 清理 | 结果 |
|---|---|---|---|
| 正数N（默认10） | 开 | 保留约N份 | 每次长任务仍写一份16–24 GB |
| **0** | 开 | **跳过** | **无限累积，最坏** |
| **负数（-1）** | **关** | 不适用 | 内核参数不注入，`core_pattern`不改成管道，**根本不产生转储** |

本机已改为`maxCrashDumpCount=-1`（`%USERPROFILE%\.wslconfig`），`wsl --shutdown`后**2026-09-14实测生效**：

- `/proc/cmdline`不再含`WSL_ENABLE_CRASH_DUMP=1`；
- `/proc/sys/kernel/core_pattern`由`|/wsl-capture-crash %t %E %p %s`变为普通的`core`；
- 同一个必定段错误的最小程序再跑一次，`wsl-crashes`目录为空，工作目录也没有`core`文件
  （默认软限制`ulimit -c`为0，管道一撤掉这个限制就重新生效），shell的提示也由
  `Segmentation fault (core dumped)`变成`Segmentation fault`。

另有`wsl2.crashDumpFolder`可把目录挪到别的盘（源码`WslCoreConfig.cpp`），本项目不需要，未使用。

同类报告：[microsoft/WSL#41314](https://github.com/microsoft/WSL/issues/41314)，重复写出58–65 GiB转储耗尽C盘，被标记为bydesign。

## 未覆盖

- 未定位被提前卸载的具体库，也未向Gazebo上游报告；这是退出流程缺陷，不影响任何采集数据
  （崩溃发生在采集与归档全部结束之后），但每次关闭要多花5秒等SIGTERM升级。
- 正数取值只做过源码阅读，未实测其清理行为。
- 若将来必须保留转储，可改用`crashDumpFolder`挪到非系统盘，该路径未验证。
- 未统计历史上已经因此写掉多少C盘空间。
