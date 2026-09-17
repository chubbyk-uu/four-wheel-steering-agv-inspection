# 从 WSLg 抓取 GZ / RViz 窗口

`tools/capture_wslg_window.ps1` 是 Windows 侧的窗口抓取脚本，供在 WSL2/WSLg 下为文档
截图和录制循环动图使用。

**为什么不用 Linux 侧的录屏工具**：WSLg 的 GUI 应用是 XWayland 客户端，`ffmpeg -f x11grab`
对 `:0` 抓到的是**全黑帧**（实测 1347×845 的 PNG 只有 3.4 KB）。WSLg 会把每个 Linux 窗口
映射成一个真实的 Windows 顶层窗口，所以从 Windows 侧抓才有画面。

需要 WSL interop 可用（`/proc/sys/fs/binfmt_misc/WSLInterop` 为 enabled）。

```bash
# 单张：按窗口标题子串匹配，抓到 Windows 路径
powershell.exe -NoProfile -ExecutionPolicy Bypass -File 'C:\path\capture_wslg_window.ps1' \
  -Match 'Gazebo Sim' -Out 'C:\Temp\shot' -Count 1

# 连拍：150 帧、间隔 20 ms（实测约 10 fps，含 PNG 编码开销）
powershell.exe -NoProfile -ExecutionPolicy Bypass -File 'C:\path\capture_wslg_window.ps1' \
  -Match 'Gazebo Sim' -Out 'C:\Temp\frames' -Count 150 -IntervalMs 20
```

脚本会把窗口提到前台再抓，并按虚拟桌面范围裁掉屏幕外部分。抓到的是**窗口矩形**，
含标题栏和边框，也可能带到背后其他窗口的边缘，合成前需要裁剪；`docs/images/` 里
Gazebo 视口用的裁剪框是 `(40,105,1058,905)`。

合成动图在 Linux 侧用 PIL：120 帧、10 fps、64 色、宽 560 得到约 6.8 MB 的 12 秒循环。

**注意**：抓的是当前屏幕内容，窗口被遮挡就会拍到遮挡物。抓图期间不要在同一台机器上
运行会杀仿真进程的命令。
