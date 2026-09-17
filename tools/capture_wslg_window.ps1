param([string]$Match, [string]$Out, [int]$Count=1, [int]$IntervalMs=0)
Add-Type -AssemblyName System.Drawing
Add-Type @"
using System;using System.Text;using System.Runtime.InteropServices;
public class W {
  public delegate bool Proc(IntPtr h, IntPtr l);
  [DllImport("user32.dll")] public static extern bool EnumWindows(Proc p, IntPtr l);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowTextW(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out R r);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  public struct R { public int L,T,Rt,B; }
}
"@
$found=[IntPtr]::Zero; $rect=New-Object W+R
$cb=[W+Proc]{ param($h,$l)
  if([W]::IsWindowVisible($h)){
    $sb=New-Object System.Text.StringBuilder 512
    [void][W]::GetWindowTextW($h,$sb,512)
    if($sb.ToString() -like "*$Match*" -and $script:found -eq [IntPtr]::Zero){
      $script:found=$h; [void][W]::GetWindowRect($h,[ref]$script:rect) } }
  return $true }
[void][W]::EnumWindows($cb,[IntPtr]::Zero)
if($found -eq [IntPtr]::Zero){ Write-Output "NOTFOUND"; exit 1 }
[void][W]::SetForegroundWindow($found); Start-Sleep -Milliseconds 400
[void][W]::GetWindowRect($found,[ref]$rect)
$sw=[System.Windows.Forms.SystemInformation]::VirtualScreen
Add-Type -AssemblyName System.Windows.Forms
$sw=[System.Windows.Forms.SystemInformation]::VirtualScreen
$x=[Math]::Max($rect.L,$sw.X); $y=[Math]::Max($rect.T,$sw.Y)
$w=[Math]::Min($rect.Rt,$sw.Right)-$x; $h=[Math]::Min($rect.B,$sw.Bottom)-$y
Write-Output ("RECT {0},{1},{2},{3}" -f $x,$y,$w,$h)
for($i=0;$i -lt $Count;$i++){
  $bmp=New-Object System.Drawing.Bitmap $w,$h
  $g=[System.Drawing.Graphics]::FromImage($bmp)
  $g.CopyFromScreen($x,$y,0,0,(New-Object System.Drawing.Size($w,$h)))
  $bmp.Save(("{0}\{1:d4}.png" -f $Out,$i),[System.Drawing.Imaging.ImageFormat]::Png)
  $g.Dispose(); $bmp.Dispose()
  if($IntervalMs -gt 0){ Start-Sleep -Milliseconds $IntervalMs }
}
Write-Output "OK $Count"
