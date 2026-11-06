param(
  [Parameter(Mandatory = $true)]
  [string]$LauncherPath,
  [string]$OutputDirectory = (Join-Path $PSScriptRoot '..\bin\Windows\Release\boot-captures'),
  [int[]]$CaptureDelaySeconds = @(1),
  [switch]$CaptureExisting,
  [string]$LogPath
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $LauncherPath -PathType Leaf)) {
  throw "Launcher not found: $LauncherPath"
}

New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$sessionTimestamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
if ($LogPath -and -not $CaptureExisting -and
    (Test-Path -LiteralPath $LogPath -PathType Leaf)) {
  $previousLog = Join-Path $OutputDirectory "xenia-boot-$sessionTimestamp-before.log"
  Move-Item -LiteralPath $LogPath -Destination $previousLog -Force
}

$commandProcess = $null
if (-not $CaptureExisting) {
  $launchCommand = '/d /c ""' + $LauncherPath + '""'
  $commandProcess = Start-Process -FilePath 'cmd.exe' -ArgumentList $launchCommand -PassThru
}

Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms
Add-Type @'
using System;
using System.Runtime.InteropServices;

public static class XenectCaptureNative {
  [StructLayout(LayoutKind.Sequential)]
  public struct RECT {
    public int Left;
    public int Top;
    public int Right;
    public int Bottom;
  }

  [DllImport("user32.dll", SetLastError = true)]
  public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);

  [DllImport("user32.dll")]
  public static extern bool BringWindowToTop(IntPtr hWnd);

  [DllImport("user32.dll")]
  public static extern bool SetForegroundWindow(IntPtr hWnd);
}
'@

$virtualScreen = [System.Windows.Forms.SystemInformation]::VirtualScreen
$left = $virtualScreen.Left
$top = $virtualScreen.Top
$width = $virtualScreen.Width
$height = $virtualScreen.Height

function Save-Capture([int]$DelaySeconds) {
  $xenia = Get-Process -Name xenia_canary -ErrorAction SilentlyContinue |
    Select-Object -First 1
  $captureScope = 'virtual-screen'
  $captureTitle = $null
  $captureLeft = $left
  $captureTop = $top
  $captureWidth = $width
  $captureHeight = $height
  if ($xenia -and $xenia.MainWindowHandle -ne 0) {
    $captureTitle = $xenia.MainWindowTitle
    [XenectCaptureNative]::BringWindowToTop($xenia.MainWindowHandle) | Out-Null
    [XenectCaptureNative]::SetForegroundWindow($xenia.MainWindowHandle) | Out-Null
    $windowRect = New-Object XenectCaptureNative+RECT
    if ([XenectCaptureNative]::GetWindowRect(
        $xenia.MainWindowHandle, [ref]$windowRect) -and
        $windowRect.Right -gt $windowRect.Left -and
        $windowRect.Bottom -gt $windowRect.Top) {
      $captureScope = 'xenia-window'
      $captureLeft = $windowRect.Left
      $captureTop = $windowRect.Top
      $captureWidth = $windowRect.Right - $windowRect.Left
      $captureHeight = $windowRect.Bottom - $windowRect.Top
    }
  }

  $bitmap = New-Object System.Drawing.Bitmap $captureWidth, $captureHeight
  $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
  try {
    $graphics.CopyFromScreen($captureLeft, $captureTop, 0, 0, $bitmap.Size)
    $capturePath = Join-Path $OutputDirectory (
      "xenia-boot-$sessionTimestamp-$DelaySeconds" + 's.png')
    $bitmap.Save($capturePath, [System.Drawing.Imaging.ImageFormat]::Png)
  } finally {
    $graphics.Dispose()
    $bitmap.Dispose()
  }

  [pscustomobject]@{
    CapturePath = $capturePath
    CaptureDelaySeconds = $DelaySeconds
    XeniaRunning = [bool]$xenia
    XeniaProcessId = if ($xenia) { $xenia.Id } else { $null }
    XeniaWindowTitle = $captureTitle
    CaptureScope = $captureScope
  }
}

$captures = @()
$elapsedSeconds = 0
foreach ($delaySeconds in ($CaptureDelaySeconds | Sort-Object -Unique)) {
  if ($delaySeconds -lt 0) {
    throw 'Capture delays must not be negative.'
  }
  Start-Sleep -Seconds ($delaySeconds - $elapsedSeconds)
  $elapsedSeconds = $delaySeconds
  $captures += Save-Capture $delaySeconds
}

$sessionLog = $null
if ($LogPath -and (Test-Path -LiteralPath $LogPath -PathType Leaf)) {
  $sessionLog = Join-Path $OutputDirectory "xenia-boot-$sessionTimestamp.log"
  Copy-Item -LiteralPath $LogPath -Destination $sessionLog -Force
}

[pscustomobject]@{
  Captures = $captures
  SessionLog = $sessionLog
  LauncherProcessId = if ($commandProcess) { $commandProcess.Id } else { $null }
}
