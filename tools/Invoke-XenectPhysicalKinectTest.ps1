param(
  [Parameter(Mandatory = $true)]
  [string]$GamePath,
  [ValidateSet('KinectSports', 'KinectAdventures', 'DanceCentral3', 'JustDance2014')]
  [string]$Game = 'KinectSports',
  [ValidateRange(15, 1800)]
  [int]$DurationSeconds = 150,
  [ValidateRange(5, 60)]
  [int]$CaptureIntervalSeconds = 15,
  [ValidateSet('PrintWindow', 'Desktop')]
  [string]$CaptureMode = 'PrintWindow',
  [ValidateRange(0, 3)]
  [int]$LogLevel = 2,
  [string]$TraceGuestFunctions = '',
  [ValidateSet('d3d12', 'vulkan')]
  [string]$GraphicsSystem = 'd3d12',
  [ValidateSet('any', 'rtv', 'rov')]
  [string]$RenderTargetPath = 'rov',
  [ValidateSet('none', 'fast', 'full')]
  [string]$ReadbackResolve = 'fast',
  [bool]$ReadbackMemexport = $false,
  [bool]$D3D12Bindless = $false,
  [bool]$StoreShaders = $false,
  [ValidateSet('fake', 'master', 'old', 'new')]
  [string]$XmaDecoder = 'new',
  [bool]$UseDedicatedXmaThread = $true,
  [bool]$SyntheticKinect = $false,
  [ValidateRange(0, 1)]
  [int]$DepthByteOrder = 0,
  [ValidateRange(0, 1)]
  [int]$DepthFormat = 0,
  [ValidateRange(0, 300)]
  [int]$SyntheticBootstrapSeconds = 0,
  [bool]$DumpCoordinateMapper = $false,
  [bool]$DumpAvatarAssets = $false,
  [bool]$AllowAvatarInitialization = $true,
  [ValidateRange(0, 1800)]
  [int]$GpuTraceAtSeconds = 0,
  [bool]$GpuStreamTrace = $false,
  [string]$XeniaPath,
  [string]$DiagnosticsRoot
)

$ErrorActionPreference = 'Stop'
if ($GpuStreamTrace -and $GpuTraceAtSeconds -gt 0) {
  throw 'GpuStreamTrace and GpuTraceAtSeconds cannot be enabled together.'
}

$runtime_directory = Join-Path $PSScriptRoot '..\bin\Windows\Release'
$xenia_path = if ($XeniaPath) {
  [System.IO.Path]::GetFullPath($XeniaPath)
} else {
  Join-Path $runtime_directory 'xenia_canary.exe'
}
$xenia_directory = Split-Path -Parent $xenia_path
$diagnostics_directory = if ($DiagnosticsRoot) {
  [System.IO.Path]::GetFullPath($DiagnosticsRoot)
} else {
  Join-Path $runtime_directory 'diagnostics'
}

if (-not (Test-Path -LiteralPath $xenia_path -PathType Leaf)) {
  throw "Xenia executable not found: $xenia_path"
}

$game_path = [System.IO.Path]::GetFullPath($GamePath)
if (-not (Test-Path -LiteralPath $game_path)) {
  throw "Game image not found: $game_path"
}

if (Get-Process -Name xenia_canary -ErrorAction SilentlyContinue) {
  throw 'A xenia_canary.exe process is already active. Close it before starting a physical Kinect test.'
}

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$session_directory = Join-Path $diagnostics_directory ("physical-$($Game.ToLowerInvariant())-$stamp")
$captures_directory = Join-Path $session_directory 'captures'
$log_path = Join-Path $session_directory 'xenia.log'
$gpu_trace_directory = Join-Path $session_directory 'gpu-trace'
$coordinate_mapper_dump_path = Join-Path $session_directory 'coordinate-mapper.bin'
$avatar_asset_dump_path = Join-Path $session_directory 'avatar-assets'
$session_config_path = Join-Path $session_directory 'xenect-session.config.toml'
New-Item -ItemType Directory -Force -Path $captures_directory | Out-Null
if ($GpuTraceAtSeconds -gt 0 -or $GpuStreamTrace) {
  New-Item -ItemType Directory -Force -Path $gpu_trace_directory | Out-Null
}
# The Xenia global config is loaded after command-line cvars. Keep the fixture
# setting in an isolated per-run config so the command cannot be overwritten by
# the user's persistent config, and so a physical run never changes it.
@"
[APU]
mute = true

[Kinect]
kinect_synthetic_test_frames = $($SyntheticKinect.ToString().ToLowerInvariant())

[Kernel]
allow_avatar_initialization = $($AllowAvatarInitialization.ToString().ToLowerInvariant())

[Profiles]
# The isolated test config replaces the persistent config passed to Xenia.
# Keep the existing offline profile attached to slot 0 so Kinect titles can
# associate a tracked skeleton with an actual signed-in local user.
logged_profile_slot_0_xuid = "E03000001DE0C750"
"@ | Set-Content -LiteralPath $session_config_path -Encoding ascii

Add-Type -TypeDefinition @'
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;

public static class XenectPhysicalCapture {
  [StructLayout(LayoutKind.Sequential)]
  public struct RECT { public int Left, Top, Right, Bottom; }
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);
  [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hWnd, IntPtr hdc, uint flags);
  [DllImport("user32.dll")] public static extern bool PostMessage(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam);
  [DllImport("user32.dll", CharSet = CharSet.Unicode)] public static extern IntPtr FindWindow(string className, string windowName);
  [DllImport("user32.dll")] public static extern bool ShowWindowAsync(IntPtr hWnd, int command);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
  public static bool RestoreAndFocus(IntPtr window) {
    if (window == IntPtr.Zero) return false;
    ShowWindowAsync(window, 9); // SW_RESTORE
    return SetForegroundWindow(window);
  }
  public static bool DismissFirstRunDisclaimer() {
    // This is the only modal dialog titled Xenia before the emulator creates
    // its main window. Select "No" so the optional browser is never opened.
    IntPtr dialog = FindWindow("#32770", "Xenia");
    return dialog != IntPtr.Zero && PostMessage(dialog, 0x0111, new IntPtr(7), IntPtr.Zero);
  }
  public static bool SaveWindow(IntPtr window, string path) {
    RECT rect;
    if (window == IntPtr.Zero || !GetWindowRect(window, out rect)) return false;
    int width = rect.Right - rect.Left, height = rect.Bottom - rect.Top;
    if (width <= 0 || height <= 0) return false;
    using (var bitmap = new Bitmap(width, height, PixelFormat.Format32bppArgb))
    using (var graphics = Graphics.FromImage(bitmap)) {
      IntPtr hdc = graphics.GetHdc();
      bool result = PrintWindow(window, hdc, 2);
      graphics.ReleaseHdc(hdc);
      if (result) bitmap.Save(path, ImageFormat.Png);
      return result;
    }
  }
  public static bool SaveDesktop(IntPtr window, string path) {
    RECT rect;
    if (window == IntPtr.Zero || !GetWindowRect(window, out rect)) return false;
    int width = rect.Right - rect.Left, height = rect.Bottom - rect.Top;
    if (width <= 0 || height <= 0) return false;
    using (var bitmap = new Bitmap(width, height, PixelFormat.Format32bppArgb))
    using (var graphics = Graphics.FromImage(bitmap)) {
      graphics.CopyFromScreen(rect.Left, rect.Top, 0, 0, bitmap.Size);
      bitmap.Save(path, ImageFormat.Png);
      return true;
    }
  }
  public static bool RequestGpuFrameTrace(IntPtr window) {
    const uint WM_KEYDOWN = 0x0100;
    const uint WM_KEYUP = 0x0101;
    const int VK_F4 = 0x73;
    if (window == IntPtr.Zero) return false;
    bool down = PostMessage(window, WM_KEYDOWN, new IntPtr(VK_F4), IntPtr.Zero);
    bool up = PostMessage(window, WM_KEYUP, new IntPtr(VK_F4), IntPtr.Zero);
    return down && up;
  }
  public static bool RequestClose(IntPtr window) {
    const uint WM_CLOSE = 0x0010;
    return window != IntPtr.Zero && PostMessage(window, WM_CLOSE, IntPtr.Zero, IntPtr.Zero);
  }
}
'@ -ReferencedAssemblies System.Drawing.dll

$arguments = @(
  ('--config={0}' -f $session_config_path),
  ('--log_level={0}' -f $LogLevel),
  ('--log_file="{0}"' -f $log_path),
  ('--gpu={0}' -f $GraphicsSystem),
  ('--readback_resolve={0}' -f $ReadbackResolve),
  ('--readback_memexport={0}' -f $ReadbackMemexport.ToString().ToLowerInvariant()),
  ('--store_shaders={0}' -f $StoreShaders.ToString().ToLowerInvariant()),
  ('--xma_decoder={0}' -f $XmaDecoder),
  ('--use_dedicated_xma_thread={0}' -f $UseDedicatedXmaThread.ToString().ToLowerInvariant()),
  '--mute=true',
  '--kinect_software_near_mode=false',
  '--kinect_seated_tracking=false',
  ('--kinect_synthetic_test_frames={0}' -f $SyntheticKinect.ToString().ToLowerInvariant()),
  ('--kinect_synthetic_bootstrap_seconds={0}' -f $SyntheticBootstrapSeconds),
  ('--kinect_pscam_depth_format={0}' -f $DepthFormat),
  ('--kinect_pscam_depth_byte_order={0}' -f $DepthByteOrder),
  '--kinect_force_zero_elevation=true'
)

if ($TraceGuestFunctions) {
  $arguments += ('--trace_guest_functions={0}' -f $TraceGuestFunctions)
}

if ($GpuTraceAtSeconds -gt 0 -or $GpuStreamTrace) {
  # The session path has no spaces. Passing it bare avoids Start-Process
  # preserving quote characters inside the cvar value.
  $arguments += ('--trace_gpu_prefix={0}' -f $gpu_trace_directory)
}
if ($GpuStreamTrace) {
  $arguments += '--trace_gpu_stream=true'
}
if ($DumpCoordinateMapper) {
  $arguments += ('--kinect_coordinate_mapper_dump_path={0}' -f $coordinate_mapper_dump_path)
}
if ($DumpAvatarAssets) {
  $arguments += ('--avatar_asset_dump_path={0}' -f $avatar_asset_dump_path)
}

if ($GraphicsSystem -eq 'd3d12') {
  $arguments += ('--render_target_path_d3d12={0}' -f $RenderTargetPath)
  $arguments += ('--d3d12_bindless={0}' -f $D3D12Bindless.ToString().ToLowerInvariant())
}
$arguments += ('"{0}"' -f $game_path)

$started = Get-Date
$xenia = Start-Process -FilePath $xenia_path -ArgumentList $arguments -WorkingDirectory $xenia_directory -PassThru
for ($attempt = 0; $attempt -lt 10; $attempt++) {
  if ([XenectPhysicalCapture]::DismissFirstRunDisclaimer()) {
    Write-Output 'Dismissed the Xenia first-run disclaimer.'
    break
  }
  Start-Sleep -Milliseconds 250
}
$capture_count = 0
$next_capture = $started.AddSeconds(1)
$next_crash_scan = $started.AddSeconds(1)
$log_scan_offset = 0L
$deadline = $started.AddSeconds($DurationSeconds)
$gpu_trace_deadline = if ($GpuTraceAtSeconds -gt 0) {
  $started.AddSeconds($GpuTraceAtSeconds)
} else {
  [DateTime]::MaxValue
}
$gpu_trace_requested = $false
$gpu_trace_request_succeeded = $false
$stop_reason = 'deadline'

try {
  while ((Get-Date) -lt $deadline) {
    $process = Get-Process -Id $xenia.Id -ErrorAction SilentlyContinue
    if (-not $process) {
      $stop_reason = 'xenia-exited'
      break
    }

    if ((Get-Date) -ge $next_crash_scan) {
      if (Test-Path -LiteralPath $log_path) {
        $stream = [System.IO.File]::Open($log_path, 'Open', 'Read', 'ReadWrite')
        try {
          if ($stream.Length -lt $log_scan_offset) {
            $log_scan_offset = 0L
          }
          [void]$stream.Seek($log_scan_offset, [System.IO.SeekOrigin]::Begin)
          $reader = [System.IO.StreamReader]::new($stream)
          try {
            $new_log_text = $reader.ReadToEnd()
            $log_scan_offset = $stream.Position
          } finally {
            $reader.Dispose()
          }
        } finally {
          $stream.Dispose()
        }
        if ($new_log_text -match '==== CRASH DUMP ====') {
          $stop_reason = 'guest-crash'
          break
        }
      }
      $next_crash_scan = (Get-Date).AddSeconds(1)
    }

    if (-not $gpu_trace_requested -and (Get-Date) -ge $gpu_trace_deadline) {
      $gpu_trace_request_succeeded =
        [XenectPhysicalCapture]::RequestGpuFrameTrace($process.MainWindowHandle)
      $gpu_trace_requested = $true
      Write-Output ("Requested GPU frame trace at {0:o}; accepted={1}" -f
        (Get-Date), $gpu_trace_request_succeeded)
    }

    if ((Get-Date) -ge $next_capture) {
      $capture_path = Join-Path $captures_directory ((Get-Date -Format 'yyyyMMdd-HHmmss') + '.png')
      try {
        [void][XenectPhysicalCapture]::RestoreAndFocus($process.MainWindowHandle)
        Start-Sleep -Milliseconds 250
        # PrintWindow can block indefinitely in a busy D3D12 present path.
        # Run it independently and enforce a short capture deadline so screen
        # diagnostics can never keep a title alive past DurationSeconds.
        $capture_job = Start-Job -ArgumentList $process.MainWindowHandle.ToInt64(),
          $capture_path, $CaptureMode -ScriptBlock {
          param([Int64]$window_handle, $path, $mode)
          Add-Type -TypeDefinition @'
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
public static class XenectCaptureWorker {
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);
  [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hWnd, IntPtr hdc, uint flags);
  public static bool Save(long windowValue, string path, bool desktop) {
    IntPtr window = new IntPtr(windowValue);
    RECT rect;
    if (window == IntPtr.Zero || !GetWindowRect(window, out rect)) return false;
    int width = rect.Right - rect.Left, height = rect.Bottom - rect.Top;
    if (width <= 0 || height <= 0) return false;
    using (var bitmap = new Bitmap(width, height, PixelFormat.Format32bppArgb))
    using (var graphics = Graphics.FromImage(bitmap)) {
      if (desktop) { graphics.CopyFromScreen(rect.Left, rect.Top, 0, 0, bitmap.Size); }
      else { IntPtr hdc = graphics.GetHdc(); bool ok = PrintWindow(window, hdc, 2); graphics.ReleaseHdc(hdc); if (!ok) return false; }
      bitmap.Save(path, ImageFormat.Png); return true;
    }
  }
}
'@ -ReferencedAssemblies System.Drawing.dll
          [XenectCaptureWorker]::Save($window_handle, $path, $mode -eq 'Desktop')
        }
        if (Wait-Job -Job $capture_job -Timeout 3) {
          $capture_succeeded = [bool](Receive-Job -Job $capture_job)
          if ($capture_succeeded) {
            $capture_count++
          } else {
            Write-Warning "Unable to capture the Xenia window at $(Get-Date -Format 'o')."
          }
        } else {
          Write-Warning "Window capture timed out at $(Get-Date -Format 'o')."
        }
        Remove-Job -Job $capture_job -Force -ErrorAction SilentlyContinue
      } catch {
        Write-Warning "Window capture failed at $(Get-Date -Format 'o'): $($_.Exception.Message)"
      }
      $next_capture = (Get-Date).AddSeconds($CaptureIntervalSeconds)
    }
    Start-Sleep -Milliseconds 500
  }
} finally {
  $process = Get-Process -Id $xenia.Id -ErrorAction SilentlyContinue
  if ($process) {
    $closed_gracefully = $false
    if ($GpuStreamTrace) {
      [void][XenectPhysicalCapture]::RequestClose($process.MainWindowHandle)
      Wait-Process -Id $xenia.Id -Timeout 5 -ErrorAction SilentlyContinue
      $process = Get-Process -Id $xenia.Id -ErrorAction SilentlyContinue
      $closed_gracefully = -not $process
    }
    if ($process) {
      Stop-Process -Id $xenia.Id -Force
    }
    $stop_reason = if ($stop_reason -eq 'deadline') {
      if ($closed_gracefully) { 'deadline-closed' } else { 'deadline-killed' }
    } else { $stop_reason }
    Wait-Process -Id $xenia.Id -ErrorAction SilentlyContinue
  }
}

$evidence_patterns = @{
  # Completion summary logging is deliberately sampled by the bridge. The
  # callback is emitted for every delivered buffer, so it is the authoritative
  # evidence for both PsCam channels in a short physical run.
  ColorFrames = 'PsCam callback:.*bytes=614400.*frame='
  DepthFrames = 'PsCam DMA depth:.*frame='
  # A physical skeleton frame with tracked=0 is valid while no body is in view.
  # Keep transport health and body recognition as separate observations.
  # Titles may consume skeletons through either the 0x2B002 message queue or
  # the 0x1402 frame buffer. Count both paths without counting the same sampled
  # log line twice.
  SkeletonFrames = 'XamNuiSkeletonMessage\(2B002\):.*host_injected=true|Windows Kinect: wrote title NUI frame'
  TrackedSkeletonFrames = 'XamNuiSkeletonMessage\(2B002\):.*tracked=[1-6]|Windows Kinect: wrote title NUI frame.*tracked=[1-6]'
  SkeletonBinding = 'Windows Kinect: bound host skeleton'
  NuiFailure = 'NUI Troubleshooter|XamNui.*error'
  DiscFailure = 'Disc Read Error|disc read error|file IO'
  Crash = 'Unhandled Exception|Fatal error|Fatal:|==== CRASH DUMP ===='
}
$evidence = [ordered]@{}
foreach ($entry in $evidence_patterns.GetEnumerator()) {
  $evidence[$entry.Key] = if (Test-Path -LiteralPath $log_path) {
    @(Select-String -LiteralPath $log_path -Pattern $entry.Value -CaseSensitive:$false).Count
  } else { 0 }
}

$gpu_trace_files = if ($GpuTraceAtSeconds -gt 0 -or $GpuStreamTrace) {
  @(Get-ChildItem -LiteralPath $gpu_trace_directory -Filter '*.xtr' -File -ErrorAction SilentlyContinue)
} else {
  @()
}
$gpu_trace_artifact_succeeded = $gpu_trace_files.Count -gt 0

$summary = [ordered]@{
  Game = $Game
  GamePath = $game_path
  XeniaPath = $xenia_path
  XeniaDirectory = $xenia_directory
  SyntheticKinect = $SyntheticKinect
  DepthByteOrder = $DepthByteOrder
  DepthFormat = $DepthFormat
  SyntheticBootstrapSeconds = $SyntheticBootstrapSeconds
  CoordinateMapperDumpPath = if ($DumpCoordinateMapper) { $coordinate_mapper_dump_path } else { $null }
  SessionConfigPath = $session_config_path
  GraphicsSystem = $GraphicsSystem
  RenderTargetPath = $RenderTargetPath
  ReadbackResolve = $ReadbackResolve
  ReadbackMemexport = $ReadbackMemexport
  D3D12Bindless = $D3D12Bindless
  StoreShaders = $StoreShaders
  XmaDecoder = $XmaDecoder
  UseDedicatedXmaThread = $UseDedicatedXmaThread
  GpuTraceAtSeconds = $GpuTraceAtSeconds
  GpuStreamTrace = $GpuStreamTrace
  GpuTraceRequested = $gpu_trace_requested
  GpuTraceRequestSucceeded = $gpu_trace_request_succeeded
  GpuTraceArtifactSucceeded = $gpu_trace_artifact_succeeded
  GpuTraceFiles = @($gpu_trace_files | ForEach-Object FullName)
  GpuTraceDirectory = if ($GpuTraceAtSeconds -gt 0 -or $GpuStreamTrace) { $gpu_trace_directory } else { $null }
  Started = $started.ToString('o')
  Finished = (Get-Date).ToString('o')
  StopReason = $stop_reason
  CaptureCount = $capture_count
  CaptureDirectory = $captures_directory
  LogPath = $log_path
  Evidence = $evidence
}
$summary | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $session_directory 'session.json') -Encoding ascii
$summary | ConvertTo-Json -Depth 4
