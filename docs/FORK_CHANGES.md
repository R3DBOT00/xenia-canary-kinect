# Fork changes

The original checkout has approximately 9,000 added lines across 109 tracked
paths relative to the pinned base. Most changes concern `xam_nui.cc` and
`xboxkrnl_usbcam.cc`. This is a working-tree snapshot, including the selected
uncommitted implementation files; it is not an upstream-reviewed patch series.

## Included development areas

- Kinect for Windows v1 backend discovery, stream handling, depth conversion,
  skeleton state, camera behavior, and NUI service compatibility.
- Kernel and XAM compatibility, including LDI/LZX work and Live-related stubs.
  Stubs do not provide a production online service.
- D3D12/Vulkan memory, texture, draw, resolve, and shader translation changes.
- CPU/JIT, timing, threading, virtual filesystem, and Windows/MinGW fixes.
- GPU trace utilities and PowerShell diagnostic launchers. Diagnostics require
  user-supplied inputs and may produce private traces or screenshots; outputs
  are excluded from version control.

## Export-specific adjustments

- Replaced hardcoded personal game image paths in the physical Kinect test
  launcher with a mandatory `-GamePath` argument.
- Kept the SDL2 MinGW configuration change as
  `patches/SDL2-mingw-disable-gles.patch` against the pinned SDL2 revision.
- Did not copy local dependency deletions or generated dependency build files.
- Omitted untracked reference excerpts with unclear provenance, prebuilt tools,
  Android packaging, upstream CI publishing workflows, standalone SPIR-V
  binaries, and reference PDFs/screenshots.
- Preserved original source notices, icon licensing, and dependency pins.

The original source checkout is unchanged. Release-specific fixes discovered
by validation are recorded in the public audit document.
