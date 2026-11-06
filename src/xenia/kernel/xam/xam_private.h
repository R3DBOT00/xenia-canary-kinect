/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2019 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XAM_XAM_PRIVATE_H_
#define XENIA_KERNEL_XAM_XAM_PRIVATE_H_

#include "xenia/cpu/export_resolver.h"
#include "xenia/kernel/xam/xam_ordinals.h"

#include <cstdint>
#include <string>
#include <vector>

namespace xe {
namespace kernel {
namespace xam {

xe::cpu::Export* RegisterExport_xam(xe::cpu::Export* export_entry);

// Starts the optional Windows Kinect 1.x backend on its MTA capture thread.
void StartWindowsNuiBackend();
bool IsWindowsNuiBackendReady();
void InitializeWindowsNuiGuestPathForMessage();
bool WindowsNuiInjectSkeletonNotification(uint8_t* notification,
                                          uint32_t notification_size);
// Writes a physical Kinect skeleton frame into a title-owned 2736-byte NUI
// buffer used by the common XamXStudioRequest(0x1402) submission path.
bool WindowsNuiWriteTitleFrame(uint32_t guest_frame, uint32_t frame_size);
// Correlates the retail NUI submission thread with its subsequent embedded
// event wait so the physical backend can discover the title-owned frame ring.
void WindowsNuiNoteFrameSubmission(uint32_t thread_id);
void WindowsNuiObserveGuestWait(uint32_t thread_id, uint32_t object_address,
                                uint32_t wait_reason, uint32_t caller);
void WindowsNuiObserveGuestEventReset(uint32_t object_address);
bool IsSyntheticKinectFrameActive();
bool WindowsNuiCopyLatestRawDepthFrame(std::vector<uint8_t>* out,
                                       uint64_t* timestamp,
                                       uint32_t* frame_number);
// Native 320x240 depth image used by the console-side diagnostic preview.
// PsCam callers must keep using the 640x480 raw frame above.
bool WindowsNuiCopyLatestNativeDepthFrame(std::vector<uint8_t>* out,
                                          uint64_t* timestamp,
                                          uint32_t* frame_number);
// Opaque calibration block returned by the physical Kinect coordinate mapper.
// Its byte layout belongs to the Kinect SDK; consumers must not reinterpret it
// as a retail PsCam structure without a verified conversion contract.
bool WindowsNuiCopyCoordinateMapperParameters(std::vector<uint8_t>* out);
bool WindowsNuiGetUniqueId(std::string* out);
uint32_t WindowsNuiTrackedCount();
uint64_t WindowsNuiSkeletonFrameCount();
uint64_t WindowsNuiColorFrameCount();
uint64_t WindowsNuiDepthFrameCount();
bool WindowsNuiGetElevationAngle(int32_t* out_angle);
bool WindowsNuiSetElevationAngle(int32_t angle);
// Retains the title-provided NUI HUD initialization contract from the 2B003
// XAM app message so the matching HUD exports observe the same state.
void WindowsNuiSetHudInitializeState(uint64_t version_0, uint64_t version_1);

// Registration functions, one per file.
#define XE_MODULE_EXPORT_GROUP(m, n)                                  \
  void Register##n##Exports(xe::cpu::ExportResolver* export_resolver, \
                            KernelState* kernel_state);
#include "xam_module_export_groups.inc"
#undef XE_MODULE_EXPORT_GROUP

}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XAM_XAM_PRIVATE_H_
