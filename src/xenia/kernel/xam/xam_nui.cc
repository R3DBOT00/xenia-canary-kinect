/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2025 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/base/cvar.h"
#include "xenia/base/clock.h"
#include "xenia/base/logging.h"
#include "xenia/cpu/cpu_flags.h"
#include "xenia/cpu/ppc/ppc_context.h"
#include "xenia/emulator.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <mutex>
#include <span>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "third_party/fmt/include/fmt/format.h"
#include "xenia/kernel/kernel_flags.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/util/object_table.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/user_profile.h"
#include "xenia/kernel/xam/user_settings.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/kernel/xam/xam_state.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_threading.h"
#include "xenia/kernel/xevent.h"
#include "xenia/kernel/xthread.h"
#include "xenia/ui/imgui_dialog.h"
#include "xenia/ui/imgui_drawer.h"
#include "xenia/ui/window.h"
#include "xenia/ui/windowed_app_context.h"
#include "xenia/vfs/virtual_file_system.h"
#include "xenia/xbox.h"

#if defined(XENIA_HAS_KINECT_V1_SDK)
#include <NuiApi.h>
#endif

DEFINE_bool(kinect_software_near_mode, false,
            "Compensate short-range Kinect v1 depth when hardware near mode "
            "is unavailable.",
            "Kinect");
DEFINE_int32(kinect_software_near_offset_mm, 450,
             "Maximum short-range depth offset used by software near mode.",
             "Kinect");
DEFINE_bool(kinect_seated_tracking, true,
            "Track the upper body when the player's legs do not fit in the "
            "Kinect v1 camera view.",
            "Kinect");
DEFINE_int32(kinect_pscam_depth_format, 0,
             "PsCam depth format: 0 = calibrated raw disparity, 1 = metric "
             "millimeters.",
             "Kinect");
DEFINE_int32(kinect_pscam_depth_byte_order, 0,
             "PsCam depth byte order: 0 = Xbox big-endian, 1 = sensor "
             "little-endian.",
             "Kinect");
DEFINE_bool(kinect_force_zero_elevation, true,
            "Keep the Windows Kinect motor at the fixed Xenect angle during "
            "a session.",
            "Kinect");
DEFINE_path(kinect_coordinate_mapper_dump_path, "",
            "Optional file path for dumping the physical Kinect v1 coordinate "
            "mapper parameters during diagnostics.",
            "Kinect");
DECLARE_bool(kinect_synthetic_test_frames);
DECLARE_int32(kinect_synthetic_bootstrap_seconds);

constexpr int32_t kXenectFixedElevationAngle = -10;
// Kinect for Windows has no Xbox biometric-profile service. Keep a stable
// guest enrollment token for the active local NUI session so the identity
// exports and delivered skeleton record describe the same player binding.
constexpr uint32_t kNuiGuestEnrollmentIndex = 0;

namespace xe {
namespace kernel {
namespace xam {

#if defined(XENIA_HAS_KINECT_V1_SDK)
bool WindowsNuiWriteTitleFrameFromNative(
    uint32_t guest_frame, uint32_t frame_size,
    const NUI_SKELETON_FRAME& native_frame);
bool WindowsNuiWriteConsumerRingFromNative(
    const NUI_SKELETON_FRAME& native_frame);
void StartWindowsNuiSyntheticRingProducer();

bool IsSyntheticKinectFrameActive() {
  if (cvars::kinect_synthetic_test_frames) {
    return true;
  }
  const int32_t bootstrap_seconds = cvars::kinect_synthetic_bootstrap_seconds;
  if (bootstrap_seconds <= 0) {
    return false;
  }
  static std::atomic<uint64_t> bootstrap_start_ms{0};
  uint64_t start_ms = bootstrap_start_ms.load(std::memory_order_acquire);
  if (!start_ms) {
    const uint64_t now_ms = GetTickCount64();
    bootstrap_start_ms.compare_exchange_strong(
        start_ms, now_ms, std::memory_order_acq_rel);
    start_ms = bootstrap_start_ms.load(std::memory_order_acquire);
    XELOGI("Windows Kinect: synthetic bootstrap started for {} seconds",
           bootstrap_seconds);
  }
  return GetTickCount64() - start_ms <
         static_cast<uint64_t>(bootstrap_seconds) * 1000;
}
#endif

// Kept in sync with the NUI HUD engagement state below. Kinect titles use
// this state to associate a tracked skeleton with the signed-in player.
extern uint32_t engaged_tracking_id;
extern uint32_t engaged_enrollment_index;
extern uint32_t nui_unknown_1;
extern char nui_unknown_2;

// XamNuiCameraTiltSetCallback is registered by every known Kinect title.
// The callback is a guest function taking (event_code, guest_payload). Its
// payload must remain valid for the guest call, so delivery is always queued
// through the kernel dispatcher instead of running on the host capture thread.
void QueueNuiTiltCallback(float elevation_degrees, const char* source);

// https://web.cs.ucdavis.edu/~okreylos/ResDev/Kinect/MainPage.html

namespace {

#if defined(XENIA_HAS_KINECT_V1_SDK)
uint16_t KinectDepthMillimetersToRawShift(uint16_t depth_mm) {
  static const std::array<uint16_t, 8192> raw_shift_by_millimeter = []() {
    std::array<uint16_t, 8192> table = {};
    table.fill(0x7FF);
    // Keep this inverse exactly paired with the calibration returned by
    // PsCam opcode 14. Opcode 24 maps the resulting disparity back to metric
    // depth before the guest builds its GPU lookup tables.
    constexpr double kZeroPlaneDistance = 120.0;
    constexpr double kZeroPlanePixelSize = 0.1042;
    constexpr double kEmitterDistance = 7.5;
    constexpr double kShiftScale = 10.0;
    constexpr double kParamCoeff = 4.0;
    constexpr double kConstShift = 200.0;
    for (uint32_t millimeters = 300; millimeters <= 4000; ++millimeters) {
      const double metric =
          kEmitterDistance *
          (1.0 - kShiftScale * kZeroPlaneDistance /
                     static_cast<double>(millimeters));
      const auto raw_shift = static_cast<int32_t>(std::lround(
          kParamCoeff *
          (metric / kZeroPlanePixelSize + 0.375 + kConstShift)));
      table[millimeters] =
          static_cast<uint16_t>(std::clamp(raw_shift, 0, 0x7FE));
    }
    return table;
  }();
  return depth_mm < raw_shift_by_millimeter.size()
             ? raw_shift_by_millimeter[depth_mm]
             : 0x7FF;
}

uint16_t ApplySoftwareNearCompensation(uint16_t depth_mm,
                                       bool software_near_enabled) {
  if (!software_near_enabled || depth_mm < 300 || depth_mm > 4000) {
    return depth_mm;
  }

  // Preserve ordering and depth relief while moving only the short-range
  // portion into the range normally expected by Xbox NUI titles. The offset
  // fades continuously from its maximum at 800 mm to zero at 1800 mm.
  constexpr double kFullCompensationDepthMm = 800.0;
  constexpr double kNoCompensationDepthMm = 1800.0;
  const double fade = std::clamp(
      (kNoCompensationDepthMm - static_cast<double>(depth_mm)) /
          (kNoCompensationDepthMm - kFullCompensationDepthMm),
      0.0, 1.0);
  const int32_t maximum_offset =
      std::clamp(cvars::kinect_software_near_offset_mm, 0, 1000);
  const auto compensated = static_cast<int32_t>(std::lround(
      static_cast<double>(depth_mm) + maximum_offset * fade));
  return static_cast<uint16_t>(std::clamp(compensated, 300, 4000));
}

class WindowsNuiBackend final {
 public:
  WindowsNuiBackend() {
    // XAM may instantiate this singleton through an unrelated NUI export after
    // the synthetic title-frame producer is already active. Do not let that
    // lazy construction start a second, physical producer: it races the
    // fixture and causes the Kinect SDK stream to be opened unnecessarily.
    if (!IsSyntheticKinectFrameActive()) {
      StartCaptureThread();
    }
  }
  ~WindowsNuiBackend() {
    std::lock_guard<std::mutex> lock(initialization_mutex_);
    Shutdown();
  }

  bool ready() {
    // The synthetic producer owns the guest-side device contract. Reporting
    // readiness here prevents callers from restarting the physical capture
    // thread merely because they queried a backend-owned helper.
    if (IsSyntheticKinectFrameActive()) {
      return true;
    }
    if (ready_.load()) {
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(initialization_mutex_);
    if (ready_.load()) {
      return true;
    }
    ReapStoppedCaptureThread();
    if (capture_running_.load() || now < next_initialization_attempt_) {
      return false;
    }
    StartCaptureThread();
    return false;
  }

  // XAM is constructed before the title starts issuing its NUI probe calls.
  // Kinect titles may cache that first device status and never poll again, so
  // give the physical sensor a bounded startup window at module creation.
  // The capture itself remains asynchronous once the title is running.
  bool WaitForInitialReady(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      if (ready()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (std::chrono::steady_clock::now() < deadline);
    return ready();
  }

  void Shutdown() {
    set_title_frame_callback({});
    capture_running_.store(false);
    // A stream read may be waiting on a Kinect runtime event. Shut the native
    // session down before joining so the wait is released during title exit.
    if (sensor_ && initialized_) {
      sensor_->NuiShutdown();
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

  // Drain the native queues from the XAM request path. The Xbox title polls
  // NUI through XAM rather than calling the Windows SDK directly.
  void pump() {
    // Capture runs independently. This request path only observes the latest
    // native frame and must never be responsible for sensor timing.
  }

  void set_title_frame_callback(
      std::function<void(const NUI_SKELETON_FRAME&)> callback) {
    std::lock_guard<std::mutex> lock(title_frame_callback_mutex_);
    title_frame_callback_ = std::move(callback);
  }

  uint32_t tracked_count() const { return tracked_count_; }
  uint64_t skeleton_frames() const { return skeleton_frames_; }
  uint64_t color_frames() const { return color_frames_; }
  uint64_t depth_frames() const { return depth_frames_; }

  bool copy_latest_skeleton(NUI_SKELETON_FRAME* out) const {
    if (!out) {
      return false;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_skeleton_valid_ && latest_depth_frame_.empty()) {
      return false;
    }
    *out = latest_skeleton_valid_ ? latest_skeleton_frame_
                                  : NUI_SKELETON_FRAME{};
    // Kinect SDK 1.8 may stop producing skeleton events while nobody is in
    // view, although the physical depth stream remains live. Retail NUI still
    // advances its frame clock in that state. Use the synchronized physical
    // depth cadence for the guest envelope without fabricating a tracked body.
    if (latest_depth_native_frame_number_ != 0) {
      out->dwFrameNumber = latest_depth_native_frame_number_;
      out->liTimeStamp.QuadPart =
          static_cast<LONGLONG>(latest_depth_timestamp_);
    }
    return true;
  }

  bool copy_latest_accelerometer(std::array<float, 4>* out,
                                 uint32_t* sample_sequence) const {
    if (!out) {
      return false;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_accelerometer_valid_) {
      return false;
    }
    *out = latest_accelerometer_;
    if (sample_sequence) {
      *sample_sequence = latest_accelerometer_sequence_;
    }
    return true;
  }

  bool remember_floor() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_skeleton_valid_) {
      return false;
    }

    // Never manufacture or freeze a floor plane. The physical Kinect SDK
    // estimates it continuously, and XAM only reports it as available when
    // that current estimate has a finite, non-degenerate normal.
    const Vector4& floor = latest_skeleton_frame_.vFloorClipPlane;
    const float normal_length_squared =
        floor.x * floor.x + floor.y * floor.y + floor.z * floor.z;
    return std::isfinite(floor.x) && std::isfinite(floor.y) &&
           std::isfinite(floor.z) && std::isfinite(floor.w) &&
           normal_length_squared > 1.0e-6f;
  }

  bool copy_latest_color(std::vector<uint8_t>* out) const {
    if (!out) {
      return false;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_color_frame_.empty()) {
      return false;
    }
    *out = latest_color_frame_;
    return true;
  }

  bool copy_latest_depth(std::vector<uint8_t>* out) const {
    if (!out) {
      return false;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_depth_frame_.empty()) {
      return false;
    }
    *out = latest_depth_frame_;
    return true;
  }

  bool copy_latest_raw_color(std::vector<uint8_t>* out, uint64_t* timestamp,
                             uint32_t* frame_number) const {
    if (!out) {
      return false;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_raw_color_frame_.empty()) {
      return false;
    }
    *out = latest_raw_color_frame_;
    if (timestamp) {
      *timestamp = latest_color_timestamp_;
    }
    if (frame_number) {
      *frame_number = latest_color_native_frame_number_;
    }
    return true;
  }

  bool copy_latest_raw_depth(std::vector<uint8_t>* out, uint64_t* timestamp,
                             uint32_t* frame_number) const {
    if (!out) {
      return false;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_raw_depth_frame_.empty()) {
      return false;
    }
    *out = latest_raw_depth_frame_;
    if (timestamp) {
      *timestamp = latest_depth_timestamp_;
    }
    if (frame_number) {
      *frame_number = latest_depth_native_frame_number_;
    }
    return true;
  }

  bool copy_latest_native_depth(std::vector<uint8_t>* out,
                                uint64_t* timestamp,
                                uint32_t* frame_number) const {
    if (!out) {
      return false;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_depth_frame_.empty()) {
      return false;
    }
    *out = latest_depth_frame_;
    if (timestamp) {
      *timestamp = latest_depth_timestamp_;
    }
    if (frame_number) {
      *frame_number = latest_depth_native_frame_number_;
    }
    return true;
  }

  bool copy_coordinate_mapper_parameters(std::vector<uint8_t>* out) const {
    if (!out) {
      return false;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (coordinate_mapper_parameters_.empty()) {
      out->clear();
      return false;
    }
    *out = coordinate_mapper_parameters_;
    return true;
  }

  int best_skeleton_index() {
    if (!ready()) {
      return -1;
    }

    pump();
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_skeleton_valid_) {
      return -1;
    }
    for (int index = 0; index < NUI_SKELETON_COUNT; ++index) {
      if (latest_skeleton_frame_.SkeletonData[index].eTrackingState ==
          NUI_SKELETON_TRACKED) {
        // Windows assigns transient host slots. The guest bridge binds the
        // first tracked body to local Xbox user 0 in canonical NUI slot 0.
        return 0;
      }
    }
    return -1;
  }

  bool get_elevation_angle(int32_t* out_angle) {
    if (!out_angle || !ready()) {
      return false;
    }

    *out_angle = last_reported_elevation_angle_.load();
    return true;
  }

  bool set_elevation_angle(int32_t requested_angle) {
    if (!ready()) {
      return false;
    }

    const int32_t effective_angle =
        cvars::kinect_force_zero_elevation ? kXenectFixedElevationAngle
                                           : requested_angle;
    const LONG native_angle = static_cast<LONG>(std::clamp(
        effective_angle, static_cast<int32_t>(NUI_CAMERA_ELEVATION_MINIMUM),
        static_cast<int32_t>(NUI_CAMERA_ELEVATION_MAXIMUM)));
    requested_elevation_angle_.store(native_angle);
    XELOGI("Windows Kinect backend: elevation request={} effective={} degrees",
           requested_angle, native_angle);
    return true;
  }

  bool copy_unique_id(std::string* out) {
    if (!out || !ready() || !sensor_) {
      return false;
    }

    // NuiUniqueId is owned by the Kinect runtime. Convert it while the native
    // sensor remains alive, then copy only an ASCII representation into guest
    // memory at the XAM boundary.
    BSTR unique_id = sensor_->NuiUniqueId();
    if (!unique_id || !SysStringLen(unique_id)) {
      return false;
    }
    const int byte_count = WideCharToMultiByte(
        CP_UTF8, 0, unique_id, static_cast<int>(SysStringLen(unique_id)),
        nullptr, 0, nullptr, nullptr);
    if (byte_count <= 0) {
      return false;
    }
    std::string value(static_cast<size_t>(byte_count), '\0');
    if (WideCharToMultiByte(CP_UTF8, 0, unique_id,
                            static_cast<int>(SysStringLen(unique_id)),
                            value.data(), byte_count, nullptr, nullptr) !=
        byte_count) {
      return false;
    }
    *out = std::move(value);
    return true;
  }

 private:
  void StartCaptureThread() {
    next_initialization_attempt_ =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    capture_running_.store(true);
    capture_thread_ = std::thread([this]() { capture_loop(); });
  }

  void ReapStoppedCaptureThread() {
    if (!capture_running_.load() && capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

  void ResetSensorState() {
    sensor_ = nullptr;
    color_event_ = nullptr;
    depth_event_ = nullptr;
    skeleton_event_ = nullptr;
    frame_end_event_ = nullptr;
    color_stream_ = nullptr;
    depth_stream_ = nullptr;
    initialized_ = false;
    color_is_raw_bayer_ = false;
    depth_has_player_index_ = false;
    near_mode_enabled_ = false;
    software_near_mode_enabled_ = false;
    skeleton_requested_ = false;
    skeleton_enabled_ = false;
    depth_width_ = 0;
    depth_height_ = 0;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_skeleton_frame_ = {};
      latest_skeleton_valid_ = false;
      latest_accelerometer_ = {};
      latest_accelerometer_valid_ = false;
      latest_accelerometer_sequence_ = 0;
      latest_color_frame_.clear();
      latest_depth_frame_.clear();
      latest_raw_color_frame_.clear();
      latest_raw_depth_frame_.clear();
      coordinate_mapper_parameters_.clear();
      latest_color_timestamp_ = 0;
      latest_depth_timestamp_ = 0;
      latest_color_native_frame_number_ = 0;
      latest_depth_native_frame_number_ = 0;
    }
    color_frames_.store(0);
    depth_frames_.store(0);
    depth_empty_results_.store(0);
    color_empty_results_.store(0);
    skeleton_empty_results_.store(0);
    capture_polls_.store(0);
    tracked_count_.store(0);
    skeleton_frames_.store(0);
    ready_.store(false);
  }

  void ShutdownSensor() {
    if (sensor_ && initialized_) {
      sensor_->NuiShutdown();
    }
    if (sensor_) {
      sensor_->Release();
    }
    if (color_event_) {
      CloseHandle(color_event_);
    }
    if (depth_event_) {
      CloseHandle(depth_event_);
    }
    if (skeleton_event_) {
      CloseHandle(skeleton_event_);
    }
    if (frame_end_event_) {
      CloseHandle(frame_end_event_);
    }
    ResetSensorState();
  }

  void ApplyRequestedElevation() {
    const int32_t requested = requested_elevation_angle_.exchange(
        kNoElevationRequest);
    if (requested == kNoElevationRequest || !sensor_ || !initialized_) {
      return;
    }

    LONG current = 0;
    if (SUCCEEDED(sensor_->NuiCameraElevationGetAngle(&current)) &&
        current == requested) {
      last_reported_elevation_angle_.store(current);
      QueueNuiTiltCallback(static_cast<float>(current), "motor-already-set");
      return;
    }

    const HRESULT result =
        sensor_->NuiCameraElevationSetAngle(static_cast<LONG>(requested));
    if (FAILED(result)) {
      XELOGW("Windows Kinect backend: elevation set {} failed {:08X}",
             requested, static_cast<uint32_t>(result));
      // A failed motor command is not a sensor disconnect and must not block
      // color/depth/skeleton capture. Report the stable guest angle instead.
      LONG reported = last_reported_elevation_angle_.load();
      sensor_->NuiCameraElevationGetAngle(&reported);
      last_reported_elevation_angle_.store(reported);
      QueueNuiTiltCallback(static_cast<float>(reported), "motor-rejected");
      return;
    }

    last_reported_elevation_angle_.store(requested);
    XELOGI("Windows Kinect backend: elevation requested={} degrees",
           requested);
    QueueNuiTiltCallback(static_cast<float>(requested), "motor-applied");
  }

  void CaptureAccelerometerReading() {
    if (!sensor_ || !initialized_) {
      return;
    }
    Vector4 reading = {};
    const HRESULT result = sensor_->NuiAccelerometerGetCurrentReading(&reading);
    if (FAILED(result)) {
      return;
    }
    uint32_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_accelerometer_ = {reading.x, reading.y, reading.z, reading.w};
      latest_accelerometer_valid_ = true;
      sequence = ++latest_accelerometer_sequence_;
    }
    if (sequence == 1 || (sequence & 0x7F) == 0) {
      XELOGD("Windows Kinect backend: accelerometer sample={} g=({}, {}, {}, {})",
             sequence, reading.x, reading.y, reading.z, reading.w);
    }
  }

  void CacheCoordinateMapperParameters() {
    INuiCoordinateMapper* mapper = nullptr;
    HRESULT result = sensor_->NuiGetCoordinateMapper(&mapper);
    if (FAILED(result) || !mapper) {
      XELOGW("Windows Kinect backend: NuiGetCoordinateMapper failed {:08X}",
             static_cast<uint32_t>(result));
      return;
    }

    ULONG byte_count = 0;
    void* data = nullptr;
    result = mapper->GetColorToDepthRelationalParameters(&byte_count, &data);
    if (SUCCEEDED(result) && data && byte_count) {
      constexpr ULONG kMaximumMapperParameters = 1 << 20;
      if (byte_count <= kMaximumMapperParameters) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        const auto* bytes = static_cast<const uint8_t*>(data);
        coordinate_mapper_parameters_.assign(bytes, bytes + byte_count);
      } else {
        XELOGW("Windows Kinect backend: rejected oversized coordinate mapper "
               "blob ({} bytes)",
               byte_count);
      }
    } else {
      XELOGW("Windows Kinect backend: mapper calibration unavailable {:08X} "
             "bytes={}",
             static_cast<uint32_t>(result), byte_count);
    }

    // The SDK allocates the variable-size result for this COM call. Copy it
    // before releasing it so capture stays independent from mapper lifetime.
    if (data) {
      CoTaskMemFree(data);
    }
    mapper->Release();

    std::lock_guard<std::mutex> lock(frame_mutex_);
    uint64_t parameter_hash = 1469598103934665603ull;
    for (uint8_t byte : coordinate_mapper_parameters_) {
      parameter_hash ^= byte;
      parameter_hash *= 1099511628211ull;
    }
    if (!cvars::kinect_coordinate_mapper_dump_path.empty()) {
      std::ofstream dump(cvars::kinect_coordinate_mapper_dump_path,
                         std::ios::binary | std::ios::trunc);
      if (dump) {
        dump.write(reinterpret_cast<const char*>(
                       coordinate_mapper_parameters_.data()),
                   static_cast<std::streamsize>(
                       coordinate_mapper_parameters_.size()));
        XELOGI("Windows Kinect backend: wrote physical coordinate mapper "
               "parameters to {}",
               cvars::kinect_coordinate_mapper_dump_path.string());
      } else {
        XELOGW("Windows Kinect backend: unable to write coordinate mapper "
               "parameters to {}",
               cvars::kinect_coordinate_mapper_dump_path.string());
      }
    }
    XELOGI("Windows Kinect backend: cached physical coordinate mapper "
           "parameters={} bytes fnv64={:016X} head={:02X}{:02X}{:02X}{:02X}"
           "{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
           coordinate_mapper_parameters_.size(), parameter_hash,
           coordinate_mapper_parameters_.size() > 0
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[0])
               : 0,
           coordinate_mapper_parameters_.size() > 1
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[1])
               : 0,
           coordinate_mapper_parameters_.size() > 2
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[2])
               : 0,
           coordinate_mapper_parameters_.size() > 3
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[3])
               : 0,
           coordinate_mapper_parameters_.size() > 4
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[4])
               : 0,
           coordinate_mapper_parameters_.size() > 5
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[5])
               : 0,
           coordinate_mapper_parameters_.size() > 6
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[6])
               : 0,
           coordinate_mapper_parameters_.size() > 7
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[7])
               : 0,
           coordinate_mapper_parameters_.size() > 8
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[8])
               : 0,
           coordinate_mapper_parameters_.size() > 9
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[9])
               : 0,
           coordinate_mapper_parameters_.size() > 10
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[10])
               : 0,
           coordinate_mapper_parameters_.size() > 11
               ? static_cast<uint32_t>(coordinate_mapper_parameters_[11])
               : 0);
  }

  bool InitializeSensor() {
    ResetSensorState();
    int sensor_count = 0;
    HRESULT result = NuiGetSensorCount(&sensor_count);
    if (FAILED(result) || sensor_count <= 0) {
      XELOGW("Windows Kinect backend: no sensor (hr={:08X}, count={})",
             static_cast<uint32_t>(result), sensor_count);
      return false;
    }

    result = NuiCreateSensorByIndex(0, &sensor_);
    if (FAILED(result) || !sensor_) {
      XELOGW("Windows Kinect backend: NuiCreateSensorByIndex failed {:08X}",
             static_cast<uint32_t>(result));
      return false;
    }

    result = sensor_->NuiStatus();
    if (FAILED(result)) {
      XELOGW("Windows Kinect backend: NuiStatus failed {:08X}",
             static_cast<uint32_t>(result));
      return false;
    }
    LONG elevation = 0;
    if (SUCCEEDED(sensor_->NuiCameraElevationGetAngle(&elevation))) {
      last_reported_elevation_angle_.store(elevation);
    }

    const DWORD stream_flags =
        NUI_INITIALIZE_FLAG_USES_COLOR |
        NUI_INITIALIZE_FLAG_USES_DEPTH |
        NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |
        NUI_INITIALIZE_FLAG_USES_SKELETON;
    // Some Kinect adapters expose the microphone array through the same NUI
    // session, while others expose only camera streams. Prefer the complete
    // session and fall back without audio instead of rejecting the sensor.
    result = sensor_->NuiInitialize(stream_flags |
                                     NUI_INITIALIZE_FLAG_USES_AUDIO);
    if (FAILED(result)) {
      XELOGW("Windows Kinect backend: audio session unavailable {:08X}; "
             "retrying camera/skeleton session",
             static_cast<uint32_t>(result));
      result = sensor_->NuiInitialize(stream_flags);
    }
    if (FAILED(result)) {
      XELOGW("Windows Kinect backend: NuiInitialize failed {:08X}",
             static_cast<uint32_t>(result));
      const DWORD camera_flags = NUI_INITIALIZE_FLAG_USES_COLOR |
                                 NUI_INITIALIZE_FLAG_USES_DEPTH;
      XELOGW("Windows Kinect backend: retrying camera-only raw session");
      result = sensor_->NuiInitialize(camera_flags);
      if (FAILED(result)) {
        XELOGW("Windows Kinect backend: camera-only NuiInitialize failed "
               "{:08X}",
               static_cast<uint32_t>(result));
        return false;
      }
      skeleton_requested_ = false;
    } else {
      skeleton_requested_ = true;
    }
    initialized_ = true;
    CacheCoordinateMapperParameters();
    // Do not issue a motor command while the sensor is bringing up its first
    // image queues. Kinect v1 adapters may reject or stall a stream if an
    // elevation request overlaps startup. Queue the fixed Xenect angle here;
    // the capture loop applies it after initialization has completed.
    requested_elevation_angle_.store(cvars::kinect_force_zero_elevation
                                         ? kXenectFixedElevationAngle
                                         : kNoElevationRequest);

    // Per-stream NUI notifications are manual-reset events. The SDK resets
    // each one when its corresponding GetNextFrame call consumes the frame.
    color_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    depth_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    skeleton_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    // NuiSetFrameEndEvent requires an auto-reset event. It is the SDK's
    // synchronization primitive for pumping multiple streams on one thread.
    frame_end_event_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    if (!color_event_ || !depth_event_ || !skeleton_event_ ||
        !frame_end_event_) {
      XELOGW("Windows Kinect backend: CreateEvent failed ({})",
             GetLastError());
      return false;
    }

    // RGB32 is the reliable continuous color stream on both Kinect v1 SKU
    // families supported by the 1.8 SDK. The raw Bayer stream can open on an
    // Xbox sensor yet stop after its first frame, which leaves titles waiting
    // on the PsCam color channel even though depth keeps advancing. The bridge
    // below derives its Bayer-compatible and UYVY outputs from RGB32.
    result = sensor_->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2,
        color_event_, &color_stream_);
    if (FAILED(result)) {
      XELOGW("Windows Kinect backend: RGB32 stream unavailable {:08X}; "
             "retrying raw Bayer",
             static_cast<uint32_t>(result));
      result = sensor_->NuiImageStreamOpen(
          NUI_IMAGE_TYPE_COLOR_RAW_BAYER, NUI_IMAGE_RESOLUTION_640x480, 0, 2,
          color_event_, &color_stream_);
      if (FAILED(result)) {
        XELOGW("Windows Kinect backend: raw Bayer stream open failed {:08X}",
               static_cast<uint32_t>(result));
        return false;
      }
      color_is_raw_bayer_ = true;
    } else {
      color_is_raw_bayer_ = false;
    }

    // Kinect v1's physical depth camera is 320x240. Some SDK drivers accept a
    // 640x480 depth request by returning an internally expanded surface, but
    // that surface is not a second physical capture and can produce doubled
    // silhouettes in Xbox depth previews. Capture the canonical packed
    // depth/player stream and expand it exactly once while building the PsCam
    // 640x480 surface below.
    result = sensor_->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240,
        0, 2,
        depth_event_, &depth_stream_);
    if (FAILED(result)) {
      XELOGW("Windows Kinect backend: native 320x240 depth/player stream "
             "open failed {:08X}",
             static_cast<uint32_t>(result));
      return false;
    }
    depth_width_ = 320;
    depth_height_ = 240;
    depth_has_player_index_ = true;
    XELOGI("Windows Kinect backend: using canonical 320x240 depth/player "
           "stream; PsCam expansion is performed once in the bridge");

    // Keep the retail Kinect v1 in its default depth mode. Requesting the
    // Kinect-for-Windows-only near-mode flags is rejected by the Xbox sensor
    // and serves no purpose when software near compensation is disabled.
    near_mode_enabled_ = false;
    software_near_mode_enabled_ = cvars::kinect_software_near_mode;

    DWORD skeleton_flags =
        near_mode_enabled_ ? NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE
                           : 0;
    if (cvars::kinect_seated_tracking) {
      skeleton_flags |= NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT;
    }
    result = skeleton_requested_
                 ? sensor_->NuiSkeletonTrackingEnable(skeleton_event_,
                                                      skeleton_flags)
                 : E_NUI_FEATURE_NOT_INITIALIZED;
    if (FAILED(result) && skeleton_requested_ && near_mode_enabled_) {
      XELOGW("Windows Kinect backend: near-range skeleton tracking rejected "
             "{:08X}; retrying without hardware near-range tracking",
             static_cast<uint32_t>(result));
      skeleton_flags &= ~NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE;
      result = sensor_->NuiSkeletonTrackingEnable(skeleton_event_,
                                                  skeleton_flags);
    }
    if (FAILED(result)) {
      XELOGW("Windows Kinect backend: SDK skeleton tracking unavailable "
             "{:08X}; raw camera streams remain enabled",
             static_cast<uint32_t>(result));
      skeleton_enabled_ = false;
    } else {
      skeleton_enabled_ = true;
      XELOGI("Windows Kinect backend: skeleton flags={:08X} seated={}",
             skeleton_flags,
              (skeleton_flags &
               NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT)
                  ? "enabled"
                  : "disabled");
    }

    result = sensor_->NuiSetFrameEndEvent(frame_end_event_, 0);
    if (FAILED(result)) {
      XELOGW("Windows Kinect backend: NuiSetFrameEndEvent failed {:08X}",
             static_cast<uint32_t>(result));
      return false;
    }

    // Seed the cache before advertising readiness. The shared retail client
    // can issue its first PsCam accelerometer query immediately after the
    // device-present transition.
    CaptureAccelerometerReading();
    ready_.store(true);
    XELOGI("Windows Kinect backend: sensor initialized, color={} depth={}x{} "
           "near={} software_near={} depth_format={} byte_order={} "
           "skeleton={}",
           color_is_raw_bayer_ ? "raw-bayer" : "rgb32", depth_width_,
           depth_height_, near_mode_enabled_ ? "enabled" : "normal",
           software_near_mode_enabled_ ? "enabled" : "disabled",
           cvars::kinect_pscam_depth_format == 1 ? "metric" : "raw-calibrated",
           cvars::kinect_pscam_depth_byte_order == 1 ? "little" : "big",
           skeleton_enabled_ ? "enabled" : "disabled");
    return true;
  }

  void capture_loop() {
    const HRESULT com_result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(com_result)) {
      XELOGW("Windows Kinect backend: MTA initialization failed {:08X}",
             static_cast<uint32_t>(com_result));
    } else if (capture_running_.load() && InitializeSensor()) {
      while (capture_running_.load()) {
        ApplyRequestedElevation();
        // Keep the initialization-time accelerometer sample cached. On Xbox
        // Kinect adapters the SDK call may block indefinitely after streaming
        // starts, which would freeze depth, color and skeleton together.
        const uint64_t poll = ++capture_polls_;
        if ((poll & 0xFF) == 1) {
          XELOGD("Windows Kinect backend: capture poll={} color={} depth={} "
                 "skeleton={}",
                 poll, color_frames_.load(), depth_frames_.load(),
                 skeleton_frames_.load());
        }
        // The aggregate event is not reliable on every Kinect v1 adapter.
        // Each SDK stream has its own readiness event, so observe all of them
        // and drain the queues from whichever notification arrived first.
        HANDLE wait_handles[] = {frame_end_event_, depth_event_, color_event_,
                                 skeleton_event_};
        const DWORD wait_handle_count = skeleton_enabled_ ? 4 : 3;
        const DWORD frame_wait = WaitForMultipleObjects(
            wait_handle_count, wait_handles, FALSE, 100);
        if (frame_wait == WAIT_TIMEOUT) {
          // Xbox 360 Kinect adapters using the 1.8 runtime may stop signaling
          // the aggregate frame-end event even while image queues continue to
          // receive data. A short watchdog dequeue keeps those queues moving
          // and allows the SDK to resume event delivery without restarting the
          // physical sensor.
          pump_depth_stream(1);
          pump_color_stream(1);
          continue;
        }
        if (frame_wait < WAIT_OBJECT_0 ||
            frame_wait >= WAIT_OBJECT_0 + wait_handle_count) {
          XELOGW("Windows Kinect backend: frame-end wait failed ({})",
                 GetLastError());
          continue;
        }
        pump_depth_stream(0);
        pump_color_stream(0);
        if (skeleton_enabled_) {

          NUI_SKELETON_FRAME skeleton_frame = {};
          const HRESULT skeleton_result =
              sensor_->NuiSkeletonGetNextFrame(0, &skeleton_frame);
          if (SUCCEEDED(skeleton_result) && skeleton_result != S_FALSE) {
            sensor_->NuiTransformSmooth(&skeleton_frame, nullptr);
            uint32_t tracked_count = 0;
            uint32_t primary_tracking_id = 0;
            for (int index = 0; index < NUI_SKELETON_COUNT; ++index) {
              if (skeleton_frame.SkeletonData[index].eTrackingState ==
                  NUI_SKELETON_TRACKED) {
                ++tracked_count;
                if (primary_tracking_id == 0) {
                  primary_tracking_id =
                      skeleton_frame.SkeletonData[index].dwTrackingID;
                }
              }
            }
            // A title may register its 2B002 notification buffer before a
            // player steps into view and then rely solely on the named NUI
            // events. Keep the XAM engagement state synchronized with the
            // physical skeleton stream rather than with that registration
            // timing, so every NUI title sees the newly tracked player.
            if (primary_tracking_id != 0) {
              engaged_tracking_id = primary_tracking_id;
              nui_unknown_1 = 1;
              nui_unknown_2 = '\1';
            }
            {
              std::lock_guard<std::mutex> lock(frame_mutex_);
              latest_skeleton_frame_ = skeleton_frame;
              latest_skeleton_valid_ = true;
              tracked_count_ = tracked_count;
              ++skeleton_frames_;
            }
            std::function<void(const NUI_SKELETON_FRAME&)> title_callback;
            {
              std::lock_guard<std::mutex> lock(title_frame_callback_mutex_);
              title_callback = title_frame_callback_;
            }
            if (title_callback) {
              title_callback(skeleton_frame);
            }
            const uint64_t skeleton_frame_count = skeleton_frames_.load();
            if (skeleton_frame_count <= 4 ||
                (skeleton_frame_count & 0x3F) == 1) {
              float tracked_body_z = 0.0f;
              float head_y = 0.0f;
              float left_hand_y = 0.0f;
              float right_hand_y = 0.0f;
              uint32_t head_state = 0;
              uint32_t left_hand_state = 0;
              uint32_t right_hand_state = 0;
              uint32_t tracked_id = 0;
              for (int index = 0; index < NUI_SKELETON_COUNT; ++index) {
                const auto& body = skeleton_frame.SkeletonData[index];
                if (body.eTrackingState == NUI_SKELETON_TRACKED) {
                  tracked_body_z = body.Position.z;
                  tracked_id = body.dwTrackingID;
                  head_y = body.SkeletonPositions[
                      NUI_SKELETON_POSITION_HEAD].y;
                  left_hand_y = body.SkeletonPositions[
                      NUI_SKELETON_POSITION_HAND_LEFT].y;
                  right_hand_y = body.SkeletonPositions[
                      NUI_SKELETON_POSITION_HAND_RIGHT].y;
                  head_state = body.eSkeletonPositionTrackingState[
                      NUI_SKELETON_POSITION_HEAD];
                  left_hand_state = body.eSkeletonPositionTrackingState[
                      NUI_SKELETON_POSITION_HAND_LEFT];
                  right_hand_state = body.eSkeletonPositionTrackingState[
                      NUI_SKELETON_POSITION_HAND_RIGHT];
                  break;
                }
              }
              XELOGI(
                "Windows Kinect backend: captured skeleton frame={} tracked={} "
                  "id={} body_z={} head_y={} left_hand_y={} right_hand_y={} "
                  "joint_states=[{},{},{}] floor=({},{},{},{})",
                  skeleton_frames_.load(), tracked_count, tracked_id,
                  tracked_body_z, head_y, left_hand_y, right_hand_y,
                  head_state, left_hand_state, right_hand_state,
                  skeleton_frame.vFloorClipPlane.x,
                  skeleton_frame.vFloorClipPlane.y,
                  skeleton_frame.vFloorClipPlane.z,
                  skeleton_frame.vFloorClipPlane.w);
            }
          } else {
            const uint64_t empty_results = ++skeleton_empty_results_;
            if (empty_results == 1 || (empty_results & 0x3FF) == 0) {
              XELOGD("Windows Kinect backend: skeleton GetNextFrame "
                     "result={:08X} consecutive_empty={}",
                     static_cast<uint32_t>(skeleton_result), empty_results);
            }
          }
        }
      }
    }
    ShutdownSensor();
    capture_running_.store(false);
    if (SUCCEEDED(com_result)) {
      CoUninitialize();
    }
  }

  void pump_depth_stream(DWORD timeout_ms) {
    if (!depth_stream_) {
      return;
    }
    NUI_IMAGE_FRAME frame = {};
    const HRESULT result = sensor_->NuiImageStreamGetNextFrame(
        depth_stream_, timeout_ms, &frame);
    if (FAILED(result) || result == S_FALSE || !frame.pFrameTexture) {
      const uint64_t empty_results = ++depth_empty_results_;
      if (empty_results == 1 || (empty_results & 0x3FF) == 0) {
        XELOGD("Windows Kinect backend: depth GetNextFrame result={:08X} "
               "texture={} consecutive_empty={}",
               static_cast<uint32_t>(result), frame.pFrameTexture ? "yes" : "no",
               empty_results);
      }
      return;
    }
    depth_empty_results_.store(0);

    // The packed texture supplies 16-bit depth/player words. The SDK's depth
    // pixel texture carries the same physical depth plus its separate hardware
    // player-index field. Prefer it whenever its documented 32-bit layout is
    // available; merely probing it and then continuing with the packed surface
    // discards the real player mask before it reaches the guest.
    INuiFrameTexture* depth_texture = frame.pFrameTexture;
    bool using_extended_depth_pixels = false;

    static std::atomic<uint64_t> extended_probe_count{0};
    const uint64_t extended_probe = ++extended_probe_count;
    BOOL extended_near_mode = FALSE;
    INuiFrameTexture* extended_texture = nullptr;
    const HRESULT extended_result =
        sensor_->NuiImageFrameGetDepthImagePixelFrameTexture(
            depth_stream_, &frame, &extended_near_mode, &extended_texture);
    if (SUCCEEDED(extended_result) && extended_texture) {
      NUI_LOCKED_RECT extended_locked = {};
      const HRESULT extended_lock_result =
          extended_texture->LockRect(0, &extended_locked, nullptr, 0);
      if (SUCCEEDED(extended_lock_result) && extended_locked.pBits &&
          extended_locked.Pitch >=
              static_cast<int>(depth_width_ * sizeof(NUI_DEPTH_IMAGE_PIXEL))) {
        if ((extended_probe & 0x1F) == 1) {
          uint32_t sampled_player_pixels = 0;
          const auto* pixels = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL*>(
              extended_locked.pBits);
          const uint32_t pitch_pixels =
              static_cast<uint32_t>(extended_locked.Pitch) /
              sizeof(NUI_DEPTH_IMAGE_PIXEL);
          for (uint32_t y = 0; y < depth_height_; y += 8) {
            for (uint32_t x = 0; x < depth_width_; x += 8) {
              sampled_player_pixels +=
                  pixels[y * pitch_pixels + x].playerIndex != 0;
            }
          }
          XELOGI("Windows Kinect backend: selected extended depth/player "
                 "surface native={}x{} pitch={} near={} "
                 "sampled_player_pixels={}",
                 depth_width_, depth_height_, extended_locked.Pitch,
                 extended_near_mode ? "enabled" : "normal",
                 sampled_player_pixels);
        }
        extended_texture->UnlockRect(0);
        depth_texture = extended_texture;
        using_extended_depth_pixels = true;
      } else {
        if ((extended_probe & 0x1F) == 1) {
          XELOGW("Windows Kinect backend: extended depth surface lock failed "
                 "hr={:08X} pitch={}",
                 static_cast<uint32_t>(extended_lock_result),
                 extended_locked.Pitch);
        }
        extended_texture->Release();
      }
    } else if (extended_probe <= 129) {
      XELOGI("Windows Kinect backend: extended depth surface unavailable "
             "hr={:08X}",
             static_cast<uint32_t>(extended_result));
    }

    NUI_LOCKED_RECT locked = {};
    const HRESULT lock_result =
        depth_texture->LockRect(0, &locked, nullptr, 0);
    if (SUCCEEDED(lock_result) && locked.pBits && locked.Pitch > 0) {
      constexpr uint32_t kRawWidth = 640;
      constexpr uint32_t kRawHeight = 480;
      constexpr uint32_t kLegacyWidth = 320;
      constexpr uint32_t kLegacyHeight = 240;
      constexpr uint32_t kBytesPerPixel = 2;
      std::vector<uint8_t> raw_depth(kRawWidth * kRawHeight * kBytesPerPixel);
      std::vector<uint8_t> legacy_depth(kLegacyWidth * kLegacyHeight *
                                        kBytesPerPixel);
      const auto* source = static_cast<const uint8_t*>(locked.pBits);
      const uint32_t source_pitch = static_cast<uint32_t>(locked.Pitch);
      uint16_t minimum_depth = UINT16_MAX;
      uint16_t maximum_depth = 0;
      uint16_t minimum_guest_depth = UINT16_MAX;
      uint16_t maximum_guest_depth = 0;
      uint16_t minimum_raw_shift = UINT16_MAX;
      uint16_t maximum_raw_shift = 0;
      uint32_t player_pixel_count = 0;
      for (uint32_t y = 0; y < kRawHeight; ++y) {
        const uint32_t source_y = y * depth_height_ / kRawHeight;
        const auto* source_row = source + source_y * source_pitch;
        auto* destination_row =
            raw_depth.data() + y * kRawWidth * kBytesPerPixel;
        for (uint32_t x = 0; x < kRawWidth; ++x) {
          const uint32_t source_x = x * depth_width_ / kRawWidth;
          const auto* source_pixel = source_row +
              source_x * (using_extended_depth_pixels ? sizeof(NUI_DEPTH_IMAGE_PIXEL) : 2u);
          const uint16_t packed_value =
              static_cast<uint16_t>(source_pixel[0]) |
              (static_cast<uint16_t>(source_pixel[1]) << 8);
          const auto* extended_pixel = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL*>(source_pixel);
          uint16_t player_index = using_extended_depth_pixels
              ? extended_pixel->playerIndex
              : packed_value & ((1u << NUI_IMAGE_PLAYER_INDEX_SHIFT) - 1u);
          const uint16_t depth_mm = using_extended_depth_pixels
              ? extended_pixel->depth
              : packed_value >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
          // Kinect SDK player labels are host-local and change as bodies are
          // acquired. The Windows bridge exposes the first Xbox profile as
          // the canonical guest player, whose depth label is 1.
          if (player_index != 0) {
            player_index = 1;
          }
          player_pixel_count += player_index != 0;
          const uint16_t guest_depth_mm = ApplySoftwareNearCompensation(
              depth_mm, software_near_mode_enabled_);
          const uint16_t raw_shift =
              KinectDepthMillimetersToRawShift(guest_depth_mm);
          const uint16_t pscam_depth =
              cvars::kinect_pscam_depth_format == 1 ? guest_depth_mm
                                                    : raw_shift;
          // PsCam depth words carry the player index in the low three bits.
          // Keep it when rewriting the depth/disparity value; dropping these
          // bits leaves a valid depth image but makes every pixel belong to
          // no player, so Kinect titles never receive a usable silhouette.
          const uint16_t guest_pscam_value = static_cast<uint16_t>(
              (pscam_depth << NUI_IMAGE_PLAYER_INDEX_SHIFT) | player_index);
          if (cvars::kinect_pscam_depth_byte_order == 1) {
            destination_row[x * 2] = static_cast<uint8_t>(guest_pscam_value);
            destination_row[x * 2 + 1] =
                static_cast<uint8_t>(guest_pscam_value >> 8);
          } else {
            destination_row[x * 2] =
                static_cast<uint8_t>(guest_pscam_value >> 8);
            destination_row[x * 2 + 1] =
                static_cast<uint8_t>(guest_pscam_value);
          }
          if (raw_shift != 0x7FF) {
            minimum_depth = std::min(minimum_depth, depth_mm);
            maximum_depth = std::max(maximum_depth, depth_mm);
            minimum_guest_depth =
                std::min(minimum_guest_depth, guest_depth_mm);
            maximum_guest_depth =
                std::max(maximum_guest_depth, guest_depth_mm);
            minimum_raw_shift = std::min(minimum_raw_shift, raw_shift);
            maximum_raw_shift = std::max(maximum_raw_shift, raw_shift);
          }
        }
      }
      for (uint32_t y = 0; y < kLegacyHeight; ++y) {
        const uint32_t source_y = y * depth_height_ / kLegacyHeight;
        const auto* source_row = source + source_y * source_pitch;
        for (uint32_t x = 0; x < kLegacyWidth; ++x) {
          const uint32_t source_x = x * depth_width_ / kLegacyWidth;
          const auto* source_pixel = source_row +
              source_x * (using_extended_depth_pixels ? sizeof(NUI_DEPTH_IMAGE_PIXEL) : 2u);
          const uint16_t packed_value =
              static_cast<uint16_t>(source_pixel[0]) |
              (static_cast<uint16_t>(source_pixel[1]) << 8);
          const auto* extended_pixel = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL*>(source_pixel);
          uint16_t player_index = using_extended_depth_pixels
              ? extended_pixel->playerIndex
              : packed_value & ((1u << NUI_IMAGE_PLAYER_INDEX_SHIFT) - 1u);
          const uint16_t depth_mm = using_extended_depth_pixels
              ? extended_pixel->depth
              : packed_value >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
          if (player_index != 0) {
            player_index = 1;
          }
          const uint16_t guest_depth_mm = ApplySoftwareNearCompensation(
              depth_mm, software_near_mode_enabled_);
          const uint16_t guest_value = static_cast<uint16_t>(
              (guest_depth_mm << NUI_IMAGE_PLAYER_INDEX_SHIFT) | player_index);
          const uint32_t destination_offset =
              (y * kLegacyWidth + x) * kBytesPerPixel;
          legacy_depth[destination_offset] =
              static_cast<uint8_t>(guest_value >> 8);
          legacy_depth[destination_offset + 1] =
              static_cast<uint8_t>(guest_value);
        }
      }
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_raw_depth_frame_ = std::move(raw_depth);
        latest_depth_frame_ = std::move(legacy_depth);
        latest_depth_timestamp_ =
            static_cast<uint64_t>(frame.liTimeStamp.QuadPart);
        latest_depth_native_frame_number_ = frame.dwFrameNumber;
        ++depth_frames_;
      }
      if (depth_frames_.load() == 1) {
        XELOGI("Windows Kinect backend: expanded canonical depth {}x{} -> "
               "PsCam 640x480 once",
               depth_width_, depth_height_);
      }
      if ((depth_frames_.load() & 0x3F) == 1) {
        XELOGI("Windows Kinect backend: captured PsCam depth frame={} "
               "native={}x{} pitch={} physical_mm={}..{} guest_mm={}..{} "
               "raw_shift={}..{} player_pixels={}",
               depth_frames_.load(), depth_width_, depth_height_, source_pitch,
               minimum_depth == UINT16_MAX ? 0 : minimum_depth, maximum_depth,
               minimum_guest_depth == UINT16_MAX ? 0 : minimum_guest_depth,
               maximum_guest_depth,
               minimum_raw_shift == UINT16_MAX ? 0 : minimum_raw_shift,
               maximum_raw_shift, player_pixel_count);
      }
      depth_texture->UnlockRect(0);
    }
    if (depth_texture != frame.pFrameTexture) {
      depth_texture->Release();
    }
    sensor_->NuiImageStreamReleaseFrame(depth_stream_, &frame);
  }

  void pump_color_stream(DWORD timeout_ms) {
    if (!color_stream_) {
      return;
    }
    NUI_IMAGE_FRAME frame = {};
    const HRESULT result = sensor_->NuiImageStreamGetNextFrame(
        color_stream_, timeout_ms, &frame);
    if (FAILED(result) || result == S_FALSE || !frame.pFrameTexture) {
      const uint64_t empty_results = ++color_empty_results_;
      if (empty_results == 1 || (empty_results & 0x3FF) == 0) {
        XELOGD("Windows Kinect backend: color GetNextFrame result={:08X} "
               "texture={} consecutive_empty={}",
               static_cast<uint32_t>(result), frame.pFrameTexture ? "yes" : "no",
               empty_results);
      }
      return;
    }
    color_empty_results_.store(0);

    NUI_LOCKED_RECT locked = {};
    const HRESULT lock_result =
        frame.pFrameTexture->LockRect(0, &locked, nullptr, 0);
    if (SUCCEEDED(lock_result) && locked.pBits && locked.Pitch > 0) {
      constexpr uint32_t kWidth = 640;
      constexpr uint32_t kHeight = 480;
      constexpr uint32_t kBgraBytesPerPixel = 4;
      const uint32_t bgra_row_bytes = kWidth * kBgraBytesPerPixel;
      std::vector<uint8_t> raw_color(kWidth * kHeight);
      std::vector<uint8_t> color(bgra_row_bytes * kHeight);
      const auto* source = static_cast<const uint8_t*>(locked.pBits);
      const uint32_t source_pitch = static_cast<uint32_t>(locked.Pitch);
      if (color_is_raw_bayer_) {
        for (uint32_t y = 0; y < kHeight; ++y) {
          std::memcpy(raw_color.data() + y * kWidth,
                      source + y * source_pitch,
                      std::min(kWidth, source_pitch));
        }
        // Kinect v1 raw Bayer is GRBG. Keep a BGRA copy for the HLE envelope
        // while the untouched Bayer bytes feed the low-level PsCam path.
        for (uint32_t y = 0; y < kHeight; y += 2) {
          for (uint32_t x = 0; x < kWidth; x += 2) {
            const uint32_t first = y * kWidth + x;
            const uint32_t second = first + kWidth;
            const uint8_t green_first = raw_color[first];
            const uint8_t red = raw_color[first + 1];
            const uint8_t blue = raw_color[second];
            const uint8_t green_second = raw_color[second + 1];
            auto set_bgra = [&](uint32_t pixel, uint8_t green) {
              auto* output = color.data() + pixel * kBgraBytesPerPixel;
              output[0] = blue;
              output[1] = green;
              output[2] = red;
              output[3] = 0xFF;
            };
            set_bgra(first, green_first);
            set_bgra(first + 1, green_first);
            set_bgra(second, green_second);
            set_bgra(second + 1, green_second);
          }
        }
      } else {
        for (uint32_t y = 0; y < kHeight; ++y) {
          std::memcpy(color.data() + y * bgra_row_bytes,
                      source + y * source_pitch,
                      std::min(bgra_row_bytes, source_pitch));
        }
        for (uint32_t y = 0; y < kHeight; ++y) {
          for (uint32_t x = 0; x < kWidth; ++x) {
            const auto* pixel =
                color.data() + (y * kWidth + x) * kBgraBytesPerPixel;
            const bool even_row = (y & 1) == 0;
            const bool even_column = (x & 1) == 0;
            raw_color[y * kWidth + x] =
                even_row ? (even_column ? pixel[1] : pixel[2])
                         : (even_column ? pixel[0] : pixel[1]);
          }
        }
      }
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_raw_color_frame_ = std::move(raw_color);
        latest_color_frame_ = std::move(color);
        latest_color_timestamp_ =
            static_cast<uint64_t>(frame.liTimeStamp.QuadPart);
        latest_color_native_frame_number_ = frame.dwFrameNumber;
        ++color_frames_;
      }
      if ((color_frames_.load() & 0x3F) == 1) {
        XELOGI("Windows Kinect backend: captured raw Bayer frame={} {}x{} "
               "pitch={} source={}",
               color_frames_.load(), kWidth, kHeight, source_pitch,
               color_is_raw_bayer_ ? "native" : "rgb32-fallback");
      }
      frame.pFrameTexture->UnlockRect(0);
    }
    sensor_->NuiImageStreamReleaseFrame(color_stream_, &frame);
  }

  static constexpr int32_t kNoElevationRequest =
      std::numeric_limits<int32_t>::max();

  INuiSensor* sensor_ = nullptr;
  HANDLE color_event_ = nullptr;
  HANDLE depth_event_ = nullptr;
  HANDLE skeleton_event_ = nullptr;
  HANDLE frame_end_event_ = nullptr;
  HANDLE color_stream_ = nullptr;
  HANDLE depth_stream_ = nullptr;
  bool initialized_ = false;
  bool color_is_raw_bayer_ = false;
  bool depth_has_player_index_ = false;
  bool near_mode_enabled_ = false;
  bool software_near_mode_enabled_ = false;
  bool skeleton_requested_ = false;
  bool skeleton_enabled_ = false;
  uint32_t depth_width_ = 0;
  uint32_t depth_height_ = 0;
  std::atomic<bool> ready_{false};
  std::mutex initialization_mutex_;
  std::chrono::steady_clock::time_point next_initialization_attempt_{};
  std::atomic<bool> capture_running_{false};
  std::thread capture_thread_;
  mutable std::mutex frame_mutex_;
  std::mutex title_frame_callback_mutex_;
  std::function<void(const NUI_SKELETON_FRAME&)> title_frame_callback_;
  NUI_SKELETON_FRAME latest_skeleton_frame_ = {};
  bool latest_skeleton_valid_ = false;
  std::array<float, 4> latest_accelerometer_ = {};
  bool latest_accelerometer_valid_ = false;
  uint32_t latest_accelerometer_sequence_ = 0;
  std::vector<uint8_t> latest_color_frame_;
  std::vector<uint8_t> latest_depth_frame_;
  std::vector<uint8_t> latest_raw_color_frame_;
  std::vector<uint8_t> latest_raw_depth_frame_;
  std::vector<uint8_t> coordinate_mapper_parameters_;
  uint64_t latest_color_timestamp_ = 0;
  uint64_t latest_depth_timestamp_ = 0;
  uint32_t latest_color_native_frame_number_ = 0;
  uint32_t latest_depth_native_frame_number_ = 0;
  std::atomic<uint64_t> color_frames_{0};
  std::atomic<uint64_t> depth_frames_{0};
  std::atomic<uint64_t> depth_empty_results_{0};
  std::atomic<uint64_t> color_empty_results_{0};
  std::atomic<uint64_t> skeleton_empty_results_{0};
  std::atomic<uint64_t> capture_polls_{0};
  std::atomic<uint32_t> tracked_count_{0};
  std::atomic<uint64_t> skeleton_frames_{0};
  std::atomic<int32_t> requested_elevation_angle_{kNoElevationRequest};
  std::atomic<int32_t> last_reported_elevation_angle_{0};
};

WindowsNuiBackend& windows_nui_backend() {
  static WindowsNuiBackend backend;
  return backend;
}
#endif

bool windows_nui_device_ready() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  return IsSyntheticKinectFrameActive() || windows_nui_backend().ready();
#else
  return false;
#endif
}

bool nui_guest_device_ready() {
  return IsSyntheticKinectFrameActive() || windows_nui_device_ready();
}

// XamShowNuiTroubleshooterUI normally enters the XAM NUI system path before
// the dashboard decides whether to display the troubleshooter. Keep that
// initialization path alive when a native Windows Kinect is attached.
void InitializeWindowsNuiGuestPath() {
  static uint32_t guest_buffer = 0;
  if (!guest_buffer) {
    guest_buffer = kernel_state()->memory()->SystemHeapAlloc(12, 4);
    if (!guest_buffer) {
      XELOGE("Windows Kinect: failed to allocate XAM NUI init buffer");
      return;
    }
    auto words = kernel_state()->memory()->TranslateVirtual<xe::be<uint32_t>*>(
        guest_buffer);
    words[0] = 0;
    words[1] = 0;
    words[2] = 0;
  }

  auto result = kernel_state()->app_manager()->DispatchMessageSync(
      0xFE, 0x00021028, guest_buffer, 12);
  XELOGI("Windows Kinect: XAM NUI init call 0xFE/0x21028 result={:08X} "
         "buffer={:08X}",
         static_cast<uint32_t>(result), guest_buffer);
}

// The recovered NUI clients in Kinect Sports, Dance Central 3, Just Dance
// 2014, and dashnui use the same XStudio setup sequence. 0x1410 describes a
// 56-byte delivery configuration, then 0x1401 supplies the first title-owned
// 2736-byte frame buffer before the recurring 0x1402 submissions begin.
// Keep this state in XAM so all three calls form one protocol rather than
// treating the setup notifications as unrelated success responses.
struct XStudioNuiContractState {
  void Reset() {
    std::lock_guard<std::mutex> lock(mutex);
    configured_version = 0;
    configuration_timestamp = 0;
    bootstrap_frame = 0;
    bootstrap_timestamp = 0;
    configured = false;
  }

  void Configure(uint32_t version, uint64_t timestamp) {
    std::lock_guard<std::mutex> lock(mutex);
    configured_version = version;
    configuration_timestamp = timestamp;
    configured = true;
  }

  bool RecordBootstrapFrame(uint32_t frame, uint32_t version,
                            uint64_t timestamp) {
    std::lock_guard<std::mutex> lock(mutex);
    bootstrap_frame = frame;
    bootstrap_timestamp = timestamp;
    // A bootstrap frame is valid even if a title issues it before its timing
    // configuration. The declared writable frame buffer is independent from
    // the optional delivery-clock state.
    return configured && configured_version == version;
  }

  bool IsConfiguredFor(uint32_t version) const {
    std::lock_guard<std::mutex> lock(mutex);
    return configured && configured_version == version;
  }

 private:
  mutable std::mutex mutex;
  uint32_t configured_version = 0;
  uint64_t configuration_timestamp = 0;
  uint32_t bootstrap_frame = 0;
  uint64_t bootstrap_timestamp = 0;
  bool configured = false;
};

struct XStudioPacket {
  std::vector<uint8_t> descriptor;
  std::vector<uint8_t> payload;
};

class XStudioPacketTransport {
 public:
  void Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    packets_.clear();
  }

  void Send(XStudioPacket packet) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // The retail producer has two rotating buffers. Bound the host queue as
      // well so a stalled consumer cannot retain unbounded guest data.
      if (packets_.size() >= 2) {
        packets_.pop_front();
      }
      packets_.push_back(std::move(packet));
    }
    condition_.notify_one();
  }

  bool Receive(uint32_t timeout_ms, XStudioPacket* packet) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                             [this] { return !packets_.empty(); })) {
      return false;
    }
    *packet = std::move(packets_.front());
    packets_.pop_front();
    return true;
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<XStudioPacket> packets_;
};

XStudioPacketTransport& xstudio_packet_transport() {
  static XStudioPacketTransport transport;
  return transport;
}

XStudioNuiContractState& xstudio_nui_contract_state() {
  static XStudioNuiContractState state;
  return state;
}

std::atomic<uint32_t> nui_submitter_thread_id{0};
std::atomic<uint64_t> nui_submission_time_ms{0};
std::atomic<uint32_t> nui_consumer_event_address{0};
std::atomic<uint32_t> nui_consumer_ring_base{0};
std::atomic<uint64_t> nui_last_synthetic_ring_frame{0};
// The SDK callback may be installed repeatedly by a title polling 0x1402.
// Keep the producer cadence tied to a new sensor sample, not to that polling
// rate, or the two-slot retail ring is overwritten before the title can
// observe its notification edge.
std::atomic<uint64_t> nui_last_physical_ring_timestamp{0};
std::atomic<bool> nui_consumer_event_reset_seen{false};
std::mutex nui_synthetic_ring_thread_mutex;
std::jthread nui_synthetic_ring_thread;
constexpr size_t kNuiWaitHistorySlots = 256;
std::array<std::atomic<uint32_t>, kNuiWaitHistorySlots> nui_last_wait_thread{};
std::array<std::atomic<uint32_t>, kNuiWaitHistorySlots> nui_last_wait_object{};
std::array<std::atomic<uint32_t>, kNuiWaitHistorySlots> nui_last_wait_caller{};

uint64_t NuiHostMonotonicMilliseconds() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

class XStudioDeviceCallbackRegistration {
 public:
  void Store(uint32_t callback, uint32_t state_mask, uint32_t event_mask) {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = callback & ~1u;
    state_mask_ = state_mask;
    event_mask_ = event_mask;
    ++generation_;
  }

  void Clear(uint32_t callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!callback || callback_ == (callback & ~1u)) {
      callback_ = 0;
      ++generation_;
    }
  }

  bool Snapshot(uint32_t* callback, uint32_t* state_mask,
                uint32_t* event_mask, uint64_t* generation) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!callback_) {
      return false;
    }
    *callback = callback_;
    *state_mask = state_mask_;
    *event_mask = event_mask_;
    *generation = generation_;
    return true;
  }

  bool Matches(uint32_t callback, uint64_t generation) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return callback_ == callback && generation_ == generation;
  }

 private:
  mutable std::mutex mutex_;
  uint32_t callback_ = 0;
  uint32_t state_mask_ = 0;
  uint32_t event_mask_ = 0;
  uint64_t generation_ = 0;
};

XStudioDeviceCallbackRegistration& xstudio_device_callback_registration() {
  static XStudioDeviceCallbackRegistration registration;
  return registration;
}

void QueueXStudioDeviceCallback(uint32_t device_state, uint32_t event_type,
                                uint32_t event_value) {
  uint32_t callback = 0;
  uint32_t state_mask = 0;
  uint32_t event_mask = 0;
  uint64_t generation = 0;
  if (!xstudio_device_callback_registration().Snapshot(
          &callback, &state_mask, &event_mask, &generation)) {
    return;
  }
  if (!(device_state & state_mask) ||
      (event_type < 32 && !(event_mask & (1u << event_type)))) {
    return;
  }
  auto* state = kernel_state();
  if (!state) {
    return;
  }
  const bool queued = state->QueueKernelDispatch(
      [state, callback, generation, device_state, event_type, event_value]() {
        if (KernelState::shared() != state ||
            !xstudio_device_callback_registration().Matches(callback,
                                                             generation)) {
          return;
        }
        auto thread = XThread::GetCurrentThread();
        if (!thread) {
          return;
        }
        uint64_t args[] = {device_state, event_type, event_value};
        state->processor()->Execute(thread->thread_state(), callback, args,
                                    xe::countof(args));
      });
  XELOGI("XamXStudioRequest: device callback state={:08X} type={} value={} "
         "callback={:08X} queued={}",
         device_state, event_type, event_value, callback, queued);
}

}  // namespace

void InitializeWindowsNuiGuestPathForMessage() {
  InitializeWindowsNuiGuestPath();
}

void StartWindowsNuiBackend() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (cvars::kinect_synthetic_test_frames) {
    XELOGI("Windows Kinect backend: synthetic fixture selected; skipped "
           "physical sensor startup wait");
    return;
  }
  // A physical Kinect v1 commonly needs longer than the old three-second
  // window before NuiInitialize and the image streams are both available.
  // Wait for the real sensor rather than exposing a transient "not present"
  // state that a title can cache for the rest of its session.
  constexpr auto kInitialNuiStartupTimeout = std::chrono::seconds(12);
  const bool ready =
      windows_nui_backend().WaitForInitialReady(kInitialNuiStartupTimeout);
  XELOGI("Windows Kinect backend: XAM startup wait={}ms ready={}",
         std::chrono::duration_cast<std::chrono::milliseconds>(
             kInitialNuiStartupTimeout)
             .count(),
         ready);
#endif
}

bool IsWindowsNuiBackendReady() { return nui_guest_device_ready(); }

void WindowsNuiPump() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (!IsSyntheticKinectFrameActive()) {
    windows_nui_backend().pump();
  }
#endif
}

bool WindowsNuiCopyLatestColorFrame(std::vector<uint8_t>* out) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  auto& backend = windows_nui_backend();
  return backend.ready() && backend.copy_latest_color(out);
#else
  if (out) {
    out->clear();
  }
  return false;
#endif
}

bool WindowsNuiCopyLatestDepthFrame(std::vector<uint8_t>* out) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  auto& backend = windows_nui_backend();
  return backend.ready() && backend.copy_latest_depth(out);
#else
  if (out) {
    out->clear();
  }
  return false;
#endif
}

bool WindowsNuiCopyLatestRawColorFrame(std::vector<uint8_t>* out,
                                       uint64_t* timestamp,
                                       uint32_t* frame_number) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  // Synthetic mode is an end-to-end PsCam fixture. Returning no host image
  // deliberately selects the deterministic producer in xboxkrnl_usbcam so
  // its color/depth pair cannot be mixed with this synthetic skeleton stream.
  if (IsSyntheticKinectFrameActive()) {
    return false;
  }
  auto& backend = windows_nui_backend();
  return backend.ready() &&
         backend.copy_latest_raw_color(out, timestamp, frame_number);
#else
  if (out) {
    out->clear();
  }
  if (timestamp) {
    *timestamp = 0;
  }
  if (frame_number) {
    *frame_number = 0;
  }
  return false;
#endif
}

bool WindowsNuiCopyLatestRawDepthFrame(std::vector<uint8_t>* out,
                                       uint64_t* timestamp,
                                       uint32_t* frame_number) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (IsSyntheticKinectFrameActive()) {
    return false;
  }
  auto& backend = windows_nui_backend();
  return backend.ready() &&
         backend.copy_latest_raw_depth(out, timestamp, frame_number);
#else
  if (out) {
    out->clear();
  }
  if (timestamp) {
    *timestamp = 0;
  }
  if (frame_number) {
    *frame_number = 0;
  }
  return false;
#endif
}

bool WindowsNuiCopyLatestNativeDepthFrame(std::vector<uint8_t>* out,
                                          uint64_t* timestamp,
                                          uint32_t* frame_number) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (IsSyntheticKinectFrameActive()) {
    return false;
  }
  auto& backend = windows_nui_backend();
  return backend.ready() &&
         backend.copy_latest_native_depth(out, timestamp, frame_number);
#else
  if (out) {
    out->clear();
  }
  if (timestamp) {
    *timestamp = 0;
  }
  if (frame_number) {
    *frame_number = 0;
  }
  return false;
#endif
}

bool WindowsNuiCopyCoordinateMapperParameters(std::vector<uint8_t>* out) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  auto& backend = windows_nui_backend();
  return backend.ready() && backend.copy_coordinate_mapper_parameters(out);
#else
  if (out) {
    out->clear();
  }
  return false;
#endif
}

bool WindowsNuiGetUniqueId(std::string* out) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  return windows_nui_backend().copy_unique_id(out);
#else
  if (out) {
    out->clear();
  }
  return false;
#endif
}

bool WindowsNuiFrameAvailable() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (IsSyntheticKinectFrameActive()) {
    return true;
  }
  auto& backend = windows_nui_backend();
  return backend.ready() &&
         (backend.skeleton_frames() != 0 || backend.color_frames() != 0 ||
          backend.depth_frames() != 0);
#else
  return false;
#endif
}

uint32_t WindowsNuiTrackedCount() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (IsSyntheticKinectFrameActive()) {
    return 1;
  }
  return windows_nui_backend().tracked_count();
#else
  return 0;
#endif
}

uint64_t WindowsNuiSkeletonFrameCount() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (IsSyntheticKinectFrameActive()) {
    return GetTickCount64() / 33 + 1;
  }
  auto& backend = windows_nui_backend();
  return backend.ready() ? backend.skeleton_frames() : 0;
#else
  return 0;
#endif
}

uint64_t WindowsNuiColorFrameCount() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  auto& backend = windows_nui_backend();
  return backend.ready() ? backend.color_frames() : 0;
#else
  return 0;
#endif
}

uint64_t WindowsNuiDepthFrameCount() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  auto& backend = windows_nui_backend();
  return backend.ready() ? backend.depth_frames() : 0;
#else
  return 0;
#endif
}

int WindowsNuiBestSkeletonIndex() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (IsSyntheticKinectFrameActive()) {
    return 0;
  }
  return windows_nui_backend().best_skeleton_index();
#else
  return -1;
#endif
}

bool WindowsNuiGetElevationAngle(int32_t* out_angle) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  return windows_nui_backend().get_elevation_angle(out_angle);
#else
  if (out_angle) {
    *out_angle = 0;
  }
  return false;
#endif
}

bool WindowsNuiSetElevationAngle(int32_t angle) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  return windows_nui_backend().set_elevation_angle(angle);
#else
  (void)angle;
  return false;
#endif
}

bool WindowsNuiCopyLatestAccelerometer(std::array<float, 4>* out,
                                       uint32_t* sample_sequence) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  auto& backend = windows_nui_backend();
  return backend.ready() &&
         backend.copy_latest_accelerometer(out, sample_sequence);
#else
  if (out) {
    *out = {};
  }
  if (sample_sequence) {
    *sample_sequence = 0;
  }
  return false;
#endif
}

bool WindowsNuiRememberFloor() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  auto& backend = windows_nui_backend();
  return backend.ready() && backend.remember_floor();
#else
  return false;
#endif
}

#if defined(XENIA_HAS_KINECT_V1_SDK)
// Xbox 360 is big-endian. Keep this layout deliberately separate from the
// Windows SDK layout so no host pointer, enum size, or compiler padding leaks
// into guest memory.
struct X_NUI_GUEST_VECTOR4 {
  xe::be<float> x;
  xe::be<float> y;
  xe::be<float> z;
  xe::be<float> w;
};

struct X_NUI_GUEST_SKELETON_DATA {
  xe::be<uint32_t> tracking_state;
  xe::be<uint32_t> tracking_id;
  xe::be<uint32_t> enrollment_index;
  xe::be<uint32_t> user_index;
  X_NUI_GUEST_VECTOR4 position;
  X_NUI_GUEST_VECTOR4 joints[NUI_SKELETON_POSITION_COUNT];
  xe::be<uint32_t> joint_tracking_state[NUI_SKELETON_POSITION_COUNT];
  xe::be<uint32_t> quality_flags;
  // The Xbox NUI ABI rounds each 436-byte Windows-compatible record to the
  // 448-byte stride used by skeleton notifications.
  xe::be<uint32_t> reserved[3];
};

struct X_NUI_GUEST_SKELETON_FRAME {
  xe::be<uint64_t> timestamp;
  xe::be<uint32_t> frame_number;
  xe::be<uint32_t> flags;
  X_NUI_GUEST_VECTOR4 floor_clip_plane;
  X_NUI_GUEST_VECTOR4 normal_to_gravity;
  X_NUI_GUEST_SKELETON_DATA skeletons[NUI_SKELETON_COUNT];
};

static_assert(sizeof(X_NUI_GUEST_SKELETON_DATA) == 448,
              "Xbox NUI skeleton record layout changed");
static_assert(sizeof(X_NUI_GUEST_SKELETON_FRAME) == 2736,
              "Xbox NUI skeleton frame layout changed");

struct X_NUI_GUEST_MEDIA_STREAM {
  xe::be<uint32_t> data;
  xe::be<uint32_t> size;
  xe::be<uint32_t> width;
  xe::be<uint32_t> height;
  xe::be<uint32_t> pitch;
  xe::be<uint32_t> format;
  xe::be<uint32_t> frame_number;
  xe::be<uint32_t> reserved;
};

// The skeleton frame remains at offset zero for compatibility with existing
// XAM callers. This extension is the bridge-owned media envelope appended to
// internally allocated frames returned by the NUI message path.
struct X_NUI_GUEST_FRAME_ENVELOPE {
  xe::be<uint32_t> magic;
  xe::be<uint32_t> version;
  xe::be<uint32_t> size;
  xe::be<uint32_t> flags;
  xe::be<uint32_t> skeleton;
  xe::be<uint32_t> skeleton_size;
  xe::be<uint64_t> timestamp;
  xe::be<uint32_t> frame_number;
  xe::be<uint32_t> tracked_count;
  X_NUI_GUEST_MEDIA_STREAM color;
  X_NUI_GUEST_MEDIA_STREAM depth;
};

static_assert(sizeof(X_NUI_GUEST_MEDIA_STREAM) == 32,
              "NUI media stream layout changed");
static_assert(sizeof(X_NUI_GUEST_FRAME_ENVELOPE) == 104,
              "NUI envelope layout changed");

static_assert(NUI_SKELETON_POSITION_COUNT == 20, "Unexpected Kinect joint count");

std::array<uint32_t, 3> windows_nui_guest_frames = {};
std::array<uint32_t, 3> windows_nui_guest_color_buffers = {};
std::array<uint32_t, 3> windows_nui_guest_depth_buffers = {};
uint32_t windows_nui_guest_frame_index = 0;

constexpr uint32_t kNuiColorWidth = 640;
constexpr uint32_t kNuiColorHeight = 480;
constexpr uint32_t kNuiColorPitch = kNuiColorWidth * 4;
constexpr uint32_t kNuiColorSize = kNuiColorPitch * kNuiColorHeight;
constexpr uint32_t kNuiDepthWidth = 320;
constexpr uint32_t kNuiDepthHeight = 240;
constexpr uint32_t kNuiDepthPitch = kNuiDepthWidth * 2;
constexpr uint32_t kNuiDepthSize = kNuiDepthPitch * kNuiDepthHeight;
constexpr uint32_t kNuiEnvelopeMagic = 0x4E554945;  // "NUIE".

uint32_t WindowsNuiGuestFrameAllocationSize() {
  return sizeof(X_NUI_GUEST_SKELETON_FRAME) +
         sizeof(X_NUI_GUEST_FRAME_ENVELOPE);
}

bool WindowsNuiIsInternalGuestFrame(uint32_t guest_ptr) {
  return std::find(windows_nui_guest_frames.begin(),
                   windows_nui_guest_frames.end(),
                   guest_ptr) != windows_nui_guest_frames.end();
}

void WindowsNuiWriteGuestEnvelope(uint32_t frame_index, uint32_t guest_frame,
                                  const NUI_SKELETON_FRAME& native_frame) {
  auto& color_ptr = windows_nui_guest_color_buffers[frame_index];
  auto& depth_ptr = windows_nui_guest_depth_buffers[frame_index];
  std::vector<uint8_t> color;
  std::vector<uint8_t> depth;

#if defined(XENIA_HAS_KINECT_V1_SDK)
  const bool has_color = windows_nui_backend().copy_latest_color(&color);
  const bool has_depth = windows_nui_backend().copy_latest_depth(&depth);
#else
  const bool has_color = false;
  const bool has_depth = false;
#endif

  if (has_color && !color_ptr) {
    color_ptr = kernel_state()->memory()->SystemHeapAlloc(kNuiColorSize, 64);
  }
  if (has_depth && !depth_ptr) {
    depth_ptr = kernel_state()->memory()->SystemHeapAlloc(kNuiDepthSize, 64);
  }
  if (has_color && color_ptr && color.size() >= kNuiColorSize) {
    std::memcpy(kernel_state()->memory()->TranslateVirtual<void*>(color_ptr),
                color.data(), kNuiColorSize);
  }
  if (has_depth && depth_ptr && depth.size() >= kNuiDepthSize) {
    std::memcpy(kernel_state()->memory()->TranslateVirtual<void*>(depth_ptr),
                depth.data(), kNuiDepthSize);
  }

  auto envelope = kernel_state()->memory()->TranslateVirtual<
      X_NUI_GUEST_FRAME_ENVELOPE*>(guest_frame +
                                    sizeof(X_NUI_GUEST_SKELETON_FRAME));
  envelope->magic = kNuiEnvelopeMagic;
  envelope->version = 1;
  envelope->size = sizeof(X_NUI_GUEST_FRAME_ENVELOPE);
  envelope->flags = 1u | (has_color && color_ptr ? 2u : 0u) |
                    (has_depth && depth_ptr ? 4u : 0u);
  envelope->skeleton = guest_frame;
  envelope->skeleton_size = sizeof(X_NUI_GUEST_SKELETON_FRAME);
  envelope->timestamp = static_cast<uint64_t>(native_frame.liTimeStamp.QuadPart);
  envelope->frame_number = native_frame.dwFrameNumber;
  envelope->tracked_count = WindowsNuiTrackedCount();
  envelope->color.data = color_ptr;
  envelope->color.size = color_ptr ? kNuiColorSize : 0;
  envelope->color.width = kNuiColorWidth;
  envelope->color.height = kNuiColorHeight;
  envelope->color.pitch = kNuiColorPitch;
  envelope->color.format = 1;  // BGRA8, as delivered by Kinect v1.
  envelope->color.frame_number = WindowsNuiColorFrameCount();
  envelope->depth.data = depth_ptr;
  envelope->depth.size = depth_ptr ? kNuiDepthSize : 0;
  envelope->depth.width = kNuiDepthWidth;
  envelope->depth.height = kNuiDepthHeight;
  envelope->depth.pitch = kNuiDepthPitch;
  envelope->depth.format = 2;  // 16-bit depth/player-index words.
  envelope->depth.frame_number = WindowsNuiDepthFrameCount();
}

void WindowsNuiConvertSkeletonFrame(X_NUI_GUEST_SKELETON_FRAME* guest,
                                    const NUI_SKELETON_FRAME& native_frame) {
  std::memset(guest, 0, sizeof(*guest));
  guest->timestamp = static_cast<uint64_t>(native_frame.liTimeStamp.QuadPart);
  guest->frame_number = native_frame.dwFrameNumber;
  guest->flags = native_frame.dwFlags;

  auto write_vector = [](X_NUI_GUEST_VECTOR4& destination,
                         const Vector4& source) {
    destination.x = source.x;
    destination.y = source.y;
    destination.z = source.z;
    destination.w = source.w;
  };
  write_vector(guest->floor_clip_plane, native_frame.vFloorClipPlane);
  write_vector(guest->normal_to_gravity, native_frame.vNormalToGravity);

  for (uint32_t skeleton_index = 0; skeleton_index < NUI_SKELETON_COUNT;
       ++skeleton_index) {
    const auto& source = native_frame.SkeletonData[skeleton_index];
    auto& destination = guest->skeletons[skeleton_index];
    destination.tracking_state = source.eTrackingState;
    destination.tracking_id = source.dwTrackingID;
    destination.enrollment_index = source.dwEnrollmentIndex;
    destination.user_index = source.dwUserIndex;
    write_vector(destination.position, source.Position);
    for (uint32_t joint = 0; joint < NUI_SKELETON_POSITION_COUNT; ++joint) {
      write_vector(destination.joints[joint], source.SkeletonPositions[joint]);
      destination.joint_tracking_state[joint] =
          source.eSkeletonPositionTrackingState[joint];
    }
    destination.quality_flags = source.dwQualityFlags;
  }
}

void WindowsNuiBindPrimarySkeletonToGuestUser(NUI_SKELETON_FRAME* frame) {
  if (!frame) {
    return;
  }

  int primary_index = -1;
  for (int index = 0; index < NUI_SKELETON_COUNT; ++index) {
    if (frame->SkeletonData[index].eTrackingState == NUI_SKELETON_TRACKED) {
      primary_index = index;
      break;
    }
  }
  if (primary_index < 0) {
    return;
  }

  const uint32_t host_slot = static_cast<uint32_t>(primary_index);
  const uint32_t host_user_index = frame->SkeletonData[primary_index].dwUserIndex;
  const uint32_t host_enrollment_index =
      frame->SkeletonData[primary_index].dwEnrollmentIndex;
  if (primary_index != 0) {
    const auto primary = frame->SkeletonData[primary_index];
    frame->SkeletonData[primary_index] = {};
    frame->SkeletonData[0] = primary;
  }

  auto& guest_body = frame->SkeletonData[0];
  // Kinect for Windows has no Xbox profile association service and reports
  // 0xFE (no user) for a physical body. The standalone backend exposes the
  // primary body as NUI slot 0, which XamUserNuiGetUserIndex maps to local
  // player 0. Preserve all measured skeleton data while applying that same
  // general XAM user binding to the frame consumed by titles.
  guest_body.dwUserIndex = 0;
  // The retail titles use the enrollment index as the key joining a tracked
  // body to their local NUI session. Windows Kinect does not supply the Xbox
  // biometric record, so bind the primary guest body to XAM's stable guest
  // enrollment token instead of exposing the SDK's no-user sentinel.
  guest_body.dwEnrollmentIndex = kNuiGuestEnrollmentIndex;

  static std::atomic<uint64_t> binding_count{0};
  static std::atomic<uint32_t> previous_host_slot{UINT32_MAX};
  static std::atomic<uint32_t> previous_host_user{UINT32_MAX};
  static std::atomic<uint32_t> previous_tracking_id{0};
  const uint64_t sequence = ++binding_count;
  const bool binding_changed =
      previous_host_slot.exchange(host_slot) != host_slot ||
      previous_host_user.exchange(host_user_index) != host_user_index ||
      previous_tracking_id.exchange(guest_body.dwTrackingID) !=
          guest_body.dwTrackingID;
  if (sequence <= 4 || (sequence & 0x7F) == 0 || binding_changed) {
    XELOGI("Windows Kinect: normalized host skeleton slot={} user={} "
           "enrollment={} tracking={} to guest slot=0 user={} enrollment={} "
           "sequence={}",
           host_slot, host_user_index, host_enrollment_index,
           guest_body.dwTrackingID, guest_body.dwUserIndex,
           guest_body.dwEnrollmentIndex, sequence);
  }
}

// The HUD query path and the title-owned 2B002 path must observe the same
// normalized frame. Derive engagement from that frame rather than from a
// separate capture-side counter, which may be one frame behind or may refer to
// a body before it is remapped to guest slot zero.
uint32_t WindowsNuiSynchronizeEngagementState(
    const NUI_SKELETON_FRAME& frame) {
  uint32_t tracked_count = 0;
  uint32_t engaged_id = 0;
  uint32_t enrollment_index = UINT32_MAX;
  for (const auto& body : frame.SkeletonData) {
    if (body.eTrackingState != NUI_SKELETON_TRACKED) {
      continue;
    }
    ++tracked_count;
    if (engaged_id == 0 && body.dwTrackingID != 0) {
      engaged_id = body.dwTrackingID;
      enrollment_index = body.dwEnrollmentIndex;
    }
  }

  engaged_tracking_id = engaged_id;
  engaged_enrollment_index = enrollment_index;
  nui_unknown_1 = engaged_id != 0 ? 1u : 0u;
  nui_unknown_2 = engaged_id != 0 ? '\1' : '\0';
  return tracked_count;
}

void WindowsNuiBuildSyntheticFrame(NUI_SKELETON_FRAME* frame) {
  if (!frame || !IsSyntheticKinectFrameActive()) {
    return;
  }

  // PsCam and skeleton consumers correlate streams by frame number and
  // timestamp. Derive both from the same 30 Hz host clock used by the
  // synthetic camera path, and keep them stable when a title polls 0x1402
  // multiple times during one sensor frame.
  const uint64_t sequence = GetTickCount64() / 33 + 1;
  frame->dwFrameNumber = static_cast<DWORD>(sequence);
  frame->liTimeStamp.QuadPart = static_cast<LONGLONG>(sequence * 33);
  // A deterministic fixture has no measured room geometry. Leave the plane
  // unavailable instead of injecting a fake floor into title calculations.
  frame->vFloorClipPlane = {};
  frame->vNormalToGravity = {0.0f, -1.0f, 0.0f, 0.0f};

  // Synthetic frames must not retain bodies from the last physical capture.
  // A mixed frame advertises two users to the guest despite this fixture
  // deliberately modeling one deterministic player.
  for (auto& skeleton : frame->SkeletonData) {
    skeleton = {};
  }
  auto& body = frame->SkeletonData[0];
  body = {};
  body.eTrackingState = NUI_SKELETON_TRACKED;
  body.dwTrackingID = 1;
  // The fixture models a signed-in local player so it can exercise title-side
  // gesture acquisition without the physical SDK's identity enrollment flow.
  body.dwUserIndex = 0;
  body.dwEnrollmentIndex = 0;
  constexpr float kSyntheticPlayerDepthMeters = 1.85f;
  body.Position = {0.0f, 0.0f, kSyntheticPlayerDepthMeters, 1.0f};
  for (uint32_t joint = 0; joint < NUI_SKELETON_POSITION_COUNT; ++joint) {
    body.eSkeletonPositionTrackingState[joint] =
        NUI_SKELETON_POSITION_TRACKED;
    body.SkeletonPositions[joint] = {
        0.0f, 0.0f, kSyntheticPlayerDepthMeters, 1.0f};
  }

  auto set_joint = [&](NUI_SKELETON_POSITION_INDEX joint, float x, float y,
                       float z) {
    body.SkeletonPositions[joint] = {x, y, z, 1.0f};
  };
  set_joint(NUI_SKELETON_POSITION_HEAD, 0.0f, 0.95f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_SHOULDER_CENTER, 0.0f, 0.65f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_SHOULDER_LEFT, -0.24f, 0.62f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_SHOULDER_RIGHT, 0.24f, 0.62f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_ELBOW_LEFT, -0.38f, 0.35f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_WRIST_LEFT, -0.40f, 0.12f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_HAND_LEFT, -0.40f, 0.04f,
            kSyntheticPlayerDepthMeters);
  // Exercise gesture recognizers with an actual transition, not a teleport
  // between two static poses: rest, raise for one second, hold, then lower.
  const uint64_t gesture_phase_ms = GetTickCount64() % 8000;
  float raise_amount = 0.0f;
  if (gesture_phase_ms >= 2000 && gesture_phase_ms < 3000) {
    raise_amount = static_cast<float>(gesture_phase_ms - 2000) / 1000.0f;
  } else if (gesture_phase_ms >= 3000 && gesture_phase_ms < 6500) {
    raise_amount = 1.0f;
  } else if (gesture_phase_ms >= 6500 && gesture_phase_ms < 7500) {
    raise_amount = 1.0f -
                   static_cast<float>(gesture_phase_ms - 6500) / 1000.0f;
  }
  const auto lerp = [raise_amount](float low, float high) {
    return low + (high - low) * raise_amount;
  };
  set_joint(NUI_SKELETON_POSITION_ELBOW_RIGHT, lerp(0.38f, 0.43f),
            lerp(0.35f, 0.92f), kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_WRIST_RIGHT, lerp(0.40f, 0.48f),
            lerp(0.12f, 1.22f), kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_HAND_RIGHT, lerp(0.40f, 0.50f),
            lerp(0.04f, 1.40f), kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_SPINE, 0.0f, 0.34f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_HIP_CENTER, 0.0f, 0.0f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_HIP_LEFT, -0.16f, 0.0f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_HIP_RIGHT, 0.16f, 0.0f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_KNEE_LEFT, -0.16f, -0.5f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_KNEE_RIGHT, 0.16f, -0.5f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_ANKLE_LEFT, -0.16f, -1.0f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_ANKLE_RIGHT, 0.16f, -1.0f,
            kSyntheticPlayerDepthMeters);
  set_joint(NUI_SKELETON_POSITION_FOOT_LEFT, -0.16f, -1.05f,
            kSyntheticPlayerDepthMeters - 0.12f);
  set_joint(NUI_SKELETON_POSITION_FOOT_RIGHT, 0.16f, -1.05f,
            kSyntheticPlayerDepthMeters - 0.12f);
}

bool WindowsNuiWriteGuestFrame(uint32_t guest_ptr) {
  if (!guest_ptr ||
      (!IsSyntheticKinectFrameActive() && !windows_nui_device_ready())) {
    return false;
  }

  NUI_SKELETON_FRAME native_frame = {};
  if (!IsSyntheticKinectFrameActive() &&
      !windows_nui_backend().copy_latest_skeleton(&native_frame)) {
    return false;
  }
  WindowsNuiBuildSyntheticFrame(&native_frame);
  WindowsNuiBindPrimarySkeletonToGuestUser(&native_frame);

  auto guest = kernel_state()->memory()->TranslateVirtual<
      X_NUI_GUEST_SKELETON_FRAME*>(guest_ptr);
  if (!guest) {
    return false;
  }
  WindowsNuiSynchronizeEngagementState(native_frame);
  WindowsNuiConvertSkeletonFrame(guest, native_frame);
  if (WindowsNuiIsInternalGuestFrame(guest_ptr)) {
    const auto frame_it = std::find(windows_nui_guest_frames.begin(),
                                    windows_nui_guest_frames.end(), guest_ptr);
    WindowsNuiWriteGuestEnvelope(
        static_cast<uint32_t>(frame_it - windows_nui_guest_frames.begin()),
        guest_ptr, native_frame);
  }
  return true;
}
#endif

#if defined(XENIA_HAS_KINECT_V1_SDK)
bool WindowsNuiWriteTitleFrameFromNative(
    uint32_t guest_frame, uint32_t frame_size,
    const NUI_SKELETON_FRAME& source_frame) {
  if (frame_size != sizeof(X_NUI_GUEST_SKELETON_FRAME)) {
    return false;
  }
  NUI_SKELETON_FRAME native_frame = source_frame;
  // NUI timestamps are elapsed milliseconds, not 100-nanosecond FILETIME
  // ticks. Preserve the synchronized physical depth clock so gesture filters
  // observe the real 30 Hz cadence. Only fill missing identity fields for a
  // startup frame without a native timestamp.
  const uint64_t producer_time_ms = GetTickCount64();
  if (!native_frame.dwFrameNumber) {
    native_frame.dwFrameNumber =
        static_cast<DWORD>(producer_time_ms / 33 + 1);
  }
  if (!native_frame.liTimeStamp.QuadPart) {
    native_frame.liTimeStamp.QuadPart =
        static_cast<LONGLONG>(producer_time_ms);
  }
  WindowsNuiBuildSyntheticFrame(&native_frame);
  WindowsNuiBindPrimarySkeletonToGuestUser(&native_frame);
  const uint32_t tracked_count =
      WindowsNuiSynchronizeEngagementState(native_frame);

  // The registered 0x1402 buffer is producer-owned after submission. Serialize
  // the initial copy and capture-thread updates so a guest never observes a
  // partially byte-swapped skeleton record.
  static std::mutex title_frame_write_mutex;
  std::lock_guard<std::mutex> write_lock(title_frame_write_mutex);
  auto guest = kernel_state()->memory()->TranslateVirtual<
      X_NUI_GUEST_SKELETON_FRAME*>(guest_frame);
  if (!guest) {
    return false;
  }
  WindowsNuiConvertSkeletonFrame(guest, native_frame);

  static std::atomic<uint64_t> title_frame_count{0};
  static std::atomic<uint32_t> previous_tracked{UINT32_MAX};
  const uint64_t sequence = ++title_frame_count;
  const uint32_t old_tracked = previous_tracked.exchange(tracked_count);
  if (sequence <= 4 || (sequence & 0x7F) == 0 ||
      tracked_count != old_tracked) {
    const auto& body = native_frame.SkeletonData[0];
    const auto& head =
        body.SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
    const auto& left_hand =
        body.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT];
    const auto& right_hand =
        body.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];
    XELOGI("Windows Kinect: wrote title NUI frame sequence={} guest={:08X} "
           "tracked={} state={} id={} user={} enrollment={} frame={} "
           "timestamp={} head=({},{},{}) hands=({},{},{})/({},{},{})",
           sequence, guest_frame, tracked_count,
           static_cast<uint32_t>(body.eTrackingState), body.dwTrackingID,
           body.dwUserIndex, body.dwEnrollmentIndex, native_frame.dwFrameNumber,
           native_frame.liTimeStamp.QuadPart, head.x, head.y, head.z,
           left_hand.x, left_hand.y, left_hand.z, right_hand.x, right_hand.y,
           right_hand.z);
  }
  return true;
}
#endif

bool WindowsNuiWriteTitleFrame(uint32_t guest_frame, uint32_t frame_size) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (frame_size != sizeof(X_NUI_GUEST_SKELETON_FRAME)) {
    return false;
  }
  NUI_SKELETON_FRAME native_frame = {};
  if (!IsSyntheticKinectFrameActive() &&
      !windows_nui_backend().copy_latest_skeleton(&native_frame)) {
    return false;
  }
  if (!WindowsNuiWriteTitleFrameFromNative(guest_frame, frame_size,
                                           native_frame)) {
    return false;
  }
  // The recovered retail wrapper may pass a stack-local output frame here.
  // Its lifetime ends when XamXStudioRequest returns, so never retain this
  // address in the physical capture thread. Continuous delivery is owned by
  // the title's 2B002 ring; each 0x1402 call only completes its current frame.
  return true;
#else
  (void)guest_frame;
  (void)frame_size;
  return false;
#endif
}

#if defined(XENIA_HAS_KINECT_V1_SDK)
bool WindowsNuiWriteConsumerRingFromNative(
    const NUI_SKELETON_FRAME& native_frame) {
  constexpr uint32_t kRingSlotSize = 2784;
  constexpr uint32_t kRingFrameOffset = 32;
  constexpr uint32_t kRingFrameSize = 2736;
  constexpr uint32_t kRingTimestampOffset = 2768;

  const uint32_t ring_base =
      nui_consumer_ring_base.load(std::memory_order_acquire);
  const uint32_t event_address =
      nui_consumer_event_address.load(std::memory_order_acquire);
  if (!ring_base || !event_address) {
    XELOGW("Windows Kinect: consumer NUI ring publish missing registration "
           "ring={:08X} event={:08X}",
           ring_base, event_address);
    return false;
  }

  const uint64_t timestamp =
      static_cast<uint64_t>(native_frame.liTimeStamp.QuadPart);
  if (timestamp &&
      nui_last_physical_ring_timestamp.load(std::memory_order_acquire) ==
          timestamp) {
    return true;
  }

  auto* selector = kernel_state()->memory()->TranslateVirtual<xe::be<uint32_t>*>(
      ring_base - 12);
  auto* event = kernel_state()->memory()->TranslateVirtual<X_KEVENT*>(
      event_address);
  if (!selector || !event) {
    XELOGW("Windows Kinect: consumer NUI ring publish translation failed "
           "selector={} event={}",
           selector != nullptr, event != nullptr);
    return false;
  }

  auto* slot0_state =
      kernel_state()->memory()->TranslateVirtual<xe::be<uint32_t>*>(ring_base);
  auto* slot1_state = kernel_state()->memory()
                          ->TranslateVirtual<xe::be<uint32_t>*>(
                              ring_base + kRingSlotSize);
  const uint32_t selector_before = static_cast<uint32_t>(*selector);
  const uint32_t state0_before = slot0_state ? static_cast<uint32_t>(*slot0_state)
                                             : UINT32_MAX;
  const uint32_t state1_before = slot1_state ? static_cast<uint32_t>(*slot1_state)
                                             : UINT32_MAX;
  if (!slot0_state || !slot1_state) {
    XELOGW("Windows Kinect: consumer NUI ring state translation failed "
           "slot0={} slot1={}",
           slot0_state != nullptr, slot1_state != nullptr);
    return false;
  }

  // The selector identifies the title's currently visible slot. Retail XAM
  // writes only its opposite slot. State 0 is free; states 1 and 2 belong to
  // the guest consumer and must never be overwritten.
  const uint32_t slot_index = (selector_before ^ 1u) & 1u;
  const uint32_t old_state = slot_index ? state1_before : state0_before;
  if (old_state != 0) {
    static std::atomic<uint64_t> ring_backpressure_count{0};
    const uint64_t dropped = ++ring_backpressure_count;
    if (dropped <= 8 || (dropped & 0x7F) == 0) {
      XELOGI("Windows Kinect: consumer NUI ring backpressure selector={} "
             "write_slot={} states=[{},{}] dropped={}",
             selector_before, slot_index, state0_before, state1_before,
             dropped);
    }
    return false;
  }

  const uint32_t slot_address = ring_base + slot_index * kRingSlotSize;
  auto* slot = kernel_state()->memory()->TranslateVirtual<uint8_t*>(slot_address);
  if (!slot) {
    XELOGW("Windows Kinect: consumer NUI ring slot translation failed "
           "slot={:08X}",
           slot_address);
    return false;
  }
  if (old_state > 2) {
    XELOGW("Windows Kinect: rejected NUI ring with invalid slot state={} "
           "base={:08X}",
           old_state, ring_base);
    nui_consumer_ring_base.store(0, std::memory_order_release);
    nui_consumer_event_address.store(0, std::memory_order_release);
    return false;
  }

  // The first 32 bytes are owned by the retail consumer. They contain the
  // state at offset zero, but may also contain per-slot metadata initialized
  // by the title. Clearing the whole header makes the first frame visible and
  // then leaves the consumer unable to release that slot. Preserve it and
  // publish only the frame body, timestamp and producer state below.
  if (!WindowsNuiWriteTitleFrameFromNative(
          slot_address + kRingFrameOffset, kRingFrameSize, native_frame)) {
    XELOGW("Windows Kinect: consumer NUI ring frame write failed "
           "frame={:08X} size={}",
           slot_address + kRingFrameOffset, kRingFrameSize);
    return false;
  }
  xe::store_and_swap<uint32_t>(slot + kRingTimestampOffset,
                               static_cast<uint32_t>(Clock::QueryGuestTickCount()));
  *selector = slot_index;
  std::atomic_thread_fence(std::memory_order_release);
  xe::store_and_swap<uint32_t>(slot, 1);
  xboxkrnl::xeKeSetEvent(event, 1, 0);
  if (timestamp) {
    nui_last_physical_ring_timestamp.store(timestamp,
                                           std::memory_order_release);
  }

  // Retail XAM also signals the optional title event referenced immediately
  // after the embedded producer event. Kinect Sports waits on this object via
  // a handle, while other clients wait directly on the embedded event.
  auto* external_event_address =
      kernel_state()->memory()->TranslateVirtual<xe::be<uint32_t>*>(
          event_address + 16);
  uint32_t external_event_value =
      external_event_address ? static_cast<uint32_t>(*external_event_address)
                             : 0;
  auto* external_event = external_event_value
                             ? kernel_state()->memory()->TranslateVirtual<
                                   X_KEVENT*>(external_event_value)
                             : nullptr;
  if (external_event &&
      (external_event->header.type == 0 || external_event->header.type == 1)) {
    xboxkrnl::xeKeSetEvent(external_event, 1, 0);
  } else {
    external_event_value = 0;
  }

  static std::atomic<uint64_t> ring_write_count{0};
  const uint64_t sequence = ++ring_write_count;
  if (sequence <= 8 || (sequence & 0x7F) == 0) {
    XELOGI("Windows Kinect: published consumer NUI ring sequence={} "
           "event={:08X} external_event={:08X} selector={} slot={} "
           "states_before=[{},{}] claimed_state={} frame={:08X}",
           sequence, event_address, external_event_value, selector_before,
           slot_index, state0_before, state1_before,
           old_state, slot_address + kRingFrameOffset);
  }
  return true;
}

void StartWindowsNuiSyntheticRingProducer() {
  std::lock_guard<std::mutex> lock(nui_synthetic_ring_thread_mutex);
  if (nui_synthetic_ring_thread.joinable()) {
    return;
  }
  nui_synthetic_ring_thread = std::jthread([](std::stop_token stop_token) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    while (!stop_token.stop_requested()) {
      if (stop_token.stop_requested() || !IsSyntheticKinectFrameActive() ||
          !nui_consumer_ring_base.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
        continue;
      }
      NUI_SKELETON_FRAME synthetic_frame = {};
      WindowsNuiBuildSyntheticFrame(&synthetic_frame);
      if (WindowsNuiWriteConsumerRingFromNative(synthetic_frame)) {
        nui_last_synthetic_ring_frame.store(
            static_cast<uint64_t>(synthetic_frame.dwFrameNumber),
            std::memory_order_release);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  });
  XELOGI("Windows Kinect: started asynchronous 30 Hz synthetic NUI ring "
         "producer");
}
#endif

bool WindowsNuiInjectSkeletonNotification(uint8_t* notification,
                                          uint32_t notification_size) {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  constexpr uint32_t kNotificationHeaderSize = 16;
  constexpr uint32_t kNotificationFrameSize = 2736;
  constexpr uint32_t kNotificationTimestampOffset = 2736;
  constexpr uint32_t kNotificationFlagsOffset = 2744;
  constexpr uint32_t kNotificationMinimumSize = 2748;
  if (!notification || notification_size < kNotificationMinimumSize ||
      (!IsSyntheticKinectFrameActive() && !windows_nui_device_ready())) {
    return false;
  }

  NUI_SKELETON_FRAME native_frame = {};
  if (!IsSyntheticKinectFrameActive() &&
      !windows_nui_backend().copy_latest_skeleton(&native_frame)) {
    return false;
  }
  WindowsNuiBuildSyntheticFrame(&native_frame);
  WindowsNuiBindPrimarySkeletonToGuestUser(&native_frame);
  const uint32_t delivered_tracked_count =
      WindowsNuiSynchronizeEngagementState(native_frame);

  // The title-owned 2B002 notification replaces the first 16 bytes of the
  // normal skeleton frame with protocol and version qwords, then appends the
  // original timestamp and flags after the 2736-byte frame body.
  X_NUI_GUEST_SKELETON_FRAME guest_frame = {};
  WindowsNuiConvertSkeletonFrame(&guest_frame, native_frame);
  const auto guest_bytes = reinterpret_cast<const uint8_t*>(&guest_frame);
  std::memcpy(notification + kNotificationHeaderSize,
              guest_bytes + kNotificationHeaderSize,
              kNotificationFrameSize - kNotificationHeaderSize);
  xe::store_and_swap<uint64_t>(notification + kNotificationTimestampOffset,
                               static_cast<uint64_t>(
                                   native_frame.liTimeStamp.QuadPart));
  xe::store_and_swap<uint32_t>(notification + kNotificationFlagsOffset,
                               native_frame.dwFlags);

  static std::atomic<uint64_t> injection_count{0};
  static std::atomic<uint32_t> previous_tracked{UINT32_MAX};
  const uint64_t sequence = ++injection_count;
  const uint32_t tracked = delivered_tracked_count;
  const uint32_t old_tracked = previous_tracked.exchange(tracked);
  if (sequence <= 4 || (sequence & 0x7F) == 0 || tracked != old_tracked) {
    const auto& body = native_frame.SkeletonData[0];
    const auto& head =
        body.SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
    const auto& hand =
        body.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];
    XELOGI("Windows Kinect: injected 2B002 skeleton sequence={} tracked={} "
           "frame={} timestamp={} floor=({}, {}, {}, {}) body0_state={} "
           "body0_id={} body0_user={} head=({}, {}, {}) hand=({}, {}, {})",
           sequence, tracked, native_frame.dwFrameNumber,
           native_frame.liTimeStamp.QuadPart, native_frame.vFloorClipPlane.x,
           native_frame.vFloorClipPlane.y, native_frame.vFloorClipPlane.z,
           native_frame.vFloorClipPlane.w,
           static_cast<uint32_t>(body.eTrackingState),
           body.dwTrackingID, body.dwUserIndex, head.x, head.y, head.z,
           hand.x, hand.y, hand.z);
  }
  return true;
#else
  (void)notification;
  (void)notification_size;
  return false;
#endif
}

uint32_t WindowsNuiGetGuestFrame() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  const uint32_t frame_index = windows_nui_guest_frame_index++ %
                               windows_nui_guest_frames.size();
  uint32_t& guest_frame = windows_nui_guest_frames[frame_index];
  if (!guest_frame) {
    guest_frame = kernel_state()->memory()->SystemHeapAlloc(
        WindowsNuiGuestFrameAllocationSize(), 16);
    if (!guest_frame) {
      XELOGE("Windows Kinect: failed to allocate guest skeleton frame");
      return 0;
    }
    std::memset(kernel_state()->memory()->TranslateVirtual<void*>(
                guest_frame),
                0, WindowsNuiGuestFrameAllocationSize());
  }
  WindowsNuiWriteGuestFrame(guest_frame);
  return guest_frame;
#else
  return 0;
#endif
}

uint32_t WindowsNuiGuestFrameSize() {
#if defined(XENIA_HAS_KINECT_V1_SDK)
  return WindowsNuiGuestFrameAllocationSize();
#else
  return 0;
#endif
}

struct X_NUI_DEVICE_STATUS {
  /* Notes:
     - for one side func of XamNuiGetDeviceStatus
       - if some data addressis less than zero then unk1 = it
       - else another func is called and its return can set unk1 = c0051200 or
     some value involving DetroitDeviceRequest
       - next PsCamDeviceRequest is called and if its return is less than zero
     then X_NUI_DEVICE_STATUS = return of PsCamDeviceRequest
       - else it equals an unknown local_1c
       - finally McaDeviceRequest is called and if its return is less than zero
     then unk2 = return of McaDeviceRequest
       - else it equals an unknown local_14
     - status can be set to X_NUI_DEVICE_STATUS[3] | 0x44 or | 0x40
  */
  xe::be<uint32_t> unk0;
  xe::be<uint32_t> unk1;
  xe::be<uint32_t> unk2;
  xe::be<uint32_t> status;
  xe::be<uint32_t> unk4;
  xe::be<uint32_t> unk5;
};
static_assert(sizeof(X_NUI_DEVICE_STATUS) == 24, "Size matters");

extern bool nui_force_device_off;
extern bool nui_natal_started;
extern uint32_t nui_camera_elevation_angle;

bool IsWritableNuiOutputRange(uint32_t address, uint32_t size);
bool IsReadableNuiInputRange(uint32_t address, uint32_t size);
void TraceXStudioEnvelope(uint32_t request, uint32_t argument);
void BroadcastNuiHardwareStatus(uint32_t device_state);

// Get
dword_result_t XamNuiGetDeviceStatus_entry(
    pointer_t<X_NUI_DEVICE_STATUS> status_ptr) {
  /* Notes:
     - it does return a value that is not always used
     - returns values are X_ERROR_SUCCESS, 0xC0050006, and others
     - 1) On func start *status_ptr = 0, status_ptr->unk1 = 0, status_ptr->unk2
     = 0, and status_ptr->status = 0
     - 2) calls XamXStudioRequest(6,&var <- = 0);
     - if return is greater than -1 && var & 0x80000000 != 0 then set
     status_ptr->unk1 = 0xC000009D, status_ptr->unk2 = 0xC000009D, and
     status_ptr->status = status_ptr[3] = 0x20
     - lots of branching functions after
  */

  status_ptr.Zero();
  const bool device_ready = nui_guest_device_ready();
  status_ptr->status = nui_force_device_off
                           ? 0
                           : (device_ready ? 0x00000044 : 0x00000020);
  XELOGI("XamNuiGetDeviceStatus: force_off={} ready={} fields={:08X} {:08X} {:08X} {:08X} {:08X} {:08X}",
         nui_force_device_off, device_ready,
         static_cast<uint32_t>(status_ptr->unk0),
         static_cast<uint32_t>(status_ptr->unk1),
         static_cast<uint32_t>(status_ptr->unk2),
         static_cast<uint32_t>(status_ptr->status),
         static_cast<uint32_t>(status_ptr->unk4),
         static_cast<uint32_t>(status_ptr->unk5));
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetDeviceStatus, kNone, kImplemented);

dword_result_t XamNuiCameraTiltGetStatus_entry(lpvoid_t unk);

dword_result_t XamXStudioRequest_entry(dword_t request, lpvoid_t argument,
                                       const ppc_context_t& ctx) {
  // XStudio exposes both the retail NUI lifecycle and development-only camera
  // commands. Camera acquisition itself remains on the retail PsCam path, but
  // the client reports each completed PsCam request back through 0x1004.
  constexpr uint32_t kOpenNuiSession = 1;
  constexpr uint32_t kQueryDeviceState = 6;
  constexpr uint32_t kRegisterDeviceCallback = 7;
  constexpr uint32_t kUnregisterDeviceCallback = 8;
  constexpr uint32_t kSendXStudioPacket = 0x251;
  constexpr uint32_t kReceiveXStudioPacket = 0x252;
  constexpr uint32_t kConfigureNuiDelivery = 0x1410;
  constexpr uint32_t kBootstrapNuiFrame = 0x1401;
  constexpr uint32_t kSubmitNuiFrame = 0x1402;
  constexpr uint32_t kDispatchPsCamRequest = 0x1003;
  constexpr uint32_t kCompletePsCamRequest = 0x1004;
  constexpr uint32_t kGetNuiTiltStatus = 0x1005;
  static std::atomic<uint64_t> request_count{0};
  static std::atomic<bool> session_open{false};
  const uint64_t sequence = ++request_count;

  const uint32_t request_value = static_cast<uint32_t>(request);
  const uint32_t argument_address = argument.guest_address();
  TraceXStudioEnvelope(request_value, argument_address);
  const bool device_ready = !nui_force_device_off && nui_guest_device_ready();

  switch (request_value) {
    case kOpenNuiSession:
      if (!device_ready) {
        session_open.store(false, std::memory_order_release);
        return X_E_DEVICE_NOT_CONNECTED;
      }
      nui_natal_started = true;
      if (!session_open.exchange(true, std::memory_order_acq_rel)) {
        xstudio_nui_contract_state().Reset();
        xstudio_packet_transport().Reset();
        InitializeWindowsNuiGuestPath();
        BroadcastNuiHardwareStatus(0x00000044);
      }
      XELOGI("XamXStudioRequest: opened retail NUI session sequence={}",
             sequence);
      return X_ERROR_SUCCESS;

    case kQueryDeviceState: {
      if (!argument || !IsWritableNuiOutputRange(argument_address,
                                                 sizeof(uint32_t))) {
        return X_E_INVALIDARG;
      }
      auto state = kernel_state()->memory()->TranslateVirtual<
          xe::be<uint32_t>*>(argument_address);
      if (!state) {
        return X_E_INVALIDARG;
      }
      // The high bit is the device-absent signal consumed by
      // XamNuiGetDeviceStatus. Preserve the rest of a caller-owned structure.
      state[0] = device_ready ? 0u : 0x80000000u;
      return X_ERROR_SUCCESS;
    }

    case kRegisterDeviceCallback: {
      constexpr uint32_t kRegistrationSize = 3 * sizeof(xe::be<uint32_t>);
      if (!argument ||
          !IsReadableNuiInputRange(argument_address, kRegistrationSize)) {
        return X_E_INVALIDARG;
      }
      const auto* registration = kernel_state()->memory()->TranslateVirtual<
          const xe::be<uint32_t>*>(argument_address);
      const uint32_t state_mask = static_cast<uint32_t>(registration[0]);
      const uint32_t event_mask = static_cast<uint32_t>(registration[1]);
      const uint32_t callback = static_cast<uint32_t>(registration[2]);
      if (!callback) {
        return X_E_INVALIDARG;
      }
      xstudio_device_callback_registration().Store(callback, state_mask,
                                                   event_mask);
      QueueXStudioDeviceCallback(device_ready ? 0x00000044u : 0u, 1, 0);
      XELOGI("XamXStudioRequest(7): registered device callback={:08X} "
             "state_mask={:08X} event_mask={:08X}",
             callback, state_mask, event_mask);
      return X_ERROR_SUCCESS;
    }

    case kUnregisterDeviceCallback: {
      uint32_t callback = 0;
      if (argument && IsReadableNuiInputRange(argument_address,
                                              sizeof(xe::be<uint32_t>))) {
        callback = xe::load_and_swap<uint32_t>(
            kernel_state()->memory()->TranslateVirtual<const uint8_t*>(
                argument_address));
      }
      xstudio_device_callback_registration().Clear(callback);
      XELOGI("XamXStudioRequest(8): cleared device callback={:08X}", callback);
      return X_ERROR_SUCCESS;
    }

    case kSendXStudioPacket:
    case kReceiveXStudioPacket: {
      // The shared retail NUI client wraps its internal producer/consumer
      // channel in this 48-byte envelope:
      // {0, timeout_ms, 0, 6, 0, 0, 0, 0,
      //  descriptor, descriptor_size, payload, payload_size}.
      // 0x251 enqueues a serialized frame and 0x252 waits for the next frame.
      // Keeping this as a byte-preserving transport lets the original client
      // own the private frame layout instead of recreating that layout in XAM.
      constexpr uint32_t kEnvelopeWords = 12;
      constexpr uint32_t kEnvelopeSize =
          kEnvelopeWords * sizeof(xe::be<uint32_t>);
      constexpr uint32_t kProtocolVersion = 6;
      constexpr uint32_t kMaxDescriptorSize = 0x1000;
      constexpr uint32_t kMaxPayloadSize = 4 * 1024 * 1024;
      constexpr uint32_t kTimeoutHresult = 0x800705B4;
      constexpr uint32_t kInsufficientBufferHresult = 0x8007007A;
      if (!argument ||
          !IsReadableNuiInputRange(argument_address, kEnvelopeSize)) {
        return X_E_INVALIDARG;
      }
      const auto* envelope = kernel_state()->memory()->TranslateVirtual<
          const xe::be<uint32_t>*>(argument_address);
      if (!envelope || static_cast<uint32_t>(envelope[3]) != kProtocolVersion) {
        return X_E_INVALIDARG;
      }
      const uint32_t timeout_ms = static_cast<uint32_t>(envelope[1]);
      const uint32_t descriptor_address = static_cast<uint32_t>(envelope[8]);
      const uint32_t descriptor_size = static_cast<uint32_t>(envelope[9]);
      const uint32_t payload_address = static_cast<uint32_t>(envelope[10]);
      const uint32_t payload_size = static_cast<uint32_t>(envelope[11]);
      if (!descriptor_size || descriptor_size > kMaxDescriptorSize ||
          !payload_size || payload_size > kMaxPayloadSize) {
        return X_E_INVALIDARG;
      }

      if (request_value == kSendXStudioPacket) {
        if (!IsReadableNuiInputRange(descriptor_address, descriptor_size) ||
            !IsReadableNuiInputRange(payload_address, payload_size)) {
          return X_E_INVALIDARG;
        }
        const auto* descriptor = kernel_state()->memory()->TranslateVirtual<
            const uint8_t*>(descriptor_address);
        const auto* payload = kernel_state()->memory()->TranslateVirtual<
            const uint8_t*>(payload_address);
        if (!descriptor || !payload) {
          return X_E_INVALIDARG;
        }
        XStudioPacket packet;
        packet.descriptor.assign(descriptor, descriptor + descriptor_size);
        packet.payload.assign(payload, payload + payload_size);
        xstudio_packet_transport().Send(std::move(packet));
        XELOGI("XamXStudioRequest(251): queued descriptor={} payload={}",
               descriptor_size, payload_size);
        return X_ERROR_SUCCESS;
      }

      if (!IsWritableNuiOutputRange(descriptor_address, descriptor_size) ||
          !IsWritableNuiOutputRange(payload_address, payload_size)) {
        return X_E_INVALIDARG;
      }
      XStudioPacket packet;
      if (!xstudio_packet_transport().Receive(timeout_ms, &packet)) {
        return kTimeoutHresult;
      }
      if (packet.descriptor.size() > descriptor_size ||
          packet.payload.size() > payload_size) {
        return kInsufficientBufferHresult;
      }
      auto* descriptor = kernel_state()->memory()->TranslateVirtual<uint8_t*>(
          descriptor_address);
      auto* payload = kernel_state()->memory()->TranslateVirtual<uint8_t*>(
          payload_address);
      if (!descriptor || !payload) {
        return X_E_INVALIDARG;
      }
      std::memcpy(descriptor, packet.descriptor.data(),
                  packet.descriptor.size());
      std::memcpy(payload, packet.payload.data(), packet.payload.size());
      XELOGI("XamXStudioRequest(252): received descriptor={} payload={}",
             packet.descriptor.size(), packet.payload.size());
      return X_ERROR_SUCCESS;
    }

    case kConfigureNuiDelivery: {
      // The shared retail helper builds a 56-byte delivery-clock envelope:
      // {56, client_version, timebase_hi, timebase_lo, ..., 0x300}.
      // The fields between the timebase and the trailing profile value are
      // client-owned state, not an enabled flag. Keep the parsed clock intact
      // without imposing a host-defined value on those words.
      constexpr uint32_t kConfigureEnvelopeWords = 14;
      constexpr uint32_t kConfigureEnvelopeSize =
          kConfigureEnvelopeWords * sizeof(xe::be<uint32_t>);
      constexpr uint32_t kConfigureEnvelopeBytesWord = 0;
      constexpr uint32_t kConfigureEnvelopeVersionWord = 1;
      constexpr uint32_t kConfigureEnvelopeTimebaseHighWord = 2;
      constexpr uint32_t kConfigureEnvelopeTimebaseLowWord = 3;
      constexpr uint32_t kConfigureEnvelopeProfileWord = 13;
      constexpr uint32_t kConfigureEnvelopeProfile = 0x300;
      if (!argument ||
          !IsReadableNuiInputRange(argument_address, kConfigureEnvelopeSize)) {
        return X_E_INVALIDARG;
      }
      const auto* configure = kernel_state()->memory()->TranslateVirtual<
          const xe::be<uint32_t>*>(argument_address);
      const uint32_t envelope_bytes =
          configure ? static_cast<uint32_t>(
                          configure[kConfigureEnvelopeBytesWord])
                    : 0;
      const uint32_t dashboard_version =
          configure ? static_cast<uint32_t>(
                          configure[kConfigureEnvelopeVersionWord])
                    : 0;
      const uint64_t timebase = configure
                                    ? (uint64_t(static_cast<uint32_t>(
                                           configure[kConfigureEnvelopeTimebaseHighWord]))
                                       << 32) |
                                          static_cast<uint32_t>(
                                              configure[kConfigureEnvelopeTimebaseLowWord])
                                    : 0;
      const uint32_t profile =
          configure ? static_cast<uint32_t>(
                          configure[kConfigureEnvelopeProfileWord])
                    : 0;
      if (envelope_bytes != kConfigureEnvelopeSize || !dashboard_version ||
          profile != kConfigureEnvelopeProfile) {
        XELOGW("XamXStudioRequest(1410): invalid bytes={} version={:04X} "
               "profile={:08X} argument={:08X}",
               envelope_bytes, dashboard_version, profile, argument_address);
        return X_E_INVALIDARG;
      }
      xstudio_nui_contract_state().Configure(dashboard_version, timebase);
      XELOGI("XamXStudioRequest(1410): configured physical NUI delivery "
             "version={:04X} timebase={:016X}",
             dashboard_version, timebase);
      return X_ERROR_SUCCESS;
    }

    case kBootstrapNuiFrame: {
      // The 0x1401 input begins with a title-owned 2736-byte skeleton buffer
      // and repeats the same dashboard version used by 0x1410. Populate the
      // declared writable range immediately so the first title tick observes
      // a real frame before it starts the regular 0x1402 submission loop.
      constexpr uint32_t kBootstrapEnvelopeWords = 12;
      constexpr uint32_t kBootstrapEnvelopeSize =
          kBootstrapEnvelopeWords * sizeof(xe::be<uint32_t>);
      constexpr uint32_t kSkeletonFrameSize = 2736;
      constexpr uint32_t kBootstrapFrameWord = 0;
      constexpr uint32_t kBootstrapFrameSizeWord = 1;
      constexpr uint32_t kBootstrapVersionWord = 2;
      constexpr uint32_t kBootstrapTimestampHighWord = 4;
      constexpr uint32_t kBootstrapTimestampLowWord = 5;
      constexpr uint32_t kBootstrapStatusWord = 7;
      if (!argument ||
          !IsReadableNuiInputRange(argument_address, kBootstrapEnvelopeSize)) {
        return X_E_INVALIDARG;
      }
      const auto* bootstrap = kernel_state()->memory()->TranslateVirtual<
          const xe::be<uint32_t>*>(argument_address);
      const uint32_t frame_address =
          bootstrap ? static_cast<uint32_t>(bootstrap[kBootstrapFrameWord]) : 0;
      const uint32_t frame_size = bootstrap
                                      ? static_cast<uint32_t>(
                                            bootstrap[kBootstrapFrameSizeWord])
                                      : 0;
      const uint32_t dashboard_version =
          bootstrap ? static_cast<uint32_t>(bootstrap[kBootstrapVersionWord])
                    : 0;
      const uint64_t timestamp = bootstrap
                                     ? (uint64_t(static_cast<uint32_t>(
                                            bootstrap[kBootstrapTimestampHighWord]))
                                        << 32) |
                                           static_cast<uint32_t>(
                                               bootstrap[kBootstrapTimestampLowWord])
                                     : 0;
      const uint32_t status_address =
          bootstrap ? static_cast<uint32_t>(bootstrap[kBootstrapStatusWord])
                    : 0;
      if (frame_size != kSkeletonFrameSize ||
          !IsWritableNuiOutputRange(frame_address, frame_size) ||
          (status_address &&
           !IsWritableNuiOutputRange(status_address, sizeof(uint32_t)))) {
        XELOGW("XamXStudioRequest(1401): invalid frame={:08X} size={} "
               "version={:04X}",
               frame_address, frame_size, dashboard_version);
        return X_E_INVALIDARG;
      }
      auto write_status = [status_address](uint32_t status) {
        if (!status_address) {
          return;
        }
        auto* output = kernel_state()->memory()->TranslateVirtual<
            xe::be<uint32_t>*>(status_address);
        if (output) {
          *output = status;
        }
      };
      const bool configured = xstudio_nui_contract_state().RecordBootstrapFrame(
          frame_address, dashboard_version, timestamp);
      if (!device_ready ||
          !WindowsNuiWriteTitleFrame(frame_address, frame_size)) {
        write_status(X_E_DEVICE_NOT_CONNECTED);
        XELOGD("XamXStudioRequest(1401): no physical frame available "
               "frame={:08X} version={:04X} configured={}",
               frame_address, dashboard_version, configured);
        return X_E_DEVICE_NOT_CONNECTED;
      }
      write_status(X_ERROR_SUCCESS);
      XELOGI("XamXStudioRequest(1401): bootstrapped physical NUI frame={} "
             "version={:04X} configured={} skeletons={} color={} depth={}",
             frame_address, dashboard_version, configured,
             WindowsNuiSkeletonFrameCount(), WindowsNuiColorFrameCount(),
             WindowsNuiDepthFrameCount());
      return X_ERROR_SUCCESS;
    }

    case kSubmitNuiFrame: {
      // Kinect Sports, Dance Central 3, Just Dance 2014 and dashnui share
      // this submission contract:
      // {frame, 2736, dashboard_version, reserved, timestamp_qword,
      //  frame_number, completion_status*}.
      // `frame` is title-owned memory for the standard 2736-byte skeleton
      // frame, not an opaque XStudio token. The recovered retail wrapper stores
      // its output HRESULT pointer in word 7 and reads it immediately after the
      // request, so completing that field is part of the ABI.
      constexpr uint32_t kSubmitEnvelopeWords = 8;
      constexpr uint32_t kSkeletonFrameSize = 2736;
      if (!argument ||
          !IsReadableNuiInputRange(
              argument_address,
              kSubmitEnvelopeWords * sizeof(xe::be<uint32_t>))) {
        return X_E_INVALIDARG;
      }
      const auto* submit = kernel_state()->memory()->TranslateVirtual<
          const xe::be<uint32_t>*>(argument_address);
      const uint32_t frame_address =
          submit ? static_cast<uint32_t>(submit[0]) : 0;
      const uint32_t frame_size =
          submit ? static_cast<uint32_t>(submit[1]) : 0;
      const uint32_t dashboard_version =
          submit ? static_cast<uint32_t>(submit[2]) : 0;
      const uint32_t status_address =
          submit ? static_cast<uint32_t>(submit[7]) : 0;
      if (frame_size != kSkeletonFrameSize ||
          !IsWritableNuiOutputRange(frame_address, frame_size) ||
          !status_address ||
          !IsWritableNuiOutputRange(status_address, sizeof(uint32_t))) {
        XELOGW("XamXStudioRequest(1402): invalid frame={:08X} size={} "
               "version={:04X} status={:08X}",
               frame_address, frame_size, dashboard_version, status_address);
        return X_E_INVALIDARG;
      }
      auto write_status = [status_address](uint32_t status) {
        auto* output = kernel_state()->memory()->TranslateVirtual<
            xe::be<uint32_t>*>(status_address);
        if (output) {
          *output = status;
        }
      };

      // Trace the title-owned frame before touching it. This distinguishes a
      // producer submission (the title already filled the NUI frame) from a
      // receive-buffer registration (the frame is empty until XAM writes it).
      // The direction matters: overwriting a submitted frame would hide the
      // output of the retail NUI processor from the dashboard and diagnostics.
      static std::atomic<uint64_t> prewrite_trace_count{0};
      const uint64_t prewrite_sequence = ++prewrite_trace_count;
      if (prewrite_sequence <= 16 || (prewrite_sequence & 0x7F) == 0) {
        const auto* submitted = kernel_state()->memory()->TranslateVirtual<
            const X_NUI_GUEST_SKELETON_FRAME*>(frame_address);
        if (submitted) {
          uint32_t submitted_tracked = 0;
          int32_t submitted_slot = -1;
          for (uint32_t slot = 0; slot < NUI_SKELETON_COUNT; ++slot) {
            if (static_cast<uint32_t>(submitted->skeletons[slot].tracking_state) ==
                NUI_SKELETON_TRACKED) {
              ++submitted_tracked;
              if (submitted_slot < 0) {
                submitted_slot = static_cast<int32_t>(slot);
              }
            }
          }
          const auto& body = submitted->skeletons[
              submitted_slot >= 0 ? static_cast<uint32_t>(submitted_slot) : 0];
          XELOGI("XamXStudioRequest(1402): prewrite frame={} timestamp={} "
                 "tracked={} slot={} state={} id={} user={} enrollment={} "
                 "head_y={} right_hand_y={}",
                 static_cast<uint32_t>(submitted->frame_number),
                 static_cast<uint64_t>(submitted->timestamp), submitted_tracked,
                 submitted_slot,
                 static_cast<uint32_t>(body.tracking_state),
                 static_cast<uint32_t>(body.tracking_id),
                 static_cast<uint32_t>(body.user_index),
                 static_cast<uint32_t>(body.enrollment_index),
                 static_cast<float>(body.joints[NUI_SKELETON_POSITION_HEAD].y),
                 static_cast<float>(
                     body.joints[NUI_SKELETON_POSITION_HAND_RIGHT].y));
        }
      }

      // Titles may begin polling 0x1402 while their client thread is still
      // preparing the 0x1410 timing envelope. The frame buffer is complete
      // and writable on its own, so provide the current physical frame rather
      // than turning that normal startup ordering into an artificial failure.
      const bool configured =
          xstudio_nui_contract_state().IsConfiguredFor(dashboard_version);
      if (!device_ready) {
        write_status(X_E_DEVICE_NOT_CONNECTED);
        XELOGD("XamXStudioRequest(1402): no physical frame available "
                "frame={:08X} version={:04X}",
                frame_address, dashboard_version);
        return X_E_DEVICE_NOT_CONNECTED;
      }

      // The retail client may issue the next request immediately after a
      // completion. Do not complete it again with the same sensor frame: that
      // turns its NUI worker into a tight loop and starves the title. Wait for
      // the 30 Hz producer, with a bounded timeout for disconnected/stalled
      // hardware.
      static std::mutex submission_mutex;
      static uint64_t last_nui_frame_count = UINT64_MAX;
      std::lock_guard<std::mutex> submission_lock(submission_mutex);
      const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(40);
      uint64_t nui_frame_count = GetTickCount64() / 33 + 1;
      while (nui_frame_count == last_nui_frame_count &&
             std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        nui_frame_count = GetTickCount64() / 33 + 1;
      }
      if (!WindowsNuiWriteTitleFrame(frame_address, frame_size)) {
        write_status(X_E_DEVICE_NOT_CONNECTED);
        return X_E_DEVICE_NOT_CONNECTED;
      }

#if defined(XENIA_HAS_KINECT_V1_SDK)
      WindowsNuiNoteFrameSubmission(XThread::GetCurrentThreadId());
      // The 0x1402 frame belongs to the caller only for the duration of this
      // request. Retaining it in the capture callback writes into freed title
      // allocations after clients rotate their receive buffers. Continuous
      // delivery has an explicit lifetime in the registered consumer ring, so
      // keep the asynchronous callback restricted to that ring.
      windows_nui_backend().set_title_frame_callback(
          [](const NUI_SKELETON_FRAME& frame) {
            if (!IsSyntheticKinectFrameActive()) {
              WindowsNuiWriteConsumerRingFromNative(frame);
            }
          });
#endif
      last_nui_frame_count = nui_frame_count;
      write_status(X_ERROR_SUCCESS);

      static std::atomic<uint64_t> submitted_frame_count{0};
      const uint64_t submitted = ++submitted_frame_count;
      if (submitted <= 8 || (submitted & 0x7F) == 0) {
        XELOGI("XamXStudioRequest(1402): delivered physical NUI frame={} "
               "size={} version={:04X} configured={} tracked={} skeletons={} "
               "color={} depth={} status={:08X}",
               frame_address, frame_size, dashboard_version, configured,
               WindowsNuiTrackedCount(), WindowsNuiSkeletonFrameCount(),
               WindowsNuiColorFrameCount(), WindowsNuiDepthFrameCount(),
               status_address);
      }
      return X_ERROR_SUCCESS;
    }

    case kDispatchPsCamRequest: {
      // Retail NUI clients pass {PsCamDeviceRequest*, completion_status}.
      // XStudio owns the forwarding status while the actual PsCam dispatcher
      // writes the operation result into the caller's second word. On retail,
      // a client only falls back to the direct PsCam export when this envelope
      // is invalid, not when the forwarded request completes asynchronously.
      constexpr uint32_t kDispatchEnvelopeWords = 2;
      constexpr uint32_t kDispatchEnvelopeSize =
          kDispatchEnvelopeWords * sizeof(xe::be<uint32_t>);
      if (!argument ||
          !IsWritableNuiOutputRange(argument_address, kDispatchEnvelopeSize)) {
        return X_E_INVALIDARG;
      }
      auto* dispatch = kernel_state()->memory()->TranslateVirtual<
          xe::be<uint32_t>*>(argument_address);
      const uint32_t ps_cam_request_address =
          dispatch ? static_cast<uint32_t>(dispatch[0]) : 0;
      if (!ps_cam_request_address) {
        return X_E_INVALIDARG;
      }

      const uint32_t ps_cam_status = xboxkrnl::PsCamDeviceRequestDispatch(
          ps_cam_request_address, 0);
      dispatch[1] = ps_cam_status;

      static std::atomic<uint64_t> dispatch_count{0};
      const uint64_t dispatch_sequence = ++dispatch_count;
      if (dispatch_sequence <= 16 || (dispatch_sequence & 0x3F) == 1) {
        XELOGI("XamXStudioRequest(1003): dispatched PsCam request={:08X} "
               "status={:08X} sequence={}",
               ps_cam_request_address, ps_cam_status, dispatch_sequence);
      }
      return X_ERROR_SUCCESS;
    }

    case kCompletePsCamRequest: {
      // All three target titles call this immediately after their PsCam DMA
      // callback. Their shared NUI client only retains the completion token
      // when the request returns success. Validate the actual guest request
      // and its callback-written status instead of accepting an arbitrary
      // XStudio envelope or returning a blanket success.
      constexpr uint32_t kPsCamRequestWords = 9;
      constexpr uint32_t kPsCamDescriptorStatusWord = 1;
      // The retail client supplies { PsCamRequest*, completion_status }. It
      // branches on this function's result to decide whether word 1 is valid,
      // then consumes word 1 as the actual completion result. Keep those two
      // result channels distinct instead of turning a completed request into
      // an XStudio failure.
      if (!argument ||
          !IsWritableNuiOutputRange(argument_address,
                                    2 * sizeof(uint32_t))) {
        return X_E_INVALIDARG;
      }
      auto* completion = kernel_state()->memory()->TranslateVirtual<
          xe::be<uint32_t>*>(argument_address);
      const uint32_t ps_cam_request_address =
          completion ? static_cast<uint32_t>(completion[0]) : 0;
      if (!ps_cam_request_address ||
          !IsReadableNuiInputRange(ps_cam_request_address,
                                   kPsCamRequestWords * sizeof(uint32_t))) {
        return X_E_INVALIDARG;
      }
      const auto* ps_cam_request = kernel_state()->memory()->TranslateVirtual<
          const xe::be<uint32_t>*>(ps_cam_request_address);
      const uint32_t opcode = static_cast<uint32_t>(ps_cam_request[0]);
      const uint32_t descriptor_address =
          static_cast<uint32_t>(ps_cam_request[4]);
      if (opcode != 5 || !descriptor_address ||
          !IsReadableNuiInputRange(
              descriptor_address,
              (kPsCamDescriptorStatusWord + 1) * sizeof(uint32_t))) {
        return X_E_INVALIDARG;
      }
      const auto* descriptor = kernel_state()->memory()->TranslateVirtual<
          const xe::be<uint32_t>*>(descriptor_address);
      const uint32_t completion_status =
          static_cast<uint32_t>(descriptor[kPsCamDescriptorStatusWord]);
      static std::atomic<uint64_t> completion_count{0};
      const uint64_t completion_sequence = ++completion_count;
      if (completion_sequence <= 8 || completion_status != X_STATUS_SUCCESS ||
           (completion_sequence & 0x7F) == 0) {
        XELOGI("XamXStudioRequest(1004): PsCam request={:08X} "
               "descriptor={:08X} completion_status={:08X}",
               ps_cam_request_address, descriptor_address, completion_status);
      }
      completion[1] = completion_status;
      return X_ERROR_SUCCESS;
    }

    case kGetNuiTiltStatus: {
      // `XtSs` is the retail 80-byte camera-status envelope. The direct
      // export clears only its result area, so use it here rather than
      // fabricating a separate XStudio response.
      constexpr uint32_t kTiltStatusEnvelopeSize = 0x50;
      constexpr uint32_t kTiltStatusSizeWord = 0;
      constexpr uint32_t kTiltStatusSignatureWord = 1;
      constexpr uint32_t kTiltStatusSignature = 0x58745373;  // XtSs
      if (!argument || !IsWritableNuiOutputRange(argument_address,
                                                  kTiltStatusEnvelopeSize)) {
        return X_E_INVALIDARG;
      }
      const auto* tilt_status = kernel_state()->memory()->TranslateVirtual<
          const xe::be<uint32_t>*>(argument_address);
      if (!tilt_status ||
          static_cast<uint32_t>(tilt_status[kTiltStatusSizeWord]) !=
              kTiltStatusEnvelopeSize ||
          static_cast<uint32_t>(tilt_status[kTiltStatusSignatureWord]) !=
              kTiltStatusSignature) {
        return X_E_INVALIDARG;
      }
      return XamNuiCameraTiltGetStatus_entry(argument);
    }

    default:
      break;
  }

  static std::mutex unsupported_trace_mutex;
  static std::unordered_map<uint32_t, uint32_t> unsupported_trace_counts;
  bool trace_unsupported = false;
  {
    std::lock_guard<std::mutex> lock(unsupported_trace_mutex);
    trace_unsupported = unsupported_trace_counts[request_value]++ < 8;
  }
  if (trace_unsupported) {
    XELOGI("XamXStudioRequest: unsupported request={:08X} argument={:08X} "
           "caller={:08X} stack={:08X} sequence={} result={:08X}",
           request_value, argument_address, uint32_t(ctx->lr),
           uint32_t(ctx->r[1]), sequence, uint32_t(X_E_INVALIDARG));
    const uint32_t caller_address = uint32_t(ctx->lr);
    const uint32_t caller_probe_address = caller_address - 8 * sizeof(uint32_t);
    const auto* caller_code = kernel_state()->memory()->TranslateVirtual<
        const xe::be<uint32_t>*>(caller_probe_address);
    if (caller_code) {
      std::string code_dump;
      for (uint32_t i = 0; i < 24; ++i) {
        code_dump += fmt::format("{:08X}{}", uint32_t(caller_code[i]),
                                 i == 23 ? "" : " ");
      }
      XELOGI("XamXStudioRequest: caller code {:08X}..{:08X} lr={:08X}: [{}]",
             caller_probe_address, caller_probe_address + 23 * 4,
             caller_address, code_dump);
    }
  }
  return X_E_INVALIDARG;
}
DECLARE_XAM_EXPORT1(XamXStudioRequest, kNone, kImplemented);

dword_result_t XamUserNuiGetUserIndex_entry(unknown_t unk, lpdword_t index) {
  constexpr uint32_t kNoUser = 0xFE;
  const uint32_t nui_slot = static_cast<uint32_t>(unk);
  // NUI slot 0 is the local player slot. It must be available even when Xenia
  // has no persisted profile object yet: the XAM user context still exposes
  // local user 0 and retail Kinect titles use it for controllerless sign-in.
  if (nui_slot == 0) {
    if (index) {
      *index = 0;
    }
    XELOGI("XamUserNuiGetUserIndex: NUI slot 0 -> local user 0");
    return X_E_SUCCESS;
  }
  if (index) {
    *index = kNoUser;
  }
  XELOGD("XamUserNuiGetUserIndex: NUI slot {} -> no user", nui_slot);
  return X_E_NO_SUCH_USER;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndex, kNone, kImplemented);

dword_result_t XamUserNuiGetUserIndexForSignin_entry(lpdword_t index) {
  for (uint32_t i = 0; i < XUserMaxUserCount; i++) {
    auto profile = kernel_state()->xam_state()->GetUserProfile(i);
    if (profile) {
      *index = i;
      return X_E_SUCCESS;
    }
  }

  return X_E_ACCESS_DENIED;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForSignin, kNone, kImplemented);

dword_result_t XamUserNuiGetUserIndexForBind_entry(lpdword_t index) {
  if (index) {
    *index = 0;
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetUserIndexForBind, kNone, kImplemented);

dword_result_t XamNuiGetDepthCalibration_entry(lpdword_t unk1) {
  /* Notes:
     - Possible returns X_STATUS_NO_SUCH_FILE, and 0x10000000
  */
  if (unk1) {
    *unk1 = 0;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetDepthCalibration, kNone, kImplemented);

// Skeleton
qword_result_t XamNuiSkeletonGetBestSkeletonIndex_entry(int_t unk) {
  (void)unk;
  return static_cast<int64_t>(WindowsNuiBestSkeletonIndex());
}
DECLARE_XAM_EXPORT1(XamNuiSkeletonGetBestSkeletonIndex, kNone, kImplemented);

/* XamNuiCamera Notes
   - most require message calls to xam in 0x0002Bxxx area
*/

dword_result_t XamNuiCameraTiltGetStatus_entry(lpvoid_t unk) {
  /* Notes:
     - Used by XamNuiCameraElevationGetAngle, and XamNuiCameraSetFlags
     - if it returns anything greater than -1 then both above functions continue
     - Both funcs send in a param of *unk = 0x50 bytes to copy
     - unk2
     - Ghidra decompile fails
  */
  static std::atomic<uint64_t> status_request_count{0};
  const uint64_t sequence = ++status_request_count;
  int32_t physical_angle = 0;
  if (!WindowsNuiGetElevationAngle(&physical_angle)) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  nui_camera_elevation_angle = static_cast<uint32_t>(physical_angle);
  auto words = unk ? kernel_state()->memory()->TranslateVirtual<
                         xe::be<uint32_t>*>(unk.guest_address())
                   : nullptr;
  if (words && sequence <= 4) {
    XELOGI("XamNuiCameraTiltGetStatus: input={:08X} {:08X} {:08X} {:08X} "
           "sequence={}",
           static_cast<uint32_t>(words[0]), static_cast<uint32_t>(words[1]),
           static_cast<uint32_t>(words[2]), static_cast<uint32_t>(words[3]),
           sequence);
  }
  if (words) {
    // XtSs is the XAM request envelope, not part of the returned camera
    // state. It is supplied as {size, signature} by retail callers and must
    // survive a successful status query.
    std::memset(reinterpret_cast<uint8_t*>(words) + 8, 0, 0x50 - 8);
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltGetStatus, kNone, kImplemented);

dword_result_t XamNuiCameraElevationGetAngle_entry(lpdword_t angle,
                                                   lpdword_t target_angle) {
  int32_t native_angle = static_cast<int32_t>(nui_camera_elevation_angle);
  if (!WindowsNuiGetElevationAngle(&native_angle)) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  nui_camera_elevation_angle = static_cast<uint32_t>(native_angle);
  if (angle) {
    *angle = static_cast<uint32_t>(native_angle);
  }
  if (target_angle) {
    *target_angle = nui_camera_elevation_angle;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationGetAngle, kNone, kImplemented);

dword_result_t XamNuiCameraGetTiltControllerType_entry() {
  /* Notes:
     - undefined unk[8]
     - undefined8 local_28;
     - undefined8 local_20;
     - undefined8 local_18;
     - undefined4 local_10;
     - local_20 = 0;
     - local_18 = 0;
     - local_10 = 0;
     - local_28 = 0xf030000000000;
     - calls DetroitDeviceRequest(unk) -> result
     - returns (ulonglong)(LZCOUNT(result) << 0x20) >> 0x25
  */
  return nui_force_device_off ? 0 : 1;
}
DECLARE_XAM_EXPORT1(XamNuiCameraGetTiltControllerType, kNone, kImplemented);

dword_result_t XamNuiCameraSetFlags_entry(qword_t unk1, dword_t unk2) {
  /* Notes:
     - if XamNuiCameraGetTiltControllerType returns 1 then operation is done
     - else 0xffffffff8007048f
  */
  X_STATUS result = X_ERROR_SUCCESS;
  int Controller_Type = XamNuiCameraGetTiltControllerType_entry();

  if (Controller_Type == 1) {
    std::array<uint8_t, 0x50> tilt_status{};
    result = XamNuiCameraTiltGetStatus_entry(tilt_status.data());
    if (XSUCCEEDED(result)) {
      (void)unk1;
      (void)unk2;
    }
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamNuiCameraSetFlags, kNone, kImplemented);

dword_result_t XamIsNuiUIActive_entry() {
  return kernel_state()->xam_state()->is_xam_dialog_present_.load();
}
DECLARE_XAM_EXPORT1(XamIsNuiUIActive, kNone, kImplemented);

dword_result_t XamNuiIsDeviceReady_entry() {
  /* device_state Notes:
   - used with XNotifyBroadcast(kXNotificationSystemNUIHardwareStatusChanged,
   device_state)
   - known values:
     - 0x0001
     - 0x0004
     - 0x0040
  */
  const uint32_t ready = nui_force_device_off || !nui_guest_device_ready()
                             ? 0
                             : 1;
  XELOGI("XamNuiIsDeviceReady: force_off={} ready={}",
         nui_force_device_off, ready);
  return ready;
}
DECLARE_XAM_EXPORT1(XamNuiIsDeviceReady, kNone, kImplemented);

dword_result_t XamIsNuiAutomationEnabled_entry(unknown_t unk1, unknown_t unk2) {
  /* Notes:
     - XamIsNuiAutomationEnabled = XamIsNatalPlaybackEnabled
     - Always returns X_E_SUCCESS? Maybe check later versions
     - Recieves param but never interacts with them
     - No Operations
  */
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT2(XamIsNuiAutomationEnabled, kNone, kStub, kHighFrequency);

dword_result_t XamIsNatalPlaybackEnabled_entry(unknown_t unk1, unknown_t unk2) {
  /* Notes:
     - XamIsNuiAutomationEnabled = XamIsNatalPlaybackEnabled
     - Always returns X_E_SUCCESS? Maybe check later versions
     - Recieves param but never interacts with them
     - No Operations
  */
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT2(XamIsNatalPlaybackEnabled, kNone, kStub, kHighFrequency);

dword_result_t XamNuiIsChatMicEnabled_entry() {
  /* Notes:
     - calls a second function with a param of uint local_20 [4];
     - Second function calls ExGetXConfigSetting(7,9,local_30,0x1c,local_40);
     - Result is sent to *local_20[0] = ^
     - Once sent back to XamNuiIsChatMicEnabled it looks for byte that
     correlates to NUI mic setting
     - return uVar2 = (~(ulonglong)local_20[0] << 0x20) >> 0x23 & 1;
     - unless the second function returns something -1 or less then
     XamNuiIsChatMicEnabled 1
  */
  return true;
}
DECLARE_XAM_EXPORT1(XamNuiIsChatMicEnabled, kNone, kImplemented);

/* HUD Notes:
   - XamNuiHudGetEngagedTrackingID, XamNuiHudIsEnabled,
   XamNuiHudSetEngagedTrackingID, XamNuiHudInterpretFrame, and
   XamNuiHudGetEngagedEnrollmentIndex all utilize the same data address
   - engaged_tracking_id set second param of XamShowNuiTroubleshooterUI
*/
uint32_t nui_unknown_1 = 0;
uint32_t engaged_tracking_id = 0;
uint32_t engaged_enrollment_index = UINT32_MAX;
char nui_unknown_2 = '\0';
bool nui_force_device_off = false;
bool nui_natal_started = false;
uint32_t nui_camera_elevation_angle = 0;
std::atomic<uint64_t> nui_hud_version_0{0};
std::atomic<uint64_t> nui_hud_version_1{0};

namespace {

class NuiTiltCallbackRegistration {
 public:
  void Store(uint32_t callback, uint32_t context) {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = callback & ~1u;
    context_ = context;
    ++generation_;
  }

  bool Snapshot(uint32_t* callback, uint32_t* context,
                uint64_t* generation) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!callback_) {
      return false;
    }
    *callback = callback_;
    *context = context_;
    *generation = generation_;
    return true;
  }

  bool Matches(uint32_t callback, uint64_t generation) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return callback_ == callback && generation_ == generation;
  }

 private:
  mutable std::mutex mutex_;
  uint32_t callback_ = 0;
  uint32_t context_ = 0;
  uint64_t generation_ = 0;
};

NuiTiltCallbackRegistration& nui_tilt_callback_registration() {
  static NuiTiltCallbackRegistration registration;
  return registration;
}

}  // namespace

void QueueNuiTiltCallback(float elevation_degrees, const char* source) {
  if (!std::isfinite(elevation_degrees)) {
    XELOGW("XamNuiCameraTilt: rejected non-finite elevation from {}", source);
    return;
  }

  uint32_t callback = 0;
  uint32_t context = 0;
  uint64_t generation = 0;
  if (!nui_tilt_callback_registration().Snapshot(&callback, &context,
                                                 &generation)) {
    return;
  }

  auto* state = kernel_state();
  if (!state) {
    return;
  }

  const bool queued = state->QueueKernelDispatch(
      [state, callback, generation, elevation_degrees]() {
        if (KernelState::shared() != state ||
            !nui_tilt_callback_registration().Matches(callback, generation)) {
          return;
        }
        auto thread = XThread::GetCurrentThread();
        if (!thread) {
          XELOGW("XamNuiCameraTilt: no guest thread for callback={:08X}",
                 callback);
          return;
        }

        auto* memory = state->memory();
        const uint32_t payload_address =
            memory->SystemHeapAlloc(sizeof(xe::be<float>), alignof(float));
        auto* payload = payload_address
                            ? memory->TranslateVirtual<xe::be<float>*>(
                                  payload_address)
                            : nullptr;
        if (!payload) {
          XELOGE("XamNuiCameraTilt: failed to allocate guest payload");
          if (payload_address) {
            memory->SystemHeapFree(payload_address);
          }
          return;
        }

        *payload = elevation_degrees;
        uint64_t args[] = {0x0002B005, payload_address};
        state->processor()->Execute(thread->thread_state(), callback, args,
                                    xe::countof(args));
        if (KernelState::shared() == state) {
          memory->SystemHeapFree(payload_address);
        }
      });

  if (queued) {
    XELOGI("XamNuiCameraTilt: queued event=0002B005 elevation={:.2f} "
           "callback={:08X} context={:08X} source={}",
           elevation_degrees, callback, context, source);
  } else {
    XELOGW("XamNuiCameraTilt: unable to queue event for callback={:08X}",
           callback);
  }
}

void WindowsNuiSetHudInitializeState(uint64_t version_0, uint64_t version_1) {
  nui_hud_version_0.store(version_0, std::memory_order_release);
  nui_hud_version_1.store(version_1, std::memory_order_release);
  XELOGI("XamNui HUD initialization: versions={:016X} {:016X}", version_0,
         version_1);
}

void WindowsNuiNotifyAuxiliaryQueueRegistered(uint32_t message) {
  // The client registers these queues before it has completed its own NUI
  // lifecycle handshake. Retain the registration, but do not inject a state
  // transition here: the retail session-open request is the ordering point
  // for a hardware-present notification.
  XELOGI("Windows Kinect: registered NUI auxiliary queue {:08X}", message);
}

void BroadcastNuiHardwareStatus(uint32_t device_state) {
  kernel_state()->BroadcastNotification(
      kXNotificationSystemNUIHardwareStatusChanged, device_state);
  QueueXStudioDeviceCallback(device_state, 1, 0);
}

dword_result_t XamNuiHudSetEngagedTrackingID_entry(dword_t id) {
  static std::atomic<uint32_t> previous_id{UINT32_MAX};
  const uint32_t prior = previous_id.exchange(static_cast<uint32_t>(id));
  engaged_tracking_id = id;
  nui_unknown_1 = 1;
  nui_unknown_2 = '\1';
  if (prior != static_cast<uint32_t>(id)) {
    XELOGI("XamNuiHudSetEngagedTrackingID: title selected tracking={}",
           static_cast<uint32_t>(id));
  }
  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudSetEngagedTrackingID, kNone, kImplemented);

qword_result_t XamNuiHudGetEngagedTrackingID_entry() {
  return engaged_tracking_id;
}
DECLARE_XAM_EXPORT1(XamNuiHudGetEngagedTrackingID, kNone, kImplemented);

dword_result_t XamNuiHudGetEngagedEnrollmentIndex_entry() {
  // Dash NUI calls this no-argument export before handing the currently
  // engaged user to its next view. It must reflect the same normalized guest
  // skeleton as the direct 2B002 notification, rather than a host SDK slot.
  static std::atomic<uint32_t> previous_index{UINT32_MAX};
  const uint32_t index = engaged_enrollment_index;
  const uint32_t previous = previous_index.exchange(index);
  if (index != previous) {
    XELOGI("Windows Kinect: HUD engaged enrollment={} tracking={}", index,
           engaged_tracking_id);
  }
  return index;
}
DECLARE_XAM_EXPORT1(XamNuiHudGetEngagedEnrollmentIndex, kNone, kImplemented);

dword_result_t XamNuiHudIsEnabled_entry() {
  /* Notes:
     - checks if XamNuiIsDeviceReady false, if nui_unknown_1 exists, and
     nui_unknown_2 is equal to null terminated string
     - only returns true if one check fails and allows for other NUI functions
     to progress
  */
  return XamNuiIsDeviceReady_entry() ? 1 : 0;
}
DECLARE_XAM_EXPORT1(XamNuiHudIsEnabled, kNone, kImplemented);

uint32_t XeXamNuiHudCheck(dword_t unk1) {
  uint32_t check = XamNuiHudIsEnabled_entry();
  if (check == 0) {
    return X_ERROR_ACCESS_DENIED;
  }

  check = XamNuiHudSetEngagedTrackingID_entry(unk1);
  if (check != 0) {
    return X_ERROR_FUNCTION_FAILED;
  }
  return X_STATUS_SUCCESS;
}

dword_result_t XamNuiHudGetInitializeFlags_entry() {
  // The third 2B003 field is a guest structure address, not this bitfield.
  // These are the two retail HUD capability bits documented for the export.
  return 0x40000200;
}
DECLARE_XAM_EXPORT1(XamNuiHudGetInitializeFlags, kNone, kImplemented);

void XamNuiHudGetVersions_entry(lpqword_t unk1, lpqword_t unk2) {
  if (unk1) {
    *unk1 = nui_hud_version_0.load(std::memory_order_acquire);
  }
  if (unk2) {
    *unk2 = nui_hud_version_1.load(std::memory_order_acquire);
  }
}
DECLARE_XAM_EXPORT1(XamNuiHudGetVersions, kNone, kImplemented);

// UI
dword_result_t XamShowNuiTroubleshooterUI_entry(dword_t user_index,
                                                dword_t tracking_id,
                                                dword_t flags) {
  /* Notes:
     - calls XamPackageManagerGetExperienceMode(&var) with var = 1
     - If returns less than zero or (var & 1) == 0 then get error message:
       - if XamPackageManagerGetExperienceMode = 0 then call XamShowMessageBoxUI
         - if XamShowMessageBoxUI returns 0x3e5 then XamShowNuiTroubleshooterUI
     returns 0
       - else XamShowNuiTroubleshooterUI returns 0x65b and call another func
     - else:
       - call XamNuiHudSetEngagedTrackingID(tracking_id) and doesn't care aboot
     return and set var2 = 2
       - checks if (flag & 0x800000) == 0
         - if true call XamNuiGetDeviceStatus.
           - if XamNuiGetDeviceStatus != 0 set var2 = 3
       - else var2 = 4
       - XamAppRequestLoadEx(var2);
       - if return = 0 then XamShowNuiTroubleshooterUI returns 5
       - else set buffer[8] and call
     XMsgSystemProcessCall(0xfe,0x21028,buffer,0xc);
     - XamNuiNatalCameraUpdateComplete calls
     XamShowNuiTroubleshooterUI(0xff,0,0) if param = -0x7ff8fffe
  */

  XELOGI("XamShowNuiTroubleshooterUI: user={:08X} tracking={:08X} flags={:08X}",
         static_cast<uint32_t>(user_index), static_cast<uint32_t>(tracking_id),
         static_cast<uint32_t>(flags));

  // A real Windows Kinect is already attached and initialized. The Xbox UI
  // troubleshooter is not the guest sensor ABI and would stop the title
  // before it can consume the emulated frame path.
  if (nui_guest_device_ready()) {
    InitializeWindowsNuiGuestPath();
    XELOGI("XamShowNuiTroubleshooterUI: suppressed for ready Windows Kinect");
    return X_ERROR_SUCCESS;
  }

  if (cvars::headless) {
    return 0;
  }

  const Emulator* emulator = kernel_state()->emulator();
  ui::Window* display_window = emulator->display_window();
  ui::ImGuiDrawer* imgui_drawer = emulator->imgui_drawer();
  if (display_window && imgui_drawer) {
    xe::threading::Fence fence;
    if (display_window->app_context().CallInUIThreadSynchronous([&]() {
          xe::ui::ImGuiDialog::ShowMessageBox(
              imgui_drawer, "NUI Troubleshooter",
              "The game has indicated there is a problem with NUI (Kinect).")
              ->Then(&fence);
        })) {
      kernel_state()->xam_state()->is_xam_dialog_present_.store(true);
      fence.Wait();
      kernel_state()->xam_state()->is_xam_dialog_present_.store(false);
    }
  }

  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamShowNuiTroubleshooterUI, kNone, kStub);

dword_result_t XamShowNuiHardwareRequiredUI_entry(unknown_t unk1) {
  if (unk1 != 0) {
    return X_ERROR_INVALID_PARAMETER;
  }

  return XamShowNuiTroubleshooterUI_entry(0xff, 0, 0x400000);
}
DECLARE_XAM_EXPORT1(XamShowNuiHardwareRequiredUI, kNone, kImplemented);

dword_result_t XamShowNuiGuideUI_entry(unknown_t unk1, unknown_t unk2) {
  /* Notes:
   - calls an unnamed function that checks XamNuiHudIsEnabled and
   XamNuiHudSetEngagedTrackingID
     - if XamNuiHudIsEnabled returns false then fuctions fails return
   X_ERROR_ACCESS_DENIED
     - else calls XamNuiHudSetEngagedTrackingID and if returns less than 0 then
   returns X_ERROR_FUNCTION_FAILED
     - else return X_ERROR_SUCCESS
   - if return offunc is X_ERROR_SUCCESS then call up ui screen
   - else return value of func
  */

  // decompiler error stops me from knowing which param gets used here
  (void)unk1;
  (void)unk2;
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamShowNuiGuideUI, kNone, kStub);

/* XamNuiIdentity Notes:
   - most require message calls to xam in 0x0002Cxxx area
*/
bool IsWritableNuiOutputRange(uint32_t address, uint32_t size);

namespace {

constexpr uint32_t kNuiIdentityDatabaseMagic = 0x584E5549;  // XNUI
constexpr uint32_t kNuiIdentityDatabaseVersion = 1;
constexpr uint32_t kNuiIdentityNoEnrollment = 0xFFFFFFFF;
constexpr uint32_t kNuiIdentityNewEnrollment = 0x80000000;
constexpr uint32_t kNuiIdentityMaxRecords = 16;
constexpr uint32_t kNuiIdentityMaxTemplateSize = 4 * 1024 * 1024;
constexpr uint32_t kNuiIdentityMaxDatabaseSize = 16 * 1024 * 1024;
constexpr uint32_t kNuiIdentityQualityTemplateStored = 1;

struct NuiIdentityRecord {
  uint32_t biometric_id = 0;
  uint32_t enrollment_index = kNuiIdentityNoEnrollment;
  uint64_t owner_xuid = 0;
  uint32_t quality_flags = 0;
  std::vector<uint8_t> template_data;
  std::vector<uint8_t> color_texture;
};

void AppendNuiIdentityU32(std::vector<uint8_t>* bytes, uint32_t value) {
  bytes->push_back(static_cast<uint8_t>(value));
  bytes->push_back(static_cast<uint8_t>(value >> 8));
  bytes->push_back(static_cast<uint8_t>(value >> 16));
  bytes->push_back(static_cast<uint8_t>(value >> 24));
}

void AppendNuiIdentityU64(std::vector<uint8_t>* bytes, uint64_t value) {
  AppendNuiIdentityU32(bytes, static_cast<uint32_t>(value));
  AppendNuiIdentityU32(bytes, static_cast<uint32_t>(value >> 32));
}

bool ReadNuiIdentityU32(std::span<const uint8_t> bytes, size_t* offset,
                        uint32_t* value) {
  if (*offset > bytes.size() || bytes.size() - *offset < sizeof(uint32_t)) {
    return false;
  }
  const uint8_t* source = bytes.data() + *offset;
  *value = uint32_t(source[0]) | (uint32_t(source[1]) << 8) |
           (uint32_t(source[2]) << 16) | (uint32_t(source[3]) << 24);
  *offset += sizeof(uint32_t);
  return true;
}

bool ReadNuiIdentityU64(std::span<const uint8_t> bytes, size_t* offset,
                        uint64_t* value) {
  uint32_t low = 0;
  uint32_t high = 0;
  if (!ReadNuiIdentityU32(bytes, offset, &low) ||
      !ReadNuiIdentityU32(bytes, offset, &high)) {
    return false;
  }
  *value = uint64_t(low) | (uint64_t(high) << 32);
  return true;
}

class NuiIdentityDatabase {
 public:
  bool BeginEnrollment(KernelState* kernel_state, uint32_t user_index) {
    auto* profile = GetProfile(kernel_state, user_index);
    // The retail ABI uses an opaque value here in some Dashboard paths. Fall
    // back to the active profile instead of rejecting that valid flow.
    if (!profile) {
      profile = GetProfile(kernel_state, 0);
    }
    if (!profile) {
      return false;
    }

    std::lock_guard lock(mutex_);
    if (!EnsureLoadedLocked(kernel_state, profile->xuid())) {
      return false;
    }
    enrollment_requested_ = true;
    enrollment_user_xuid_ = profile->xuid();
    return true;
  }

  void AbortEnrollment() {
    std::lock_guard lock(mutex_);
    enrollment_requested_ = false;
    enrollment_user_xuid_ = 0;
  }

  bool GetEnrollmentIndex(KernelState* kernel_state, uint64_t xuid,
                          uint32_t* enrollment_index) {
    std::lock_guard lock(mutex_);
    if (!EnsureLoadedLocked(kernel_state, xuid)) {
      return false;
    }
    const NuiIdentityRecord* record = FindRecordForOwnerLocked(xuid);
    if (!record) {
      return false;
    }
    *enrollment_index = record->enrollment_index;
    return true;
  }

  bool HasEnrollment(KernelState* kernel_state, uint64_t xuid) {
    uint32_t enrollment_index = 0;
    return GetEnrollmentIndex(kernel_state, xuid, &enrollment_index);
  }

  X_RESULT RemoveEnrollment(KernelState* kernel_state, uint64_t xuid) {
    std::lock_guard lock(mutex_);
    if (!EnsureLoadedLocked(kernel_state, xuid)) {
      return X_ERROR_FUNCTION_FAILED;
    }

    bool removed = false;
    for (auto record = records_.begin(); record != records_.end();) {
      if (record->second.owner_xuid == xuid) {
        record = records_.erase(record);
        removed = true;
      } else {
        ++record;
      }
    }
    if (!removed) {
      return X_ERROR_FILE_NOT_FOUND;
    }
    if (!SaveLocked(kernel_state, xuid)) {
      return X_ERROR_FUNCTION_FAILED;
    }
    XELOGI("XAM NUI identity: removed enrollment(s) for XUID {:016X}", xuid);
    return X_ERROR_SUCCESS;
  }

  X_RESULT ReadTemplate(KernelState* kernel_state, uint64_t xuid,
                        uint32_t biometric_id, uint32_t* record_index,
                        std::vector<uint8_t>* template_data) {
    std::lock_guard lock(mutex_);
    if (!EnsureLoadedLocked(kernel_state, xuid)) {
      return X_ERROR_FUNCTION_FAILED;
    }
    const NuiIdentityRecord* record = FindRecordLocked(biometric_id, xuid);
    if (!record || record->template_data.empty()) {
      return X_ERROR_FILE_NOT_FOUND;
    }
    *record_index = record->enrollment_index;
    *template_data = record->template_data;
    return X_ERROR_SUCCESS;
  }

  X_RESULT WriteTemplate(KernelState* kernel_state, uint64_t xuid,
                         uint32_t biometric_id, uint32_t requested_index,
                         std::span<const uint8_t> template_data,
                         uint32_t* record_index) {
    if (template_data.empty() ||
        template_data.size() > kNuiIdentityMaxTemplateSize) {
      return X_ERROR_INVALID_PARAMETER;
    }

    std::lock_guard lock(mutex_);
    if (!EnsureLoadedLocked(kernel_state, xuid)) {
      return X_ERROR_FUNCTION_FAILED;
    }

    auto existing = records_.find(biometric_id);
    if (existing == records_.end() && records_.size() >= kNuiIdentityMaxRecords) {
      return X_ERROR_INSUFFICIENT_BUFFER;
    }

    NuiIdentityRecord& record = records_[biometric_id];
    record.biometric_id = biometric_id;
    record.owner_xuid = xuid;
    if (record.enrollment_index == kNuiIdentityNoEnrollment ||
        requested_index == kNuiIdentityNewEnrollment ||
        requested_index == kNuiIdentityNoEnrollment) {
      record.enrollment_index = AllocateEnrollmentIndexLocked();
    } else {
      record.enrollment_index = requested_index;
    }
    record.template_data.assign(template_data.begin(), template_data.end());
    record.quality_flags = kNuiIdentityQualityTemplateStored;

    if (!SaveLocked(kernel_state, xuid)) {
      return X_ERROR_FUNCTION_FAILED;
    }

    enrollment_requested_ = false;
    enrollment_user_xuid_ = 0;
    *record_index = record.enrollment_index;
    XELOGI("XAM NUI identity: stored biometric {:08X}, enrollment {} for "
           "XUID {:016X} ({} bytes)",
           biometric_id, record.enrollment_index, xuid, template_data.size());
    return X_ERROR_SUCCESS;
  }

 private:
  static UserProfile* GetProfile(KernelState* kernel_state,
                                 uint32_t user_index) {
    if (!kernel_state || !kernel_state->xam_state()) {
      return nullptr;
    }
    return kernel_state->xam_state()->GetUserProfile(user_index);
  }

  static std::string DatabasePath(uint64_t xuid) {
    return fmt::format("User_{:016X}:\\NuiIdentity.xdb", xuid);
  }

  bool EnsureLoadedLocked(KernelState* kernel_state, uint64_t xuid) {
    if (loaded_ && loaded_xuid_ == xuid) {
      return true;
    }

    records_.clear();
    enrollment_requested_ = false;
    enrollment_user_xuid_ = 0;
    loaded_ = false;
    loaded_xuid_ = xuid;

    auto* entry = kernel_state->file_system()->ResolvePath(DatabasePath(xuid));
    if (!entry) {
      loaded_ = true;
      return true;
    }
    if (entry->size() < 12 || entry->size() > kNuiIdentityMaxDatabaseSize) {
      XELOGW("XAM NUI identity: ignoring malformed database for XUID {:016X}",
             xuid);
      loaded_ = true;
      return true;
    }

    vfs::File* file = nullptr;
    if (entry->Open(vfs::FileAccess::kFileReadData, &file) != X_STATUS_SUCCESS ||
        !file) {
      XELOGW("XAM NUI identity: unable to open database for XUID {:016X}",
             xuid);
      return false;
    }
    std::vector<uint8_t> bytes(entry->size());
    size_t bytes_read = 0;
    const X_STATUS read_status = file->ReadSync(bytes, 0, &bytes_read);
    file->Destroy();
    if (read_status != X_STATUS_SUCCESS || bytes_read != bytes.size()) {
      XELOGW("XAM NUI identity: unable to read database for XUID {:016X}",
             xuid);
      return false;
    }

    size_t offset = 0;
    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t record_count = 0;
    if (!ReadNuiIdentityU32(bytes, &offset, &magic) ||
        !ReadNuiIdentityU32(bytes, &offset, &version) ||
        !ReadNuiIdentityU32(bytes, &offset, &record_count) ||
        magic != kNuiIdentityDatabaseMagic ||
        version != kNuiIdentityDatabaseVersion ||
        record_count > kNuiIdentityMaxRecords) {
      XELOGW("XAM NUI identity: invalid database header for XUID {:016X}",
             xuid);
      loaded_ = true;
      return true;
    }

    for (uint32_t record_number = 0; record_number < record_count;
         ++record_number) {
      NuiIdentityRecord record;
      uint32_t template_size = 0;
      uint32_t texture_size = 0;
      if (!ReadNuiIdentityU32(bytes, &offset, &record.biometric_id) ||
          !ReadNuiIdentityU32(bytes, &offset, &record.enrollment_index) ||
          !ReadNuiIdentityU64(bytes, &offset, &record.owner_xuid) ||
          !ReadNuiIdentityU32(bytes, &offset, &record.quality_flags) ||
          !ReadNuiIdentityU32(bytes, &offset, &template_size) ||
          !ReadNuiIdentityU32(bytes, &offset, &texture_size) ||
          template_size > kNuiIdentityMaxTemplateSize ||
          texture_size > kNuiIdentityMaxTemplateSize || offset > bytes.size() ||
          template_size > bytes.size() - offset ||
          texture_size > bytes.size() - offset - template_size) {
        XELOGW("XAM NUI identity: invalid record {} for XUID {:016X}",
               record_number, xuid);
        records_.clear();
        loaded_ = true;
        return true;
      }
      record.template_data.assign(bytes.begin() + offset,
                                  bytes.begin() + offset + template_size);
      offset += template_size;
      record.color_texture.assign(bytes.begin() + offset,
                                  bytes.begin() + offset + texture_size);
      offset += texture_size;
      if (record.enrollment_index != kNuiIdentityNoEnrollment &&
          !record.template_data.empty()) {
        records_.insert_or_assign(record.biometric_id, std::move(record));
      }
    }

    if (offset != bytes.size()) {
      XELOGW("XAM NUI identity: ignored trailing data for XUID {:016X}", xuid);
    }
    loaded_ = true;
    XELOGI("XAM NUI identity: loaded {} enrollment record(s) for XUID {:016X}",
           records_.size(), xuid);
    return true;
  }

  bool SaveLocked(KernelState* kernel_state, uint64_t xuid) {
    std::vector<const NuiIdentityRecord*> records;
    records.reserve(records_.size());
    for (const auto& [biometric_id, record] : records_) {
      (void)biometric_id;
      records.push_back(&record);
    }
    std::sort(records.begin(), records.end(),
              [](const NuiIdentityRecord* left, const NuiIdentityRecord* right) {
                return left->biometric_id < right->biometric_id;
              });

    std::vector<uint8_t> bytes;
    bytes.reserve(12 + records.size() * 32);
    AppendNuiIdentityU32(&bytes, kNuiIdentityDatabaseMagic);
    AppendNuiIdentityU32(&bytes, kNuiIdentityDatabaseVersion);
    AppendNuiIdentityU32(&bytes, static_cast<uint32_t>(records.size()));
    for (const NuiIdentityRecord* record : records) {
      if (record->template_data.size() > kNuiIdentityMaxTemplateSize ||
          record->color_texture.size() > kNuiIdentityMaxTemplateSize) {
        return false;
      }
      AppendNuiIdentityU32(&bytes, record->biometric_id);
      AppendNuiIdentityU32(&bytes, record->enrollment_index);
      AppendNuiIdentityU64(&bytes, record->owner_xuid);
      AppendNuiIdentityU32(&bytes, record->quality_flags);
      AppendNuiIdentityU32(&bytes,
                           static_cast<uint32_t>(record->template_data.size()));
      AppendNuiIdentityU32(&bytes,
                           static_cast<uint32_t>(record->color_texture.size()));
      bytes.insert(bytes.end(), record->template_data.begin(),
                   record->template_data.end());
      bytes.insert(bytes.end(), record->color_texture.begin(),
                   record->color_texture.end());
      if (bytes.size() > kNuiIdentityMaxDatabaseSize) {
        return false;
      }
    }

    vfs::File* file = nullptr;
    vfs::FileAction action;
    if (kernel_state->file_system()->OpenFile(
            nullptr, DatabasePath(xuid), vfs::FileDisposition::kOverwriteIf,
            vfs::FileAccess::kGenericWrite, false, true, &file, &action) !=
            X_STATUS_SUCCESS ||
        !file) {
      XELOGE("XAM NUI identity: unable to save database for XUID {:016X}",
             xuid);
      return false;
    }
    size_t bytes_written = 0;
    const X_STATUS write_status = file->WriteSync(bytes, 0, &bytes_written);
    file->Destroy();
    return write_status == X_STATUS_SUCCESS && bytes_written == bytes.size();
  }

  const NuiIdentityRecord* FindRecordLocked(uint32_t biometric_id,
                                             uint64_t xuid) const {
    const auto exact = records_.find(biometric_id);
    if (exact != records_.end() && exact->second.owner_xuid == xuid) {
      return &exact->second;
    }
    return FindRecordForOwnerLocked(xuid);
  }

  const NuiIdentityRecord* FindRecordForOwnerLocked(uint64_t xuid) const {
    for (const auto& [biometric_id, record] : records_) {
      (void)biometric_id;
      if (record.owner_xuid == xuid && !record.template_data.empty() &&
          record.enrollment_index != kNuiIdentityNoEnrollment) {
        return &record;
      }
    }
    return nullptr;
  }

  uint32_t AllocateEnrollmentIndexLocked() const {
    uint32_t next_index = 0;
    for (const auto& [biometric_id, record] : records_) {
      (void)biometric_id;
      if (record.enrollment_index != kNuiIdentityNoEnrollment &&
          record.enrollment_index != kNuiIdentityNewEnrollment) {
        next_index = std::max(next_index, record.enrollment_index + 1);
      }
    }
    return next_index;
  }

  std::mutex mutex_;
  uint64_t loaded_xuid_ = 0;
  bool loaded_ = false;
  bool enrollment_requested_ = false;
  uint64_t enrollment_user_xuid_ = 0;
  std::unordered_map<uint32_t, NuiIdentityRecord> records_;
};

NuiIdentityDatabase& nui_identity_database() {
  static NuiIdentityDatabase database;
  return database;
}

UserProfile* GetNuiUserProfile(uint32_t user_index) {
  auto* state = kernel_state();
  return state && state->xam_state()
             ? state->xam_state()->GetUserProfile(user_index)
             : nullptr;
}

X_RESULT CompleteNuiBiometricOperation(dword_t overlapped, X_RESULT result,
                                       uint32_t transferred) {
  if (!overlapped) {
    return result;
  }
  kernel_state()->CompleteOverlappedImmediateEx(
      static_cast<uint32_t>(overlapped), result, result, transferred);
  return X_ERROR_IO_PENDING;
}

}  // namespace

uint64_t NUI_Session_Id = 0;

qword_result_t XamNuiIdentityGetSessionId_entry() {
  if (NUI_Session_Id == 0) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto* profile = GetNuiUserProfile(0);
    const uint64_t xuid = profile ? profile->xuid() : 0;
    NUI_Session_Id = static_cast<uint64_t>(now.count()) ^
                     (xuid * UINT64_C(0x9E3779B97F4A7C15)) ^
                     UINT64_C(0x58454E49414E5549);
    if (!NUI_Session_Id) {
      NUI_Session_Id = UINT64_C(0x58454E49414E5549);
    }
  }
  return NUI_Session_Id;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetSessionId, kNone, kImplemented);

dword_result_t XamNuiIdentityEnrollForSignIn_entry(dword_t unk1, qword_t unk2,
                                                   qword_t unk3, dword_t unk4) {
  if (!nui_identity_database().BeginEnrollment(kernel_state(),
                                                static_cast<uint32_t>(unk1))) {
    return X_E_NO_SUCH_USER;
  }
  (void)unk2;
  (void)unk3;
  (void)unk4;
  XELOGI("XAM NUI identity: enrollment requested for user {}",
         static_cast<uint32_t>(unk1));
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityEnrollForSignIn, kNone, kImplemented);

dword_result_t XamNuiIdentityAbort_entry(dword_t unk) {
  (void)unk;
  nui_identity_database().AbortEnrollment();
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityAbort, kNone, kImplemented);

// Other
dword_result_t XamUserNuiEnableBiometric_entry(dword_t user_index,
                                               int_t enable) {
  auto* profile = GetNuiUserProfile(static_cast<uint32_t>(user_index));
  if (!profile) {
    return X_E_NO_SUCH_USER;
  }
  UserSetting setting(UserSettingId::XPROFILE_NUI_BIOMETRIC_SIGNIN,
                      int32_t(enable ? 1 : 0));
  kernel_state()->xam_state()->user_tracker()->UpsertSetting(
      profile->xuid(), kDashboardID, &setting);
  XELOGI("XAM NUI identity: biometric sign-in {} for user {} (XUID {:016X})",
         enable ? "enabled" : "disabled", static_cast<uint32_t>(user_index),
         profile->xuid());
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamUserNuiEnableBiometric, kNone, kImplemented);

dword_result_t XamUserNuiIsBiometricEnabled_entry(dword_t user_index) {
  auto* profile = GetNuiUserProfile(static_cast<uint32_t>(user_index));
  if (!profile) {
    return 0;
  }
  const auto setting = kernel_state()->xam_state()->user_tracker()->GetSetting(
      profile, kDashboardID,
      static_cast<uint32_t>(UserSettingId::XPROFILE_NUI_BIOMETRIC_SIGNIN));
  if (!setting) {
    return 0;
  }
  const UserDataTypes value = setting->get_host_data();
  const auto* enabled = std::get_if<int32_t>(&value);
  return enabled && *enabled != 0 ? 1 : 0;
}
DECLARE_XAM_EXPORT1(XamUserNuiIsBiometricEnabled, kNone, kImplemented);

void XamNuiPlayerEngagementUpdate_entry(qword_t unk1, unknown_t unk2,
                                        lpunknown_t unk3) {
  static std::atomic<uint32_t> engagement_update_count{0};
  const uint32_t sequence = ++engagement_update_count;
  if (sequence <= 8 || (sequence & 0x7F) == 0) {
    XELOGI("XamNuiPlayerEngagementUpdate: sequence={} arg0={:016X} "
           "arg1={:08X} arg2={:08X} tracked={} engaged={}",
           sequence, static_cast<uint64_t>(unk1),
           static_cast<uint32_t>(unk2), static_cast<uint32_t>(unk3),
           WindowsNuiTrackedCount(), engaged_tracking_id);
  }
}
DECLARE_XAM_EXPORT1(XamNuiPlayerEngagementUpdate, kNone, kImplemented);

dword_result_t XamNuiSetForceDeviceOff_entry(dword_t force_off) {
  nui_force_device_off = force_off != 0;
  BroadcastNuiHardwareStatus(nui_force_device_off ? 0x00000000 : 0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiSetForceDeviceOff, kNone, kImplemented);

dword_result_t XamNuiNatalCameraUpdateStarting_entry() {
  nui_natal_started = true;
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiNatalCameraUpdateStarting, kNone, kImplemented);

dword_result_t XamNuiNatalCameraUpdateComplete_entry() {
  nui_natal_started = true;
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiNatalCameraUpdateComplete, kNone, kImplemented);

dword_result_t XamNuiHudInterpretFrame_entry(dword_t frame_ptr) {
  const uint32_t guest_frame = frame_ptr ? static_cast<uint32_t>(frame_ptr)
                                         : WindowsNuiGetGuestFrame();
  const bool copied = WindowsNuiWriteGuestFrame(guest_frame);
  XELOGI("XamNuiHudInterpretFrame: frame={:08X} tracked={} frames={}",
         guest_frame, WindowsNuiTrackedCount(), WindowsNuiSkeletonFrameCount());
  return copied ? X_ERROR_SUCCESS : X_ERROR_DEVICE_NOT_CONNECTED;
}
DECLARE_XAM_EXPORT1(XamNuiHudInterpretFrame, kNone, kImplemented);

dword_result_t XamNuiHudEnableInputFilter_entry() {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiHudEnableInputFilter, kNone, kImplemented);

dword_result_t XamNuiCameraElevationAutoTilt_entry() {
  nui_natal_started = true;
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationAutoTilt, kNone, kImplemented);

dword_result_t XamNuiCameraElevationStopMovement_entry() {
  nui_natal_started = true;
  int32_t physical_angle = 0;
  if (!WindowsNuiGetElevationAngle(&physical_angle) ||
      !WindowsNuiSetElevationAngle(physical_angle)) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  nui_camera_elevation_angle = static_cast<uint32_t>(physical_angle);
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationStopMovement, kNone, kImplemented);

dword_result_t XamNuiCameraElevationReverseAutoTilt_entry() {
  nui_natal_started = true;
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationReverseAutoTilt, kNone, kImplemented);

dword_result_t XamNuiCameraElevationSetCallback_entry() {
  nui_natal_started = true;
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationSetCallback, kNone, kImplemented);

dword_result_t XamNuiCameraTiltReportStatus_entry() {
  nui_natal_started = true;
  int32_t physical_angle = static_cast<int32_t>(nui_camera_elevation_angle);
  if (!WindowsNuiGetElevationAngle(&physical_angle)) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  nui_camera_elevation_angle = static_cast<uint32_t>(physical_angle);
  XELOGD("XamNuiCameraTiltReportStatus: physical elevation={} degrees",
         physical_angle);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltReportStatus, kNone, kImplemented);

dword_result_t XamNuiCameraAdjustTilt_entry() {
  nui_natal_started = true;
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraAdjustTilt, kNone, kImplemented);

dword_result_t XamNuiCameraRememberFloor_entry() {
  nui_natal_started = true;
  if (!WindowsNuiRememberFloor()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }
  XELOGI("XamNuiCameraRememberFloor: captured physical floor plane");
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraRememberFloor, kNone, kImplemented);

dword_result_t XamNuiCameraElevationSetAngle_entry(dword_t angle,
                                                   dword_t flags) {
  const int32_t requested_angle = static_cast<int32_t>(angle);
  const int32_t clamped_angle =
      std::clamp(requested_angle, -27, 27);
  const int32_t physical_target =
      cvars::kinect_force_zero_elevation ? kXenectFixedElevationAngle
                                         : clamped_angle;
  nui_natal_started = true;
  (void)flags;
  if (!WindowsNuiSetElevationAngle(clamped_angle)) {
    XELOGW("XamNuiCameraElevationSetAngle: Kinect unavailable, requested={} "
           "clamped={}",
           requested_angle, clamped_angle);
    return X_E_DEVICE_NOT_CONNECTED;
  }
  nui_camera_elevation_angle = static_cast<uint32_t>(physical_target);
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraElevationSetAngle, kNone, kImplemented);

dword_result_t XamNuiCameraTiltSetCallback_entry(lpvoid_t callback,
                                                 lpvoid_t context) {
  nui_natal_started = true;
  const uint32_t callback_address = callback.guest_address();
  const uint32_t context_address = context.guest_address();
  nui_tilt_callback_registration().Store(callback_address, context_address);
  if (!callback_address) {
    XELOGI("XamNuiCameraTiltSetCallback: callback cleared");
    return X_ERROR_SUCCESS;
  }

  int32_t physical_angle = 0;
  if (WindowsNuiGetElevationAngle(&physical_angle)) {
    nui_camera_elevation_angle = static_cast<uint32_t>(physical_angle);
    QueueNuiTiltCallback(static_cast<float>(physical_angle),
                          "callback-registration");
  } else {
    XELOGW("XamNuiCameraTiltSetCallback: registered {:08X}, but no physical "
           "Kinect elevation is available",
           callback_address);
    return X_E_DEVICE_NOT_CONNECTED;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiCameraTiltSetCallback, kNone, kImplemented);

dword_result_t XamUserNuiGetEnrollmentIndex_entry(dword_t user_index,
                                                  lpdword_t index) {
  if (!index || !IsWritableNuiOutputRange(index.guest_address(),
                                           sizeof(uint32_t))) {
    return X_E_INVALIDARG;
  }
  *index = kNuiIdentityNoEnrollment;

  auto* profile = GetNuiUserProfile(static_cast<uint32_t>(user_index));
  uint32_t enrollment_index = kNuiIdentityNoEnrollment;
  if (profile && nui_identity_database().GetEnrollmentIndex(
                     kernel_state(), profile->xuid(), &enrollment_index)) {
    *index = enrollment_index;
    return X_ERROR_SUCCESS;
  }

  // A Kinect body bound to local user 0 is a valid controllerless guest even
  // without a persisted biometric template. The frame delivery path uses this
  // same token, and biometric read/write APIs still report the absence of a
  // real template rather than manufacturing one.
  if (static_cast<uint32_t>(user_index) == 0 && nui_guest_device_ready()) {
    *index = kNuiGuestEnrollmentIndex;
    return X_ERROR_SUCCESS;
  }
  return X_E_NO_SUCH_USER;
}
DECLARE_XAM_EXPORT1(XamUserNuiGetEnrollmentIndex, kNone, kImplemented);

dword_result_t XamShowNuiControllerRequiredUI_entry(dword_t user_index,
                                                    dword_t flags) {
  (void)user_index;
  (void)flags;
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamShowNuiControllerRequiredUI, kNone, kImplemented);

dword_result_t XamShowNuiGamerCardUIForXUID_entry(dword_t user_index,
                                                  qword_t xuid,
                                                  dword_t flags) {
  (void)user_index;
  (void)xuid;
  (void)flags;
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamShowNuiGamerCardUIForXUID, kNone, kImplemented);

dword_result_t XamShowNuiMarketplaceUI_entry(dword_t user_index,
                                             dword_t ui_type,
                                             qword_t offer_id,
                                             dword_t offer_type,
                                             dword_t content_category,
                                             dword_t title_id) {
  (void)user_index;
  (void)ui_type;
  (void)offer_id;
  (void)offer_type;
  (void)content_category;
  (void)title_id;
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamShowNuiMarketplaceUI, kNone, kImplemented);

dword_result_t XamShowNuiDeviceSelectorUI_entry(dword_t user_index,
                                                dword_t content_type,
                                                dword_t content_flags,
                                                qword_t total_requested,
                                                lpdword_t device_id_ptr,
                                                pointer_t<XAM_OVERLAPPED>
                                                    overlapped) {
  (void)user_index;
  (void)content_type;
  (void)content_flags;
  (void)total_requested;
  (void)device_id_ptr;
  (void)overlapped;
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamShowNuiDeviceSelectorUI, kNone, kImplemented);

dword_result_t XamEnableNuiAutomation_entry() {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamEnableNuiAutomation, kNone, kImplemented);

dword_result_t XamEnableNatalPlayback_entry() {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamEnableNatalPlayback, kNone, kImplemented);

dword_result_t XamReportKinectSettingsChangedEvent_entry() {
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamReportKinectSettingsChangedEvent, kNone, kImplemented);

dword_result_t XamNuiEnableChatMic_entry() {
  BroadcastNuiHardwareStatus(0x00000044);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiEnableChatMic, kNone, kImplemented);

dword_result_t XamNuiStoreDepthCalibration_entry() {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiStoreDepthCalibration, kNone, kImplemented);

dword_result_t XamNuiIdentityGetEnrollmentInfo_entry() {
  auto* profile = GetNuiUserProfile(0);
  return profile && nui_identity_database().HasEnrollment(kernel_state(),
                                                           profile->xuid())
             ? X_ERROR_SUCCESS
             : X_HRESULT_FROM_WIN32(X_ERROR_FILE_NOT_FOUND);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetEnrollmentInfo, kNone, kImplemented);

dword_result_t XamNuiIdentityUnenroll_entry(dword_t user_index) {
  // The recovered Dashboard code never invokes this ordinal. Retail callers
  // that do use it supply a local user index; constrain it to that form rather
  // than treating an unknown register value as an enrollment handle.
  if (static_cast<uint32_t>(user_index) >= 4) {
    return X_E_INVALIDARG;
  }
  auto* profile = GetNuiUserProfile(static_cast<uint32_t>(user_index));
  if (!profile) {
    return X_E_NO_SUCH_USER;
  }
  const X_RESULT result = nui_identity_database().RemoveEnrollment(
      kernel_state(), profile->xuid());
  return result == X_ERROR_SUCCESS ? result : X_HRESULT_FROM_WIN32(result);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityUnenroll, kNone, kImplemented);

dword_result_t XamNuiIdentityGetColorTexture_entry() {
  auto* profile = GetNuiUserProfile(0);
  return profile && nui_identity_database().HasEnrollment(kernel_state(),
                                                           profile->xuid())
             ? X_ERROR_SUCCESS
             : X_HRESULT_FROM_WIN32(X_ERROR_FILE_NOT_FOUND);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetColorTexture, kNone, kImplemented);

dword_result_t XamNuiIdentityGetQualityFlagsMessage_entry() {
  auto* profile = GetNuiUserProfile(0);
  return profile && nui_identity_database().HasEnrollment(kernel_state(),
                                                           profile->xuid())
             ? X_ERROR_SUCCESS
             : X_HRESULT_FROM_WIN32(X_ERROR_FILE_NOT_FOUND);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetQualityFlagsMessage, kNone, kImplemented);

dword_result_t XamNuiIdentityGetQualityFlags_entry() {
  auto* profile = GetNuiUserProfile(0);
  return profile && nui_identity_database().HasEnrollment(kernel_state(),
                                                           profile->xuid())
             ? X_ERROR_SUCCESS
             : X_HRESULT_FROM_WIN32(X_ERROR_FILE_NOT_FOUND);
}
DECLARE_XAM_EXPORT1(XamNuiIdentityGetQualityFlags, kNone, kImplemented);

dword_result_t XamNuiIdentityIdentifyWithBiometric_entry(
    dword_t tracking_id, lpvoid_t identity_context,
    lpdword_t enrollment_index_out, lpdword_t confidence_out) {
  if ((enrollment_index_out &&
       !IsWritableNuiOutputRange(enrollment_index_out.guest_address(),
                                 sizeof(uint32_t))) ||
      (confidence_out &&
       !IsWritableNuiOutputRange(confidence_out.guest_address(),
                                 sizeof(uint32_t)))) {
    return X_E_INVALIDARG;
  }
  if (enrollment_index_out) {
    *enrollment_index_out = kNuiIdentityNoEnrollment;
  }
  if (confidence_out) {
    *confidence_out = 0;
  }

  auto* profile = GetNuiUserProfile(0);
  uint32_t enrollment_index = kNuiIdentityNoEnrollment;
  const bool has_saved_enrollment =
      profile && nui_identity_database().GetEnrollmentIndex(
                     kernel_state(), profile->xuid(), &enrollment_index);
  if (!has_saved_enrollment) {
    // Identification of the live primary body is also used by retail clients
    // to join a Kinect cursor to the active local session. With the Windows
    // sensor there is no Xbox biometric store to query, but this particular
    // tracking ID has already been observed and bound by the frame producer.
    const uint32_t current_tracking_id = engaged_tracking_id;
    if (!tracking_id || !nui_guest_device_ready() ||
        static_cast<uint32_t>(tracking_id) != current_tracking_id) {
      return X_HRESULT_FROM_WIN32(X_ERROR_FILE_NOT_FOUND);
    }
    enrollment_index = kNuiGuestEnrollmentIndex;
  }

  // The sensor has already supplied a live skeleton through the NUI frame
  // path. The identity service associates that active tracking ID with the
  // enrollment saved for the signed-in local profile.
  if (enrollment_index_out) {
    *enrollment_index_out = enrollment_index;
  }
  if (confidence_out) {
    *confidence_out = 100;
  }
  XELOGI("XAM NUI identity: tracking {} matched enrollment {} (context "
         "{:08X})",
         static_cast<uint32_t>(tracking_id), enrollment_index,
         identity_context.guest_address());
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiIdentityIdentifyWithBiometric, kNone, kImplemented);

dword_result_t XamNuiGetDeviceSerialNumber_entry(lpvoid_t serial_out,
                                                  dword_t serial_capacity) {
  constexpr uint32_t kMinimumSerialCapacity = 2;
  const uint32_t capacity = static_cast<uint32_t>(serial_capacity);
  if (!serial_out || capacity < kMinimumSerialCapacity ||
      !IsWritableNuiOutputRange(serial_out.guest_address(), capacity)) {
    return X_E_INVALIDARG;
  }

  std::string unique_id;
  if (!WindowsNuiGetUniqueId(&unique_id) || unique_id.empty()) {
    return X_E_DEVICE_NOT_CONNECTED;
  }

  const uint32_t bytes_to_copy = std::min<uint32_t>(
      capacity - 1, static_cast<uint32_t>(unique_id.size()));
  auto* destination = kernel_state()->memory()->TranslateVirtual<char*>(
      serial_out.guest_address());
  if (!destination) {
    return X_E_INVALIDARG;
  }
  std::memcpy(destination, unique_id.data(), bytes_to_copy);
  destination[bytes_to_copy] = '\0';
  XELOGI("XamNuiGetDeviceSerialNumber: returned Kinect unique ID chars={} "
         "capacity={}",
         bytes_to_copy, capacity);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetDeviceSerialNumber, kNone, kImplemented);

bool IsWritableNuiOutputRange(uint32_t address, uint32_t size) {
  if (!address || !size || address > UINT32_MAX - (size - 1)) {
    return false;
  }
  auto memory = kernel_memory();
  auto heap = memory->LookupHeap(address);
  const uint32_t last_address = address + size - 1;
  if (!heap || memory->LookupHeap(last_address) != heap) {
    return false;
  }
  return heap->QueryRangeAccess(address, last_address) ==
         xe::memory::PageAccess::kReadWrite;
}

bool IsReadableNuiInputRange(uint32_t address, uint32_t size) {
  if (!address || !size || address > UINT32_MAX - (size - 1)) {
    return false;
  }
  auto memory = kernel_memory();
  auto heap = memory->LookupHeap(address);
  const uint32_t last_address = address + size - 1;
  if (!heap || memory->LookupHeap(last_address) != heap) {
    return false;
  }
  return heap->QueryRangeAccess(address, last_address) !=
         xe::memory::PageAccess::kNoAccess;
}

void WindowsNuiNoteFrameSubmission(uint32_t thread_id) {
  nui_submitter_thread_id.store(thread_id, std::memory_order_release);
  nui_submission_time_ms.store(NuiHostMonotonicMilliseconds(),
                               std::memory_order_release);
  const size_t history_slot = thread_id % kNuiWaitHistorySlots;
  if (nui_last_wait_thread[history_slot].load(std::memory_order_acquire) ==
      thread_id) {
    const uint32_t wait_object =
        nui_last_wait_object[history_slot].load(std::memory_order_acquire);
    const uint32_t wait_caller =
        nui_last_wait_caller[history_slot].load(std::memory_order_acquire);
    if (wait_object) {
      WindowsNuiObserveGuestWait(thread_id, wait_object, 3, wait_caller);
    }
  }
}

void WindowsNuiObserveGuestWait(uint32_t thread_id, uint32_t object_address,
                                uint32_t wait_reason, uint32_t caller) {
  constexpr uint32_t kSelectorOffset = 28;
  constexpr uint32_t kRingOffset = 40;
  constexpr uint32_t kSlotSize = 2784;
  constexpr uint64_t kRegistrationWindowMs = 5000;

  if (!thread_id || wait_reason != 3) {
    return;
  }
  const auto* observed_header = kernel_state()->memory()->TranslateVirtual<
      const X_DISPATCH_HEADER*>(object_address);
  if (observed_header && observed_header->type <= 1) {
    const size_t history_slot = thread_id % kNuiWaitHistorySlots;
    nui_last_wait_object[history_slot].store(object_address,
                                             std::memory_order_release);
    nui_last_wait_caller[history_slot].store(caller,
                                             std::memory_order_release);
    nui_last_wait_thread[history_slot].store(thread_id,
                                             std::memory_order_release);
  }
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (nui_consumer_ring_base.load(std::memory_order_acquire)) {
    return;
  }
#else
  if (nui_consumer_ring_base.load(std::memory_order_acquire)) {
    return;
  }
#endif
  const uint64_t submitted_at =
      nui_submission_time_ms.load(std::memory_order_acquire);
  const uint64_t now = NuiHostMonotonicMilliseconds();
  if (!submitted_at || now < submitted_at ||
      now - submitted_at > kRegistrationWindowMs) {
    return;
  }

  const uint32_t selector_address = object_address + kSelectorOffset;
  const uint32_t ring_base = object_address + kRingOffset;
  const bool event_writable =
      IsWritableNuiOutputRange(object_address, sizeof(X_KEVENT));
  const bool selector_writable =
      IsWritableNuiOutputRange(selector_address, sizeof(uint32_t));
  const bool slot0_writable =
      IsWritableNuiOutputRange(ring_base, kSlotSize);
  const bool slot1_writable =
      IsWritableNuiOutputRange(ring_base + kSlotSize, kSlotSize);
  static std::atomic<uint32_t> access_rejection_log_count{0};
  if (!event_writable || !selector_writable || !slot0_writable ||
      !slot1_writable) {
    if (access_rejection_log_count.fetch_add(1, std::memory_order_relaxed) <
        32) {
      XELOGI("Windows Kinect: NUI wait candidate access rejected waiter={} "
             "caller={:08X} event={:08X} age_ms={} writable=[{},{},{},{}]",
             thread_id, caller, object_address, now - submitted_at,
             event_writable, selector_writable, slot0_writable,
             slot1_writable);
    }
    return;
  }
  const auto* event = kernel_state()->memory()->TranslateVirtual<const X_KEVENT*>(
      object_address);
  const auto* selector = kernel_state()->memory()->TranslateVirtual<
      const xe::be<uint32_t>*>(selector_address);
  auto* slot0 = kernel_state()->memory()->TranslateVirtual<xe::be<uint32_t>*>(
      ring_base);
  auto* slot1 = kernel_state()->memory()->TranslateVirtual<xe::be<uint32_t>*>(
      ring_base + kSlotSize);
  static std::atomic<uint32_t> rejected_candidate_log_count{0};
  const uint32_t submitter_thread =
      nui_submitter_thread_id.load(std::memory_order_acquire);
  if (!event || !selector || !slot0 || !slot1) {
    if (rejected_candidate_log_count.fetch_add(1, std::memory_order_relaxed) <
        32) {
      XELOGI("Windows Kinect: NUI wait candidate unreadable submitter={} "
             "waiter={} caller={:08X} event={:08X} age_ms={}",
             submitter_thread, thread_id, caller, object_address,
             now - submitted_at);
    }
    return;
  }

  const uint32_t event_type = event->header.type;
  const uint32_t selector_value = *selector;
  const uint32_t slot0_state = *slot0;
  const uint32_t slot1_state = *slot1;
  // Notification and synchronization events are dispatcher types 0 and 1.
  const auto valid_slot_state = [](uint32_t state) {
    return state <= 2 || state == UINT32_MAX;
  };
  if (event_type > 1 || selector_value > 1 ||
      !valid_slot_state(slot0_state) || !valid_slot_state(slot1_state)) {
    if (rejected_candidate_log_count.fetch_add(1, std::memory_order_relaxed) <
        32) {
      XELOGI("Windows Kinect: NUI wait candidate rejected submitter={} "
             "waiter={} caller={:08X} event={:08X} age_ms={} type={} selector={} "
             "states=[{},{}]",
             submitter_thread, thread_id, caller, object_address,
             now - submitted_at, event_type, selector_value, slot0_state,
             slot1_state);
    }
    return;
  }

  nui_consumer_event_address.store(object_address, std::memory_order_release);
  nui_consumer_ring_base.store(ring_base, std::memory_order_release);
  nui_last_physical_ring_timestamp.store(0, std::memory_order_release);
  nui_consumer_event_reset_seen.store(false, std::memory_order_release);
  if (slot0_state == UINT32_MAX) {
    *slot0 = 0;
  }
  if (slot1_state == UINT32_MAX) {
    *slot1 = 0;
  }
  XELOGI("Windows Kinect: discovered retail NUI consumer ring thread={} "
         "event={:08X} selector={} ring={:08X} states=[{},{}]",
         thread_id, object_address, selector_value, ring_base, slot0_state,
         slot1_state);
#if defined(XENIA_HAS_KINECT_V1_SDK)
  if (IsSyntheticKinectFrameActive()) {
    StartWindowsNuiSyntheticRingProducer();
  }
#endif
}

void WindowsNuiObserveGuestEventReset(uint32_t object_address) {
  if (object_address &&
      object_address ==
          nui_consumer_event_address.load(std::memory_order_acquire)) {
    nui_consumer_event_reset_seen.store(true, std::memory_order_release);
    static std::atomic<uint32_t> reset_log_count{0};
    const uint32_t ring_base =
        nui_consumer_ring_base.load(std::memory_order_acquire);
    const auto* selector = ring_base
                               ? kernel_state()->memory()->TranslateVirtual<
                                     const xe::be<uint32_t>*>(ring_base - 12)
                               : nullptr;
    const auto* slot0 = ring_base
                            ? kernel_state()->memory()->TranslateVirtual<
                                  const xe::be<uint32_t>*>(ring_base)
                            : nullptr;
    const auto* slot1 = ring_base
                            ? kernel_state()->memory()->TranslateVirtual<
                                  const xe::be<uint32_t>*>(ring_base + 2784)
                            : nullptr;
    const uint32_t reset_count =
        reset_log_count.fetch_add(1, std::memory_order_relaxed) + 1;
    if (reset_count <= 8 || (reset_count & 0x7F) == 0) {
      XELOGI("Windows Kinect: retail NUI consumer reset event={:08X} "
             "selector={} states=[{},{}] count={}",
             object_address, selector ? static_cast<uint32_t>(*selector) : UINT32_MAX,
             slot0 ? static_cast<uint32_t>(*slot0) : UINT32_MAX,
             slot1 ? static_cast<uint32_t>(*slot1) : UINT32_MAX, reset_count);
    }
  }
}

void TraceXStudioEnvelope(uint32_t request, uint32_t argument) {
  // The XStudio ABI is private. Capture a bounded sample of the actual guest
  // envelopes before adding a handler so every response is based on a title's
  // observed buffer contract rather than a generic success stub.
  static std::mutex trace_mutex;
  static std::unordered_map<uint32_t, uint32_t> trace_count_by_request;
  bool should_trace = false;
  {
    std::lock_guard<std::mutex> lock(trace_mutex);
    auto& trace_count = trace_count_by_request[request];
    should_trace = trace_count++ < 3;
  }
  if (!should_trace) {
    return;
  }

  const uint32_t envelope_probe_size = request == 0x301 ? 128u : 48u;
  if (!IsReadableNuiInputRange(argument, envelope_probe_size)) {
    XELOGI("XamXStudioRequest envelope: request={:08X} argument={:08X} "
           "is not a readable {}-byte guest range",
           request, argument, envelope_probe_size);
    return;
  }

  const auto* words = kernel_state()->memory()->TranslateVirtual<
      const xe::be<uint32_t>*>(argument);
  if (!words) {
    XELOGI("XamXStudioRequest envelope: request={:08X} argument={:08X} "
           "could not be translated",
           request, argument);
    return;
  }
  XELOGI("XamXStudioRequest envelope: request={:08X} argument={:08X} "
         "words=[{:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} "
         "{:08X} {:08X} {:08X} {:08X}]",
         request, argument, static_cast<uint32_t>(words[0]),
         static_cast<uint32_t>(words[1]), static_cast<uint32_t>(words[2]),
         static_cast<uint32_t>(words[3]), static_cast<uint32_t>(words[4]),
         static_cast<uint32_t>(words[5]), static_cast<uint32_t>(words[6]),
         static_cast<uint32_t>(words[7]), static_cast<uint32_t>(words[8]),
         static_cast<uint32_t>(words[9]), static_cast<uint32_t>(words[10]),
         static_cast<uint32_t>(words[11]));

  if (request == 0x301) {
    std::string envelope_dump;
    for (uint32_t i = 0; i < envelope_probe_size / sizeof(uint32_t); ++i) {
      envelope_dump += fmt::format("{:08X}{}", uint32_t(words[i]),
                                   i + 1 == envelope_probe_size / 4 ? "" : " ");
    }
    XELOGI("XamXStudioRequest(301): envelope words=[{}]", envelope_dump);

    for (uint32_t word_index = 4; word_index <= 6; ++word_index) {
      const uint32_t pointed_address = static_cast<uint32_t>(words[word_index]);
      constexpr uint32_t kPointedProbeSize = 96;
      if (!IsReadableNuiInputRange(pointed_address, kPointedProbeSize)) {
        XELOGI("XamXStudioRequest(301): word[{}]={:08X} is not readable",
               word_index, pointed_address);
        continue;
      }
      const auto* pointed_bytes = kernel_state()->memory()->TranslateVirtual<
          const uint8_t*>(pointed_address);
      std::string pointed_dump;
      for (uint32_t i = 0; pointed_bytes && i < kPointedProbeSize; ++i) {
        pointed_dump += fmt::format("{:02X}{}", pointed_bytes[i],
                                    i + 1 == kPointedProbeSize ? "" : " ");
      }
      XELOGI("XamXStudioRequest(301): word[{}]={:08X} bytes=[{}]",
             word_index, pointed_address, pointed_dump);
    }
  }

  if (request == 0x252) {
    const uint32_t output_address = static_cast<uint32_t>(words[8]);
    const uint32_t output_size = static_cast<uint32_t>(words[9]);
    const uint32_t payload_address = static_cast<uint32_t>(words[10]);
    const uint32_t payload_size = static_cast<uint32_t>(words[11]);
    XELOGI("XamXStudioRequest(252): output={:08X}/{} payload={:08X}/{}",
           output_address, output_size, payload_address, payload_size);
    if (output_size && output_size <= 64 &&
        IsReadableNuiInputRange(output_address, output_size)) {
      const auto* bytes = kernel_state()->memory()->TranslateVirtual<
          const uint8_t*>(output_address);
      std::string dump;
      for (uint32_t i = 0; bytes && i < output_size; ++i) {
        dump += fmt::format("{:02X}{}", bytes[i],
                            i + 1 == output_size ? "" : " ");
      }
      XELOGI("XamXStudioRequest(252): output bytes before=[{}]", dump);
    }
    if (payload_size && payload_size <= 0x200000 &&
        IsReadableNuiInputRange(payload_address, payload_size)) {
      const auto* payload = kernel_state()->memory()->TranslateVirtual<
          const uint8_t*>(payload_address);
      constexpr uint32_t kSampleSize = 64;
      const uint32_t head_size = std::min(payload_size, kSampleSize);
      const uint32_t tail_offset = payload_size > kSampleSize
                                       ? payload_size - kSampleSize
                                       : 0;
      std::string head_dump;
      std::string tail_dump;
      uint32_t nonzero_count = 0;
      uint32_t rolling_hash = 2166136261u;
      for (uint32_t i = 0; payload && i < payload_size; ++i) {
        const uint8_t value = payload[i];
        nonzero_count += value != 0;
        rolling_hash = (rolling_hash ^ value) * 16777619u;
        if (i < head_size) {
          head_dump += fmt::format("{:02X}{}", value,
                                   i + 1 == head_size ? "" : " ");
        }
        if (i >= tail_offset) {
          tail_dump += fmt::format("{:02X}{}", value,
                                   i + 1 == payload_size ? "" : " ");
        }
      }
      XELOGI("XamXStudioRequest(252): payload nonzero={}/{} hash={:08X} "
             "head=[{}] tail=[{}]",
             nonzero_count, payload_size, rolling_hash, head_dump, tail_dump);
    }
  }
}

void ZeroNuiOutput(const char* function_name, lpvoid_t output, uint32_t size,
                   const ppc_context_t& ctx) {
  const uint32_t guest_address = output.guest_address();
  if (!guest_address) {
    return;
  }
  if (!IsWritableNuiOutputRange(guest_address, size)) {
    XELOGW("{}: ignoring non-writable output={:08X} size={} caller={:08X}",
           function_name, guest_address, size, uint32_t(ctx->lr));
    return;
  }
  XELOGI("{}: output={:08X} size={} caller={:08X}", function_name,
         guest_address, size, uint32_t(ctx->lr));
  std::memset(output, 0, size);
}

dword_result_t XamNuiGetLoadedDepthCalibration_entry(
    lpvoid_t calibration_out, const ppc_context_t& ctx) {
  ZeroNuiOutput("XamNuiGetLoadedDepthCalibration", calibration_out, 0x100,
                ctx);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetLoadedDepthCalibration, kNone, kImplemented);

dword_result_t XamNuiGetTrueColorInfo_entry(
    dword_t device_handle, dword_t query, lpvoid_t output,
    dword_t output_capacity, lpdword_t output_size,
    const ppc_context_t& ctx) {
  constexpr float kNominalFrameInterval = 34.188f;
  constexpr std::array<float, 21> kExposureTable = {
      49.1f,   7.5f,  0.1333f, 144.5f, 10.0f, 0.1f,   362.2f,
      15.0f,   0.0667f, 482.9f, 30.0f, 0.0333f, 724.3f, 30.0f,
      0.025f,  1448.7f, 30.0f,  0.0167f, 4102.6f, 30.0f, 0.0083f,
  };

  const uint32_t query_value = static_cast<uint32_t>(query);
  const uint32_t required_size =
      query_value == 0 ? sizeof(float)
                       : query_value == 1
                             ? uint32_t(kExposureTable.size() * sizeof(float))
                             : 0;
  XELOGI("XamNuiGetTrueColorInfo: device={} query={} output={:08X} "
         "capacity={} size_out={:08X} caller={:08X}",
         static_cast<uint32_t>(device_handle), query_value,
         output.guest_address(), static_cast<uint32_t>(output_capacity),
         output_size.guest_address(), uint32_t(ctx->lr));

  if (!required_size || !output_size ||
      !IsWritableNuiOutputRange(output_size.guest_address(), sizeof(uint32_t))) {
    return X_ERROR_INVALID_PARAMETER;
  }
  *output_size = required_size;
  if (!output || static_cast<uint32_t>(output_capacity) < required_size ||
      !IsWritableNuiOutputRange(output.guest_address(), required_size)) {
    return X_ERROR_INSUFFICIENT_BUFFER;
  }

  auto output_floats = output.as<xe::be<float>*>();
  if (query_value == 0) {
    output_floats[0] = kNominalFrameInterval;
  } else {
    for (size_t index = 0; index < kExposureTable.size(); ++index) {
      output_floats[index] = kExposureTable[index];
    }
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetTrueColorInfo, kNone, kImplemented);

dword_result_t XamNuiGetCameraIntrinsics_entry(
    lpvoid_t intrinsics_out, const ppc_context_t& ctx) {
  ZeroNuiOutput("XamNuiGetCameraIntrinsics", intrinsics_out, 0x80, ctx);
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetCameraIntrinsics, kNone, kImplemented);

dword_result_t XamNuiGetFanRate_entry(lpdword_t fan_rate_out) {
  if (fan_rate_out) {
    *fan_rate_out = 0;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetFanRate, kNone, kImplemented);

dword_result_t XamNuiGetSupportString_entry(lpvoid_t string_out) {
  if (string_out) {
    constexpr char kSupportString[] = "XENIA KINECT READY";
    std::memset(string_out, 0, 0x40);
    std::memcpy(string_out, kSupportString, sizeof(kSupportString));
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamNuiGetSupportString, kNone, kImplemented);

dword_result_t XamKinectGetHardwareType_entry() {
  return 1;
}
DECLARE_XAM_EXPORT1(XamKinectGetHardwareType, kNone, kImplemented);

dword_result_t XamFitnessInitialize_entry(lpdword_t fitness_handle_out) {
  if (!fitness_handle_out) {
    return X_E_INVALIDARG;
  }
  // Retail callers only use this value to distinguish an initialized Fitness
  // session from the optional no-service state.
  *fitness_handle_out = 1;
  XELOGI("XamFitnessInitialize: handle=1");
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamFitnessInitialize, kNone, kImplemented);

dword_result_t XamFitnessUnInitialize_entry() {
  XELOGI("XamFitnessUnInitialize");
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamFitnessUnInitialize, kNone, kImplemented);

dword_result_t XamReadBiometricData_entry(
    dword_t biometric_id, lpdword_t record_index_out, lpvoid_t buffer,
    lpdword_t buffer_size_inout, dword_t overlapped) {
  if (!buffer_size_inout ||
      !IsWritableNuiOutputRange(buffer_size_inout.guest_address(),
                                sizeof(uint32_t)) ||
      (record_index_out &&
       !IsWritableNuiOutputRange(record_index_out.guest_address(),
                                 sizeof(uint32_t)))) {
    return CompleteNuiBiometricOperation(overlapped, X_ERROR_INVALID_PARAMETER,
                                         0);
  }

  auto* profile = GetNuiUserProfile(0);
  if (!profile) {
    if (record_index_out) {
      *record_index_out = kNuiIdentityNoEnrollment;
    }
    *buffer_size_inout = 0;
    return CompleteNuiBiometricOperation(overlapped, X_ERROR_FILE_NOT_FOUND,
                                         0);
  }

  uint32_t record_index = kNuiIdentityNoEnrollment;
  std::vector<uint8_t> template_data;
  const X_RESULT result = nui_identity_database().ReadTemplate(
      kernel_state(), profile->xuid(), static_cast<uint32_t>(biometric_id),
      &record_index, &template_data);
  if (result != X_ERROR_SUCCESS) {
    if (record_index_out) {
      *record_index_out = kNuiIdentityNoEnrollment;
    }
    *buffer_size_inout = 0;
    return CompleteNuiBiometricOperation(overlapped, result, 0);
  }

  if (record_index_out) {
    *record_index_out = record_index;
  }
  const uint32_t required_size = static_cast<uint32_t>(template_data.size());
  const uint32_t supplied_size = *buffer_size_inout;
  *buffer_size_inout = required_size;
  if (!buffer || supplied_size < required_size ||
      !IsWritableNuiOutputRange(buffer.guest_address(), required_size)) {
    return CompleteNuiBiometricOperation(overlapped, X_ERROR_INSUFFICIENT_BUFFER,
                                         0);
  }

  std::memcpy(buffer, template_data.data(), template_data.size());
  XELOGI("XAM NUI identity: read biometric {:08X}, enrollment {} ({} bytes)",
         static_cast<uint32_t>(biometric_id), record_index, required_size);
  return CompleteNuiBiometricOperation(overlapped, X_ERROR_SUCCESS,
                                       required_size);
}
DECLARE_XAM_EXPORT1(XamReadBiometricData, kNone, kImplemented);

dword_result_t XamWriteBiometricData_entry(
    dword_t biometric_id, lpdword_t record_index_inout, lpvoid_t buffer,
    lpdword_t buffer_size_inout, dword_t overlapped) {
  if (!record_index_inout || !buffer_size_inout ||
      !IsWritableNuiOutputRange(record_index_inout.guest_address(),
                                sizeof(uint32_t)) ||
      !IsWritableNuiOutputRange(buffer_size_inout.guest_address(),
                                sizeof(uint32_t))) {
    return CompleteNuiBiometricOperation(overlapped, X_ERROR_INVALID_PARAMETER,
                                         0);
  }

  const uint32_t template_size = *buffer_size_inout;
  if (!buffer || !template_size ||
      template_size > kNuiIdentityMaxTemplateSize ||
      !IsWritableNuiOutputRange(buffer.guest_address(), template_size)) {
    return CompleteNuiBiometricOperation(overlapped, X_ERROR_INVALID_PARAMETER,
                                         0);
  }

  auto* profile = GetNuiUserProfile(0);
  if (!profile) {
    return CompleteNuiBiometricOperation(overlapped, X_ERROR_FILE_NOT_FOUND,
                                         0);
  }

  std::vector<uint8_t> template_data(template_size);
  std::memcpy(template_data.data(), buffer, template_size);
  uint32_t enrollment_index = *record_index_inout;
  const X_RESULT result = nui_identity_database().WriteTemplate(
      kernel_state(), profile->xuid(), static_cast<uint32_t>(biometric_id),
      enrollment_index, template_data, &enrollment_index);
  if (result == X_ERROR_SUCCESS) {
    *record_index_inout = enrollment_index;
  }
  return CompleteNuiBiometricOperation(
      overlapped, result, result == X_ERROR_SUCCESS ? template_size : 0);
}
DECLARE_XAM_EXPORT1(XamWriteBiometricData, kNone, kImplemented);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(NUI);
