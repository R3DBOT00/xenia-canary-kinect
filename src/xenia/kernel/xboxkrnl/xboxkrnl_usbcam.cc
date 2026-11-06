/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "xenia/base/cvar.h"
#include "xenia/base/logging.h"
#include "xenia/cpu/processor.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/kernel/xthread.h"
#include "xenia/xbox.h"

DEFINE_bool(kinect_synthetic_test_frames, false,
            "Generate deterministic PsCam frames when no Kinect frame is "
            "available.",
            "Kinect");
DEFINE_int32(kinect_synthetic_bootstrap_seconds, 0,
             "Use coherent synthetic Kinect streams for this many seconds "
             "after the first NUI frame request, then switch to the physical "
             "sensor. Intended for end-to-end diagnostics.",
             "Kinect");
DECLARE_int32(kinect_pscam_depth_format);
DECLARE_int32(kinect_pscam_depth_byte_order);

namespace xe {
namespace kernel {

namespace xam {
bool IsWindowsNuiBackendReady();
bool IsSyntheticKinectFrameActive();
bool WindowsNuiCopyLatestColorFrame(std::vector<uint8_t>* out);
bool WindowsNuiCopyLatestRawColorFrame(std::vector<uint8_t>* out,
                                       uint64_t* timestamp,
                                       uint32_t* frame_number);
bool WindowsNuiCopyLatestRawDepthFrame(std::vector<uint8_t>* out,
                                       uint64_t* timestamp,
                                       uint32_t* frame_number);
bool WindowsNuiCopyLatestAccelerometer(std::array<float, 4>* out,
                                       uint32_t* sample_sequence);
bool WindowsNuiCopyCoordinateMapperParameters(std::vector<uint8_t>* out);
uint64_t WindowsNuiColorFrameCount();
uint64_t WindowsNuiDepthFrameCount();
}  // namespace xam

namespace xboxkrnl {

namespace {

constexpr uint32_t kPsCamColorChannel = 2;
constexpr uint32_t kPsCamDepthChannel = 4;
constexpr uint32_t kColorWidth = 640;
constexpr uint32_t kColorHeight = 480;
constexpr uint32_t kDepthWidth = 640;
constexpr uint32_t kDepthHeight = 480;
constexpr uint32_t kNativeDepthWidth = 320;
constexpr uint32_t kNativeDepthHeight = 240;
constexpr uint32_t kPsCamPlayerIndexBits = 3;

constexpr uint32_t kPsCamCalibrationSize = 4272;
constexpr uint32_t kPsCamRegistrationOffset = 8;
constexpr uint32_t kPsCamRegistrationCount = 29;
constexpr uint32_t kPsCamShiftToDepthOffset = 124;
constexpr uint32_t kPsCamShiftToDepthCount = 2048;
constexpr uint32_t kPsCamDepthNumeratorOffset = 4232;
constexpr uint32_t kPsCamZeroPlanePixelSizeOffset = 4240;
constexpr uint32_t kPsCamPixelSizeFactorOffset = 4248;
constexpr uint32_t kPsCamParamCoeffOffset = 4250;
constexpr uint32_t kPsCamDepthToShiftOffsetOffset = 4256;
constexpr uint32_t kPsCamEmitterDistanceOffset = 4264;

// INuiCoordinateMapper serializes this Kinect v1 factory data in the physical
// CDRP blob. These offsets are the host ABI, not guest-visible layout.
constexpr size_t kMapperShiftToDepthOffset = 0x001A;
constexpr size_t kMapperDepthToMetricOffset = 0x101A;
constexpr size_t kMapperRegistrationOffset = 0x3020;
constexpr size_t kMapperReferenceDistanceOffset = 0x30F4;
constexpr size_t kMapperReferencePixelSizeOffset = 0x30F8;
constexpr size_t kMapperMinimumSize = 0x30FC;

constexpr double kEmitterDistance = 7.5;
constexpr double kShiftScale = 10.0;
constexpr double kParamCoeff = 4.0;
constexpr double kConstShift = 200.0;
constexpr double kPixelSizeFactor = 1.0;

struct PsCamPhysicalCalibration {
  std::array<int32_t, kPsCamRegistrationCount> registration = {};
  std::array<uint16_t, kPsCamShiftToDepthCount> metric_shift_to_depth = {};
  std::array<uint16_t, 4096> metric_depth_table = {};
  double reference_distance = 0.0;
  double reference_pixel_size = 0.0;
};

uint16_t ReadLittleEndianU16(const std::vector<uint8_t>& bytes,
                             size_t offset) {
  return static_cast<uint16_t>(bytes[offset]) |
         static_cast<uint16_t>(bytes[offset + 1]) << 8;
}

uint32_t ReadLittleEndianU32(const std::vector<uint8_t>& bytes,
                             size_t offset) {
  return static_cast<uint32_t>(bytes[offset]) |
         static_cast<uint32_t>(bytes[offset + 1]) << 8 |
         static_cast<uint32_t>(bytes[offset + 2]) << 16 |
         static_cast<uint32_t>(bytes[offset + 3]) << 24;
}

float ReadLittleEndianFloat(const std::vector<uint8_t>& bytes, size_t offset) {
  const uint32_t bits = ReadLittleEndianU32(bytes, offset);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool ReadPhysicalKinectCalibration(PsCamPhysicalCalibration* calibration) {
  if (!calibration) {
    return false;
  }

  std::vector<uint8_t> mapper_parameters;
  if (!xam::WindowsNuiCopyCoordinateMapperParameters(&mapper_parameters) ||
      mapper_parameters.size() < kMapperMinimumSize) {
    return false;
  }

  PsCamPhysicalCalibration parsed;
  for (uint32_t index = 0; index < kPsCamRegistrationCount; ++index) {
    parsed.registration[index] = static_cast<int32_t>(ReadLittleEndianU32(
        mapper_parameters, kMapperRegistrationOffset + index * sizeof(int32_t)));
  }
  for (uint32_t index = 0; index < kPsCamShiftToDepthCount; ++index) {
    parsed.metric_shift_to_depth[index] = ReadLittleEndianU16(
        mapper_parameters, kMapperShiftToDepthOffset + index * sizeof(uint16_t));
  }
  for (uint32_t index = 0; index < parsed.metric_depth_table.size(); ++index) {
    parsed.metric_depth_table[index] = ReadLittleEndianU16(
        mapper_parameters, kMapperDepthToMetricOffset + index * sizeof(uint16_t));
  }

  parsed.reference_distance = ReadLittleEndianFloat(
      mapper_parameters, kMapperReferenceDistanceOffset);
  parsed.reference_pixel_size = ReadLittleEndianFloat(
      mapper_parameters, kMapperReferencePixelSizeOffset);
  if (!std::isfinite(parsed.reference_distance) ||
      !std::isfinite(parsed.reference_pixel_size) ||
      parsed.reference_distance <= 0.0 || parsed.reference_pixel_size <= 0.0 ||
      (parsed.registration[0] == 0 && parsed.registration[1] == 0 &&
       parsed.registration[2] == 0)) {
    return false;
  }

  *calibration = parsed;
  return true;
}

int32_t KinectGravityToMilligravity(float gravity) {
  if (!std::isfinite(gravity)) {
    return 0;
  }
  // The Kinect motor reports acceleration in 0.001 g units. The Windows
  // Kinect v1 API exposes that same physical vector in g, so this is a unit
  // conversion, not a synthetic orientation estimate.
  return static_cast<int32_t>(std::clamp(
      std::lround(static_cast<double>(gravity) * 1000.0), -32768L, 32767L));
}

uint16_t PsCamRawShiftToDepthMillimeters(
    uint32_t raw_shift, const PsCamPhysicalCalibration& calibration) {
  if (raw_shift >= kPsCamShiftToDepthCount) {
    return 0;
  }
  const double fixed_reference_x =
      (static_cast<double>(raw_shift) - kParamCoeff * kConstShift) /
          kParamCoeff -
      0.375;
  const double metric = fixed_reference_x * calibration.reference_pixel_size *
                        kPixelSizeFactor;
  const double denominator = kEmitterDistance - metric;
  if (denominator <= 0.0) {
    return 0;
  }
  const double depth =
      kShiftScale *
      (metric * calibration.reference_distance / denominator +
       calibration.reference_distance);
  if (depth <= 0.0 || depth >= 4000.0) {
    return 0;
  }
  return static_cast<uint16_t>(std::lround(depth));
}

bool ReadPsCamCalibration(PsCamPhysicalCalibration* calibration) {
  if (!xam::IsSyntheticKinectFrameActive()) {
    return ReadPhysicalKinectCalibration(calibration);
  }
  if (!calibration) {
    return false;
  }

  // Keep the synthetic PsCam fixture self-contained when no SDK sensor is
  // attached. These are deterministic Kinect v1 factory-scale parameters;
  // generated color and depth surfaces are already pixel-aligned, so the
  // remaining polynomial terms intentionally stay zero.
  PsCamPhysicalCalibration generated;
  generated.registration[0] = -520;
  generated.registration[1] = -94;
  generated.registration[2] = -14;
  generated.reference_distance = 120.0;
  generated.reference_pixel_size = 0.1042;
  for (uint32_t shift = 0; shift < generated.metric_shift_to_depth.size();
       ++shift) {
    generated.metric_shift_to_depth[shift] =
        PsCamRawShiftToDepthMillimeters(shift, generated);
  }
  for (uint32_t depth = 0; depth < generated.metric_depth_table.size();
       ++depth) {
    generated.metric_depth_table[depth] = static_cast<uint16_t>(depth);
  }
  *calibration = generated;
  return true;
}

void WritePsCamCalibration(uint8_t* response,
                            const PsCamPhysicalCalibration& calibration) {
  std::memset(response + 4, 0, kPsCamCalibrationSize - 4);
  auto registration = reinterpret_cast<xe::be<int32_t>*>(
      response + kPsCamRegistrationOffset);
  for (uint32_t index = 0; index < kPsCamRegistrationCount; ++index) {
    registration[index] = calibration.registration[index];
  }

  auto shift_to_depth = reinterpret_cast<xe::be<uint16_t>*>(
      response + kPsCamShiftToDepthOffset);
  for (uint32_t shift = 0; shift < kPsCamShiftToDepthCount; ++shift) {
    shift_to_depth[shift] = cvars::kinect_pscam_depth_format == 1
                                ? calibration.metric_shift_to_depth[shift]
                                : PsCamRawShiftToDepthMillimeters(shift,
                                                                   calibration);
  }

  // sub_82A73FC0 rebuilds the inverse depth-to-shift table as:
  //   shift = K * (1 - H / depth) + offset
  // These precomputed values express the same Kinect v1 calibration model.
  *reinterpret_cast<xe::be<double>*>(
      response + kPsCamDepthNumeratorOffset) =
      kShiftScale * calibration.reference_distance / kParamCoeff;
  *reinterpret_cast<xe::be<double>*>(
      response + kPsCamZeroPlanePixelSizeOffset) =
      calibration.reference_pixel_size;
  *reinterpret_cast<xe::be<uint16_t>*>(
      response + kPsCamPixelSizeFactorOffset) =
      static_cast<uint16_t>(kPixelSizeFactor * kDepthWidth);
  *reinterpret_cast<xe::be<uint16_t>*>(response +
                                      kPsCamParamCoeffOffset) =
      static_cast<uint16_t>(kParamCoeff);
  *reinterpret_cast<xe::be<double>*>(
      response + kPsCamDepthToShiftOffsetOffset) =
      0.375 * kParamCoeff + kParamCoeff * kConstShift / kPixelSizeFactor;
  *reinterpret_cast<xe::be<double>*>(
      response + kPsCamEmitterDistanceOffset) = kEmitterDistance;
}

uint64_t SyntheticFrameCount() {
  return GetTickCount64() / 33 + 1;
}

bool WriteSyntheticColorFrame(uint8_t* destination,
                              uint32_t destination_length,
                              uint32_t* bytes_written) {
  constexpr uint32_t kBayerSize = kColorWidth * kColorHeight;
  if (destination_length < kBayerSize) {
    return false;
  }
  const uint32_t phase = static_cast<uint32_t>(SyntheticFrameCount());
  for (uint32_t y = 0; y < kColorHeight; ++y) {
    for (uint32_t x = 0; x < kColorWidth; ++x) {
      destination[y * kColorWidth + x] = static_cast<uint8_t>(
          24 + ((x / 4 + y / 3 + phase) % 208));
    }
  }
  *bytes_written = kBayerSize;
  return true;
}

bool WriteSyntheticDepthFrame(uint8_t* destination,
                              uint32_t destination_length,
                              uint32_t* bytes_written) {
  constexpr uint32_t kDepthSize = kDepthWidth * kDepthHeight * 2;
  if (destination_length < kDepthSize) {
    return false;
  }
  const uint64_t gesture_phase_ms = GetTickCount64() % 8000;
  const bool hand_raised =
      gesture_phase_ms >= 2500 && gesture_phase_ms < 7000;
  for (uint32_t y = 0; y < kDepthHeight; ++y) {
    for (uint32_t x = 0; x < kDepthWidth; ++x) {
      const int32_t px = static_cast<int32_t>(x);
      const int32_t py = static_cast<int32_t>(y);
      const int32_t head_x = px - 320;
      const int32_t head_y = py - 105;
      const bool head = head_x * head_x + head_y * head_y < 38 * 38;
      const bool torso = px >= 270 && px <= 370 && py >= 140 && py <= 325;
      const bool left_leg = px >= 275 && px <= 315 && py >= 300 && py <= 465;
      const bool right_leg = px >= 325 && px <= 365 && py >= 300 && py <= 465;
      const bool left_arm = px >= 235 && px <= 285 && py >= 165 && py <= 330;
      const bool right_arm = hand_raised
                                 ? px >= 360 && px <= 425 && py >= 55 &&
                                       py <= 230
                                 : px >= 355 && px <= 405 && py >= 165 &&
                                       py <= 330;
      const bool foreground =
          head || torso || left_leg || right_leg || left_arm || right_arm;
      const uint16_t packed_depth = foreground ? 904 : 992;
      const uint16_t player_index = foreground ? 1 : 0;
      const uint16_t guest_value = static_cast<uint16_t>(
          (packed_depth << kPsCamPlayerIndexBits) | player_index);
      const uint32_t offset = (y * kDepthWidth + x) * 2;
      if (cvars::kinect_pscam_depth_byte_order == 1) {
        destination[offset] = static_cast<uint8_t>(guest_value);
        destination[offset + 1] = static_cast<uint8_t>(guest_value >> 8);
      } else {
        destination[offset] = static_cast<uint8_t>(guest_value >> 8);
        destination[offset + 1] = static_cast<uint8_t>(guest_value);
      }
    }
  }
  *bytes_written = kDepthSize;
  return true;
}

struct PsCamPendingRequest {
  KernelState* kernel_state;
  uint32_t request_ptr;
  uint32_t callback;
  uint32_t channel;
  uint32_t buffer_ptr;
  uint32_t buffer_length;
};

bool QueuePsCamCallback(KernelState* kernel_state, uint32_t callback,
                        uint32_t request_ptr, X_STATUS status,
                        uint32_t bytes_transferred) {
  if (!kernel_state || !callback) {
    return false;
  }
  return kernel_state->QueueKernelDispatch(
      [kernel_state, callback, request_ptr, status, bytes_transferred]() {
        if (KernelState::shared() != kernel_state) {
          return;
        }
        auto thread = XThread::GetCurrentThread();
        if (!thread) {
          return;
        }
        uint64_t args[] = {request_ptr, status, bytes_transferred};
        kernel_state->processor()->Execute(thread->thread_state(),
                                           callback & ~1u, args,
                                           xe::countof(args));

        // PsCam clients update their transfer descriptor from the callback.
        // Sample that guest-visible completion record after the callback has
        // returned so an ABI or byte-count mismatch is observable without
        // changing the result delivered to a title.
        auto memory = kernel_state->memory();
        auto request_words =
            memory->TranslateVirtual<xe::be<uint32_t>*>(request_ptr);
        const uint32_t descriptor_ptr =
            request_words ? static_cast<uint32_t>(request_words[4]) : 0;
        auto descriptor = descriptor_ptr
                              ? memory->TranslateVirtual<xe::be<uint32_t>*>(
                                    descriptor_ptr)
                              : nullptr;
        if (!descriptor) {
          return;
        }
        // PsCam completions expose the result through both the callback
        // argument and descriptor word 1. Retail clients inspect the
        // descriptor after the callback returns, so leaving its previous
        // value intact makes a successful asynchronous request look failed.
        descriptor[1] = static_cast<uint32_t>(status);
        static std::atomic<uint64_t> callback_trace_count{0};
        const uint64_t trace = ++callback_trace_count;
        const X_STATUS guest_status =
            static_cast<X_STATUS>(static_cast<uint32_t>(descriptor[1]));
        if (trace <= 16 || XFAILED(guest_status) || (trace & 0x7F) == 0) {
          XELOGI("PsCam callback: request={:08X} descriptor={:08X} "
                 "callback_status={:08X} callback_bytes={} "
                 "descriptor_status={:08X} descriptor_frame={:08X}",
                 request_ptr, descriptor_ptr, static_cast<uint32_t>(status),
                 bytes_transferred, static_cast<uint32_t>(guest_status),
                 static_cast<uint32_t>(descriptor[18]));
        }
      });
}

bool WriteColorFrame(uint8_t* destination, uint32_t destination_length,
                     uint32_t* bytes_written, uint64_t* timestamp,
                     uint32_t* frame_number) {
  constexpr uint32_t kBayerSize = kColorWidth * kColorHeight;
  constexpr uint32_t kUyvySize = kColorWidth * kColorHeight * 2;
  constexpr uint32_t kBgraSize = kColorWidth * kColorHeight * 4;

  if (xam::IsSyntheticKinectFrameActive()) {
    const uint64_t synthetic_frame = SyntheticFrameCount();
    *timestamp = synthetic_frame * 333333;
    *frame_number = static_cast<uint32_t>(synthetic_frame);
    if (destination_length < kBayerSize) {
      return false;
    }
    std::vector<uint8_t> synthetic_bayer(kBayerSize);
    if (!WriteSyntheticColorFrame(synthetic_bayer.data(), kBayerSize,
                                  bytes_written)) {
      return false;
    }
    if (destination_length >= kUyvySize) {
      for (uint32_t pixel = 0; pixel < kColorWidth * kColorHeight; pixel += 2) {
        const uint8_t y0 = synthetic_bayer[pixel];
        const uint8_t y1 = synthetic_bayer[pixel + 1];
        destination[pixel * 2] = 128;
        destination[pixel * 2 + 1] = y0;
        destination[pixel * 2 + 2] = 128;
        destination[pixel * 2 + 3] = y1;
      }
      *bytes_written = kUyvySize;
    } else {
      std::memcpy(destination, synthetic_bayer.data(), kBayerSize);
      *bytes_written = kBayerSize;
    }
    return true;
  }

  std::vector<uint8_t> raw_bayer;
  if (!xam::WindowsNuiCopyLatestRawColorFrame(&raw_bayer, timestamp,
                                              frame_number)) {
    if (!xam::IsSyntheticKinectFrameActive()) {
      return false;
    }
    const uint64_t synthetic_frame = SyntheticFrameCount();
    *timestamp = synthetic_frame * 333333;
    *frame_number = static_cast<uint32_t>(synthetic_frame);
    if (destination_length < kBayerSize) {
      return false;
    }
    std::vector<uint8_t> synthetic_bayer(kBayerSize);
    if (!WriteSyntheticColorFrame(synthetic_bayer.data(), kBayerSize,
                                  bytes_written)) {
      return false;
    }
    if (destination_length >= kUyvySize) {
      // Expand the deterministic Bayer fixture into valid neutral UYVY so
      // 16-bit PsCam consumers exercise the same byte-count contract.
      for (uint32_t pixel = 0; pixel < kColorWidth * kColorHeight;
           pixel += 2) {
        const uint8_t y0 = synthetic_bayer[pixel];
        const uint8_t y1 = synthetic_bayer[pixel + 1];
        destination[pixel * 2] = 128;
        destination[pixel * 2 + 1] = y0;
        destination[pixel * 2 + 2] = 128;
        destination[pixel * 2 + 3] = y1;
      }
      *bytes_written = kUyvySize;
    } else {
      std::memcpy(destination, synthetic_bayer.data(), kBayerSize);
    }
    return true;
  }

  // Kinect SDK v1 image timestamps are elapsed milliseconds. PsCam DMA
  // descriptors use the 100 ns camera clock also produced by the synthetic
  // backend, while XStudio skeleton envelopes intentionally remain in ms.
  *timestamp *= 10000;
  if (raw_bayer.size() < kBayerSize || destination_length < kBayerSize) {
    return false;
  }
  if (destination_length < kUyvySize) {
    std::memcpy(destination, raw_bayer.data(), kBayerSize);
    *bytes_written = kBayerSize;
    return true;
  }

  std::vector<uint8_t> bgra;
  if (!xam::WindowsNuiCopyLatestColorFrame(&bgra) ||
      bgra.size() < kBgraSize) {
    return false;
  }

  // Kinect's 640x480 16-bit color stream is packed UYVY. Some titles,
  // including Dance Central, allocate that negotiated size rather than the
  // 8-bit Bayer surface used by the retail camera's default stream. Convert
  // Xenia's physical BGRA capture per pixel pair so the completion size and
  // pixel encoding match the request instead of returning a short Bayer frame.
  auto clamp_byte = [](int32_t value) -> uint8_t {
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
  };
  for (uint32_t y = 0; y < kColorHeight; ++y) {
    for (uint32_t x = 0; x < kColorWidth; x += 2) {
      const auto* first = bgra.data() + (y * kColorWidth + x) * 4;
      const auto* second = first + 4;
      const int32_t b0 = first[0], g0 = first[1], r0 = first[2];
      const int32_t b1 = second[0], g1 = second[1], r1 = second[2];
      const uint8_t y0 = clamp_byte((77 * r0 + 150 * g0 + 29 * b0) >> 8);
      const uint8_t y1 = clamp_byte((77 * r1 + 150 * g1 + 29 * b1) >> 8);
      const uint8_t u = clamp_byte(
          ((-43 * r0 - 85 * g0 + 128 * b0 - 43 * r1 - 85 * g1 +
            128 * b1) >>
               9) +
          128);
      const uint8_t v = clamp_byte(
          ((128 * r0 - 107 * g0 - 21 * b0 + 128 * r1 - 107 * g1 -
            21 * b1) >>
               9) +
          128);
      auto* output = destination + (y * kColorWidth + x) * 2;
      output[0] = u;
      output[1] = y0;
      output[2] = v;
      output[3] = y1;
    }
  }

  *bytes_written = kUyvySize;
  static std::atomic<uint64_t> uyvy_frame_count{0};
  const uint64_t uyvy_frame = ++uyvy_frame_count;
  if (uyvy_frame <= 4 || (uyvy_frame & 0x7F) == 0) {
    XELOGI("PsCamDeviceRequest: converted physical Kinect BGRA 640x480 to "
           "UYVY bytes={} frame={}",
           kUyvySize, *frame_number);
  }
  return true;
}

bool WriteDepthFrame(uint8_t* destination, uint32_t destination_length,
                     uint32_t* bytes_written, uint64_t* timestamp,
                     uint32_t* frame_number) {
  if (xam::IsSyntheticKinectFrameActive()) {
    const uint64_t synthetic_frame = SyntheticFrameCount();
    *timestamp = synthetic_frame * 333333;
    *frame_number = static_cast<uint32_t>(synthetic_frame);
    return WriteSyntheticDepthFrame(destination, destination_length,
                                    bytes_written);
  }

  std::vector<uint8_t> source;
  if (!xam::WindowsNuiCopyLatestRawDepthFrame(&source, timestamp,
                                              frame_number)) {
    if (!xam::IsSyntheticKinectFrameActive()) {
      return false;
    }
    const uint64_t synthetic_frame = SyntheticFrameCount();
    *timestamp = synthetic_frame * 333333;
    *frame_number = static_cast<uint32_t>(synthetic_frame);
    return WriteSyntheticDepthFrame(destination, destination_length,
                                    bytes_written);
  }

  // Keep the physical image descriptors on PsCam's 100 ns clock. Skeleton
  // frames use the SDK millisecond clock and are converted separately into
  // the title-owned NUI structure.
  *timestamp *= 10000;

  constexpr uint32_t kDepthSize = kDepthWidth * kDepthHeight * 2;
  constexpr uint32_t kNativeDepthSize =
      kNativeDepthWidth * kNativeDepthHeight * 2;
  if (destination_length < kDepthSize) {
    return false;
  }
  if (source.size() >= kDepthSize) {
    std::memcpy(destination, source.data(), kDepthSize);
  } else if (source.size() >= kNativeDepthSize) {
    // Kinect v1 exposes depth at 320x240 while the Xbox PsCam client used by
    // retail titles requests a 640x480 depth surface. Expand each packed
    // depth/player-index word to a 2x2 block without decoding it, preserving
    // the byte order and low three player-index bits prepared by xam_nui.
    for (uint32_t source_y = 0; source_y < kNativeDepthHeight; ++source_y) {
      const uint8_t* source_row =
          source.data() + source_y * kNativeDepthWidth * 2;
      uint8_t* destination_row = destination + source_y * 2 * kDepthWidth * 2;
      for (uint32_t source_x = 0; source_x < kNativeDepthWidth; ++source_x) {
        const uint8_t* sample = source_row + source_x * 2;
        uint8_t* first = destination_row + source_x * 2 * 2;
        std::memcpy(first, sample, 2);
        std::memcpy(first + 2, sample, 2);
        std::memcpy(first + kDepthWidth * 2, sample, 2);
        std::memcpy(first + kDepthWidth * 2 + 2, sample, 2);
      }
    }
    static std::atomic<uint64_t> expanded_depth_frames{0};
    const uint64_t expanded = ++expanded_depth_frames;
    if (expanded <= 4 || (expanded & 0x7F) == 0) {
      XELOGI("PsCamDeviceRequest: expanded Kinect v1 depth {}x{} -> {}x{}",
             kNativeDepthWidth, kNativeDepthHeight, kDepthWidth,
             kDepthHeight);
    }
  } else {
    return false;
  }
  *bytes_written = kDepthSize;
  return true;
}

class PsCamRequestPump final {
 public:
  PsCamRequestPump() : worker_([this]() { WorkerMain(); }) {}

  ~PsCamRequestPump() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    worker_.join();
  }

  bool Queue(PsCamPendingRequest request) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto duplicate = std::find_if(
        pending_.begin(), pending_.end(), [&](const auto& pending) {
          return pending.request_ptr == request.request_ptr;
        });
    if (duplicate != pending_.end()) {
      return true;
    }
    if (pending_.size() >= 16) {
      return false;
    }
    pending_.push_back(std::move(request));
    condition_.notify_all();
    return true;
  }

 private:
  void WorkerMain() {
    while (true) {
      PsCamPendingRequest request = {};
      uint64_t native_frame = 0;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(2), [this]() {
          return stopping_ || !pending_.empty();
        });
        if (stopping_) {
          return;
        }

        std::erase_if(pending_, [](const auto& pending) {
          return KernelState::shared() != pending.kernel_state;
        });

        auto ready = pending_.end();
        for (auto it = pending_.begin(); it != pending_.end(); ++it) {
          const uint64_t frame =
              it->channel == kPsCamColorChannel
                  ? xam::WindowsNuiColorFrameCount()
                  : xam::WindowsNuiDepthFrameCount();
          const uint64_t available_frame =
              frame ? frame
                    : (xam::IsSyntheticKinectFrameActive()
                           ? SyntheticFrameCount()
                           : 0);
          const uint64_t last_frame =
              it->channel == kPsCamColorChannel ? last_color_frame_
                                                : last_depth_frame_;
          if (available_frame && available_frame != last_frame) {
            ready = it;
            native_frame = available_frame;
            break;
          }
        }
        if (ready == pending_.end()) {
          continue;
        }
        request = std::move(*ready);
        pending_.erase(ready);
      }

      uint32_t bytes_written = 0;
      const bool completed = Complete(request, native_frame, &bytes_written);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (completed) {
          if (request.channel == kPsCamColorChannel) {
            last_color_frame_ = native_frame;
          } else {
            last_depth_frame_ = native_frame;
          }
        } else if (!stopping_) {
          pending_.push_front(std::move(request));
        }
      }
      if (!completed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }

  bool Complete(const PsCamPendingRequest& request, uint64_t native_frame,
                uint32_t* bytes_written) {
    if (KernelState::shared() != request.kernel_state) {
      return true;
    }

    auto destination = request.kernel_state->memory()->TranslateVirtual<
        uint8_t*>(request.buffer_ptr);
    if (!destination) {
      return true;
    }

    uint64_t timestamp = 0;
    uint32_t frame_number = 0;
    const bool copied =
        request.channel == kPsCamColorChannel
            ? WriteColorFrame(destination, request.buffer_length, bytes_written,
                              &timestamp, &frame_number)
            : WriteDepthFrame(destination, request.buffer_length, bytes_written,
                              &timestamp, &frame_number);
    if (!copied) {
      return false;
    }

    // Validate the final guest-visible depth words, rather than only the host
    // capture vector. This makes byte order, player bits, and DMA destination
    // failures distinguishable without changing the data supplied to a title.
    if (request.channel == kPsCamDepthChannel &&
        *bytes_written >= kDepthWidth * kDepthHeight * 2) {
      const uint64_t trace = ++guest_depth_trace_count_;
      if (trace <= 8 || (trace & 0x3F) == 0) {
        uint16_t minimum_shift = UINT16_MAX;
        uint16_t maximum_shift = 0;
        uint32_t player_samples = 0;
        for (uint32_t y = 0; y < kDepthHeight; y += 8) {
          for (uint32_t x = 0; x < kDepthWidth; x += 8) {
            const uint32_t offset = (y * kDepthWidth + x) * 2;
            const uint16_t packed = cvars::kinect_pscam_depth_byte_order == 1
                                        ? static_cast<uint16_t>(
                                              destination[offset] |
                                              (destination[offset + 1] << 8))
                                        : static_cast<uint16_t>(
                                              (destination[offset] << 8) |
                                              destination[offset + 1]);
            const uint16_t shift = packed >> kPsCamPlayerIndexBits;
            if (shift && shift != 0x7FF) {
              minimum_shift = std::min(minimum_shift, shift);
              maximum_shift = std::max(maximum_shift, shift);
            }
            player_samples += (packed & ((1u << kPsCamPlayerIndexBits) - 1u)) !=
                              0;
          }
        }
        const uint32_t center_offset =
            ((kDepthHeight / 2) * kDepthWidth + (kDepthWidth / 2)) * 2;
        const uint16_t center_word =
            cvars::kinect_pscam_depth_byte_order == 1
                ? static_cast<uint16_t>(destination[center_offset] |
                                        (destination[center_offset + 1] << 8))
                : static_cast<uint16_t>((destination[center_offset] << 8) |
                                        destination[center_offset + 1]);
        XELOGI("PsCam DMA depth: buffer={:08X} frame={} order={} "
               "shift={}..{} sampled_player_pixels={} center={:04X}",
               request.buffer_ptr, frame_number,
               cvars::kinect_pscam_depth_byte_order,
               minimum_shift == UINT16_MAX ? 0 : minimum_shift,
               maximum_shift, player_samples, center_word);
      }
    }

    auto memory = request.kernel_state->memory();
    auto request_words =
        memory->TranslateVirtual<xe::be<uint32_t>*>(request.request_ptr);
    const uint32_t descriptor_ptr =
        request_words ? static_cast<uint32_t>(request_words[4]) : 0;
    auto descriptor = descriptor_ptr
                          ? memory->TranslateVirtual<uint8_t*>(descriptor_ptr)
                          : nullptr;
    if (!descriptor) {
      XELOGE("PsCamDeviceRequest: missing transfer descriptor request={:08X} "
             "descriptor={:08X}",
             request.request_ptr, descriptor_ptr);
      return true;
    }

    const auto descriptor_words =
        reinterpret_cast<const xe::be<uint32_t>*>(descriptor);
    const uint64_t descriptor_trace = ++descriptor_trace_count_;
    if (descriptor_trace <= 16) {
      XELOGI(
          "PsCam descriptor pre channel={} request={:08X} flags={:08X} "
          "error={:08X} packet={:02X} frame={:08X} timestamp={:016X} "
          "cb_frame={:08X} cb_clock={:08X} cb_global={:08X} "
          "private100={:08X} private104={:08X}",
          request.channel, request.request_ptr,
          static_cast<uint32_t>(descriptor_words[0]),
          static_cast<uint32_t>(descriptor_words[1]), descriptor[52],
          static_cast<uint32_t>(descriptor_words[18]),
          static_cast<uint64_t>(
              *reinterpret_cast<const xe::be<uint64_t>*>(descriptor + 80)),
          static_cast<uint32_t>(descriptor_words[22]),
          static_cast<uint32_t>(descriptor_words[23]),
          static_cast<uint32_t>(descriptor_words[24]),
          static_cast<uint32_t>(descriptor_words[25]),
          static_cast<uint32_t>(descriptor_words[26]));
    }

    descriptor[52] = static_cast<uint8_t>(frame_number);
    *reinterpret_cast<xe::be<uint32_t>*>(descriptor + 72) = frame_number;
    *reinterpret_cast<xe::be<uint64_t>*>(descriptor + 80) = timestamp;

    if (!QueuePsCamCallback(request.kernel_state, request.callback,
                            request.request_ptr, X_STATUS_SUCCESS,
                            *bytes_written)) {
      return false;
    }
    const uint64_t completion = ++completion_count_;
    if ((completion & 0x3F) == 1) {
      XELOGI("PsCamDeviceRequest: completed frame channel={} request={:08X} "
             "descriptor={:08X} buffer={:08X} bytes={} frame={} "
             "timestamp={} source_frame={} total={}",
             request.channel, request.request_ptr, descriptor_ptr,
             request.buffer_ptr, *bytes_written, frame_number, timestamp,
             native_frame, completion);
    }
    return true;
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<PsCamPendingRequest> pending_;
  bool stopping_ = false;
  uint64_t last_color_frame_ = 0;
  uint64_t last_depth_frame_ = 0;
  std::atomic<uint64_t> completion_count_{0};
  std::atomic<uint64_t> descriptor_trace_count_{0};
  std::atomic<uint64_t> guest_depth_trace_count_{0};
  std::thread worker_;
};

PsCamRequestPump& ps_cam_request_pump() {
  static PsCamRequestPump pump;
  return pump;
}

}  // namespace

dword_result_t XUsbcamCreate_entry(dword_t buffer,
                                   dword_t buffer_size,  // 0x4B000 640x480?
                                   lpdword_t handle_out) {
  if (!handle_out || !buffer || !buffer_size) {
    return X_STATUS_INVALID_PARAMETER;
  }
  if (!xam::IsWindowsNuiBackendReady()) {
    *handle_out = 0;
    return X_STATUS_INVALID_DEVICE_REQUEST;
  }
  *handle_out = 1;
  XELOGD("XUsbcamCreate(buffer={:08X}, size={:08X}) -> physical Kinect",
         static_cast<uint32_t>(buffer), static_cast<uint32_t>(buffer_size));
  return X_STATUS_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(XUsbcamCreate, kNone, kImplemented);

dword_result_t XUsbcamGetState_entry() {
  return xam::IsWindowsNuiBackendReady() ? 2 : 0;
}
DECLARE_XBOXKRNL_EXPORT1(XUsbcamGetState, kNone, kImplemented);

uint32_t PsCamDeviceRequestDispatch(uint32_t guest_request,
                                    uint32_t caller_address) {
  if (!guest_request) {
    return X_STATUS_INVALID_PARAMETER;
  }

  auto words = kernel_state()->memory()->TranslateVirtual<xe::be<uint32_t>*>(
      guest_request);
  if (!words) {
    return X_STATUS_INVALID_PARAMETER;
  }

  const uint32_t opcode = words[0];
  const uint32_t callback = words[5];
  const uint32_t channel = words[6];
  const uint32_t buffer_ptr = words[7];
  const uint32_t buffer_length = words[8];
  static std::atomic<uint64_t> request_count{0};
  const uint64_t sequence = ++request_count;
  if (sequence <= 16 || (sequence & 0x3F) == 1) {
    XELOGI("PsCamDeviceRequest: op={} request={:08X} callback={:08X} "
           "channel={} buffer={:08X} length={} sequence={} caller={:08X}",
           opcode, guest_request, callback, channel, buffer_ptr, buffer_length,
           sequence, caller_address);
  }
  if (sequence <= 4 || (opcode == 5 && sequence <= 16) || opcode == 36) {
    XELOGI(
        "PsCamDeviceRequest words: {:08X} {:08X} {:08X} {:08X} {:08X} "
        "{:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} "
        "{:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X}",
        static_cast<uint32_t>(words[0]), static_cast<uint32_t>(words[1]),
        static_cast<uint32_t>(words[2]), static_cast<uint32_t>(words[3]),
        static_cast<uint32_t>(words[4]), static_cast<uint32_t>(words[5]),
        static_cast<uint32_t>(words[6]), static_cast<uint32_t>(words[7]),
        static_cast<uint32_t>(words[8]), static_cast<uint32_t>(words[9]),
        static_cast<uint32_t>(words[10]), static_cast<uint32_t>(words[11]),
        static_cast<uint32_t>(words[12]), static_cast<uint32_t>(words[13]),
        static_cast<uint32_t>(words[14]), static_cast<uint32_t>(words[15]),
        static_cast<uint32_t>(words[16]), static_cast<uint32_t>(words[17]),
        static_cast<uint32_t>(words[18]), static_cast<uint32_t>(words[19]),
        static_cast<uint32_t>(words[20]), static_cast<uint32_t>(words[21]),
        static_cast<uint32_t>(words[22]), static_cast<uint32_t>(words[23]));
  }

  switch (opcode) {
    case 0:
      // The retail NUI state machine enters stream initialization for state 4.
      // State 3 only reports a present device and exits without opening streams.
      words[1] = 4;
      return X_STATUS_SUCCESS;

    case 2:
      // Device startup is completed through the request callback on retail.
      // Returning pending keeps the guest wrapper's event lifetime intact.
      if (callback) {
        if (QueuePsCamCallback(kernel_state(), callback, guest_request,
                               X_STATUS_SUCCESS, 0)) {
          return X_STATUS_PENDING;
        }
      }
      return X_STATUS_SUCCESS;

    case 13: {
      auto bytes = kernel_state()->memory()->TranslateVirtual<uint8_t*>(
          guest_request);
      std::memset(bytes + 4, 0, 28);
      bytes[4] = 5;
      bytes[5] = 1;
      return X_STATUS_SUCCESS;
    }

    case 14: {
      PsCamPhysicalCalibration calibration;
      if (!ReadPsCamCalibration(&calibration)) {
        XELOGW("PsCamDeviceRequest: physical Kinect calibration unavailable "
               "for opcode 14");
        return X_STATUS_INVALID_DEVICE_REQUEST;
      }
      auto bytes = kernel_state()->memory()->TranslateVirtual<uint8_t*>(
          guest_request);
      WritePsCamCalibration(bytes, calibration);
      XELOGI("PsCamDeviceRequest: supplied physical Kinect calibration "
             "registration[0..2]={},{},{} reference_distance={} "
             "reference_pixel_size={}",
             calibration.registration[0], calibration.registration[1],
             calibration.registration[2], calibration.reference_distance,
             calibration.reference_pixel_size);
      return X_STATUS_SUCCESS;
    }

    case 24: {
      PsCamPhysicalCalibration calibration;
      if (!ReadPsCamCalibration(&calibration)) {
        XELOGW("PsCamDeviceRequest: physical Kinect depth table unavailable "
               "for opcode 24");
        return X_STATUS_INVALID_DEVICE_REQUEST;
      }
      const uint32_t output_ptr = words[2];
      const uint32_t output_count = words[3];
      if (output_ptr && output_count && output_count <= 0x10000) {
        auto output = kernel_state()->memory()->TranslateVirtual<
            xe::be<uint16_t>*>(output_ptr);
        for (uint32_t index = 0; index < output_count; ++index) {
          output[index] =
              cvars::kinect_pscam_depth_format == 1 &&
                      index < calibration.metric_depth_table.size()
                  ? calibration.metric_depth_table[index]
                  : PsCamRawShiftToDepthMillimeters(index, calibration);
        }
        XELOGI("PsCamDeviceRequest: supplied {} depth correction table "
               "count={} bytes={} samples[678,842,983]={},{},{}",
               cvars::kinect_pscam_depth_format == 1 ? "metric-identity"
                                                     : "raw-to-metric",
               output_count, output_count * 2,
               output_count > 678 ? static_cast<uint16_t>(output[678]) : 0,
               output_count > 842 ? static_cast<uint16_t>(output[842]) : 0,
               output_count > 983 ? static_cast<uint16_t>(output[983]) : 0);
      }
      return X_STATUS_SUCCESS;
    }

    case 25:
      // USB scheduling values are expressed in 50 microsecond units.
      for (uint32_t index = 1; index <= 15; ++index) {
        words[index] = 667;
      }
      return X_STATUS_SUCCESS;

    case 33:
      // All three retail clients issue this after loading the depth conversion
      // table and before configuring their stream queue. The request selects
      // the packed depth-and-player-index producer; reject it only when the
      // physical capture backend is absent instead of silently acknowledging
      // an unconfigured camera path.
      if (!xam::IsWindowsNuiBackendReady()) {
        return X_STATUS_INVALID_DEVICE_REQUEST;
      }
      words[1] = 1;
      XELOGI("PsCamDeviceRequest: enabled physical depth/player stream "
             "request={:08X}",
             guest_request);
      return X_STATUS_SUCCESS;

    case 27:
    case 35:
      // The shared Kinect retail client submits these control requests with a
      // completion callback, then waits for STATUS_PENDING to resolve. They
      // carry no image payload, but completing synchronously skips the
      // client's device-state transition entirely. Route them through the
      // same kernel callback path as stream setup so titles observe actual
      // asynchronous completion and the supplied status argument.
      if (!callback) {
        return X_STATUS_INVALID_PARAMETER;
      }
      if (!QueuePsCamCallback(kernel_state(), callback, guest_request,
                              X_STATUS_SUCCESS, 0)) {
        return X_STATUS_UNSUCCESSFUL;
      }
      XELOGI("PsCamDeviceRequest: queued control completion op={} "
             "request={:08X} callback={:08X}",
             opcode, guest_request, callback);
      return X_STATUS_PENDING;

    case 36: {
      // All three target NUI clients issue this synchronous-over-async query
      // and then consume a four-component accelerometer sample from words
      // 19-22. The physical Kinect motor exposes exactly that vector through
      // the SDK in g; retail PsCam represents it as signed milligravity.
      if (!callback) {
        return X_STATUS_INVALID_PARAMETER;
      }
      std::array<float, 4> gravity = {};
      uint32_t sample_sequence = 0;
      if (!xam::WindowsNuiCopyLatestAccelerometer(&gravity,
                                                   &sample_sequence)) {
        return X_STATUS_INVALID_DEVICE_REQUEST;
      }
      words[7] = 0xF;
      words[19] = static_cast<uint32_t>(
          KinectGravityToMilligravity(gravity[0]));
      words[20] = static_cast<uint32_t>(
          KinectGravityToMilligravity(gravity[1]));
      words[21] = static_cast<uint32_t>(
          KinectGravityToMilligravity(gravity[2]));
      words[22] = static_cast<uint32_t>(
          KinectGravityToMilligravity(gravity[3]));
      // The shared retail client routes a positive value in this field to its
      // camera event dispatcher. Reading the accelerometer produces no
      // separate motor event, so report the real idle-event state instead of
      // manufacturing a timestamp or a notification code.
      words[23] = 0;
      if (!QueuePsCamCallback(kernel_state(), callback, guest_request,
                              X_STATUS_SUCCESS, 0)) {
        return X_STATUS_UNSUCCESSFUL;
      }
      XELOGI("PsCamDeviceRequest: queued physical accelerometer sample={} "
             "millig=({}, {}, {}, {})",
             sample_sequence, KinectGravityToMilligravity(gravity[0]),
             KinectGravityToMilligravity(gravity[1]),
             KinectGravityToMilligravity(gravity[2]),
             KinectGravityToMilligravity(gravity[3]));
      return X_STATUS_PENDING;
    }

    case 5: {
      if (!callback || !buffer_ptr || !buffer_length ||
          (channel != kPsCamColorChannel &&
           channel != kPsCamDepthChannel)) {
        return X_STATUS_INVALID_PARAMETER;
      }
      if (!ps_cam_request_pump().Queue(
              {kernel_state(), guest_request, callback, channel, buffer_ptr,
               buffer_length})) {
        return X_STATUS_UNSUCCESSFUL;
      }
      return X_STATUS_PENDING;
    }

    case 15:
      // Stream configuration completes through the driver dispatch callback.
      if (callback) {
        if (QueuePsCamCallback(kernel_state(), callback, guest_request,
                               X_STATUS_SUCCESS, 0)) {
          return X_STATUS_PENDING;
        }
      }
      return X_STATUS_SUCCESS;

    default:
      XELOGW("PsCamDeviceRequest: unsupported opcode={} request={:08X}",
             opcode, guest_request);
      return X_STATUS_INVALID_DEVICE_REQUEST;
  }
}

dword_result_t PsCamDeviceRequest_entry(dword_t request_ptr,
                                        const ppc_context_t& context) {
  return PsCamDeviceRequestDispatch(static_cast<uint32_t>(request_ptr),
                                    static_cast<uint32_t>(context->lr));
}
DECLARE_XBOXKRNL_EXPORT1(PsCamDeviceRequest, kNone, kImplemented);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(Usbcam);
