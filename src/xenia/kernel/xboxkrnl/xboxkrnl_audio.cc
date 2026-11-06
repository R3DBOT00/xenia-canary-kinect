/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <atomic>
#include <cmath>

#include "xenia/apu/audio_system.h"
#include "xenia/emulator.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/xbox.h"

DECLARE_uint32(audio_flag);

namespace xe {
namespace kernel {
namespace xboxkrnl {

dword_result_t XAudioGetSpeakerConfig_entry(lpdword_t config_ptr) {
  kernel_state()->xconfig()->ReadSetting(
      XCONFIG_USER_CATEGORY,
      XCONFIG_USER_CATEGORY_ENTRIES::XCONFIG_USER_AUDIO_FLAGS, config_ptr);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(XAudioGetSpeakerConfig, kAudio, kImplemented);

dword_result_t XAudioGetVoiceCategoryVolumeChangeMask_entry(
    lpunknown_t driver_ptr, lpdword_t out_ptr) {
  assert_true((driver_ptr.guest_address() & 0xFFFF0000) == 0x41550000);

  xe::threading::MaybeYield();

  // Checking these bits to see if any voice volume changed.
  // I think.
  *out_ptr = 0;
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT2(XAudioGetVoiceCategoryVolumeChangeMask, kAudio, kStub,
                         kHighFrequency);

dword_result_t XAudioGetVoiceCategoryVolume_entry(dword_t unk,
                                                  lpfloat_t out_ptr) {
  // Expects a floating point single. Volume %?
  *out_ptr = 1.0f;

  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT2(XAudioGetVoiceCategoryVolume, kAudio, kStub,
                         kHighFrequency);

namespace {

std::atomic<bool> xaudio_ducker_enabled{false};
std::atomic<float> xaudio_ducker_level{1.0f};
std::atomic<float> xaudio_ducker_threshold{0.0f};
std::atomic<float> xaudio_ducker_attack_time{0.01f};
std::atomic<float> xaudio_ducker_release_time{0.01f};
std::atomic<float> xaudio_ducker_hold_time{0.0f};

float SanitizeDuckerValue(float value, float fallback,
                          bool must_be_positive = false) {
  if (!std::isfinite(value) || (must_be_positive && value <= 0.0f)) {
    return fallback;
  }
  return value;
}

}  // namespace

dword_result_t XAudioEnableDucker_entry(dword_t enabled) {
  xaudio_ducker_enabled.store(enabled != 0, std::memory_order_relaxed);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(XAudioEnableDucker, kAudio, kImplemented);

dword_result_t XAudioIsDuckerEnabled_entry() {
  return xaudio_ducker_enabled.load(std::memory_order_relaxed) ? 1u : 0u;
}
DECLARE_XBOXKRNL_EXPORT1(XAudioIsDuckerEnabled, kAudio, kImplemented);

float_result_t XAudioGetDuckerLevel_entry() {
  return xaudio_ducker_level.load(std::memory_order_relaxed);
}
DECLARE_XBOXKRNL_EXPORT1(XAudioGetDuckerLevel, kAudio, kImplemented);

dword_result_t XAudioSetDuckerLevel_entry(float_t value) {
  xaudio_ducker_level.store(SanitizeDuckerValue(value, 1.0f),
                            std::memory_order_relaxed);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(XAudioSetDuckerLevel, kAudio, kImplemented);

#define DEFINE_XAUDIO_DUCKER_PROPERTY(name, storage, fallback, positive)       \
  float_result_t XAudioGetDucker##name##_entry() {                            \
    return storage.load(std::memory_order_relaxed);                           \
  }                                                                            \
  DECLARE_XBOXKRNL_EXPORT1(XAudioGetDucker##name, kAudio, kImplemented);       \
  dword_result_t XAudioSetDucker##name##_entry(float_t value) {               \
    storage.store(SanitizeDuckerValue(value, fallback, positive),             \
                  std::memory_order_relaxed);                                  \
    return X_ERROR_SUCCESS;                                                    \
  }                                                                            \
  DECLARE_XBOXKRNL_EXPORT1(XAudioSetDucker##name, kAudio, kImplemented)

DEFINE_XAUDIO_DUCKER_PROPERTY(Threshold, xaudio_ducker_threshold, 0.0f, false);
DEFINE_XAUDIO_DUCKER_PROPERTY(AttackTime, xaudio_ducker_attack_time, 0.01f,
                              true);
DEFINE_XAUDIO_DUCKER_PROPERTY(ReleaseTime, xaudio_ducker_release_time, 0.01f,
                              true);
DEFINE_XAUDIO_DUCKER_PROPERTY(HoldTime, xaudio_ducker_hold_time, 0.0f, false);

#undef DEFINE_XAUDIO_DUCKER_PROPERTY

dword_result_t XAudioRegisterRenderDriverClient_entry(lpdword_t callback_ptr,
                                                      lpdword_t driver_ptr) {
  if (!callback_ptr) {
    return X_E_INVALIDARG;
  }

  uint32_t callback = callback_ptr[0];

  if (!callback) {
    return X_E_INVALIDARG;
  }
  uint32_t callback_arg = callback_ptr[1];

  auto audio_system = kernel_state()->emulator()->audio_system();

  size_t index;
  auto result = audio_system->RegisterClient(callback, callback_arg, &index);
  if (XFAILED(result)) {
    return result;
  }

  assert_true(!(index & ~0x0000FFFF));
  *driver_ptr = 0x41550000 | (static_cast<uint32_t>(index) & 0x0000FFFF);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(XAudioRegisterRenderDriverClient, kAudio,
                         kImplemented);

dword_result_t XAudioUnregisterRenderDriverClient_entry(
    lpunknown_t driver_ptr) {
  assert_true((driver_ptr.guest_address() & 0xFFFF0000) == 0x41550000);

  auto audio_system = kernel_state()->emulator()->audio_system();
  audio_system->UnregisterClient(driver_ptr.guest_address() & 0x0000FFFF);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(XAudioUnregisterRenderDriverClient, kAudio,
                         kImplemented);

dword_result_t XAudioSubmitRenderDriverFrame_entry(lpunknown_t driver_ptr,
                                                   lpunknown_t samples_ptr) {
  assert_true((driver_ptr.guest_address() & 0xFFFF0000) == 0x41550000);

  auto audio_system = kernel_state()->emulator()->audio_system();
  auto samples =
      kernel_state()->memory()->TranslateVirtual<float*>(samples_ptr);
  audio_system->SubmitFrame(driver_ptr.guest_address() & 0x0000FFFF, samples);

  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT2(XAudioSubmitRenderDriverFrame, kAudio, kImplemented,
                         kHighFrequency);

namespace {

void LogMecCall(std::string_view name, const ppc_context_t& context) {
  static std::atomic<uint32_t> call_count{0};
  const uint32_t sequence = ++call_count;
  if (sequence > 16) {
    return;
  }

  XELOGI("{}: r3={:08X} r4={:08X} r5={:08X} r6={:08X} r7={:08X} r8={:08X} "
         "r9={:08X} r10={:08X} lr={:08X}",
         name, static_cast<uint32_t>(context->r[3]),
         static_cast<uint32_t>(context->r[4]), static_cast<uint32_t>(context->r[5]),
         static_cast<uint32_t>(context->r[6]), static_cast<uint32_t>(context->r[7]),
         static_cast<uint32_t>(context->r[8]), static_cast<uint32_t>(context->r[9]),
         static_cast<uint32_t>(context->r[10]), static_cast<uint32_t>(context->lr));
}

}  // namespace

// The MEC exports are present in late retail kernels but aren't represented by
// the normal render-driver client interface. Keep their successful no-device
// behavior while capturing the ABI used by titles before writing any output.
dword_result_t XAudioRegisterRenderDriverMECClient_entry(
    const ppc_context_t& context) {
  LogMecCall("XAudioRegisterRenderDriverMECClient", context);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(XAudioRegisterRenderDriverMECClient, kAudio, kStub);

dword_result_t XAudioUnregisterRenderDriverMECClient_entry(
    const ppc_context_t& context) {
  LogMecCall("XAudioUnregisterRenderDriverMECClient", context);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(XAudioUnregisterRenderDriverMECClient, kAudio, kStub);

dword_result_t XAudioCaptureRenderDriverFrame_entry(const ppc_context_t& context) {
  LogMecCall("XAudioCaptureRenderDriverFrame", context);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT2(XAudioCaptureRenderDriverFrame, kAudio, kStub,
                         kHighFrequency);

dword_result_t XAudioGetRenderDriverTic_entry(const ppc_context_t& context) {
  LogMecCall("XAudioGetRenderDriverTic", context);
  return X_ERROR_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT2(XAudioGetRenderDriverTic, kAudio, kStub,
                         kHighFrequency);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(Audio);
