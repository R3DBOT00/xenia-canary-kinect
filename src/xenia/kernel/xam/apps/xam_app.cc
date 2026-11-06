/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2021 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/xam/apps/xam_app.h"

#include <atomic>

#include "xenia/base/logging.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/xam/xam_content_device.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/kernel/xenumerator.h"

/* Notes:
   - Messages ids that start with 0x00021xxx are UI calls
   - Messages ids that start with 0x00023xxx are used for the user profile
   - Messages ids that start with 0x0002Bxxx are used by the Kinect device
   usually for camera related functions
   - Messages ids that start with 0x0002Cxxx are used by the XamNuiIdentity
   functions
*/

namespace xe {
namespace kernel {
namespace xam {

void WindowsNuiNotifyAuxiliaryQueueRegistered(uint32_t message);

namespace apps {

XamApp::XamApp(KernelState* kernel_state) : App(kernel_state, 0xFE) {}

X_HRESULT XamApp::DispatchMessageSync(uint32_t message, uint32_t buffer_ptr,
                                      uint32_t buffer_length) {
  // NOTE: buffer_length may be zero or valid.
  auto buffer = memory_->TranslateVirtual(buffer_ptr);
  switch (message) {
    case 0x0002000E: {
      X_ENUMERATE_PARAM* data_ptr =
          reinterpret_cast<X_ENUMERATE_PARAM*>(buffer);

      XELOGD(
          "XEnumerateCrossTitle({:04X}, {:04X}, {:04X}, {:04X}, {}, {}, "
          "{:04X})",
          data_ptr->user_index.get(), data_ptr->flags.get(),
          data_ptr->private_enum_structure_ptr.get(),
          data_ptr->buffer_ptr.get(), data_ptr->buffer_size.get(),
          data_ptr->items_requested.get(), data_ptr->items_returned_ptr.get());

      if (!data_ptr->buffer_ptr || !data_ptr->private_enum_structure_ptr) {
        return X_E_INVALIDARG;
      }

      auto enum_struct =
          memory_->TranslateVirtual<X_KENUMERATOR_CONTENT_AGGREGATE*>(
              data_ptr->private_enum_structure_ptr);

      auto e = kernel_state_->object_table()->LookupObject<XEnumerator>(
          enum_struct->handle);

      if (!e) {
        return X_E_INVALIDARG;
      }

      assert_true(enum_struct->magic == kXObjSignature);

      XCONTENT_CROSS_TITLE_DATA cross_title_data = {};
      uint8_t* cross_title_data_ptr =
          reinterpret_cast<uint8_t*>(&cross_title_data);

      uint32_t item_count = 0;
      X_RESULT result = e->WriteItems(cross_title_data_ptr,
                                      data_ptr->buffer_size, &item_count);

      XCONTENT_DATA_INTERNAL* content_data_ptr =
          memory_->TranslateVirtual<XCONTENT_DATA_INTERNAL*>(
              data_ptr->buffer_ptr);

      assert_true(data_ptr->buffer_size == sizeof(XCONTENT_DATA_INTERNAL));

      std::memset(content_data_ptr, 0, data_ptr->buffer_size);

      if (!result) {
        content_data_ptr->device_id = cross_title_data.content_data.device_id;
        content_data_ptr->content_type =
            cross_title_data.content_data.content_type;
        content_data_ptr->set_display_name(
            cross_title_data.content_data.display_name());
        content_data_ptr->set_file_name(
            cross_title_data.content_data.file_name());
        content_data_ptr->padding[0] = content_data_ptr->padding[1] = 0;
        content_data_ptr->title_id = cross_title_data.title_id;
      }

      result = X_HRESULT_FROM_WIN32(result);

      xe::be<uint32_t>* items_returned_ptr =
          memory_->TranslateVirtual<xe::be<uint32_t>*>(
              data_ptr->items_returned_ptr);

      *items_returned_ptr = item_count;

      return result;
    }
    case 0x00020021: {
      struct XContentQueryVolumeDeviceType {
        char root_name[64];
        xe::be<uint32_t> is_title_process;
        xe::be<DeviceType> device_type_ptr;
        xe::be<uint32_t> overlapped_ptr;
      }* data = reinterpret_cast<XContentQueryVolumeDeviceType*>(buffer);
      assert_true(buffer_length == sizeof(XContentQueryVolumeDeviceType));

      std::string target;
      if (!kernel_state_->file_system()->FindSymbolicLink(
              std::string(data->root_name) + ':', target)) {
        return X_E_INVALIDARG;
      }

      // Only apply this check to XContent packages
      if (!target.starts_with("\\Device\\Package_")) {
        return X_E_INVALIDARG;
      }

      xe::be<DeviceType>* device_type_ptr =
          memory_->TranslateVirtual<xe::be<DeviceType>*>(
              static_cast<uint32_t>(data->device_type_ptr.get()));

      switch (kernel_state_->deployment_type_) {
        case XDeploymentType::kDownload:
        case XDeploymentType::kInstalledToHDD: {
          *device_type_ptr = DeviceType::HDD;
        } break;
        case XDeploymentType::kOpticalDisc: {
          *device_type_ptr = DeviceType::ODD;
        } break;
        default: {
          *device_type_ptr = DeviceType::Invalid;
        } break;
      }

      XELOGD("XContentQueryVolumeDeviceType('{}', {:08X}, {:08X}, {:08X})",
             data->root_name,
             static_cast<uint32_t>(data->is_title_process.get()),
             static_cast<uint32_t>(data->device_type_ptr.get()),
             static_cast<uint32_t>(data->overlapped_ptr.get()));

      return X_E_SUCCESS;
    }
    case 0x00021012: {
      uint32_t enabled = xe::load_and_swap<uint32_t>(buffer);
      XELOGD("XEnableGuestSignin: {}", enabled ? "true" : "false");
      return X_E_SUCCESS;
    }
    case 0x00022005: {
      struct XTITLE_GET_DEPLOYMENT_TYPE {
        xe::be<uint32_t> deployment_type_ptr;
        xe::be<uint32_t> overlapped_ptr;
      }* data = reinterpret_cast<XTITLE_GET_DEPLOYMENT_TYPE*>(buffer);
      assert_true(!buffer_length ||
                  buffer_length == sizeof(XTITLE_GET_DEPLOYMENT_TYPE));
      auto deployment_type =
          memory_->TranslateVirtual<uint32_t*>(data->deployment_type_ptr);
      *deployment_type = static_cast<uint32_t>(kernel_state_->deployment_type_);
      XELOGD("XTitleGetDeploymentType({:08X}, {:08X}",
             data->deployment_type_ptr.get(), data->overlapped_ptr.get());
      return X_E_SUCCESS;
    }
    case 0x0002B001: {
      // Retail NUI sends this four-byte camera lifecycle notification before
      // starting the PsCam channels. The payload is title-owned; XAM only
      // acknowledges delivery.
      if (!buffer || (buffer_length && buffer_length < sizeof(uint32_t))) {
        return X_E_INVALIDARG;
      }
      const uint32_t value = xe::load_and_swap<uint32_t>(buffer);
      XELOGI("XamNuiCameraMessage(2B001, value={:08X})", value);
      return X_E_SUCCESS;
    }
    case 0x0002B003: {
      // Kinect titles use this as the HUD/NUI initialization handshake.
      // Keep it generic: the guest owns the payload, while XAM enters the
      // same NUI system path used by the camera and skeleton messages.
      struct {
        xe::be<uint64_t> unk1;
        xe::be<uint64_t> unk2;
        xe::be<uint64_t> unk3;
      }* args = memory_->TranslateVirtual<decltype(args)>(buffer_ptr);
      if (!args || (buffer_length && buffer_length < sizeof(*args))) {
        return X_E_INVALIDARG;
      }

      InitializeWindowsNuiGuestPathForMessage();
      WindowsNuiSetHudInitializeState(args->unk1.get(), args->unk2.get());
      XELOGI("XamNuiInitializeMessage(2B003): args={:016X} {:016X} {:016X}",
             args->unk1.get(), args->unk2.get(), args->unk3.get());
      return X_E_SUCCESS;
    }
    case 0x0002B002: {
      // The title sends a 2784-byte NUI skeleton notification here. XAM owns
      // the notification, but the title retains the payload and only consumes
      // the HRESULT. The six 448-byte skeleton records begin at offset 48.
      constexpr uint32_t kSkeletonOffset = 48;
      constexpr uint32_t kSkeletonStride = 448;
      constexpr uint32_t kSkeletonCount = 6;
      constexpr uint32_t kTimestampOffset = 2736;
      if (!buffer) {
        return X_E_INVALIDARG;
      }

      const bool host_skeleton_injected =
          WindowsNuiInjectSkeletonNotification(
              buffer, buffer_length ? buffer_length : 2784);

      uint32_t tracked_count = 0;
      uint32_t position_only_count = 0;
      uint32_t tracking_states[kSkeletonCount] = {};
      for (uint32_t i = 0; i < kSkeletonCount; ++i) {
        tracking_states[i] = xe::load_and_swap<uint32_t>(
            buffer + kSkeletonOffset + i * kSkeletonStride);
        tracked_count += tracking_states[i] == 2;
        position_only_count += tracking_states[i] == 1;
      }

      const uint32_t message_count = ++nui_skeleton_message_count_;
      if (message_count <= 4 || (message_count & 0x7F) == 0 ||
          tracked_count != nui_skeleton_last_tracked_count_) {
        const uint64_t protocol = xe::load_and_swap<uint64_t>(buffer);
        const uint64_t message_version =
            xe::load_and_swap<uint64_t>(buffer + 8);
        const uint64_t timestamp =
            xe::load_and_swap<uint64_t>(buffer + kTimestampOffset);
        XELOGI(
            "XamNuiSkeletonMessage(2B002): count={} protocol={} "
            "version={:04X} timestamp={} tracked={} position_only={} "
            "host_injected={} "
            "states=[{},{},{},{},{},{}]",
            message_count, protocol, message_version, timestamp, tracked_count,
            position_only_count, host_skeleton_injected, tracking_states[0],
            tracking_states[1], tracking_states[2], tracking_states[3],
            tracking_states[4], tracking_states[5]);
        nui_skeleton_last_tracked_count_ = tracked_count;
      }
      return X_E_SUCCESS;
    }
    case 0x00021028: {
      XELOGD("XamNuiSystemProcessCall(21028, {:08X}, {:08X})",
             buffer_ptr, buffer_length);
      if (buffer && buffer_length >= 12) {
        // This payload is a trio of guest pointers to NUI request/result
        // structures. Clearing the 12-byte envelope destroys the pointers
        // before the Kinect path can service the request.
        auto pointers = memory_->TranslateVirtual<xe::be<uint32_t>*>(
            buffer_ptr);
        XELOGD("XamNuiSystemProcessCall: pointers {:08X} {:08X} {:08X}",
               pointers[0].get(), pointers[1].get(), pointers[2].get());
      }
      return X_E_SUCCESS;
    }
    case 0x0002C00C: {
      // Identity messages are sent with a zero declared payload, but the
      // pointer refers to a title-owned recipe envelope. The recipe strings
      // are consumed by the retail NuiIdentity binary; XAM must preserve the
      // caller-owned buffers and must not manufacture a completion.
      static std::atomic<uint32_t> message_count{0};
      const uint32_t count = ++message_count;
      if (!buffer) {
        return X_E_INVALIDARG;
      }
      if (count <= 16 || (count & 0x7F) == 0) {
        constexpr size_t kEnvelopeWords = 8;
        std::array<uint32_t, kEnvelopeWords> words = {};
        std::string word_dump;
        for (size_t index = 0; index < words.size(); ++index) {
          words[index] = xe::load_and_swap<uint32_t>(buffer + index * 4);
          word_dump += fmt::format("{:08X}{}", words[index],
                                   index + 1 == words.size() ? "" : " ");
        }
        XELOGI("XamNuiIdentityMessage(2C00C): count={} buffer={:08X} "
               "words=[{}]",
               count, buffer_ptr, word_dump);
      }
      return X_E_SUCCESS;
    }
    case 0x0002A005:
    case 0x0002A006:
    case 0x0002A007:
    case 0x0002A008:
    case 0x0002A009: {
      // NUI clients register five independent system queues. Their payloads
      // contain title-owned callback and result storage, so retain the
      // registration and publish the system state transition without writing
      // into opaque guest fields.
      static std::atomic<uint32_t> queue_trace_count{0};
      const uint32_t trace_count = ++queue_trace_count;
      if (!buffer || buffer_length < 24) {
        XELOGW("XamAuxiliaryQueueMessage({:08X}) invalid buffer={:08X} "
               "length={}",
               message, buffer_ptr, buffer_length);
        return X_E_INVALIDARG;
      }
      const auto* words = reinterpret_cast<const xe::be<uint32_t>*>(buffer);
      const size_t queue_index = message - 0x0002A005;
      auto& registration = nui_auxiliary_queues_[queue_index];
      registration.message = message;
      for (size_t index = 0; index < registration.words.size(); ++index) {
        registration.words[index] = static_cast<uint32_t>(words[index]);
      }
      registration.generation = ++nui_auxiliary_generation_;
      if (trace_count <= 16) {
        XELOGI("XamAuxiliaryQueueMessage({:08X}): buffer={:08X} length={} "
               "words=[{:08X} {:08X} {:08X} {:08X} {:08X} {:08X}]",
               message, buffer_ptr, buffer_length,
               static_cast<uint32_t>(words[0]),
               static_cast<uint32_t>(words[1]),
               static_cast<uint32_t>(words[2]),
               static_cast<uint32_t>(words[3]),
               static_cast<uint32_t>(words[4]),
               static_cast<uint32_t>(words[5]));
      }
      WindowsNuiNotifyAuxiliaryQueueRegistered(message);
      return X_E_SUCCESS;
    }
    case 0x0002B004: {
      XELOGI("XamNuiCameraMessage(2B004)");
      return X_E_SUCCESS;
    }
    case 0x0002C009: {
      if (!buffer) {
        return X_E_INVALIDARG;
      }
      XELOGI("XamNuiIdentityMessage(2C009): buffer={:08X} words={:08X} "
             "{:08X} {:08X} {:08X}",
             buffer_ptr, xe::load_and_swap<uint32_t>(buffer + 0),
             xe::load_and_swap<uint32_t>(buffer + 4),
             xe::load_and_swap<uint32_t>(buffer + 8),
             xe::load_and_swap<uint32_t>(buffer + 12));
      return X_E_SUCCESS;
    }
    // Causes dashboard to correctly process language/region change. It does not
    // contain any buffer.
    case 0x8000000D: {
      const bool is_pc_enabled =
          (kernel_state_->xconfig()->ReadSetting<uint8_t>(
               XCONFIG_USER_CATEGORY, XCONFIG_USER_PC_FLAGS) &
           X_PC_FLAGS::PCEnabled) != 0;

      return is_pc_enabled ? X_E_ACCESS_DENIED : X_E_SUCCESS;
    }
  }
  XELOGE(
      "Unimplemented XAM message app={:08X}, msg={:08X}, arg1={:08X}, "
      "arg2={:08X}",
      app_id(), message, buffer_ptr, buffer_length);
  return X_E_FAIL;
}

}  // namespace apps
}  // namespace xam
}  // namespace kernel
}  // namespace xe
