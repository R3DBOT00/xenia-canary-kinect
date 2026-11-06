/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2025 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/xam/xam_avatar.h"

#include <atomic>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <vector>

#include "xenia/base/logging.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/xam_private.h"

DEFINE_bool(allow_avatar_initialization, true,
            "Enable Xbox 360 avatar subsystem initialization",
            "Kernel");
DEFINE_string(avatar_asset_dump_path, "",
              "Directory where custom avatar STRB payloads are captured",
              "Kernel");

namespace xe {
namespace kernel {
namespace xam {

namespace {

// These are title-provided avatar resources, not files manufactured by Xenia.
// XamAvatarGetAssets will consume this cache once its result-buffer hierarchy is
// implemented; retaining the source payloads is necessary for that ABI work.
std::mutex avatar_custom_assets_mutex;
std::vector<std::vector<uint8_t>> avatar_custom_assets;

bool IsAvatarStrbPayload(const uint8_t* data, size_t size) {
  // STRB has a fixed 27-byte header, padded to the declared block alignment.
  // Rejecting malformed input here prevents an arbitrary guest buffer from
  // becoming a persistent avatar resource.
  if (!data || size < 27 || std::memcmp(data, "STRB", 4) != 0) {
    return false;
  }

  const uint8_t id_size = data[22];
  const uint8_t span_size = data[23];
  const uint8_t alignment = data[4] ? data[26] : 1;
  return id_size != 0 && id_size <= 8 && span_size != 0 && span_size <= 4 &&
         alignment != 0 && (alignment & (alignment - 1)) == 0;
}

void ClearAvatarCustomAssets() {
  std::lock_guard<std::mutex> lock(avatar_custom_assets_mutex);
  avatar_custom_assets.clear();
}

}  // namespace

// Start/End
dword_result_t XamAvatarInitialize_entry(
    dword_t coordinate_system,  // 1, 2, 4, etc
    dword_t unk2,               // 0 or 1
    dword_t processor_number,   // for thread creation?
    lpdword_t function_ptrs,    // 20b, 5 pointers
    lpdword_t unk5,             // data segment ptr
    dword_t unk6                // flags - 0x00300000, 0x30, etc
) {
  ClearAvatarCustomAssets();
  if (kernel_state()->title_id() == kAvatarEditorID) {
    return X_STATUS_SUCCESS;
  }

  return cvars::allow_avatar_initialization ? X_STATUS_SUCCESS : ~0u;
}
DECLARE_XAM_EXPORT1(XamAvatarInitialize, kAvatars, kStub);

void XamAvatarShutdown_entry() {
  // Calls XMsgStartIORequestEx(0xf3,0x600002,0,0,0,0).
  // in 12611 its XamUnloadSysApp(0xf2,1)
  ClearAvatarCustomAssets();
}
DECLARE_XAM_EXPORT1(XamAvatarShutdown, kAvatars, kStub);

// Get & Set
dword_result_t XamAvatarGetManifestLocalUser_entry(
    dword_t user_index, pointer_t<X_AVATAR_METADATA> avatar_metadata_ptr,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  auto run = [=](uint32_t& extended_error, uint32_t& length) {
    extended_error = X_ERROR_SUCCESS;
    length = 0;

    if (user_index >= XUserMaxUserCount) {
      extended_error = X_E_INVALIDARG;
      return X_ERROR_INVALID_PARAMETER;
    }

    if (!avatar_metadata_ptr) {
      extended_error = X_E_INVALIDARG;
      return X_ERROR_INVALID_PARAMETER;
    }

    const auto user_profile =
        kernel_state()->xam_state()->GetUserProfile(user_index);

    if (!user_profile) {
      extended_error = X_E_NO_SUCH_USER;
      return X_ERROR_FUNCTION_FAILED;
    }

    const uint32_t avatar_info_id =
        static_cast<uint32_t>(UserSettingId::XPROFILE_GAMERCARD_AVATAR_INFO_1);

    X_USER_PROFILE_SETTING avatar_info_setting_data = {};

    avatar_info_setting_data.user_index = static_cast<uint32_t>(user_index);
    avatar_info_setting_data.setting_id = avatar_info_id;
    avatar_info_setting_data.data.type = X_USER_DATA_TYPE::BINARY;
    avatar_info_setting_data.data.data.binary.size = kMaxUserDataSize;
    avatar_info_setting_data.data.data.binary.ptr = 0;

    uint32_t avatar_metadata_address = avatar_metadata_ptr.guest_address();

    const bool has_avatar_info_setting =
        kernel_state()->xam_state()->user_tracker()->GetUserSetting(
            user_profile->xuid(), kDashboardID, avatar_info_id,
            &avatar_info_setting_data, avatar_metadata_address);

    // Avatar metadata is opaque. A zero-filled blob is not a valid fallback:
    // callers are allowed to pass a successful result directly to the asset
    // loader, which makes an invented manifest unsafe.
    if (!has_avatar_info_setting ||
        !avatar_info_setting_data.data.data.binary.ptr) {
      extended_error = X_E_NO_SUCH_USER;
      return X_ERROR_FUNCTION_FAILED;
    }

    return X_ERROR_SUCCESS;
  };

  if (!overlapped_ptr) {
    uint32_t extended_error, length;
    X_RESULT result = run(extended_error, length);

    return result == X_ERROR_SUCCESS ? result : extended_error;
  }

  kernel_state()->CompleteOverlappedDeferredEx(run, overlapped_ptr);
  return X_ERROR_IO_PENDING;
}
DECLARE_XAM_EXPORT1(XamAvatarGetManifestLocalUser, kAvatars, kStub);

dword_result_t XamAvatarGetManifestsByXuid_entry(
    dword_t user_index, dword_t xuid_count, lpqword_t xuid, dword_t unk,
    dword_t avatar_info_ptr, pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  // set unk4 to 0 or 0x80990001 after

  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_ERROR_SUCCESS);
    return X_ERROR_IO_PENDING;
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarGetManifestsByXuid, kAvatars, kStub)

dword_result_t XamAvatarGetAssetsResultSize_entry(
    dword_t avatar_component_mask, lpdword_t result_buffer_size_ptr,
    lpdword_t gpu_resource_buffer_size_ptr) {
  XELOGI(
      "XamAvatarGetAssetsResultSize: mask={:08X} result_size_ptr={:08X} "
      "gpu_size_ptr={:08X}",
      static_cast<uint32_t>(avatar_component_mask),
      result_buffer_size_ptr.guest_address(),
      gpu_resource_buffer_size_ptr.guest_address());
  if (result_buffer_size_ptr) {
    *result_buffer_size_ptr = 0;
  }
  if (gpu_resource_buffer_size_ptr) {
    *gpu_resource_buffer_size_ptr = 0;
  }

  // The result buffer is a hierarchy containing pointers into the GPU resource
  // buffer. We do not have a serializer for that hierarchy yet, so a success
  // status with zero-sized buffers is invalid: titles allocate a page and then
  // dereference a result that was never produced. Return the documented failure
  // path until real assets can be serialized from the installed package data.
  XELOGW("XamAvatarGetAssetsResultSize: avatar asset hierarchy unavailable "
         "for mask={:08X}",
         static_cast<uint32_t>(avatar_component_mask));
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamAvatarGetAssetsResultSize, kAvatars, kStub);

dword_result_t XamAvatarGetAssets_entry(
    pointer_t<X_AVATAR_METADATA> avatar_metadata_ptr,
    dword_t avatar_component_mask, dword_t flags, lpdword_t result_buffer_ptr,
    lpdword_t gpu_resource_buffer_ptr,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  XELOGI(
      "XamAvatarGetAssets: metadata={:08X} mask={:08X} flags={:08X} "
      "result={:08X} gpu={:08X} overlapped={:08X}",
      avatar_metadata_ptr.guest_address(),
      static_cast<uint32_t>(avatar_component_mask), static_cast<uint32_t>(flags),
      result_buffer_ptr.guest_address(),
      gpu_resource_buffer_ptr.guest_address(), overlapped_ptr.guest_address());
  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediateEx(
        overlapped_ptr, X_ERROR_FUNCTION_FAILED, X_E_FAIL, 0);
    return X_ERROR_IO_PENDING;
  }

  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamAvatarGetAssets, kAvatars, kStub);

dword_result_t XamAvatarSetCustomAsset_entry(
    dword_t buffer_size, lpdword_t asset_data_ptr, dword_t custom_color_count,
    lpdword_t custom_colors_ptr,
    pointer_t<X_AVATAR_METADATA> avatar_metadata_ptr) {
  if (!asset_data_ptr || !buffer_size) {
    return X_E_INVALIDARG;
  }

  const auto* asset_data = kernel_state()->memory()->TranslateVirtual(
      asset_data_ptr.guest_address());
  if (!IsAvatarStrbPayload(asset_data, static_cast<uint32_t>(buffer_size))) {
    XELOGW("XamAvatarSetCustomAsset: rejected malformed STRB payload size={}",
           static_cast<uint32_t>(buffer_size));
    return X_E_INVALIDARG;
  }

  {
    std::lock_guard<std::mutex> lock(avatar_custom_assets_mutex);
    avatar_custom_assets.emplace_back(asset_data,
                                      asset_data + static_cast<uint32_t>(buffer_size));
  }

  if (!cvars::avatar_asset_dump_path.empty()) {
    static std::atomic<uint32_t> dump_sequence = 0;
    std::error_code error;
    const std::filesystem::path dump_directory =
        cvars::avatar_asset_dump_path;
    std::filesystem::create_directories(dump_directory, error);

    const uint32_t sequence = dump_sequence.fetch_add(1);
    const auto dump_path = dump_directory /
                           fmt::format("{:08X}-{:04}-{:08X}.strb",
                                       kernel_state()->title_id(), sequence,
                                       static_cast<uint32_t>(buffer_size));
    std::ofstream dump(dump_path, std::ios::binary | std::ios::trunc);
    if (dump) {
      dump.write(reinterpret_cast<const char*>(asset_data),
                 static_cast<std::streamsize>(buffer_size));
      XELOGI("XamAvatarSetCustomAsset: captured {} bytes to {}",
             static_cast<uint32_t>(buffer_size), dump_path.string());
    } else {
      XELOGE("XamAvatarSetCustomAsset: failed to create {}",
             dump_path.string());
    }
  }
  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarSetCustomAsset, kAvatars, kStub)

dword_result_t XamAvatarSetManifest_entry(
    dword_t user_index, dword_t avatar_info_ptr,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  auto run = [=](uint32_t& extended_error, uint32_t& length) {
    extended_error = X_ERROR_SUCCESS;
    length = 0;
    // Update and save settings.
    const auto& user_profile =
        kernel_state()->xam_state()->GetUserProfile(user_index);

    // Skip writing data about users with id != 0 they're not supported
    if (!user_profile) {
      extended_error = X_E_NO_SUCH_USER;
      return X_ERROR_FUNCTION_FAILED;
    }

    const uint32_t avatar_info_id =
        static_cast<uint32_t>(UserSettingId::XPROFILE_GAMERCARD_AVATAR_INFO_1);

    X_USER_PROFILE_SETTING setting_data;
    setting_data.user_index = static_cast<uint32_t>(user_index);
    setting_data.setting_id = avatar_info_id;
    setting_data.data.type = X_USER_DATA_TYPE::BINARY;
    setting_data.data.data.binary.size = kMaxUserDataSize;
    setting_data.data.data.binary.ptr = static_cast<uint32_t>(avatar_info_ptr);

    const UserSetting setting = UserSetting(&setting_data);

    kernel_state()->xam_state()->user_tracker()->UpsertSetting(
        user_profile->xuid(), kDashboardID, &setting);

    return X_STATUS_SUCCESS;
  };

  if (!overlapped_ptr) {
    uint32_t extended_error, length;
    X_RESULT result = run(extended_error, length);

    return result == X_ERROR_SUCCESS ? result : extended_error;
  }

  kernel_state()->CompleteOverlappedDeferredEx(run, overlapped_ptr);
  return X_ERROR_IO_PENDING;
}
DECLARE_XAM_EXPORT1(XamAvatarSetManifest, kAvatars, kStub);

dword_result_t XamAvatarGetMetadataRandom_entry(
    dword_t body_type, dword_t avatars_count,
    pointer_t<X_AVATAR_METADATA> avatar_metadata_ptr,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  if (!avatars_count) {
    return X_STATUS_SUCCESS;
  }
  if (!avatar_metadata_ptr) {
    return X_E_INVALIDARG;
  }

  // Generating the opaque retail metadata format has not been implemented.
  // Do not claim success while leaving an array of invalid manifests behind.
  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_E_FAIL);
    return X_ERROR_IO_PENDING;
  }
  return X_E_FAIL;
}
DECLARE_XAM_EXPORT1(XamAvatarGetMetadataRandom, kAvatars, kStub);

dword_result_t XamAvatarGetMetadataSignedOutProfileCount_entry(
    lpdword_t profile_count_ptr, pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  if (!profile_count_ptr) {
    return X_E_INVALIDARG;
  }
  // Xenia currently has no persisted signed-out local-profile roster.
  *profile_count_ptr = 0;
  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_ERROR_SUCCESS);
    return X_ERROR_IO_PENDING;
  }
  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarGetMetadataSignedOutProfileCount, kAvatars, kStub);

dword_result_t XamAvatarGetMetadataSignedOutProfile_entry(
    dword_t profile_index, pointer_t<X_AVATAR_METADATA> avatar_metadata_ptr,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  if (!avatar_metadata_ptr) {
    return X_E_INVALIDARG;
  }
  // The count export reports zero, so every index is out of range. Returning
  // success here previously exposed untouched guest memory as avatar metadata.
  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_E_NO_SUCH_USER);
    return X_ERROR_IO_PENDING;
  }
  return X_E_NO_SUCH_USER;
}
DECLARE_XAM_EXPORT1(XamAvatarGetMetadataSignedOutProfile, kAvatars, kStub);

dword_result_t XamAvatarManifestGetBodyType_entry(
    pointer_t<X_AVATAR_METADATA> avatar_metadata_ptr) {
  return static_cast<uint8_t>(X_AVATAR_BODY_TYPE::Male);
}
DECLARE_XAM_EXPORT1(XamAvatarManifestGetBodyType, kAvatars, kStub);

dword_result_t XamAvatarGetInstrumentation_entry(qword_t unk1, lpdword_t unk2) {
  /* Notes:
     - unk1 not used?
     - unk1 recieves values of 1, 2, and 6
     - mark implemented once confirmed first param not used and params named
  */
  if (unk2) {
    *unk2 = 0;
  }
  return 1;
}
DECLARE_XAM_EXPORT1(XamAvatarGetInstrumentation, kAvatars, kStub);

dword_result_t XamAvatarGetAssetIcon_entry(
    lpqword_t unk1, dword_t unk2, lpqword_t unk3, lpqword_t unk4,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_ERROR_SUCCESS);
    return X_ERROR_IO_PENDING;
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarGetAssetIcon, kAvatars, kStub);

dword_result_t XamAvatarGetAssetBinary_entry(
    lpvoid_t asset_metadata, dword_t unk2, dword_t unk3, dword_t unk4,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_ERROR_SUCCESS);
    return X_ERROR_IO_PENDING;
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarGetAssetBinary, kAvatars, kStub);

void XamAvatarGetInstalledAssetPackageDescription_entry(
    pointer_t<X_ASSET_ID> asset_id_ptr,
    pointer_t<XCONTENT_AGGREGATE_DATA>
        content_data_ptr  // pointer_t<XCONTENT_DATA_INTERNAL>
) {
  XCONTENT_AGGREGATE_DATA content_data;
  content_data.content_type = XContentType::kAvatarItem;
  content_data.xuid = 0;
  content_data.title_id = asset_id_ptr->title_id;
  std::string file_name =
      fmt::format("{:016X}{:08X}{:08X}", asset_id_ptr->data,
                  asset_id_ptr->data2, asset_id_ptr->title_id);
  content_data.set_file_name(file_name);
  *content_data_ptr = content_data;
  XELOGD("Looking for avatar asset: {:X}", file_name);
}
DECLARE_XAM_EXPORT1(XamAvatarGetInstalledAssetPackageDescription, kAvatars,
                    kSketchy);

void XamAvatarSetMocks_entry() {
  // No-op.
}
DECLARE_XAM_EXPORT1(XamAvatarSetMocks, kAvatars, kStub);

// Animation
const static std::map<uint64_t, std::string> XAnimationTypeMap = {
    // Animation Generic Stand
    {0x0040000000030003, "Animation Generic Stand 0"},
    {0x0040000000040003, "Animation Generic Stand 1"},
    {0x0040000000050003, "Animation Generic Stand 2"},
    {0x0040000000270003, "Animation Generic Stand 3"},
    {0x0040000000280003, "Animation Generic Stand 4"},
    {0x0040000000290003, "Animation Generic Stand 5"},
    {0x00400000002A0003, "Animation Generic Stand 6"},
    {0x00400000002B0003, "Animation Generic Stand 7"},
    // Animation Idle
    {0x0040000000130001, "Animation Male Idle Looks Around"},
    {0x0040000000140001, "Animation Male Idle Stretch"},
    {0x0040000000150001, "Animation Male Idle Shifts Weight"},
    {0x0040000000260001, "Animation Male Idle Checks Hand"},
    {0x0040000000090002, "Animation Female Idle Check Nails"},
    {0x00400000000A0002, "Animation Female Idle Looks Around"},
    {0x00400000000B0002, "Animation Female Idle Shifts Weight"},
    {0x00400000000C0002, "Animation Female Idle Fixes Shoe"},
};

// https://github.com/xenia-canary/xenia-canary/commit/212c99eee2724de15f471148d10197d89794ff32
dword_result_t XamAvatarLoadAnimation_entry(
    lpqword_t asset_id_ptr, dword_t flags, lpvoid_t output,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  /* Notes:
      - unknown[4] & unknown[0] = 0x10000000
      - Calls XMsgStartIORequestEx(0xf3, 0x60000F, overlapped_ptr, stack1, 0x18,
     unknown) 0xf2 12611
  */
  assert_true(asset_id_ptr);

  std::string summary = "Request to load avatar animation: ";

  if (XAnimationTypeMap.find(*asset_id_ptr) != XAnimationTypeMap.cend()) {
    summary += XAnimationTypeMap.at(*asset_id_ptr);
  } else {
    summary += fmt::format("Unknown animation 0x{:016x}",
                           static_cast<uint64_t>(*asset_id_ptr));
  }

  XELOGD("{}", summary);

  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_ERROR_SUCCESS);
    return X_ERROR_IO_PENDING;
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarLoadAnimation, kAvatars, kStub);

dword_result_t XamAvatarGenerateMipMaps_entry(
    lpdword_t avatar_assets_ptr, dword_t flags, dword_t buffer_size,
    lpdword_t mip_map_buffer_ptr, pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_ERROR_SUCCESS);
    return X_ERROR_IO_PENDING;
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarGenerateMipMaps, kAvatars, kStub);

// Enum
dword_result_t XamAvatarBeginEnumAssets_entry(
    dword_t unk1, dword_t unk2, dword_t unk3, word_t unk4, dword_t unk5,
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  // unknown[4] & unknown[0] = 0x20080002
  // buffer_ptr[8]
  // XMsgStartIORequestEx(0xf3, 0x60000c, overlapped_ptr, buffer_ptr, 0x14,
  // unknown) 0xf2 12611

  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_ERROR_SUCCESS);
    return X_ERROR_IO_PENDING;
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarBeginEnumAssets, kAvatars, kStub);

dword_result_t XamAvatarEnumAssets_entry(
    lpvoid_t unk1, lpqword_t unk2, pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  // unk1 pointer to a struct of size 0x8a48
  // buffer_ptr = concat(unk1, unk2)
  // unknown = 0x20080002
  // XMsgStartIORequestEx(0xf3, 0x60000d, overlapped_ptr, &buffer_ptr, 8,
  // &unknown) 0xf2 12611

  return X_E_NO_MORE_FILES;  // Stop it from calling endlessly
}
DECLARE_XAM_EXPORT1(XamAvatarEnumAssets, kAvatars, kStub);

dword_result_t XamAvatarEndEnumAssets_entry(
    pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  // unknown[4]
  // unknown[0] = 0x20080002
  // XMsgStartIORequestEx(0xf2,0x60000e,overlapped_ptr,0,0,unknown); 12611

  if (overlapped_ptr) {
    kernel_state()->CompleteOverlappedImmediate(overlapped_ptr,
                                                X_ERROR_SUCCESS);
    return X_ERROR_IO_PENDING;
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarEndEnumAssets, kAvatars, kStub);

// Other
dword_result_t XamAvatarWearNow_entry(
    qword_t unk1, lpdword_t unk2, pointer_t<XAM_OVERLAPPED> overlapped_ptr) {
  X_RESULT result = X_ERROR_SUCCESS;
  if (kernel_state()->title_id() == kAvatarEditorID) {
    /*
      - ops
    XamSendMessageToLoadedApps(0xffffffff8000000e,0xffffffff80050018,lVar5);
    XNotifyBroadcast(0xffffffff80050018,lVar5);
    if (overlapped_ptr) {
      XMsgCompleteIORequest(overlapped_ptr,0,0,0);
    }
    */
  } else {
    // buffer_ptr = concat(unk1, *unk2);
    //  XMsgStartIORequestEx(0xf3,0x600018,overlapped_ptr,&buffer_ptr,0x14,0);
    if (overlapped_ptr) {
      kernel_state()->CompleteOverlappedImmediate(overlapped_ptr, result);
      return X_ERROR_IO_PENDING;
    }
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamAvatarWearNow, kAvatars, kStub);

dword_result_t XamAvatarReinstallAwardedAsset_entry(lpstring_t string_out_ptr,
                                                    dword_t string_size,
                                                    lpdword_t unk_ptr) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamAvatarReinstallAwardedAsset, kAvatars, kStub);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(Avatar);
