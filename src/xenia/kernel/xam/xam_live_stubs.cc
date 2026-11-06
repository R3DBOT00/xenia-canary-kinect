/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2026 Xenia Canary. All rights reserved.                          *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xam/xam_private.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace xam {

namespace {

constexpr uint32_t kXLiveBaseAppId = 0xFC;
constexpr uint32_t kXLiveBaseLogonGetHrMessage = 0x00058003;

uint32_t GetLiveBaseLogonHr() {
  return kernel_state()->app_manager()->DispatchMessageSync(
      kXLiveBaseAppId, kXLiveBaseLogonGetHrMessage, 0, 0);
}

}  // namespace

dword_result_t NetDll_XHttpStartup_entry(dword_t caller, dword_t version) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpStartup, kNetworking, kImplemented);

dword_result_t NetDll_XHttpShutdown_entry(dword_t caller) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpShutdown, kNetworking, kImplemented);

dword_result_t NetDll_XHttpOpen_entry(dword_t caller, dword_t unk0,
                                      dword_t unk1) {
  return 1;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpOpen, kNetworking, kStub);

dword_result_t NetDll_XHttpCloseHandle_entry(dword_t caller, dword_t handle) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpCloseHandle, kNetworking, kImplemented);

dword_result_t NetDll_XHttpConnect_entry(dword_t caller, dword_t handle,
                                         dword_t unk0, dword_t unk1) {
  return 1;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpConnect, kNetworking, kStub);

dword_result_t NetDll_XHttpSetStatusCallback_entry(dword_t caller,
                                                   dword_t handle,
                                                   dword_t callback,
                                                   dword_t unk0) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpSetStatusCallback, kNetworking, kImplemented);

dword_result_t NetDll_XHttpOpenRequest_entry(dword_t caller, dword_t handle,
                                             dword_t unk0, dword_t unk1,
                                             dword_t unk2, dword_t unk3,
                                             dword_t unk4) {
  return 1;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpOpenRequest, kNetworking, kStub);

dword_result_t NetDll_XHttpOpenRequestUsingMemory_entry(
    dword_t caller, dword_t handle, dword_t unk0, dword_t unk1, dword_t unk2,
    dword_t unk3, dword_t unk4, dword_t unk5) {
  return 1;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpOpenRequestUsingMemory, kNetworking, kStub);

dword_result_t NetDll_XHttpSendRequest_entry(dword_t caller, dword_t handle,
                                             dword_t unk0, dword_t unk1,
                                             dword_t unk2, dword_t unk3,
                                             dword_t unk4, dword_t unk5) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpSendRequest, kNetworking, kImplemented);

dword_result_t NetDll_XHttpReceiveResponse_entry(dword_t caller,
                                                 dword_t handle,
                                                 dword_t unk0) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpReceiveResponse, kNetworking, kImplemented);

dword_result_t NetDll_XHttpQueryHeaders_entry(dword_t caller, dword_t handle,
                                             dword_t query_index,
                                             dword_t query_type,
                                             dword_t buffer_ptr,
                                             dword_t buffer_length,
                                             dword_t bytes_written_ptr,
                                             dword_t unk0) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpQueryHeaders, kNetworking, kImplemented);

dword_result_t NetDll_XHttpReadData_entry(dword_t caller, dword_t handle,
                                          dword_t buffer_ptr,
                                          dword_t buffer_length,
                                          lpdword_t bytes_read_ptr) {
  if (bytes_read_ptr) {
    *bytes_read_ptr = 0;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpReadData, kNetworking, kImplemented);

dword_result_t NetDll_XHttpWriteData_entry(dword_t caller, dword_t handle,
                                           dword_t buffer_ptr,
                                           dword_t buffer_length,
                                           lpdword_t bytes_written_ptr) {
  if (bytes_written_ptr) {
    *bytes_written_ptr = 0;
  }
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpWriteData, kNetworking, kImplemented);

dword_result_t NetDll_XHttpQueryOption_entry(dword_t caller, dword_t handle,
                                             dword_t option, dword_t buffer_ptr,
                                             dword_t buffer_length) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpQueryOption, kNetworking, kImplemented);

dword_result_t NetDll_XHttpSetOption_entry(dword_t caller, dword_t handle,
                                           dword_t option, dword_t buffer_ptr,
                                           dword_t buffer_length) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpSetOption, kNetworking, kImplemented);

dword_result_t NetDll_XHttpDoWork_entry(dword_t caller, dword_t handle) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpDoWork, kNetworking, kImplemented);

dword_result_t NetDll_XHttpSetCredentials_entry(dword_t caller, dword_t handle,
                                                dword_t unk0, dword_t unk1,
                                                dword_t unk2, dword_t unk3,
                                                dword_t unk4) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpSetCredentials, kNetworking, kImplemented);

dword_result_t NetDll_XHttpQueryAuthSchemes_entry(dword_t caller,
                                                  dword_t handle,
                                                  dword_t buffer_ptr,
                                                  dword_t buffer_length,
                                                  dword_t unknown_ptr) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpQueryAuthSchemes, kNetworking, kImplemented);

dword_result_t NetDll_XHttpCrackUrlW_entry(dword_t caller, dword_t unk0,
                                           dword_t unk1, dword_t unk2) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpCrackUrlW, kNetworking, kImplemented);

dword_result_t NetDll_XHttpCrackUrl_entry(dword_t caller, dword_t unk0,
                                          dword_t unk1, dword_t unk2) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpCrackUrl, kNetworking, kImplemented);

dword_result_t NetDll_XHttpCreateUrl_entry(dword_t caller, dword_t unk0,
                                           dword_t unk1, dword_t unk2) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpCreateUrl, kNetworking, kImplemented);

dword_result_t NetDll_XHttpCreateUrlW_entry(dword_t caller, dword_t unk0,
                                            dword_t unk1, dword_t unk2) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpCreateUrlW, kNetworking, kImplemented);

dword_result_t NetDll_XHttpResetPerfCounters_entry(dword_t caller,
                                                   dword_t handle) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpResetPerfCounters, kNetworking, kImplemented);

dword_result_t NetDll_XHttpGetPerfCounters_entry(dword_t caller,
                                                 dword_t handle,
                                                 dword_t buffer_ptr,
                                                 dword_t buffer_length) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(NetDll_XHttpGetPerfCounters, kNetworking, kImplemented);

dword_result_t XamGetLiveHiveValueA_entry(lpstring_t value_name,
                                          lpdword_t value_ptr) {
  auto result = GetLiveBaseLogonHr();
  if (value_ptr) {
    *value_ptr = result;
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamGetLiveHiveValueA, kNone, kImplemented);

dword_result_t XamGetLiveHiveValueW_entry(lpvoid_t value_name,
                                          lpdword_t value_ptr) {
  auto result = GetLiveBaseLogonHr();
  if (value_ptr) {
    *value_ptr = result;
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamGetLiveHiveValueW, kNone, kImplemented);

dword_result_t XamGetLiveHiveValueDuringLogonAttemptA_entry(
    lpstring_t value_name, lpdword_t value_ptr) {
  auto result = GetLiveBaseLogonHr();
  if (value_ptr) {
    *value_ptr = result;
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamGetLiveHiveValueDuringLogonAttemptA, kNone,
                    kImplemented);

dword_result_t XamGetLiveHiveValueDuringLogonAttemptExA_entry(
    lpstring_t value_name, lpdword_t value_ptr) {
  auto result = GetLiveBaseLogonHr();
  if (value_ptr) {
    *value_ptr = result;
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamGetLiveHiveValueDuringLogonAttemptExA, kNone,
                    kImplemented);

dword_result_t XamQueryLiveHiveA_entry(lpstring_t value_name,
                                       lpdword_t value_ptr) {
  auto result = GetLiveBaseLogonHr();
  if (value_ptr) {
    *value_ptr = result;
  }
  return result;
}
DECLARE_XAM_EXPORT1(XamQueryLiveHiveA, kNone, kImplemented);

dword_result_t XamLrcSetTitlePort_entry(dword_t port) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamLrcSetTitlePort, kNone, kImplemented);

dword_result_t XamLrcVerifyClientId_entry(dword_t client_id, dword_t unk0,
                                          dword_t unk1) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamLrcVerifyClientId, kNone, kImplemented);

dword_result_t XamLrcEncryptDecryptTitleMessage_entry(dword_t buffer_ptr,
                                                     dword_t buffer_length,
                                                     dword_t unk0,
                                                     dword_t unk1) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamLrcEncryptDecryptTitleMessage, kNone, kImplemented);

dword_result_t XamLrcLogSessionSummary_entry(dword_t unk0, dword_t unk1,
                                            dword_t unk2, dword_t unk3) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamLrcLogSessionSummary, kNone, kImplemented);

dword_result_t XamLrcLogError_entry(dword_t unk0, dword_t unk1, dword_t unk2,
                                    dword_t unk3) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamLrcLogError, kNone, kImplemented);

dword_result_t XamXlfsInitializeUploadQueue_entry(lpvoid_t config,
                                                  lpdword_t handle_out) {
  if (!handle_out) {
    return X_E_INVALIDARG;
  }
  *handle_out = 1;
  XELOGI("XamXlfsInitializeUploadQueue: handle=1");
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamXlfsInitializeUploadQueue, kNone, kImplemented);

dword_result_t XamXlfsUninitializeUploadQueue_entry(dword_t handle,
                                                    lpvoid_t config) {
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamXlfsUninitializeUploadQueue, kNone, kImplemented);

dword_result_t XamXlfsMountUploadQueueInstance_entry(
    dword_t handle, lpvoid_t config, pointer_t<uint8_t> mounted_out,
    lpdword_t instance_out) {
  if (mounted_out) {
    *mounted_out = 1;
  }
  if (instance_out) {
    *instance_out = 1;
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamXlfsMountUploadQueueInstance, kNone, kImplemented);

dword_result_t XamXlfsUnmountUploadQueueInstance_entry(
    dword_t handle, pointer_t<uint8_t> mounted_out, lpvoid_t config) {
  if (mounted_out) {
    *mounted_out = 0;
  }
  return X_E_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamXlfsUnmountUploadQueueInstance, kNone, kImplemented);

dword_result_t XamCacheOpenFile_entry(lpvoid_t cache_file) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamCacheOpenFile, kNone, kImplemented);

dword_result_t XamCacheCloseFile_entry(lpvoid_t cache_file, dword_t flags) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamCacheCloseFile, kNone, kImplemented);

dword_result_t XamCacheReset_entry(dword_t cache_type) {
  return X_ERROR_SUCCESS;
}
DECLARE_XAM_EXPORT1(XamCacheReset, kNone, kImplemented);

}  // namespace xam
}  // namespace kernel
}  // namespace xe

DECLARE_XAM_EMPTY_REGISTER_EXPORTS(Live);
