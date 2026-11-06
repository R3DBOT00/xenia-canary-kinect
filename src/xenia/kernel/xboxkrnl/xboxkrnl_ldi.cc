/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "xenia/base/byte_order.h"
#include "xenia/base/logging.h"
#include "xenia/cpu/lzx.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace xboxkrnl {

namespace {

constexpr uint32_t kLdiSignature = 0x4349444C;  // 'CIDL'
constexpr uint32_t kDefaultWindowSize = 0x8000;
constexpr uint32_t kDefaultCpuType = 1;

#pragma pack(push, 1)
struct LDI_CONTEXT_HEADER {
  xe::be<uint32_t> signature;
  xe::be<uint32_t> window_size;
  xe::be<uint32_t> cpu_type;
  xe::be<uint32_t> reserved;
};
#pragma pack(pop)
static_assert_size(LDI_CONTEXT_HEADER, 16);

struct LdiCallInfo {
  uint32_t context_ptr;
  uint32_t input_ptr;
  uint32_t input_size;
  uint32_t output_ptr;
  uint32_t output_size;
  uint32_t window_size;
  uint32_t cpu_type;
};

struct LdiContextState {
  uint32_t frame_size = kDefaultWindowSize;
  uint32_t window_size = kDefaultWindowSize;
  uint32_t scratch_ptr = 0;
  lzx_stream* stream = nullptr;
  std::mutex mutex;

  ~LdiContextState() { lzx_stream_destroy(stream); }
};

std::mutex ldi_context_mutex;
std::unordered_map<uint32_t, std::shared_ptr<LdiContextState>> ldi_contexts;
std::atomic<uint32_t> next_ldi_context{0x4C440001};

uint32_t ReadGuestU32(const ppc_context_t& ctx, uint32_t guest_ptr,
                      uint32_t fallback = 0) {
  if (!guest_ptr) {
    return fallback;
  }
  auto value = ctx.TranslateVirtual<const xe::be<uint32_t>*>(guest_ptr);
  return value ? static_cast<uint32_t>(*value) : fallback;
}

LdiCallInfo ReadDecompressCallInfo(const ppc_context_t& ctx) {
  LdiCallInfo info{};
  info.context_ptr = static_cast<uint32_t>(ctx->r[3]);
  info.input_ptr = static_cast<uint32_t>(ctx->r[4]);
  info.input_size = static_cast<uint32_t>(ctx->r[5]);
  info.output_ptr = static_cast<uint32_t>(ctx->r[6]);
  info.output_size = ReadGuestU32(ctx, static_cast<uint32_t>(ctx->r[7]));
  return info;
}

std::shared_ptr<LdiContextState> GetLdiContext(uint32_t context_ptr) {
  std::lock_guard<std::mutex> lock(ldi_context_mutex);
  auto it = ldi_contexts.find(context_ptr);
  return it != ldi_contexts.end() ? it->second : nullptr;
}

}  // namespace

dword_result_t LDICreateDecompression_entry(const ppc_context_t& ctx) {
  const uint32_t window_size_ptr = static_cast<uint32_t>(ctx->r[3]);
  const uint32_t block_size_ptr = static_cast<uint32_t>(ctx->r[4]);
  const uint32_t alloc_callback = static_cast<uint32_t>(ctx->r[5]);
  const uint32_t free_callback = static_cast<uint32_t>(ctx->r[6]);
  const uint32_t scratch_ptr = static_cast<uint32_t>(ctx->r[7]);
  const uint32_t scratch_size_ptr = static_cast<uint32_t>(ctx->r[8]);
  const uint32_t context_out_ptr = static_cast<uint32_t>(ctx->r[9]);
  uint32_t frame_size =
      ReadGuestU32(ctx, window_size_ptr, kDefaultWindowSize);
  uint32_t window_size =
      ReadGuestU32(ctx, block_size_ptr, kDefaultWindowSize);
  if (!frame_size || frame_size > 0x8000) {
    frame_size = kDefaultWindowSize;
  }
  if (window_size < 0x8000 || window_size > 0x200000 ||
      (window_size & (window_size - 1))) {
    window_size = kDefaultWindowSize;
  }

  auto context_out =
      ctx.TranslateVirtual<xe::be<uint32_t>*>(context_out_ptr);
  if (!context_out) {
    return X_STATUS_INVALID_PARAMETER;
  }

  uint32_t context_handle = scratch_ptr;
  if (!context_handle) {
    context_handle = next_ldi_context.fetch_add(1, std::memory_order_relaxed);
  }

  auto state = std::make_shared<LdiContextState>();
  state->frame_size = frame_size;
  state->window_size = window_size;
  state->scratch_ptr = scratch_ptr;
  state->stream = lzx_stream_create(window_size);
  if (!state->stream) {
    return X_STATUS_UNSUCCESSFUL;
  }
  {
    std::lock_guard<std::mutex> lock(ldi_context_mutex);
    ldi_contexts[context_handle] = std::move(state);
  }
  *context_out = context_handle;

  if (scratch_ptr) {
    auto header = ctx.TranslateVirtual<LDI_CONTEXT_HEADER*>(scratch_ptr);
    if (header) {
      header->signature = kLdiSignature;
      header->window_size = window_size;
      header->cpu_type = kDefaultCpuType;
      header->reserved = 0;
    }
  }

  XELOGI(
      "LDICreateDecompression: frame={:08X} window={:08X} alloc={:08X} "
      "free={:08X} scratch={:08X} scratch_size={:08X} out={:08X} "
      "context={:08X}",
      frame_size, window_size, alloc_callback, free_callback, scratch_ptr,
      ReadGuestU32(ctx, scratch_size_ptr), context_out_ptr, context_handle);

  return X_STATUS_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(LDICreateDecompression, kNone, kImplemented);

dword_result_t LDIDecompress_entry(const ppc_context_t& ctx) {
  const LdiCallInfo info = ReadDecompressCallInfo(ctx);
  const uint32_t output_size_ptr = static_cast<uint32_t>(ctx->r[7]);
  auto state = GetLdiContext(info.context_ptr);
  if (!state) {
    return X_STATUS_INVALID_PARAMETER;
  }

  XELOGI(
      "LDIDecompress: ctx={:08X} in={:08X}({:08X}) out={:08X}({:08X}) "
      "out_size_ptr={:08X} window={:08X}",
      info.context_ptr, info.input_ptr, info.input_size, info.output_ptr,
      info.output_size, output_size_ptr, state->window_size);

  if (!info.input_ptr || !info.output_ptr || !info.input_size ||
      !info.output_size) {
    return X_STATUS_INVALID_PARAMETER;
  }

  const uint8_t* input = ctx.TranslateVirtual<const uint8_t*>(info.input_ptr);
  uint8_t* output = ctx.TranslateVirtual<uint8_t*>(info.output_ptr);
  if (!input || !output) {
    return X_STATUS_INVALID_PARAMETER;
  }

  std::lock_guard<std::mutex> state_lock(state->mutex);
  const int result = lzx_stream_decompress(
      state->stream, input, info.input_size, output, info.output_size);
  if (result != 0) {
    XELOGE("LDIDecompress: LZX failed for context {:08X} with code {}",
           info.context_ptr, result);
    auto output_size =
        ctx.TranslateVirtual<xe::be<uint32_t>*>(output_size_ptr);
    if (output_size) {
      *output_size = 0;
    }
    return X_STATUS_UNSUCCESSFUL;
  }

  XELOGI("LDIDecompress: success context={:08X} produced={:08X}",
         info.context_ptr, info.output_size);
  return X_STATUS_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(LDIDecompress, kNone, kImplemented);

dword_result_t LDIDestroyDecompression_entry(const ppc_context_t& ctx) {
  const uint32_t context_ptr = static_cast<uint32_t>(ctx->r[3]);
  XELOGI("LDIDestroyDecompression: ctx={:08X}", context_ptr);

  std::shared_ptr<LdiContextState> state;
  {
    std::lock_guard<std::mutex> lock(ldi_context_mutex);
    auto it = ldi_contexts.find(context_ptr);
    if (it != ldi_contexts.end()) {
      state = std::move(it->second);
      ldi_contexts.erase(it);
    }
  }

  const uint32_t scratch_ptr = state ? state->scratch_ptr : 0;
  if (scratch_ptr) {
    auto header = ctx.TranslateVirtual<LDI_CONTEXT_HEADER*>(scratch_ptr);
    if (header) {
      header->signature = 0;
      header->window_size = 0;
      header->cpu_type = 0;
      header->reserved = 0;
    }
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(LDIDestroyDecompression, kNone, kImplemented);

dword_result_t LDIResetDecompression_entry(const ppc_context_t& ctx) {
  const uint32_t context_ptr = static_cast<uint32_t>(ctx->r[3]);
  XELOGI("LDIResetDecompression: ctx={:08X}", context_ptr);

  auto state = GetLdiContext(context_ptr);
  if (!state) {
    return X_STATUS_INVALID_PARAMETER;
  }
  std::lock_guard<std::mutex> state_lock(state->mutex);
  lzx_stream_destroy(state->stream);
  state->stream = lzx_stream_create(state->window_size);
  if (!state->stream) {
    return X_STATUS_UNSUCCESSFUL;
  }

  return X_STATUS_SUCCESS;
}
DECLARE_XBOXKRNL_EXPORT1(LDIResetDecompression, kNone, kImplemented);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(Ldi);
