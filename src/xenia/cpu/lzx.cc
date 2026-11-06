/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2013 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/cpu/lzx.h"

#include <algorithm>
#include <climits>
#include <vector>

#include "xenia/base/byte_order.h"
#include "xenia/base/logging.h"
#include "xenia/base/math.h"
#include "xenia/base/memory.h"
#include "xenia/kernel/util/xex2_info.h"

#include "third_party/mspack/lzx.h"
#include "third_party/mspack/mspack.h"

typedef struct mspack_memory_file_t {
  mspack_system sys;
  void* buffer;
  off_t buffer_size;
  off_t offset;
} mspack_memory_file;

mspack_memory_file* mspack_memory_open(mspack_system* sys, void* buffer,
                                       const size_t buffer_size) {
  assert_true(buffer_size < INT_MAX);
  if (buffer_size >= INT_MAX) {
    return NULL;
  }
  auto memfile =
      (mspack_memory_file*)std::calloc(1, sizeof(mspack_memory_file));
  if (!memfile) {
    return NULL;
  }
  memfile->buffer = buffer;
  memfile->buffer_size = (off_t)buffer_size;
  memfile->offset = 0;
  return memfile;
}

void mspack_memory_close(mspack_memory_file* file) {
  auto memfile = (mspack_memory_file*)file;
  std::free(memfile);
}

int mspack_memory_read(mspack_file* file, void* buffer, int chars) {
  auto memfile = (mspack_memory_file*)file;
  const off_t remaining = memfile->buffer_size - memfile->offset;
  const off_t total = std::min(static_cast<off_t>(chars), remaining);
  std::memcpy(buffer, (uint8_t*)memfile->buffer + memfile->offset, total);
  memfile->offset += total;
  return (int)total;
}

int mspack_memory_write(mspack_file* file, void* buffer, int chars) {
  auto memfile = (mspack_memory_file*)file;
  const off_t remaining = memfile->buffer_size - memfile->offset;
  const off_t total = std::min(static_cast<off_t>(chars), remaining);
  std::memcpy((uint8_t*)memfile->buffer + memfile->offset, buffer, total);
  memfile->offset += total;
  return (int)total;
}

void* mspack_memory_alloc(mspack_system* sys, size_t chars) {
  return std::calloc(chars, 1);
}

void mspack_memory_free(void* ptr) { std::free(ptr); }

void mspack_memory_copy(void* src, void* dest, size_t chars) {
  std::memcpy(dest, src, chars);
}

mspack_system* mspack_memory_sys_create() {
  auto sys = (mspack_system*)std::calloc(1, sizeof(mspack_system));
  if (!sys) {
    return NULL;
  }
  sys->read = mspack_memory_read;
  sys->write = mspack_memory_write;
  sys->alloc = mspack_memory_alloc;
  sys->free = mspack_memory_free;
  sys->copy = mspack_memory_copy;
  return sys;
}

void mspack_memory_sys_destroy(struct mspack_system* sys) { free(sys); }

struct lzx_stream_input_file {
  std::vector<uint8_t> data;
  size_t offset = 0;
};

struct lzx_stream_output_file {
  uint8_t* data = nullptr;
  size_t size = 0;
  size_t offset = 0;
};

struct lzx_stream {
  mspack_system sys{};
  lzx_stream_input_file input;
  lzx_stream_output_file output;
  lzxd_stream* decoder = nullptr;
};

int lzx_stream_read(mspack_file* file, void* buffer, int chars) {
  auto input = reinterpret_cast<lzx_stream_input_file*>(file);
  const size_t remaining = input->data.size() - input->offset;
  const size_t total = std::min<size_t>(static_cast<size_t>(chars), remaining);
  if (total) {
    std::memcpy(buffer, input->data.data() + input->offset, total);
    input->offset += total;
  }
  return static_cast<int>(total);
}

int lzx_stream_write(mspack_file* file, void* buffer, int chars) {
  auto output = reinterpret_cast<lzx_stream_output_file*>(file);
  const size_t remaining = output->size - output->offset;
  const size_t total = std::min<size_t>(static_cast<size_t>(chars), remaining);
  if (total) {
    std::memcpy(output->data + output->offset, buffer, total);
    output->offset += total;
  }
  return static_cast<int>(total);
}

lzx_stream* lzx_stream_create(uint32_t window_size) {
  uint32_t window_bits;
  if (!xe::bit_scan_forward(window_size, &window_bits) ||
      window_size != (1u << window_bits)) {
    return nullptr;
  }

  auto stream = new lzx_stream();
  stream->sys.read = lzx_stream_read;
  stream->sys.write = lzx_stream_write;
  stream->sys.alloc = mspack_memory_alloc;
  stream->sys.free = mspack_memory_free;
  stream->sys.copy = mspack_memory_copy;
  stream->decoder = lzxd_init(
      &stream->sys, reinterpret_cast<mspack_file*>(&stream->input),
      reinterpret_cast<mspack_file*>(&stream->output), window_bits, 0, 2, 0,
      0);
  if (!stream->decoder) {
    delete stream;
    return nullptr;
  }
  return stream;
}

int lzx_stream_decompress(lzx_stream* stream, const void* lzx_data,
                          size_t lzx_len, void* dest, size_t dest_len) {
  if (!stream || !stream->decoder || !lzx_data || !lzx_len || !dest ||
      !dest_len || lzx_len >= INT_MAX || dest_len >= INT_MAX) {
    return MSPACK_ERR_ARGS;
  }

  const auto input = static_cast<const uint8_t*>(lzx_data);
  const size_t input_offset_before = stream->input.offset;
  const unsigned int frame_before = stream->decoder->frame;
  stream->input.data.insert(stream->input.data.end(), input, input + lzx_len);
  // LDI supplies compressed input one frame at a time. Allow libmspack to
  // probe the end of each supplied chunk without treating the next one as an
  // attempt to read past a permanent EOF.
  stream->decoder->input_end = 0;
  stream->output.data = static_cast<uint8_t*>(dest);
  stream->output.size = dest_len;
  stream->output.offset = 0;
  lzxd_set_output_length(stream->decoder,
                         stream->decoder->offset + static_cast<off_t>(dest_len));
  // libmspack's end-frame calculation includes the frame at the exact end
  // offset. Stop one byte short and then drain that byte from its buffered
  // output so a 32 KiB LDI call doesn't create a zero-length phantom frame.
  const off_t final_offset =
      stream->decoder->offset + static_cast<off_t>(dest_len);
  const bool ends_on_frame_boundary =
      (final_offset % LZX_FRAME_SIZE) == 0 && dest_len > 1;
  const off_t first_output_size = static_cast<off_t>(
      ends_on_frame_boundary ? dest_len - 1 : dest_len);
  int result = lzxd_decompress(stream->decoder, first_output_size);
  if (result == MSPACK_ERR_OK && ends_on_frame_boundary) {
    result = lzxd_decompress(stream->decoder, 1);
  }
  XELOGI(
      "LZX stream: input_added={} consumed={} pending={} output={} "
      "frame={}->{} bits={} block_remaining={} result={}",
      lzx_len, stream->input.offset - input_offset_before,
      stream->input.data.size() - stream->input.offset, dest_len, frame_before,
      stream->decoder->frame, stream->decoder->bits_left,
      stream->decoder->block_remaining, result);
  if (result == MSPACK_ERR_OK && stream->output.offset != dest_len) {
    return MSPACK_ERR_WRITE;
  }
  if (result == MSPACK_ERR_OK) {
    // XMem/LDI chunks are individually padded to a 16-bit boundary. Keep the
    // LZX window, Huffman tables and repeated offsets, but don't carry the
    // padding or prefetched word into the next chunk.
    stream->decoder->bit_buffer = 0;
    stream->decoder->bits_left = 0;
    stream->decoder->i_ptr = stream->decoder->inbuf;
    stream->decoder->i_end = stream->decoder->inbuf;
    stream->decoder->input_end = 0;
    stream->input.data.clear();
    stream->input.offset = 0;
  }
  return result;
}

void lzx_stream_destroy(lzx_stream* stream) {
  if (!stream) {
    return;
  }
  if (stream->decoder) {
    lzxd_free(stream->decoder);
  }
  delete stream;
}

int lzx_decompress(const void* lzx_data, size_t lzx_len, void* dest,
                   size_t dest_len, uint32_t window_size, void* window_data,
                   size_t window_data_len) {
  int result_code = 1;

  uint32_t window_bits;
  if (!xe::bit_scan_forward(window_size, &window_bits)) {
    return result_code;
  }

  mspack_system* sys = mspack_memory_sys_create();
  mspack_memory_file* lzxsrc =
      mspack_memory_open(sys, (void*)lzx_data, lzx_len);
  mspack_memory_file* lzxdst = mspack_memory_open(sys, dest, dest_len);
  lzxd_stream* lzxd = lzxd_init(sys, (mspack_file*)lzxsrc, (mspack_file*)lzxdst,
                                window_bits, 0, 0x8000, (off_t)dest_len, 0);

  if (lzxd) {
    if (window_data) {
      // zero the window and then copy window_data to the end of it
      auto padding_len = window_size - window_data_len;
      std::memset(&lzxd->window[0], 0, padding_len);
      std::memcpy(&lzxd->window[padding_len], window_data, window_data_len);
      // TODO(gibbed): should this be set regardless if source window data is
      // available or not?
      lzxd->ref_data_size = window_size;
    }

    result_code = lzxd_decompress(lzxd, (off_t)dest_len);

    lzxd_free(lzxd);
    lzxd = NULL;
  }

  if (lzxsrc) {
    mspack_memory_close(lzxsrc);
    lzxsrc = NULL;
  }

  if (lzxdst) {
    mspack_memory_close(lzxdst);
    lzxdst = NULL;
  }

  if (sys) {
    mspack_memory_sys_destroy(sys);
    sys = NULL;
  }

  return result_code;
}

int lzxdelta_apply_patch(xe::xex2_delta_patch* patch, size_t patch_len,
                         uint32_t window_size, void* dest) {
  void* patch_end = (char*)patch + patch_len;
  auto* cur_patch = patch;

  while (patch_end > cur_patch) {
    int patch_sz = -4;  // 0 byte patches need us to remove 4 byte from next
                        // patch addr because of patch_data field
    if (cur_patch->compressed_len == 0 && cur_patch->uncompressed_len == 0 &&
        cur_patch->new_addr == 0 && cur_patch->old_addr == 0) {
      break;
    }
    switch (cur_patch->compressed_len) {
      case 0:  // fill with 0
        std::memset((char*)dest + cur_patch->new_addr, 0,
                    cur_patch->uncompressed_len);
        break;
      case 1:  // copy from old -> new
        std::memcpy((char*)dest + cur_patch->new_addr,
                    (char*)dest + cur_patch->old_addr,
                    cur_patch->uncompressed_len);
        break;
      default:  // delta patch
        patch_sz =
            cur_patch->compressed_len - 4;  // -4 because of patch_data field

        int result = lzx_decompress(
            cur_patch->patch_data, cur_patch->compressed_len,
            (char*)dest + cur_patch->new_addr, cur_patch->uncompressed_len,
            window_size, (char*)dest + cur_patch->old_addr,
            cur_patch->uncompressed_len);

        if (result) {
          return result;
        }
        break;
    }

    cur_patch++;
    cur_patch = (xe::xex2_delta_patch*)((char*)cur_patch +
                                        patch_sz);  // TODO: make this less ugly
  }

  return 0;
}
