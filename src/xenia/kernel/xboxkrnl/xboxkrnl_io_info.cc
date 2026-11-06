/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include <array>

#include "xenia/base/logging.h"
#include "xenia/kernel/info/file.h"
#include "xenia/kernel/info/volume.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/util/shim_utils.h"
#include "xenia/kernel/xboxkrnl/xboxkrnl_private.h"
#include "xenia/kernel/xfile.h"
#include "xenia/vfs/device.h"
#include "xenia/vfs/devices/disc_image_entry.h"
#include "xenia/xbox.h"

namespace xe {
namespace kernel {
namespace xboxkrnl {

namespace {

constexpr std::array<uint8_t, 4> kXctdHeader = {0x0F, 0xF5, 0x12, 0xED};
constexpr uint32_t kXctdCompressionPresent = 1;

bool IsXctdCompressedFile(XFile* file) {
  std::array<uint8_t, kXctdHeader.size()> header{};
  size_t bytes_read = 0;
  const X_STATUS status = file->file()->ReadSync(header, 0, &bytes_read);
  return XSUCCEEDED(status) && bytes_read == header.size() &&
         header == kXctdHeader;
}

}  // namespace

uint32_t GetQueryFileInfoMinimumLength(uint32_t info_class) {
  switch (info_class) {
    case XFileInternalInformation:
      return sizeof(X_FILE_INTERNAL_INFORMATION);
    case XFilePositionInformation:
      return sizeof(X_FILE_POSITION_INFORMATION);
    case XFileXctdCompressionInformation:
      return sizeof(X_FILE_XCTD_COMPRESSION_INFORMATION);
    case XFileNetworkOpenInformation:
      return sizeof(X_FILE_NETWORK_OPEN_INFORMATION);
    // TODO(gibbed): structures to get the size of.
    case XFileModeInformation:
    case XFileAlignmentInformation:
    case XFileSectorInformation:
    case XFileIoPriorityInformation:
      return 4;
    case XFileNameInformation:
    case XFileAllocationInformation:
      return 8;
    case XFileBasicInformation:
      return 40;
    default:
      return 0;
  }
}

dword_result_t NtQueryInformationFile_entry(
    dword_t file_handle, pointer_t<X_IO_STATUS_BLOCK> io_status_block_ptr,
    lpvoid_t info_ptr, dword_t info_length, dword_t info_class) {
  uint32_t minimum_length = GetQueryFileInfoMinimumLength(info_class);
  if (!minimum_length) {
    return X_STATUS_INVALID_INFO_CLASS;
  }

  if (info_length < minimum_length) {
    return X_STATUS_INFO_LENGTH_MISMATCH;
  }

  auto file = kernel_state()->object_table()->LookupObject<XFile>(file_handle);
  if (!file) {
    return X_STATUS_INVALID_HANDLE;
  }

  info_ptr.Zero(info_length);

  X_STATUS status = X_STATUS_SUCCESS;
  uint32_t out_length;

  switch (info_class) {
    case XFileInternalInformation: {
      // Internal unique file pointer. Not sure why anyone would want this.
      // TODO(benvanik): use pointer to fs::entry?
      auto info = info_ptr.as<X_FILE_INTERNAL_INFORMATION*>();
      info->index_number = xe::memory::hash_combine(0, file->path());
      out_length = sizeof(*info);
      break;
    }
    case XFilePositionInformation: {
      auto info = info_ptr.as<X_FILE_POSITION_INFORMATION*>();
      info->current_byte_offset = file->position();
      out_length = sizeof(*info);
      break;
    }
    case XFileAlignmentInformation: {
      // Requested by XMountUtilityDrive XAM-task
      auto info = info_ptr.as<uint32_t*>();
      *info = 0;  // FILE_BYTE_ALIGNMENT?
      out_length = sizeof(*info);
      break;
    }
    case XFileSectorInformation: {
      auto info = info_ptr.as<uint32_t*>();
      if (auto* disc_entry =
              dynamic_cast<vfs::DiscImageEntry*>(file->entry())) {
        const uint32_t bytes_per_sector = file->device()->bytes_per_sector();
        if (!bytes_per_sector) {
          status = X_STATUS_INVALID_DEVICE_REQUEST;
          out_length = 0;
          break;
        }
        // DiscImageEntry retains the XISO byte position of each file. This is
        // the physical sector identity requested by retail disc titles.
        *info = static_cast<uint32_t>(disc_entry->data_offset() /
                                      bytes_per_sector);
        XELOGD("NtQueryInformationFile sector: path='{}' sector={}",
               file->path(), static_cast<uint32_t>(*info));
      } else {
        // Host-directory mounts have no XISO sector table. Retain their stable
        // file identity without representing it as a physical disc position.
        const size_t path_identity =
            xe::memory::hash_combine(82589933LL, file->path());
        *info = static_cast<uint32_t>(path_identity ^ (path_identity >> 32));
        XELOGD("NtQueryInformationFile sector identity: path='{}' value={}",
               file->path(), static_cast<uint32_t>(*info));
      }
      out_length = sizeof(uint32_t);
      break;
    }
    case XFileXctdCompressionInformation: {
      // Extracted games retain the compression header but not the original
      // media metadata. The Xbox ABI exposes this as a one-word boolean.
      const bool is_xctd = IsXctdCompressedFile(file.get());
      auto info = info_ptr.as<X_FILE_XCTD_COMPRESSION_INFORMATION*>();
      info->unknown = is_xctd ? kXctdCompressionPresent : 0;
      out_length = sizeof(*info);
      XELOGD("NtQueryInformationFile XCTD: path='{}' compressed={}",
             file->path(), is_xctd);
      break;
    };
    case XFileNetworkOpenInformation: {
      // Make sure we're working with up-to-date information, just in case the
      // file size has changed via something other than NtSetInfoFile
      // (eg. seems NtWriteFile might extend the file in some cases)
      file->entry()->update();

      auto info = info_ptr.as<X_FILE_NETWORK_OPEN_INFORMATION*>();
      info->creation_time = file->entry()->create_timestamp();
      info->last_access_time = file->entry()->access_timestamp();
      info->last_write_time = file->entry()->write_timestamp();
      info->change_time = file->entry()->write_timestamp();
      info->allocation_size = file->entry()->allocation_size();
      info->end_of_file = file->entry()->size();
      info->attributes = file->entry()->attributes();
      out_length = sizeof(*info);
      break;
    }
    default: {
      // Unsupported, for now.
      assert_always();
      status = X_STATUS_INVALID_PARAMETER;
      out_length = 0;
      break;
    }
  }

  if (io_status_block_ptr) {
    io_status_block_ptr->status = status;
    io_status_block_ptr->information = out_length;
  }

  return status;
}
DECLARE_XBOXKRNL_EXPORT1(NtQueryInformationFile, kFileSystem, kImplemented);

uint32_t GetSetFileInfoMinimumLength(uint32_t info_class) {
  switch (info_class) {
    case XFileRenameInformation:
      return sizeof(X_FILE_RENAME_INFORMATION);
    case XFileDispositionInformation:
      return sizeof(X_FILE_DISPOSITION_INFORMATION);
    case XFilePositionInformation:
      return sizeof(X_FILE_POSITION_INFORMATION);
    case XFileCompletionInformation:
      return sizeof(X_FILE_COMPLETION_INFORMATION);
    // TODO(gibbed): structures to get the size of.
    case XFileModeInformation:
    case XFileIoPriorityInformation:
      return 4;
    case XFileAllocationInformation:
    case XFileEndOfFileInformation:
    case XFileMountPartitionInformation:
      return 8;
    case XFileLinkInformation:
      return 16;
    case XFileBasicInformation:
      return 40;
    case XFileMountPartitionsInformation:
      return 152;
    default:
      return 0;
  }
}

dword_result_t NtSetInformationFile_entry(
    dword_t file_handle, pointer_t<X_IO_STATUS_BLOCK> io_status_block,
    lpvoid_t info_ptr, dword_t info_length, dword_t info_class) {
  uint32_t minimum_length = GetSetFileInfoMinimumLength(info_class);
  if (!minimum_length) {
    return X_STATUS_INVALID_INFO_CLASS;
  }

  if (info_length < minimum_length) {
    return X_STATUS_INFO_LENGTH_MISMATCH;
  }

  auto file = kernel_state()->object_table()->LookupObject<XFile>(file_handle);
  if (!file) {
    return X_STATUS_INVALID_HANDLE;
  }

  X_STATUS result = X_STATUS_SUCCESS;
  uint32_t out_length;

  switch (info_class) {
    case XFileBasicInformation: {
      auto info = info_ptr.as<X_FILE_BASIC_INFORMATION*>();

      bool basic_result = true;
      if (info->creation_time) {
        basic_result &= file->entry()->SetCreateTimestamp(info->creation_time);
      }

      if (info->last_access_time) {
        basic_result &=
            file->entry()->SetAccessTimestamp(info->last_access_time);
      }

      if (info->last_write_time) {
        basic_result &= file->entry()->SetWriteTimestamp(info->last_write_time);
      }

      basic_result &= file->entry()->SetAttributes(info->attributes);
      if (!basic_result) {
        result = X_STATUS_UNSUCCESSFUL;
      }

      out_length = sizeof(*info);
      break;
    }
    case XFileRenameInformation: {
      auto info = info_ptr.as<X_FILE_RENAME_INFORMATION*>();
      // Compute path, possibly attrs relative.
      std::filesystem::path target_path =
          util::TranslateAnsiPath(kernel_memory(), &info->ansi_string);

      // Place IsValidPath in path from where it can be accessed everywhere
      if (!IsValidPath(target_path.string(), false)) {
        return X_STATUS_OBJECT_NAME_INVALID;
      }

      if (!target_path.has_filename()) {
        return X_STATUS_INVALID_PARAMETER;
      }

      file->Rename(target_path);
      out_length = sizeof(*info);
      break;
    }
    case XFileDispositionInformation: {
      auto info = info_ptr.as<X_FILE_DISPOSITION_INFORMATION*>();
      bool delete_on_close = info->delete_file ? true : false;
      file->entry()->SetForDeletion(static_cast<bool>(info->delete_file));
      out_length = 0;
      XELOGW("NtSetInformationFile set deleting flag for {} on close to: {}",
             file->name(), delete_on_close);
      break;
    }
    case XFilePositionInformation: {
      auto info = info_ptr.as<X_FILE_POSITION_INFORMATION*>();
      file->set_position(info->current_byte_offset);
      out_length = sizeof(*info);
      break;
    }
    case XFileAllocationInformation: {
      XELOGW("NtSetInformationFile ignoring alloc");
      out_length = 8;
      break;
    }
    case XFileEndOfFileInformation: {
      auto info = info_ptr.as<X_FILE_END_OF_FILE_INFORMATION*>();
      result = file->SetLength(info->end_of_file);
      out_length = sizeof(*info);

      // Update the files vfs::Entry information
      file->entry()->update();
      break;
    }
    case XFileCompletionInformation: {
      // Info contains IO Completion handle and completion key
      auto info = info_ptr.as<X_FILE_COMPLETION_INFORMATION*>();
      auto handle = uint32_t(info->handle);
      auto key = uint32_t(info->key);
      out_length = sizeof(*info);
      auto port =
          kernel_state()->object_table()->LookupObject<XIOCompletion>(handle);
      if (!port) {
        result = X_STATUS_INVALID_HANDLE;
      } else {
        file->RegisterIOCompletionPort(key, port);
      }
      break;
    }
    default:
      // Unsupported, for now.
      assert_always();
      out_length = 0;
      break;
  }

  if (io_status_block) {
    io_status_block->status = result;
    io_status_block->information = out_length;
  }

  return result;
}
DECLARE_XBOXKRNL_EXPORT2(NtSetInformationFile, kFileSystem, kImplemented,
                         kHighFrequency);

uint32_t GetQueryVolumeInfoMinimumLength(uint32_t info_class) {
  switch (info_class) {
    case XFileFsVolumeInformation:
      return sizeof(X_FILE_FS_VOLUME_INFORMATION);
    case XFileFsSizeInformation:
      return sizeof(X_FILE_FS_SIZE_INFORMATION);
    case XFileFsDeviceInformation:
      return sizeof(X_FILE_FS_DEVICE_INFORMATION);
    case XFileFsAttributeInformation:
      return sizeof(X_FILE_FS_ATTRIBUTE_INFORMATION);
    // TODO(gibbed): structures to get the size of.
    default:
      XELOGW("Unimplemented Info Class: 0x{:08x}", info_class);
      return 0;
  }
}

dword_result_t NtQueryVolumeInformationFile_entry(
    dword_t file_handle, pointer_t<X_IO_STATUS_BLOCK> io_status_block_ptr,
    lpvoid_t info_ptr, dword_t info_length, dword_t info_class) {
  uint32_t minimum_length = GetQueryVolumeInfoMinimumLength(info_class);
  if (!minimum_length) {
    return X_STATUS_INVALID_INFO_CLASS;
  }

  if (info_length < minimum_length) {
    return X_STATUS_INFO_LENGTH_MISMATCH;
  }

  auto file = kernel_state()->object_table()->LookupObject<XFile>(file_handle);
  if (!file) {
    return X_STATUS_INVALID_HANDLE;
  }

  info_ptr.Zero(info_length);

  X_STATUS status = X_STATUS_SUCCESS;
  uint32_t out_length;

  switch (info_class) {
    case XFileFsVolumeInformation: {
      auto info = info_ptr.as<X_FILE_FS_VOLUME_INFORMATION*>();
      info->creation_time = 0;
      info->serial_number = 0;  // set for FATX, but we don't do that currently
      info->supports_objects = 0;
      info->label_length = 0;
      out_length = offsetof(X_FILE_FS_VOLUME_INFORMATION, label);
      break;
    }
    case XFileFsSizeInformation: {
      auto device = file->device();
      auto info = info_ptr.as<X_FILE_FS_SIZE_INFORMATION*>();
      info->total_allocation_units = device->total_allocation_units();
      info->available_allocation_units = device->available_allocation_units();
      info->sectors_per_allocation_unit = device->sectors_per_allocation_unit();
      info->bytes_per_sector = device->bytes_per_sector();
      // TODO(gibbed): sanity check, XCTD userland code seems to require this.
      assert_true(info->bytes_per_sector == 0x200);
      out_length = sizeof(*info);
      break;
    }
    case XFileFsAttributeInformation: {
      auto device = file->device();
      const auto& name = device->name();
      auto info = info_ptr.as<X_FILE_FS_ATTRIBUTE_INFORMATION*>();
      info->attributes = device->attributes();
      info->component_name_max_length = device->component_name_max_length();
      info->name_length = uint32_t(name.size());
      if (info_length >= 12 + name.size()) {
        std::memcpy(info->name, name.data(), name.size());
        out_length =
            offsetof(X_FILE_FS_ATTRIBUTE_INFORMATION, name) + info->name_length;
      } else {
        status = X_STATUS_BUFFER_OVERFLOW;
        out_length = offsetof(X_FILE_FS_ATTRIBUTE_INFORMATION, name);
      }
      break;
    }
    case XFileFsDeviceInformation: {
      auto info = info_ptr.as<X_FILE_FS_DEVICE_INFORMATION*>();
      auto file_device = file->device();
      info->device_type =
          static_cast<X_FILE_DEVICE_TYPE>(file_device->device_type());
      info->characteristics = file_device->device_characteristics();
      XELOGD("XFileFsDeviceInformation: device={} type={:08X} "
             "characteristics={:08X}",
             file_device->name(), file_device->device_type(),
             file_device->device_characteristics());
      out_length = sizeof(X_FILE_FS_DEVICE_INFORMATION);
      break;
    }
    default: {
      assert_always();
      out_length = 0;
      break;
    }
  }

  if (io_status_block_ptr) {
    io_status_block_ptr->status = status;
    io_status_block_ptr->information = out_length;
  }

  return status;
}
DECLARE_XBOXKRNL_EXPORT1(NtQueryVolumeInformationFile, kFileSystem,
                         kImplemented);

}  // namespace xboxkrnl
}  // namespace kernel
}  // namespace xe

DECLARE_XBOXKRNL_EMPTY_REGISTER_EXPORTS(IoInfo);
