/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/base/console_app_main.h"
#include "xenia/base/cvar.h"
#include "xenia/base/literals.h"
#include "xenia/base/logging.h"
#include "xenia/base/math.h"

#include "xenia/vfs/devices/disc_image_device.h"
#include "xenia/vfs/devices/xcontent_container_device.h"
#include "xenia/vfs/file.h"
#include "xenia/vfs/virtual_file_system.h"

namespace xe {
namespace vfs {

DEFINE_transient_path(source, "", "Specifies the file to dump from.",
                      "General");

DEFINE_transient_path(dump_path, "",
                      "Specifies the directory to dump files to.", "General");

DEFINE_transient_string(guest_path, "",
                        "Optionally extracts only this guest path.", "General");

int vfs_dump_main(const std::vector<std::string>& args) {
  if (cvars::source.empty() || cvars::dump_path.empty()) {
    XELOGE("Usage: {} [source] [dump_path]", args[0]);
    return 1;
  }

  std::filesystem::path base_path = cvars::dump_path;
  std::unique_ptr<vfs::Device> device =
      vfs::XContentContainerDevice::CreateContentDevice("", cvars::source);
  bool initialized = device && device->Initialize();

  if (!initialized) {
    device = std::make_unique<vfs::DiscImageDevice>("", cvars::source);
    initialized = device->Initialize();
  }

  if (!initialized) {
    XELOGE("Failed to initialize device");
    return 1;
  }

  uint64_t progress = 0;
  if (!cvars::guest_path.empty()) {
    auto entry = device->ResolvePath(cvars::guest_path);
    if (!entry || (entry->attributes() & kFileAttributeDirectory)) {
      XELOGE("Guest file was not found: {}", cvars::guest_path);
      return 1;
    }

    std::error_code error_code;
    std::filesystem::create_directories(base_path, error_code);
    if (error_code) {
      XELOGE("Failed to create dump directory: {}", error_code.message());
      return 1;
    }

    return VirtualFileSystem::ExtractContentFile(entry, base_path, progress,
                                                 true);
  }

  return VirtualFileSystem::ExtractContentFiles(device.get(), base_path,
                                                progress);
}

}  // namespace vfs
}  // namespace xe

XE_DEFINE_CONSOLE_APP("xenia-vfs-dump", xe::vfs::vfs_dump_main,
                      "[source] [dump_path] [guest_path]", "source",
                      "dump_path", "guest_path");
