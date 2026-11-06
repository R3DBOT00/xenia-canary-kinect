/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2020 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#ifndef XENIA_KERNEL_XAM_APPS_XAM_APP_H_
#define XENIA_KERNEL_XAM_APPS_XAM_APP_H_

#include <array>

#include "xenia/kernel/kernel_state.h"
#include "xenia/kernel/xam/app_manager.h"

namespace xe {
namespace kernel {
namespace xam {
namespace apps {

class XamApp : public App {
 public:
  explicit XamApp(KernelState* kernel_state);

  X_HRESULT DispatchMessageSync(uint32_t message, uint32_t buffer_ptr,
                                uint32_t buffer_length) override;

 private:
  struct NuiAuxiliaryQueueRegistration {
    uint32_t message = 0;
    std::array<uint32_t, 6> words = {};
    uint32_t generation = 0;
  };

  uint32_t nui_skeleton_message_count_ = 0;
  uint32_t nui_skeleton_last_tracked_count_ = UINT32_MAX;
  std::array<NuiAuxiliaryQueueRegistration, 5> nui_auxiliary_queues_ = {};
  uint32_t nui_auxiliary_generation_ = 0;
};

}  // namespace apps
}  // namespace xam
}  // namespace kernel
}  // namespace xe

#endif  // XENIA_KERNEL_XAM_APPS_XAM_APP_H_
