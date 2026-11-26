#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace robodog_protocol {

struct UsbBulkEndpointDescriptor {
  int interface_number{-1};
  int alternate_setting{0};
  std::uint8_t address{0};
  std::uint16_t max_packet_size{0};
};

struct UsbDeviceSessionConfig {
  std::uint16_t vendor_id{0};
  std::uint16_t product_id{0};
  std::string serial_number;
};

class LibusbDeviceSession {
public:
  struct Api {
    std::function<void *()> init_context;
    std::function<void(void *)> exit_context;
    std::function<void *(void *, std::uint16_t, std::uint16_t, const std::string &)> open_device;
    std::function<void(void *)> close_device;
    std::function<bool(void *, std::vector<UsbBulkEndpointDescriptor> &)> get_bulk_endpoints;
    std::function<int(void *, int)> kernel_driver_active;
    std::function<int(void *, int)> detach_kernel_driver;
    std::function<int(void *, int)> attach_kernel_driver;
    std::function<int(void *, int)> claim_interface;
    std::function<int(void *, int)> release_interface;
    std::function<int(void *, int, int)> set_alternate_setting;
    int success_code{0};
  };

  explicit LibusbDeviceSession(Api api);
  LibusbDeviceSession(const LibusbDeviceSession &) = delete;
  LibusbDeviceSession & operator=(const LibusbDeviceSession &) = delete;
  ~LibusbDeviceSession();

  bool open(const UsbDeviceSessionConfig & config);
  bool close() noexcept;
  bool is_open() const noexcept { return context_ != nullptr && device_ != nullptr && claimed_; }

  void * native_context() const noexcept { return context_; }
  void * native_device() const noexcept { return device_; }
  const UsbBulkEndpointDescriptor & bulk_in() const noexcept { return bulk_in_; }
  const UsbBulkEndpointDescriptor & bulk_out() const noexcept { return bulk_out_; }
  int last_error() const noexcept { return last_error_; }

private:
  bool select_bulk_pair(const std::vector<UsbBulkEndpointDescriptor> & endpoints);

  Api api_;
  void * context_{nullptr};
  void * device_{nullptr};
  int claimed_interface_{-1};
  int alternate_setting_{0};
  bool detached_kernel_driver_{false};
  bool claimed_{false};
  UsbBulkEndpointDescriptor bulk_in_{};
  UsbBulkEndpointDescriptor bulk_out_{};
  int last_error_{0};
};

}  // namespace robodog_protocol
