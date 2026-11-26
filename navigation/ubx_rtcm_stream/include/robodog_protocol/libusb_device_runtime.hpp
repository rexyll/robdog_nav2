#pragma once

#include "robodog_protocol/libusb_event_loop.hpp"
#include "robodog_protocol/libusb_device_session.hpp"

#include <cstdint>
#include <functional>
#include <memory>

namespace robodog_protocol {

// Top-level owner that enforces EventLoop -> TransferTransport -> DeviceSession
// teardown ordering. The API factory receives the live libusb context.
class LibusbDeviceRuntime {
public:
  using TransferApiFactory =
    std::function<LibusbTransferTransport::Api(void * native_context)>;

  LibusbDeviceRuntime(
    LibusbDeviceSession::Api device_api, TransferApiFactory transfer_api_factory);
  LibusbDeviceRuntime(const LibusbDeviceRuntime &) = delete;
  LibusbDeviceRuntime & operator=(const LibusbDeviceRuntime &) = delete;
  ~LibusbDeviceRuntime();

  bool open(const UsbDeviceSessionConfig & config, unsigned int poll_timeout_ms = 50);
  bool shutdown(std::uint32_t timeout_ms = 5000);

  bool is_open() const noexcept;
  LibusbTransferTransport * transport() noexcept { return transport_.get(); }
  const LibusbDeviceSession & device_session() const noexcept { return device_session_; }
  int last_error() const noexcept { return last_error_; }

private:
  LibusbDeviceSession device_session_;
  TransferApiFactory transfer_api_factory_;
  std::unique_ptr<LibusbTransferTransport> transport_;
  std::unique_ptr<LibusbEventLoop> event_loop_;
  int last_error_{0};
};

}  // namespace robodog_protocol
