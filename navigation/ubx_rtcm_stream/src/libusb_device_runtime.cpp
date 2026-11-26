#include "robodog_protocol/libusb_device_runtime.hpp"

#include <exception>
#include <utility>

namespace robodog_protocol {

LibusbDeviceRuntime::LibusbDeviceRuntime(
  LibusbDeviceSession::Api device_api, TransferApiFactory transfer_api_factory)
: device_session_(std::move(device_api)),
  transfer_api_factory_(std::move(transfer_api_factory)) {}

LibusbDeviceRuntime::~LibusbDeviceRuntime()
{
  if ((event_loop_ || transport_ || device_session_.native_context()) && !shutdown(5000)) {
    std::terminate();
  }
}

bool LibusbDeviceRuntime::open(
  const UsbDeviceSessionConfig & config, unsigned int poll_timeout_ms)
{
  if (event_loop_ || transport_ || device_session_.is_open() || !transfer_api_factory_) return false;
  if (!device_session_.open(config)) {
    last_error_ = device_session_.last_error();
    return false;
  }

  try {
    auto transfer_api = transfer_api_factory_(device_session_.native_context());
    // DeviceSession is the sole owner of the native handle. The transfer
    // adapter only borrows it and must not call libusb_close() itself.
    transfer_api.close_device_on_close = false;
    transport_ = std::make_unique<LibusbTransferTransport>(
      std::move(transfer_api), device_session_.native_device());
    if (!transport_->valid()) {
      last_error_ = -6;
      (void)transport_->close();
      transport_.reset();
      (void)device_session_.close();
      return false;
    }
    event_loop_ = std::make_unique<LibusbEventLoop>(*transport_, poll_timeout_ms);
    if (!event_loop_->start()) {
      last_error_ = -4;
      (void)event_loop_->shutdown(1000);
      event_loop_.reset();
      if (!transport_->close()) return false;
      transport_.reset();
      (void)device_session_.close();
      return false;
    }
  } catch (...) {
    last_error_ = -5;
    if (event_loop_) {
      if (!event_loop_->shutdown(1000)) return false;
      event_loop_.reset();
    }
    if (transport_) {
      if (!transport_->close()) return false;
      transport_.reset();
    }
    (void)device_session_.close();
    return false;
  }

  last_error_ = 0;
  return true;
}

bool LibusbDeviceRuntime::shutdown(std::uint32_t timeout_ms)
{
  if (event_loop_) {
    if (!event_loop_->shutdown(timeout_ms)) return false;
    event_loop_.reset();
  }
  if (transport_) {
    if (!transport_->close()) return false;
    transport_.reset();
  }
  if (device_session_.native_context()) {
    if (!device_session_.close()) {
      last_error_ = device_session_.last_error();
      return false;
    }
  }
  return true;
}

bool LibusbDeviceRuntime::is_open() const noexcept
{
  return device_session_.is_open() && transport_ != nullptr && event_loop_ != nullptr;
}

}  // namespace robodog_protocol
