#include "robodog_protocol/libusb_api_binding.hpp"

#include <new>
#include <sys/time.h>

namespace robodog_protocol {
namespace {

struct CallbackBridge {
  LibusbTransferTransport::NativeCallback callback{};
  void * user_data{};
};

void native_callback(libusb_transfer * transfer)
{
  if (!transfer) return;
  auto * bridge = static_cast<CallbackBridge *>(transfer->user_data);
  if (!bridge) return;

  const auto callback = bridge->callback;
  void * const user_data = bridge->user_data;
  const int status = static_cast<int>(transfer->status);
  const int actual_length = transfer->actual_length;

  // The bridge belongs to this one native callback. The adapter's Context
  // remains owned by its pending registry until handle_events returns.
  transfer->user_data = nullptr;
  delete bridge;
  if (callback) callback(transfer, status, actual_length, user_data);
}

}  // namespace

LibusbTransferTransport::Api make_libusb_api(libusb_context * context)
{
  LibusbTransferTransport::Api api;
  api.alloc_transfer = [](int iso_packets) -> void * {
    return libusb_alloc_transfer(iso_packets);
  };
  api.free_transfer = [](void * native_transfer) {
    auto * transfer = static_cast<libusb_transfer *>(native_transfer);
    // Adapter calls free only before submit failure or after callback drain.
    // In either safe state, any still-attached bridge can be reclaimed here.
    delete static_cast<CallbackBridge *>(transfer->user_data);
    transfer->user_data = nullptr;
    libusb_free_transfer(transfer);
  };
  api.fill_bulk_transfer = [](
    void * native_transfer, void * device_handle, std::uint8_t endpoint,
    std::uint8_t * buffer, int length, unsigned int timeout_ms,
    LibusbTransferTransport::NativeCallback callback, void * user_data) -> int
  {
    auto * bridge = new (std::nothrow) CallbackBridge{callback, user_data};
    if (!bridge) return LIBUSB_ERROR_NO_MEM;

    libusb_fill_bulk_transfer(
      static_cast<libusb_transfer *>(native_transfer),
      static_cast<libusb_device_handle *>(device_handle), endpoint,
      buffer, length, &native_callback, bridge, timeout_ms);
    return LIBUSB_SUCCESS;
  };
  api.submit_transfer = [](void * native_transfer) -> int {
    auto * transfer = static_cast<libusb_transfer *>(native_transfer);
    const int result = libusb_submit_transfer(transfer);
    if (result != LIBUSB_SUCCESS) {
      // libusb did not accept the transfer, so no callback will consume bridge.
      delete static_cast<CallbackBridge *>(transfer->user_data);
      transfer->user_data = nullptr;
    }
    return result;
  };
  api.cancel_transfer = [](void * native_transfer) -> int {
    return libusb_cancel_transfer(static_cast<libusb_transfer *>(native_transfer));
  };
  api.handle_events = [context](unsigned int timeout_ms) -> int {
    if (!context) return LIBUSB_ERROR_INVALID_PARAM;
    timeval timeout{};
    timeout.tv_sec = static_cast<decltype(timeout.tv_sec)>(timeout_ms / 1000u);
    timeout.tv_usec = static_cast<decltype(timeout.tv_usec)>((timeout_ms % 1000u) * 1000u);
    return libusb_handle_events_timeout_completed(context, &timeout, nullptr);
  };
  api.close_device = [](void * device_handle) {
    libusb_close(static_cast<libusb_device_handle *>(device_handle));
  };

  api.success_code = LIBUSB_SUCCESS;
  api.completed_status = LIBUSB_TRANSFER_COMPLETED;
  api.error_status = LIBUSB_TRANSFER_ERROR;
  api.timeout_status = LIBUSB_TRANSFER_TIMED_OUT;
  api.cancelled_status = LIBUSB_TRANSFER_CANCELLED;
  api.stall_status = LIBUSB_TRANSFER_STALL;
  api.no_device_status = LIBUSB_TRANSFER_NO_DEVICE;
  api.overflow_status = LIBUSB_TRANSFER_OVERFLOW;
  return api;
}

}  // namespace robodog_protocol
