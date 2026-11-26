#include "robodog_protocol/libusb_api_binding.hpp"

#include <new>
#include <algorithm>
#include <array>
#include <string>
#include <sys/types.h>
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
    std::uint8_t * buffer, int length, unsigned int timeout_ms, std::uint8_t flags,
    LibusbTransferTransport::NativeCallback callback, void * user_data) -> int
  {
    auto * bridge = new (std::nothrow) CallbackBridge{callback, user_data};
    if (!bridge) return LIBUSB_ERROR_NO_MEM;

    libusb_fill_bulk_transfer(
      static_cast<libusb_transfer *>(native_transfer),
      static_cast<libusb_device_handle *>(device_handle), endpoint,
      buffer, length, &native_callback, bridge, timeout_ms);
    static_cast<libusb_transfer *>(native_transfer)->flags = flags;
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

LibusbDeviceSession::Api make_libusb_device_api()
{
  LibusbDeviceSession::Api api;
  api.init_context = []() -> void * {
    libusb_context * context = nullptr;
    return libusb_init(&context) == LIBUSB_SUCCESS ? context : nullptr;
  };
  api.exit_context = [](void * context) {
    libusb_exit(static_cast<libusb_context *>(context));
  };
  api.open_device = [](
    void * context, std::uint16_t vendor_id, std::uint16_t product_id,
    const std::string & requested_serial) -> void *
  {
    libusb_device ** devices = nullptr;
    const ssize_t count = libusb_get_device_list(
      static_cast<libusb_context *>(context), &devices);
    if (count < 0) return nullptr;

    libusb_device_handle * selected = nullptr;
    for (ssize_t i = 0; i < count; ++i) {
      libusb_device_descriptor descriptor{};
      if (libusb_get_device_descriptor(devices[i], &descriptor) != LIBUSB_SUCCESS ||
          descriptor.idVendor != vendor_id || descriptor.idProduct != product_id) continue;

      libusb_device_handle * candidate = nullptr;
      if (libusb_open(devices[i], &candidate) != LIBUSB_SUCCESS || !candidate) continue;
      if (!requested_serial.empty()) {
        if (descriptor.iSerialNumber == 0) {
          libusb_close(candidate);
          continue;
        }
        std::array<unsigned char, 256> serial{};
        const int length = libusb_get_string_descriptor_ascii(
          candidate, descriptor.iSerialNumber, serial.data(),
          static_cast<int>(serial.size()));
        const std::string actual = length > 0
          ? std::string(reinterpret_cast<const char *>(serial.data()), static_cast<std::size_t>(length))
          : std::string{};
        if (actual != requested_serial) {
          libusb_close(candidate);
          continue;
        }
      }
      selected = candidate;
      break;
    }
    libusb_free_device_list(devices, 1);
    return selected;
  };
  api.close_device = [](void * handle) {
    libusb_close(static_cast<libusb_device_handle *>(handle));
  };
  api.get_bulk_endpoints = [](
    void * handle, std::vector<UsbBulkEndpointDescriptor> & output) -> bool
  {
    output.clear();
    auto * device = libusb_get_device(static_cast<libusb_device_handle *>(handle));
    if (!device) return false;
    libusb_config_descriptor * config = nullptr;
    const int rc = libusb_get_active_config_descriptor(device, &config);
    if (rc != LIBUSB_SUCCESS || !config) return false;

    for (std::uint8_t i = 0; i < config->bNumInterfaces; ++i) {
      const libusb_interface & interface = config->interface[i];
      for (int a = 0; a < interface.num_altsetting; ++a) {
        const libusb_interface_descriptor & alt = interface.altsetting[a];
        for (std::uint8_t e = 0; e < alt.bNumEndpoints; ++e) {
          const libusb_endpoint_descriptor & endpoint = alt.endpoint[e];
          if ((endpoint.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK) continue;
          output.push_back(UsbBulkEndpointDescriptor{
            static_cast<int>(alt.bInterfaceNumber),
            static_cast<int>(alt.bAlternateSetting),
            endpoint.bEndpointAddress,
            endpoint.wMaxPacketSize});
        }
      }
    }
    libusb_free_config_descriptor(config);
    return !output.empty();
  };
  api.kernel_driver_active = [](void * handle, int interface_number) {
    return libusb_kernel_driver_active(
      static_cast<libusb_device_handle *>(handle), interface_number);
  };
  api.detach_kernel_driver = [](void * handle, int interface_number) {
    return libusb_detach_kernel_driver(
      static_cast<libusb_device_handle *>(handle), interface_number);
  };
  api.attach_kernel_driver = [](void * handle, int interface_number) {
    return libusb_attach_kernel_driver(
      static_cast<libusb_device_handle *>(handle), interface_number);
  };
  api.claim_interface = [](void * handle, int interface_number) {
    return libusb_claim_interface(
      static_cast<libusb_device_handle *>(handle), interface_number);
  };
  api.release_interface = [](void * handle, int interface_number) {
    return libusb_release_interface(
      static_cast<libusb_device_handle *>(handle), interface_number);
  };
  api.set_alternate_setting = [](void * handle, int interface_number, int alternate_setting) {
    return libusb_set_interface_alt_setting(
      static_cast<libusb_device_handle *>(handle), interface_number, alternate_setting);
  };
  api.success_code = LIBUSB_SUCCESS;
  return api;
}

}  // namespace robodog_protocol
