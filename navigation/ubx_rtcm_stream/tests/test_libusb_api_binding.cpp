#include "robodog_protocol/libusb_api_binding.hpp"

#include <array>
#include <cassert>
#include <cstdint>

namespace {
bool callback_called = false;
int callback_status = -1;
int callback_actual_length = -1;
void * callback_user_data = nullptr;
void * callback_transfer = nullptr;

void capture_callback(void * transfer, int status, int actual_length, void * user_data) {
  callback_called = true;
  callback_status = status;
  callback_actual_length = actual_length;
  callback_user_data = user_data;
  callback_transfer = transfer;
}
}

int main()
{
  libusb_context * context = nullptr;
  assert(libusb_init(&context) == LIBUSB_SUCCESS);

  auto api = robodog_protocol::make_libusb_api(context);
  assert(api.alloc_transfer && api.free_transfer && api.fill_bulk_transfer);
  assert(api.submit_transfer && api.cancel_transfer && api.handle_events && api.close_device);
  assert(api.success_code == LIBUSB_SUCCESS);
  assert(api.completed_status == LIBUSB_TRANSFER_COMPLETED);
  assert(api.no_device_status == LIBUSB_TRANSFER_NO_DEVICE);

  auto * transfer = static_cast<libusb_transfer *>(api.alloc_transfer(0));
  assert(transfer != nullptr);
  std::array<std::uint8_t, 16> buffer{};
  const int fill_result = api.fill_bulk_transfer(
    transfer, nullptr, 0x81, buffer.data(), static_cast<int>(buffer.size()),
    250, 0, &capture_callback, reinterpret_cast<void *>(0x1234));
  assert(fill_result == LIBUSB_SUCCESS);

  transfer->status = LIBUSB_TRANSFER_COMPLETED;
  transfer->actual_length = 6;
  transfer->callback(transfer);  // Exercise the native-signature bridge directly.
  assert(callback_called);
  assert(callback_status == LIBUSB_TRANSFER_COMPLETED);
  assert(callback_actual_length == 6);
  assert(callback_user_data == reinterpret_cast<void *>(0x1234));
  assert(callback_transfer == transfer);
  assert(transfer->user_data == nullptr);
  api.free_transfer(transfer);

  // Also verify the never-submitted path reclaims a bridge still in user_data.
  auto * unsubmitted = static_cast<libusb_transfer *>(api.alloc_transfer(0));
  assert(unsubmitted != nullptr);
  assert(api.fill_bulk_transfer(
    unsubmitted, nullptr, 0x01, buffer.data(), static_cast<int>(buffer.size()),
    250, 0, &capture_callback, reinterpret_cast<void *>(0x5678)) == LIBUSB_SUCCESS);
  api.free_transfer(unsubmitted);
  libusb_exit(context);
  return 0;
}
