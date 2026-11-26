#include "robodog_protocol/libusb_api_binding.hpp"

#include <array>
#include <cassert>
#include <cstdint>

namespace {
void noop_callback(void *, int, int, void *) {}
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
    250, 0, &noop_callback, reinterpret_cast<void *>(0x1234));
  assert(fill_result == LIBUSB_SUCCESS);

  // No submit/callback is attempted: free_transfer must safely reclaim the
  // unconsumed native callback bridge in this never-submitted state.
  api.free_transfer(transfer);
  libusb_exit(context);
  return 0;
}
