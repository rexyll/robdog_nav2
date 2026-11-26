#include "robodog_protocol/libusb_device_session.hpp"

#include <utility>

namespace robodog_protocol {

LibusbDeviceSession::LibusbDeviceSession(Api api)
: api_(std::move(api)) {}

LibusbDeviceSession::~LibusbDeviceSession()
{
  (void)close();
}

bool LibusbDeviceSession::select_bulk_pair(
  const std::vector<UsbBulkEndpointDescriptor> & endpoints)
{
  for (const auto & in : endpoints) {
    if ((in.address & 0x80u) == 0u) continue;
    for (const auto & out : endpoints) {
      if ((out.address & 0x80u) != 0u) continue;
      if (in.interface_number != out.interface_number ||
          in.alternate_setting != out.alternate_setting ||
          (in.address & 0x0fu) == 0u || (out.address & 0x0fu) == 0u) continue;
      bulk_in_ = in;
      bulk_out_ = out;
      claimed_interface_ = in.interface_number;
      alternate_setting_ = in.alternate_setting;
      return true;
    }
  }
  return false;
}

bool LibusbDeviceSession::open(const UsbDeviceSessionConfig & config)
{
  if (is_open() || context_ != nullptr || device_ != nullptr) return false;
  if (!api_.init_context || !api_.exit_context || !api_.open_device ||
      !api_.close_device || !api_.get_bulk_endpoints ||
      !api_.kernel_driver_active || !api_.detach_kernel_driver ||
      !api_.attach_kernel_driver || !api_.claim_interface ||
      !api_.release_interface || !api_.set_alternate_setting ||
      config.vendor_id == 0 || config.product_id == 0) return false;

  context_ = api_.init_context();
  if (!context_) { last_error_ = -1; return false; }

  device_ = api_.open_device(context_, config.vendor_id, config.product_id, config.serial_number);
  if (!device_) { last_error_ = -2; (void)close(); return false; }

  std::vector<UsbBulkEndpointDescriptor> endpoints;
  if (!api_.get_bulk_endpoints(device_, endpoints) || !select_bulk_pair(endpoints)) {
    last_error_ = -3;
    (void)close();
    return false;
  }

  const int active = api_.kernel_driver_active(device_, claimed_interface_);
  if (active < 0 && active != api_.kernel_driver_not_supported_code) {
    last_error_ = active;
    (void)close();
    return false;
  }
  if (active > 0) {
    const int rc = api_.detach_kernel_driver(device_, claimed_interface_);
    if (rc != api_.success_code) { last_error_ = rc; (void)close(); return false; }
    detached_kernel_driver_ = true;
  }

  const int claim_rc = api_.claim_interface(device_, claimed_interface_);
  if (claim_rc != api_.success_code) { last_error_ = claim_rc; (void)close(); return false; }
  claimed_ = true;

  if (alternate_setting_ != 0) {
    const int alt_rc = api_.set_alternate_setting(device_, claimed_interface_, alternate_setting_);
    if (alt_rc != api_.success_code) { last_error_ = alt_rc; (void)close(); return false; }
  }
  last_error_ = api_.success_code;
  return true;
}

bool LibusbDeviceSession::close() noexcept
{
  bool ok = true;
  if (device_ && claimed_) {
    const int rc = api_.release_interface(device_, claimed_interface_);
    if (rc != api_.success_code) { last_error_ = rc; ok = false; }
    claimed_ = false;
  }
  if (device_ && detached_kernel_driver_) {
    const int rc = api_.attach_kernel_driver(device_, claimed_interface_);
    if (rc != api_.success_code) { last_error_ = rc; ok = false; }
    detached_kernel_driver_ = false;
  }
  if (device_) {
    api_.close_device(device_);
    device_ = nullptr;
  }
  if (context_) {
    api_.exit_context(context_);
    context_ = nullptr;
  }
  claimed_interface_ = -1;
  alternate_setting_ = 0;
  bulk_in_ = {};
  bulk_out_ = {};
  return ok;
}

}  // namespace robodog_protocol
