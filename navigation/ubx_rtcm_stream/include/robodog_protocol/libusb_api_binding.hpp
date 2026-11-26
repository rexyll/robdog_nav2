#pragma once

#include "robodog_protocol/libusb_transfer_transport.hpp"
#include "robodog_protocol/libusb_device_session.hpp"

#include <libusb.h>

namespace robodog_protocol {

// Builds the production function table for libusb-1.0. The caller owns the
// libusb_context and must keep it alive until the transport is closed and the
// event-pump thread has been joined.
LibusbTransferTransport::Api make_libusb_api(libusb_context * context);
LibusbDeviceSession::Api make_libusb_device_api();

}  // namespace robodog_protocol
