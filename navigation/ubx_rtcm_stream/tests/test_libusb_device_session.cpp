#include "robodog_protocol/libusb_device_session.hpp"

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

using robodog_protocol::LibusbDeviceSession;
using robodog_protocol::UsbBulkEndpointDescriptor;
using robodog_protocol::UsbDeviceSessionConfig;

struct FakeDeviceApi {
  int active{1};
  int kernel_driver_not_supported{-12};
  int claim_result{0};
  int alt_result{0};
  int release_result{0};
  int detach_count{0};
  int attach_count{0};
  int claim_count{0};
  int release_count{0};
  int close_count{0};
  int exit_count{0};
  int open_count{0};
  std::vector<UsbBulkEndpointDescriptor> endpoints{
    {1, 0, 0x81, 64}, {1, 0, 0x02, 64}, {2, 0, 0x83, 64}
  };

  LibusbDeviceSession::Api api() {
    LibusbDeviceSession::Api a;
    a.init_context = [] { return reinterpret_cast<void *>(0x10); };
    a.exit_context = [this](void *) { ++exit_count; };
    a.open_device = [this](void *, std::uint16_t vid, std::uint16_t pid, const std::string & serial) {
      ++open_count;
      assert(vid == 0x1546 && pid == 0x01a9);
      assert(serial == "F9P-TEST");
      return reinterpret_cast<void *>(0x20);
    };
    a.close_device = [this](void *) { ++close_count; };
    a.get_bulk_endpoints = [this](void *, std::vector<UsbBulkEndpointDescriptor> & out) {
      out = endpoints; return true;
    };
    a.kernel_driver_active = [this](void *, int) { return active; };
    a.detach_kernel_driver = [this](void *, int) { ++detach_count; return 0; };
    a.attach_kernel_driver = [this](void *, int) { ++attach_count; return 0; };
    a.claim_interface = [this](void *, int) { ++claim_count; return claim_result; };
    a.release_interface = [this](void *, int) { ++release_count; return release_result; };
    a.set_alternate_setting = [this](void *, int, int) { return alt_result; };
    a.kernel_driver_not_supported_code = kernel_driver_not_supported;
    return a;
  }
};

int main() {
  UsbDeviceSessionConfig config{0x1546, 0x01a9, "F9P-TEST"};
  {
    FakeDeviceApi fake;
    LibusbDeviceSession session(fake.api());
    assert(session.open(config));
    assert(session.is_open());
    assert(session.bulk_in().address == 0x81);
    assert(session.bulk_out().address == 0x02);
    assert(session.bulk_in().interface_number == 1);
    assert(session.close());
    assert(fake.detach_count == 1 && fake.attach_count == 1);
    assert(fake.claim_count == 1 && fake.release_count == 1);
    assert(fake.close_count == 1 && fake.exit_count == 1);
  }
  {
    FakeDeviceApi fake;
    fake.endpoints = {{1, 0, 0x81, 64}, {2, 0, 0x02, 64}};
    LibusbDeviceSession session(fake.api());
    assert(!session.open(config));
    assert(fake.claim_count == 0);
    assert(fake.close_count == 1 && fake.exit_count == 1);
  }
  {
    FakeDeviceApi fake;
    fake.active = fake.kernel_driver_not_supported;
    LibusbDeviceSession session(fake.api());
    assert(session.open(config));
    assert(fake.detach_count == 0);
    assert(session.close());
    assert(fake.attach_count == 0);
  }
  {
    FakeDeviceApi fake;
    fake.claim_result = -6;
    LibusbDeviceSession session(fake.api());
    assert(!session.open(config));
    assert(fake.detach_count == 1);
    assert(fake.attach_count == 1);
    assert(fake.release_count == 0);
    assert(fake.close_count == 1 && fake.exit_count == 1);
  }
  return 0;
}
