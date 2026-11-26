#include "robodog_protocol/fake_usb_transport.hpp"

#include <cassert>
#include <cstdint>

using robodog_protocol::FakeUsbTransport;
using robodog_protocol::TransferLifecycle;
using robodog_protocol::TransferSession;
using robodog_protocol::TransferState;
using robodog_protocol::UsbCompletionStatus;

int main() {
  {
    TransferLifecycle lifecycle(1);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(1));
    assert(transport.complete(1, UsbCompletionStatus::Completed));
    assert(transport.poll_events() == 1);
    assert(lifecycle.snapshot(1)->state == TransferState::Released);
    assert(session.shutdown());
  }
  {
    TransferLifecycle lifecycle(4);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(10));
    assert(session.cancel(10));
    assert(transport.poll_events() == 1);
    assert(lifecycle.snapshot(10)->state == TransferState::Released);
    assert(session.shutdown());
  }
  {
    const UsbCompletionStatus statuses[] = {
      UsbCompletionStatus::NoDevice, UsbCompletionStatus::Error, UsbCompletionStatus::Cancelled
    };
    std::uint64_t id = 20;
    for (const auto status : statuses) {
      TransferLifecycle lifecycle(1);
      FakeUsbTransport transport;
      TransferSession session(lifecycle, transport);
      assert(session.submit(id));
      assert(transport.complete(id, status));
      assert(transport.poll_events() == 1);
      assert(lifecycle.snapshot(id)->state == TransferState::Released);
      assert(session.shutdown());
      ++id;
    }
  }
  {
    TransferLifecycle lifecycle(1);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    transport.fail_next_submit();
    assert(!session.submit(30));
    assert(lifecycle.snapshot(30)->state == TransferState::Released);
    assert(transport.pending_count() == 0);
    assert(session.shutdown());
  }
  {
    TransferLifecycle lifecycle(1);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(40));
    transport.fail_next_cancel();
    assert(!session.cancel(40));
    assert(lifecycle.snapshot(40)->state == TransferState::CancelRequested);
    assert(transport.complete(40, UsbCompletionStatus::Completed));
    assert(transport.poll_events() == 1);
    assert(lifecycle.snapshot(40)->state == TransferState::Released);
    assert(session.shutdown());
  }
  {
    TransferLifecycle lifecycle(9);
    FakeUsbTransport transport;
    TransferSession session(lifecycle, transport);
    assert(session.submit(50));
    assert(session.shutdown());
    assert(lifecycle.snapshot(50)->state == TransferState::Released);
    assert(transport.pending_count() == 0);
  }
  return 0;
}
