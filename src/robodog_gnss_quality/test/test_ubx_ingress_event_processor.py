import struct
import unittest

from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError
from robodog_gnss_quality.ubx_ingress_event_processor import (
    GnssIngressEvent,
    GnssIngressEventProcessor,
)
from robodog_gnss_quality.ubx_time_mapper import UbxReceiverTimeMapper


SESSION = "f9p-process-A"
GENERATION = 1


def make_ubx_frame(message_class, message_id, payload):
    body = bytes([message_class, message_id]) + len(payload).to_bytes(2, "little") + payload
    ck_a = 0
    ck_b = 0
    for value in body:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xB5\x62" + body + bytes([ck_a, ck_b])


def timegps_frame(i_tow=123456):
    payload = struct.pack("<IihbBI", i_tow, 0, 2450, 18, 0x07, 50)
    return make_ubx_frame(0x01, 0x20, payload)


def timels_frame(i_tow=123456):
    payload = bytearray(24)
    payload[0:4] = i_tow.to_bytes(4, "little")
    payload[8] = 2
    payload[9] = 18
    payload[10] = 2
    payload[11] = 1
    payload[12:16] = (86400).to_bytes(4, "little", signed=True)
    payload[16:18] = (2451).to_bytes(2, "little")
    payload[18:20] = (7).to_bytes(2, "little")
    payload[23] = 0x01  # current offset valid; future event unavailable
    return make_ubx_frame(0x01, 0x26, bytes(payload))


def event(kind, sequence, *, session=SESSION, generation=GENERATION,
          frame_bytes=None, protocol=None, dropped_frame_events_total=0):
    return GnssIngressEvent(
        event_type=kind,
        process_session_id=session,
        connection_generation=generation,
        ingress_sequence=sequence,
        receive_monotonic_ns=1_000_000 + sequence * 1000,
        dropped_frame_events_total=dropped_frame_events_total,
        protocol=protocol,
        frame_bytes=frame_bytes,
    )


def make_processor():
    mapper = UbxReceiverTimeMapper(
        trusted_leap_sources={2},
        require_ingress_sequence=True,
    )
    return GnssIngressEventProcessor(mapper)


class TestUbxIngressEventProcessor(unittest.TestCase):
    def test_generation_and_raw_time_frames_drive_canonical_mapper(self):
        processor = make_processor()
        self.assertIsNone(processor.process(event("GenerationStarted", 0)))
        self.assertIsNone(processor.process(event(
            "Frame", 1, frame_bytes=timegps_frame(), protocol=0
        )))
        mapping = processor.process(event(
            "Frame", 2, frame_bytes=timels_frame(), protocol=0
        ))
        self.assertIsNotNone(mapping)
        self.assertTrue(mapping.valid)
        self.assertTrue(mapping.utc_stamp_valid)
        self.assertEqual(mapping.process_session_id, SESSION)
        self.assertEqual(mapping.connection_generation, GENERATION)
        self.assertEqual(mapping.gps_week, 2450)
        stats = processor.stats()
        self.assertEqual(stats.processed_events, 3)
        self.assertEqual(stats.decoded_timegps, 1)
        self.assertEqual(stats.decoded_timels, 1)
        self.assertEqual(stats.emitted_mappings, 1)

    def test_non_time_frames_consume_event_order_without_entering_mapper(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0))
        generic = make_ubx_frame(0x01, 0x07, b"\x00\x01")
        self.assertIsNone(processor.process(event(
            "Frame", 1, frame_bytes=generic, protocol=0
        )))
        processor.process(event("Frame", 2, frame_bytes=timegps_frame(), protocol=0))
        mapping = processor.process(event(
            "Frame", 3, frame_bytes=timels_frame(), protocol=0
        ))
        self.assertIsNotNone(mapping)
        self.assertEqual(processor.stats().ignored_frames, 1)

    def test_device_lost_resets_mapper_and_next_generation_is_fenced(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0))
        processor.process(event("Frame", 1, frame_bytes=timegps_frame(), protocol=0))
        processor.process(event("DeviceLost", 2))
        processor.process(event("GenerationStarted", 3, generation=2))
        with self.assertRaisesRegex(EpochPairingError, "stale"):
            processor.process(event(
                "Frame", 4, generation=1, frame_bytes=timels_frame(), protocol=0
            ))
        processor.process(event(
            "Frame", 5, generation=2, frame_bytes=timegps_frame(), protocol=0
        ))
        mapping = processor.process(event(
            "Frame", 6, generation=2, frame_bytes=timels_frame(), protocol=0
        ))
        self.assertIsNotNone(mapping)
        self.assertEqual(mapping.connection_generation, 2)

    def test_stopped_after_device_lost_is_valid_but_later_frame_is_rejected(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0))
        processor.process(event("DeviceLost", 1))
        processor.process(event("Stopped", 2))
        with self.assertRaisesRegex(EpochPairingError, "stale or reordered|outside active"):
            processor.process(event(
                "Frame", 3, frame_bytes=timegps_frame(), protocol=0
            ))

    def test_duplicate_or_reordered_event_sequence_is_rejected(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0))
        generic = make_ubx_frame(0x01, 0x07, b"\x00\x01")
        processor.process(event("Frame", 1, frame_bytes=generic, protocol=0))
        with self.assertRaisesRegex(EpochPairingError, "sequence"):
            processor.process(event("Frame", 1, frame_bytes=generic, protocol=0))

    def test_new_process_session_resets_global_sequence_origin(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0))
        processor.process(event("Stopped", 1))
        processor.process(event(
            "GenerationStarted", 0, session="f9p-process-B", generation=1
        ))
        mapping = processor.process(event(
            "Frame", 1, session="f9p-process-B", generation=1,
            frame_bytes=timegps_frame(), protocol=0
        ))
        self.assertIsNone(mapping)
        self.assertEqual(processor.stats().decoded_timegps, 1)

    def test_receive_monotonic_timestamp_must_not_move_backwards(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0))
        generic = make_ubx_frame(0x01, 0x07, b"\\x00\\x01")
        with self.assertRaisesRegex(EpochPairingError, "receive_monotonic_ns"):
            processor.process(GnssIngressEvent(
                event_type="Frame",
                process_session_id=SESSION,
                connection_generation=GENERATION,
                ingress_sequence=1,
                receive_monotonic_ns=999_999,
                protocol=0,
                frame_bytes=generic,
            ))
        # Equal timestamps are valid: multiple frames can be decoded from one
        # USB completion callback and share its callback-entry timestamp.
        self.assertIsNone(processor.process(GnssIngressEvent(
            event_type="Frame",
            process_session_id=SESSION,
            connection_generation=GENERATION,
            ingress_sequence=1,
            receive_monotonic_ns=1_000_000,
            protocol=0,
            frame_bytes=generic,
        )))
        self.assertEqual(processor.stats().ignored_frames, 1)


    def test_queue_drop_watermark_accounts_for_sequence_gaps(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0, dropped_frame_events_total=1))
        # Sequence 1 was accepted and later evicted by DropOldest. Its loss
        # watermark may already be visible on the earlier popped start event.
        processor.process(event(
            "Frame", 2, frame_bytes=timegps_frame(), protocol=0,
            dropped_frame_events_total=1
        ))
        mapping = processor.process(event(
            "Frame", 3, frame_bytes=timels_frame(), protocol=0,
            dropped_frame_events_total=1
        ))
        self.assertIsNotNone(mapping)

    def test_unexplained_ingress_sequence_gap_fails_closed(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0))
        with self.assertRaisesRegex(EpochPairingError, "not explained"):
            processor.process(event(
                "Frame", 2, frame_bytes=timegps_frame(), protocol=0
            ))

    def test_unaccounted_drop_watermark_must_be_resolved_before_terminal(self):
        processor = make_processor()
        processor.process(event("GenerationStarted", 0, dropped_frame_events_total=1))
        with self.assertRaisesRegex(EpochPairingError, "unexplained dropped-frame"):
            processor.process(event("Stopped", 1, dropped_frame_events_total=1))

    def test_mapper_must_enforce_ingress_sequence(self):
        with self.assertRaisesRegex(ValueError, "require ingress_sequence"):
            GnssIngressEventProcessor(UbxReceiverTimeMapper())


if __name__ == "__main__":
    unittest.main()
