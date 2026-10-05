#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python

 (C)2009 - Luc Jean - luc.jean@gmail.com
 (C)2009 - Apidev - http://www.apidev.fr

 This is distributed under GNU LGPL license, see license.txt
"""

import struct
import sys
import unittest
from fractions import Fraction

import modbus_tk
from modbus_tk.utils import calculate_crc
from modbus_tk import rtu_replay
from modbus_tk.rtu_replay import (
    Arrival, FrameReceived, CrcError, FrameInterrupted,
    MalformedFrame, UnterminatedTail, calculate_rtu_limits, replay,
)

LOGGER = modbus_tk.utils.create_logger()


def make_frame(address, pdu):
    """Build a valid RTU ADU (little endian CRC on the wire, swapped by helper)"""
    body = struct.pack(">B", address) + bytes(bytearray(pdu))
    crc = calculate_crc(body)
    return body + struct.pack(">H", crc)


def feed_bytes(replayer, first_ts, data, step_us=100):
    """Feed one byte at a time with a constant intra-frame gap"""
    events = []
    for index, byte in enumerate(data):
        events.extend(replayer.feed(first_ts + index * step_us, byte))
    return events


class TestRtuLimits(unittest.TestCase):
    """Check the exact RTU timing computation"""

    def test_fixed_limits_above_19200(self):
        """Above 19200 baud the fixed 750/1750 us limits apply"""
        limits = calculate_rtu_limits(115200)
        self.assertEqual(limits.t1_5_us, 750)
        self.assertEqual(limits.t3_5_us, 1750)
        self.assertEqual(calculate_rtu_limits(38400).t1_5_us, 750)
        self.assertEqual(calculate_rtu_limits(19201).t3_5_us, 1750)

    def test_exact_limits_at_9600(self):
        """At 9600 baud a character lasts 11/9600 s and limits stay exact"""
        limits = calculate_rtu_limits(9600)
        self.assertEqual(limits.char_time_us, Fraction(6875, 6))
        self.assertEqual(limits.t1_5_us, Fraction(6875, 4))
        self.assertEqual(limits.t3_5_us, Fraction(48125, 12))

    def test_exact_limits_at_19200(self):
        """19200 baud still uses computed timing, not the fixed limits"""
        limits = calculate_rtu_limits(19200)
        self.assertEqual(limits.t1_5_us, Fraction(6875, 8))
        self.assertEqual(limits.t3_5_us, Fraction(48125, 24))


class TestFrameDecoding(unittest.TestCase):
    """Check valid frames, broadcasts and exception responses"""

    def test_valid_request_frame(self):
        """A frame followed by t3.5 silence is decoded"""
        frame = make_frame(17, [3, 0, 10, 0, 2])
        replayer = rtu_replay.RtuReplayer(115200)
        feed_bytes(replayer, 0, frame, step_us=100)
        events = replayer.flush(1000 + 1750)

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertIsInstance(event, FrameReceived)
        self.assertEqual(event.address, 17)
        self.assertEqual(event.function_code, 3)
        self.assertEqual(event.pdu, bytes(bytearray([3, 0, 10, 0, 2])))
        self.assertFalse(event.broadcast)
        self.assertFalse(event.exception)
        self.assertEqual(event.frame, frame)
        self.assertEqual(event.start_time, 0)
        self.assertEqual(event.end_time, 700)

    def test_broadcast_frame(self):
        """Address 0 is reported as a broadcast frame"""
        frame = make_frame(0, [6, 0, 5, 0, 0xFF])
        events = replay(
            [Arrival(0, frame)], baudrate=115200, end_timestamp_us=1750
        )
        self.assertEqual(len(events), 1)
        self.assertIsInstance(events[0], FrameReceived)
        self.assertTrue(events[0].broadcast)
        self.assertEqual(events[0].address, 0)

    def test_exception_response_frame(self):
        """Function code with 0x80 is reported as an exception response"""
        frame = make_frame(24, [0x83, 0x02])
        events = replay(
            [Arrival(1000, frame)], baudrate=115200, end_timestamp_us=2750
        )
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertIsInstance(event, FrameReceived)
        self.assertTrue(event.exception)
        self.assertEqual(event.function_code, 0x83)
        self.assertEqual(event.exception_code, 0x02)
        self.assertFalse(event.broadcast)

    def test_short_frame_is_malformed(self):
        """A silence-delimited frame below 4 bytes is malformed"""
        replayer = rtu_replay.RtuReplayer(115200)
        replayer.feed(0, bytes(bytearray([1, 3])))
        events = replayer.flush(1750)
        self.assertEqual(len(events), 1)
        self.assertIsInstance(events[0], MalformedFrame)
        self.assertEqual(events[0].frame, bytes(bytearray([1, 3])))

    def test_batch_arrivals_share_timestamp(self):
        """Several bytes logged at the same timestamp are accepted together"""
        frame = make_frame(1, [3, 0])
        events = replay(
            [(0, frame)], baudrate=115200, end_timestamp_us=1750
        )
        self.assertEqual(len(events), 1)
        self.assertIsInstance(events[0], FrameReceived)


class TestTimingBoundaries(unittest.TestCase):
    """Check gaps exactly on both sides of the t1.5 and t3.5 thresholds"""

    def test_t1_5_boundaries_high_speed(self):
        """749/750 us stay in-frame, 751 us interrupts the frame"""
        frame = make_frame(5, [3, 0, 10, 0, 2])

        for gap in (749, 750):
            replayer = rtu_replay.RtuReplayer(115200)
            events = feed_bytes(replayer, 0, frame[:4], step_us=100)
            events.extend(replayer.feed(300 + gap, frame[4:]))
            events.extend(replayer.flush(300 + gap + 100 + 1750))
            self.assertEqual(len(events), 1, "gap={0}".format(gap))
            self.assertIsInstance(events[0], FrameReceived)

        replayer = rtu_replay.RtuReplayer(115200)
        events = feed_bytes(replayer, 0, frame[:4], step_us=100)
        events.extend(replayer.feed(300 + 751, frame[4:]))
        events.extend(replayer.flush(300 + 751 + 100 + 1750))
        kinds = [type(event) for event in events]
        self.assertIn(FrameInterrupted, kinds)
        interrupted = [event for event in events if isinstance(event, FrameInterrupted)][0]
        self.assertEqual(interrupted.gap_us, 751)
        self.assertEqual(interrupted.limit_us, 750)

    def test_t1_5_exact_ratio_boundary(self):
        """Exact rational timestamps decide the t1.5 boundary without float"""
        frame = make_frame(1, [3, 0, 10, 0, 2])
        t1_5 = Fraction(6875, 4)

        replayer = rtu_replay.RtuReplayer(9600)
        events = feed_bytes(replayer, 0, frame[:4], step_us=Fraction(1, 10))
        events.extend(replayer.feed(Fraction(3, 10) + t1_5, frame[4:]))
        events.extend(replayer.flush(Fraction(3, 10) + t1_5 + 5000))
        self.assertEqual(len(events), 1)
        self.assertIsInstance(events[0], FrameReceived)

        replayer = rtu_replay.RtuReplayer(9600)
        events = feed_bytes(replayer, 0, frame[:4], step_us=Fraction(1, 10))
        over = t1_5 + Fraction(1, 1000)
        events.extend(replayer.feed(Fraction(3, 10) + over, frame[4:]))
        events.extend(replayer.flush(Fraction(3, 10) + over + 1000))
        self.assertIsInstance(events[0], FrameInterrupted)
        self.assertEqual(events[0].limit_us, t1_5)

    def test_t3_5_flush_boundaries(self):
        """Silence exactly at t3.5 closes; one microsecond less keeps the tail"""
        frame = make_frame(9, [4, 2, 0xCD, 0x01])
        last_byte_time = 600

        replayer = rtu_replay.RtuReplayer(115200)
        feed_bytes(replayer, 0, frame, step_us=100)
        events = replayer.flush(last_byte_time + 1750)
        self.assertEqual(len(events), 1)
        self.assertIsInstance(events[0], FrameReceived)

        replayer = rtu_replay.RtuReplayer(115200)
        feed_bytes(replayer, 0, frame, step_us=100)
        events = replayer.flush(last_byte_time + 1749)
        self.assertEqual(len(events), 1)
        self.assertIsInstance(events[0], UnterminatedTail)

    def test_explicit_flush_required(self):
        """Buffered bytes are not reported until flush is called"""
        frame = make_frame(9, [4, 2, 0xCD, 0x01])
        replayer = rtu_replay.RtuReplayer(115200)
        events = feed_bytes(replayer, 0, frame, step_us=100)
        self.assertEqual(events, [])

    def test_increasing_timestamp_enforced(self):
        """A timestamp going backwards is rejected"""
        replayer = rtu_replay.RtuReplayer(115200)
        replayer.feed(10, 1)
        with self.assertRaises(ValueError):
            replayer.feed(9, 2)


class TestCrcRecovery(unittest.TestCase):
    """Check CRC errors followed immediately by a good frame"""

    def test_bad_crc_then_good_crc(self):
        """A bad frame and the next good frame are separated by t3.5"""
        good1 = bytearray(make_frame(1, [3, 0, 10, 0, 2]))
        good1[-1] ^= 0xFF
        bad = bytes(good1)
        good2 = make_frame(2, [3, 0, 20, 0, 1])

        events = [Arrival(0, bad)]
        events.append(Arrival(2000, good2))
        decoded = replay(events, baudrate=115200, end_timestamp_us=4000)

        self.assertEqual(len(decoded), 2)
        self.assertIsInstance(decoded[0], CrcError)
        self.assertEqual(decoded[0].frame, bad)
        self.assertNotEqual(decoded[0].expected_crc, decoded[0].actual_crc)
        self.assertIsInstance(decoded[1], FrameReceived)
        self.assertEqual(decoded[1].frame, good2)
        self.assertEqual(decoded[1].address, 2)

    def test_bad_crc_without_silence_stays_one_bad_frame(self):
        """Frames touching each other without t3.5 merge into one CRC error"""
        good1 = bytearray(make_frame(1, [3, 0, 10, 0, 2]))
        good1[-1] ^= 0xFF
        bad = bytes(good1)
        good2 = make_frame(2, [3, 0, 20, 0, 1])

        decoded = replay(
            [Arrival(0, bad + good2)],
            baudrate=115200, end_timestamp_us=1750
        )
        self.assertEqual(len(decoded), 1)
        self.assertIsInstance(decoded[0], CrcError)
        self.assertEqual(decoded[0].frame, bad + good2)


class TestBaudrateDependentSplit(unittest.TestCase):
    """The same bytes split differently depending on the baudrate"""

    def test_same_stream_different_baudrates(self):
        """2 ms separates frames at 115200 but breaks them at 9600"""
        frame1 = make_frame(1, [3, 0, 10, 0, 2])
        frame2 = make_frame(2, [3, 0, 20, 0, 1])

        high = replay(
            [Arrival(0, frame1), Arrival(2000, frame2)],
            baudrate=115200, end_timestamp_us=4000
        )
        self.assertEqual(len(high), 2)
        self.assertIsInstance(high[0], FrameReceived)
        self.assertIsInstance(high[1], FrameReceived)
        self.assertEqual(high[0].frame, frame1)
        self.assertEqual(high[1].frame, frame2)

        low = replay(
            [Arrival(0, frame1), Arrival(2000, frame2)],
            baudrate=9600, end_timestamp_us=7000
        )
        self.assertEqual([type(event) for event in low],
                         [FrameInterrupted, FrameReceived])
        self.assertEqual(low[0].partial, frame1)
        self.assertEqual(low[1].frame, frame2)

    def test_gap_around_high_speed_t1_5(self):
        """800 us interrupts at 115200 but stays in-frame at 9600"""
        frame1 = make_frame(1, [3, 0, 10, 0, 2])
        frame2 = make_frame(2, [3, 0, 20, 0, 1])
        arrivals = [Arrival(0, frame1), Arrival(800, frame2)]

        high = replay(arrivals, baudrate=115200, end_timestamp_us=6000)
        self.assertEqual([type(event) for event in high],
                         [FrameInterrupted, FrameReceived])
        self.assertEqual(high[0].partial, frame1)
        self.assertEqual(high[1].frame, frame2)

        low = replay(arrivals, baudrate=9600, end_timestamp_us=6000)
        self.assertEqual(len(low), 1)
        self.assertIsInstance(low[0], CrcError)
        self.assertEqual(low[0].frame, frame1 + frame2)


if __name__ == '__main__':
    unittest.main(argv=sys.argv)
