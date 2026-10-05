#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python

 (C)2009 - Apidev - http://www.apidev.fr

 This is distributed under GNU LGPL license, see license.txt

Tests for the serial-free RTU capture replayer (modbus_tk.rtu_replay).
"""

import ast
import unittest
from fractions import Fraction

from modbus_tk import rtu_replay
from modbus_tk.rtu_replay import (
    RtuTiming, RtuReplayer, append_crc, replay,
    crc_wire_bytes, CRC_LITTLE_ENDIAN, CRC_BIG_ENDIAN,
    FrameReceived, CrcError, MalformedFrame, FrameInterrupted, FrameTruncated,
)
from modbus_tk import utils


def frame(address, pdu_bytes):
    """Build a Modbus RTU frame with a correct CRC."""
    return append_crc(bytes([address]) + bytes(pdu_bytes))


class TestRtuTiming(unittest.TestCase):
    """t1.5 / t3.5 derivation from the serial line parameters."""

    def test_spec_table_low_speed(self):
        """9600 8N1 gives the canonical 1718.75 / 4010.416666 us values."""
        timing = RtuTiming(9600)
        self.assertEqual(timing.bits_per_char, 11)
        self.assertEqual(timing.inter_char, Fraction(13750, 8))
        self.assertEqual(timing.inter_frame, Fraction(48125, 12))

    def test_all_standard_eight_bit_formats_time_as_11_bits(self):
        """Parity slot is carried as an extra stop bit when parity is none."""
        for bytesize, parity, stopbits in (
                (8, 'N', 1), (8, 'N', 2), (8, 'E', 1), (8, 'O', 1)):
            timing = RtuTiming(9600, bytesize, parity, stopbits)
            self.assertEqual(timing.bits_per_char, 11)

    def test_high_speed_uses_fixed_limits(self):
        """Above 19200 bps t1.5=750us and t3.5=1750us, independent of format."""
        for baudrate in (19201, 38400, 115200):
            timing = RtuTiming(baudrate)
            self.assertEqual(timing.inter_char, 750)
            self.assertEqual(timing.inter_frame, 1750)
        self.assertEqual(RtuTiming(19200).inter_char, Fraction(13750, 16))

    def test_classify_gap_boundaries(self):
        """Exactly t1.5 stays in-frame, exactly t3.5 closes the frame."""
        timing = RtuTiming(115200)
        self.assertEqual(timing.classify_gap(0), 'same')
        self.assertEqual(timing.classify_gap(750), 'same')
        self.assertEqual(timing.classify_gap(751), 'interrupt')
        self.assertEqual(timing.classify_gap(1749), 'interrupt')
        self.assertEqual(timing.classify_gap(1750), 'frame')

    def test_classify_gap_fraction_boundaries(self):
        """Exact ratios remove any ambiguity at the low-speed thresholds."""
        timing = RtuTiming(9600)
        self.assertEqual(timing.classify_gap(timing.inter_char), 'same')
        self.assertEqual(timing.classify_gap(timing.inter_char + Fraction(1, 1000)),
                         'interrupt')
        self.assertEqual(timing.classify_gap(timing.inter_frame - Fraction(1, 1000)),
                         'interrupt')
        self.assertEqual(timing.classify_gap(timing.inter_frame), 'frame')

    def test_invalid_parameters(self):
        self.assertRaises(ValueError, RtuTiming, 0)
        self.assertRaises(ValueError, RtuTiming, -9600)
        self.assertRaises(ValueError, RtuTiming, 9600, bytesize=6)
        self.assertRaises(ValueError, RtuTiming, 9600, parity='X')
        self.assertRaises(ValueError, RtuTiming, 9600, stopbits=3)
        self.assertRaises(ValueError, RtuTiming, 9600, inter_char_multiplier=0)


class TestFrameDecoding(unittest.TestCase):
    """Frames closed by t3.5 are CRC checked and decoded."""

    def setUp(self):
        self.replayer = RtuReplayer(115200)

    def feed_frame(self, data, start=0, closing_gap=2000):
        events = self.replayer.feed(start, data)
        events += self.replayer.flush(start + (len(data) - 1) * 10 + closing_gap)
        return events

    def test_valid_response(self):
        data = frame(1, [3, 4, 0, 1, 0, 2])
        event, = self.feed_frame(data)
        self.assertIsInstance(event, FrameReceived)
        self.assertEqual(event.address, 1)
        self.assertEqual(event.function_code, 3)
        self.assertEqual(event.pdu, bytes([3, 4, 0, 1, 0, 2]))
        self.assertFalse(event.is_broadcast)
        self.assertFalse(event.is_exception)

    def test_broadcast_request(self):
        data = frame(0, [6, 0, 1, 0, 0x10])
        event, = self.feed_frame(data)
        self.assertIsInstance(event, FrameReceived)
        self.assertTrue(event.is_broadcast)
        self.assertEqual(event.address, 0)

    def test_exception_response(self):
        data = frame(5, [0x83, 0x02])
        event, = self.feed_frame(data)
        self.assertIsInstance(event, FrameReceived)
        self.assertTrue(event.is_exception)
        self.assertEqual(event.function_code, 0x83)
        self.assertEqual(event.exception_code, 0x02)

    def test_crc_error(self):
        data = frame(1, [3, 4, 0, 1, 0, 2])
        corrupted = data[:-1] + bytes([data[-1] ^ 0xFF])
        event, = self.feed_frame(corrupted)
        self.assertIsInstance(event, CrcError)
        self.assertEqual(event.data, corrupted)
        self.assertNotEqual(event.expected_crc, event.actual_crc)

    def test_too_short_frame_is_malformed(self):
        event, = self.feed_frame(bytes([1, 2]))
        self.assertIsInstance(event, MalformedFrame)

    def test_crc_error_followed_by_good_frame(self):
        """A bad frame and the immediately following good frame are distinct."""
        good = frame(1, [3, 4, 0, 1, 0, 2])
        bad = good[:-1] + bytes([good[-1] ^ 0xFF])
        events = replay([(0, bad), (2000, good)], baudrate=115200, end_time=12000)
        self.assertEqual(2, len(events))
        self.assertIsInstance(events[0], CrcError)
        self.assertEqual(events[0].data, bad)
        self.assertIsInstance(events[1], FrameReceived)
        self.assertEqual(events[1].data, good)


class TestIntraFrameInterruption(unittest.TestCase):
    """A gap above t1.5 but below t3.5 aborts the pending partial frame."""

    def test_high_speed_interrupt_zone(self):
        data = frame(1, [3, 4, 0, 1, 0, 2])
        for gap in (751, 1749):
            replayer = RtuReplayer(115200)
            events = replayer.feed(0, data[:3])
            events += replayer.feed(gap, data[3:])
            self.assertEqual(1, len(events), gap)
            self.assertIsInstance(events[0], FrameInterrupted)
            self.assertEqual(events[0].data, data[:3])
            self.assertEqual(events[0].gap, gap)

    def test_gap_at_t1_5_does_not_interrupt(self):
        data = frame(1, [3, 4, 0, 1, 0, 2])
        replayer = RtuReplayer(115200)
        self.assertEqual([], replayer.feed(0, data[:3]))
        self.assertEqual([], replayer.feed(750, data[3:]))
        event, = replayer.flush(8000)
        self.assertIsInstance(event, FrameReceived)


class TestBaudrateDependentSplitting(unittest.TestCase):
    """Identical bytes must be segmented differently per baudrate."""

    def _capture(self, first, second, gap):
        events = [(index * 10, byte_value)
                  for index, byte_value in enumerate(first)]
        offset = (len(first) - 1) * 10 + gap
        events += [(offset + index * 10, byte_value)
                   for index, byte_value in enumerate(second)]
        return events

    def test_same_bytes_different_baudrate(self):
        first = frame(1, [3, 4, 0, 1, 0, 2])
        second = frame(2, [3, 4, 0, 9, 0, 8])

        # A 1000us pause: below t1.5 (1718.75us) at 9600 but inside the
        # (750, 1750) interruption zone at 115200.
        capture = self._capture(first, second, 1000)
        low_speed = replay(capture, baudrate=9600,
                           end_time=capture[-1][0] + 10000)
        high_speed = replay(capture, baudrate=115200,
                            end_time=capture[-1][0] + 10000)

        self.assertEqual(1, len(low_speed))
        self.assertIsInstance(low_speed[0], CrcError)
        self.assertEqual(low_speed[0].data, first + second)

        self.assertEqual(2, len(high_speed))
        self.assertIsInstance(high_speed[0], FrameInterrupted)
        self.assertEqual(high_speed[0].data, first)
        self.assertIsInstance(high_speed[1], FrameReceived)
        self.assertEqual(high_speed[1].data, second)

    def test_boundary_gap_is_baudrate_dependent(self):
        """1750us closes a frame at 115200 but only interrupts at 9600."""
        first = frame(1, [3, 4, 0, 1, 0, 2])
        second = frame(2, [3, 4, 0, 9, 0, 8])
        capture = self._capture(first, second, 1750)
        high_speed = replay(capture, baudrate=115200,
                            end_time=capture[-1][0] + 10000)
        low_speed = replay(capture, baudrate=9600,
                           end_time=capture[-1][0] + 10000)
        self.assertEqual([FrameReceived, FrameReceived],
                         [type(e) for e in high_speed])
        self.assertEqual([FrameInterrupted, FrameReceived],
                         [type(e) for e in low_speed])


class TestFlush(unittest.TestCase):
    """End of data is explicit; a tail below the boundary is not a frame."""

    def setUp(self):
        self.data = frame(1, [3, 4, 0, 1, 0, 2])

    def test_flush_without_silence_is_truncated(self):
        replayer = RtuReplayer(115200)
        replayer.feed(0, self.data)
        event, = replayer.flush()
        self.assertIsInstance(event, FrameTruncated)
        self.assertEqual(event.data, self.data)
        self.assertEqual([], replayer.flush())

    def test_flush_below_t1_5_is_truncated(self):
        replayer = RtuReplayer(115200)
        replayer.feed(0, self.data)
        event, = replayer.flush(750)
        self.assertIsInstance(event, FrameTruncated)

    def test_flush_in_interrupt_zone(self):
        replayer = RtuReplayer(115200)
        replayer.feed(0, self.data)
        event, = replayer.flush(1000)
        self.assertIsInstance(event, FrameInterrupted)
        self.assertEqual(event.gap, 1000)

    def test_flush_at_t3_5_decodes_frame(self):
        replayer = RtuReplayer(115200)
        replayer.feed(0, self.data)
        event, = replayer.flush(1750)
        self.assertIsInstance(event, FrameReceived)


class TestFeedAndInputs(unittest.TestCase):
    """Arrival-event input shapes and time representation rules."""

    def setUp(self):
        self.data = frame(1, [3, 4, 0, 1, 0, 2])

    def test_feed_forms(self):
        per_byte = RtuReplayer(115200)
        for index, byte_value in enumerate(self.data):
            self.assertEqual([], per_byte.feed(index * 10, byte_value))
        event, = per_byte.flush((len(self.data) - 1) * 10 + 1750)
        self.assertIsInstance(event, FrameReceived)

        batched = RtuReplayer(115200)
        self.assertEqual([], batched.feed(0, self.data))
        self.assertIsInstance(batched.flush(1750)[0], FrameReceived)

        tuple_form = RtuReplayer(115200)
        self.assertEqual([], tuple_form.feed((0, self.data)))
        self.assertIsInstance(tuple_form.flush(1750)[0], FrameReceived)

    def test_fraction_timestamps(self):
        timing = RtuTiming(9600)
        replayer = RtuReplayer(timing=timing)
        replayer.feed(Fraction(0), self.data)
        event, = replayer.flush(timing.inter_frame)
        self.assertIsInstance(event, FrameReceived)

    def test_float_timestamp_rejected(self):
        replayer = RtuReplayer()
        self.assertRaises(TypeError, replayer.feed, 1.5, 1)
        replayer.feed(0, self.data)
        self.assertRaises(TypeError, replayer.flush, 1.5)

    def test_non_monotonic_timestamp_rejected(self):
        replayer = RtuReplayer(115200)
        replayer.feed(10, self.data)
        self.assertRaises(ValueError, replayer.feed, 9, 0)

    def test_byte_range(self):
        replayer = RtuReplayer()
        self.assertRaises(ValueError, replayer.feed, 0, 256)

    def test_no_sleep_or_pyserial_dependency(self):
        """Only stdlib + modbus_tk.utils are imported and nothing sleeps."""
        source = open(rtu_replay.__file__).read()
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                root = node.names[0].name.split('.')[0]
                self.assertNotEqual(root, 'serial')
            elif isinstance(node, ast.ImportFrom) and node.module:
                self.assertNotEqual(node.module.split('.')[0], 'serial')
        for node in ast.walk(tree):
            self.assertFalse(
                isinstance(node, ast.Attribute) and node.attr == 'sleep'
            )


class TestCrcEncoding(unittest.TestCase):
    """CRC vector, standard wire order and optional big-endian interop."""

    def test_standard_check_vector(self):
        # The standard Modbus CRC16 of "123456789" is the register value 0x4B37
        self.assertEqual(utils.calculate_crc(b"123456789"), 0x374B)

    def test_default_wire_order_is_low_byte_first(self):
        body = bytes([1, 3, 4, 0, 1, 0, 2])
        wire = append_crc(body)
        self.assertEqual(wire[-2:], bytes([0x32, 0x2A]))

    def test_crc_error_reports_registers(self):
        good = append_crc(bytes([1, 3, 4, 0, 1, 0, 2]))
        bad = good[:-1] + bytes([good[-1] ^ 0xFF])
        event, = replay([(0, bad)], baudrate=115200, end_time=2000)
        self.assertIsInstance(event, CrcError)
        self.assertEqual(event.expected_crc, 0x2A32)
        self.assertEqual(event.actual_crc, 0xD532)

    def test_big_endian_crc_interop(self):
        """Frames built like the historical RtuQuery (pack >H) still decode."""
        import struct
        body = bytes([17]) + struct.pack(">BBHHH", 3, 0, 10, 0, 1)
        wire = body + struct.pack(">H", utils.calculate_crc(body))
        self.assertEqual(crc_wire_bytes(body, CRC_BIG_ENDIAN), wire[-2:])

        replayer = RtuReplayer(115200, crc_byte_order=CRC_BIG_ENDIAN)
        self.assertEqual([], replayer.feed(0, wire))
        event, = replayer.flush(2000)
        self.assertIsInstance(event, FrameReceived)
        self.assertEqual(event.address, 17)


if __name__ == '__main__':
    unittest.main()
