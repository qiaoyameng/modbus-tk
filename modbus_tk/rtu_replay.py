#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python

 (C)2009 - Apidev - http://www.apidev.fr

 This is distributed under GNU LGPL license, see license.txt

Replay Modbus RTU frames from a "timestamp + byte" capture without any
serial hardware (and without importing pyserial).

The caller feeds arrival events (:meth:`RtuReplayer.feed`) and provides the
serial parameters (baudrate, bytesize, parity, stopbits). The replayer keeps
no wall clock of its own: every timestamp is supplied by the caller, expressed
in microseconds as an :class:`int` or as an exact :class:`~fractions.Fraction`
ratio. Nothing ever sleeps, so the same capture can be replayed deterministically.

Frame delimiting follows the Modbus over Serial Line specification:

* a gap longer than 1.5 character time inside a frame aborts the frame
  (:class:`FrameInterrupted`);
* a gap of at least 3.5 character times closes a complete frame which is then
  CRC checked;
* above 19200 bps the spec mandates fixed limits of 750 us / 1750 us.
"""

from fractions import Fraction

from modbus_tk import utils

# Event kinds
FRAME = "frame"
CRC_ERROR = "crc_error"
MALFORMED_FRAME = "malformed_frame"
FRAME_INTERRUPTED = "frame_interrupted"
FRAME_TRUNCATED = "frame_truncated"

# Event kinds kept as aliases for callers preferring the term "received"
FRAME_RECEIVED = FRAME

MICROSECONDS_PER_SECOND = 1000000

# Modbus over Serial Line: above 19200 bps the timers are fixed values
HIGH_SPEED_BAUD_THRESHOLD = 19200
HIGH_SPEED_T1_5_US = 750
HIGH_SPEED_T3_5_US = 1750

BROADCAST_ADDRESS = 0


def _as_fraction(value, name):
    """Convert an int/Fraction/float numeric to an exact Fraction."""
    if isinstance(value, Fraction):
        return value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("{0} must be an int, float or Fraction".format(name))
    return Fraction(value)


def _as_exact_time(value, name):
    """Timestamps/durations: int microseconds or an exact Fraction ratio.

    Floats are rejected on purpose so that threshold comparisons never suffer
    from binary floating point rounding.
    """
    if isinstance(value, bool) or not isinstance(value, (int, Fraction)):
        raise TypeError(
            "{0} must be an int number of microseconds or a Fraction".format(name)
        )
    return value if isinstance(value, Fraction) else Fraction(value)


class RtuTiming(object):
    """Character-time timers derived from the serial line parameters.

    All values are exposed in microseconds as exact :class:`Fraction`
    instances. ``inter_char`` is t1.5 (max intra-frame gap) and
    ``inter_frame`` is t3.5 (frame-to-frame silence).
    """

    def __init__(self, baudrate, bytesize=8, parity='N', stopbits=1,
                 inter_char_multiplier=1.5, inter_frame_multiplier=3.5):
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError("baudrate must be a positive integer")
        if bytesize not in (7, 8):
            raise ValueError("bytesize must be 7 or 8")
        parity = parity.upper()
        if parity not in ('N', 'E', 'O', 'M', 'S'):
            raise ValueError("parity must be one of N, E, O, M, S")
        if stopbits not in (1, 1.5, 2):
            raise ValueError("stopbits must be 1, 1.5 or 2")

        multiplier_char = _as_fraction(inter_char_multiplier, "inter_char_multiplier")
        multiplier_frame = _as_fraction(inter_frame_multiplier, "inter_frame_multiplier")
        if multiplier_char <= 0 or multiplier_frame <= 0:
            raise ValueError("time multipliers must be positive")

        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.inter_char_multiplier = multiplier_char
        self.inter_frame_multiplier = multiplier_frame

        # A Modbus RTU character spans 11 bit times per the over-serial-line
        # specification (start + 8 data + parity slot + stop). Without parity
        # the parity slot is carried as an extra stop bit, so all the standard
        # 8-bit configurations below time out to exactly 11 bits; non-standard
        # formats such as 7-bit frames are still accounted for.
        parity_slot = 0 if (parity == 'N' and Fraction(stopbits) >= 2) else 1
        bits_per_char = 1 + bytesize + parity_slot + Fraction(stopbits)
        char_time_us = MICROSECONDS_PER_SECOND * bits_per_char / baudrate

        if baudrate > HIGH_SPEED_BAUD_THRESHOLD:
            # Above 19200 bps the spec mandates fixed timers, whatever the
            # actual character frame length is.
            self.char_time = Fraction(HIGH_SPEED_T3_5_US) / Fraction(7, 2)
            self.inter_char = Fraction(HIGH_SPEED_T1_5_US)
            self.inter_frame = Fraction(HIGH_SPEED_T3_5_US)
        else:
            self.char_time = char_time_us
            self.inter_char = multiplier_char * char_time_us
            self.inter_frame = multiplier_frame * char_time_us
        self.bits_per_char = bits_per_char

    def classify_gap(self, gap_us):
        """Return ``'frame'``, ``'interrupt'`` or ``'same'`` for a gap.

        ``same``  : gap <= t1.5, the next byte still belongs to the frame
        ``frame`` : gap >= t3.5, the previous frame is complete
        ``interrupt`` : t1.5 < gap < t3.5, the partial frame is aborted
        """
        gap = _as_exact_time(gap_us, "gap")
        if gap >= self.inter_frame:
            return 'frame'
        if gap > self.inter_char:
            return 'interrupt'
        return 'same'


class RtuEvent(object):
    """Base class for events emitted while replaying a capture."""

    kind = None

    def __init__(self, data, start_time, end_time):
        self.data = bytes(data)
        self.start_time = start_time
        self.end_time = end_time

    @property
    def address(self):
        """Slave address (first byte) or None for an empty event."""
        return self.data[0] if self.data else None

    def __repr__(self):
        return "{0}(data={1!r})".format(type(self).__name__, self.data)

    def __eq__(self, other):
        return (
            type(self) is type(other)
            and self.data == other.data
            and self.start_time == other.start_time
            and self.end_time == other.end_time
        )

    def __ne__(self, other):
        return not self.__eq__(other)


class FrameReceived(RtuEvent):
    """A complete frame whose CRC is valid."""

    kind = FRAME

    def __init__(self, data, start_time, end_time, pdu, exception_code=None):
        super(FrameReceived, self).__init__(data, start_time, end_time)
        self.pdu = bytes(pdu)
        self.function_code = pdu[0] if pdu else None
        self.is_broadcast = bool(self.data) and self.data[0] == BROADCAST_ADDRESS
        self.is_exception = exception_code is not None
        self.exception_code = exception_code


class CrcError(RtuEvent):
    """A complete (timing-wise) frame whose CRC does not match."""

    kind = CRC_ERROR

    def __init__(self, data, start_time, end_time, expected_crc=None, actual_crc=None):
        super(CrcError, self).__init__(data, start_time, end_time)
        self.expected_crc = expected_crc
        self.actual_crc = actual_crc


class MalformedFrame(RtuEvent):
    """A frame closed by timing but too short to contain a CRC."""

    kind = MALFORMED_FRAME


class FrameInterrupted(RtuEvent):
    """Bytes abandoned because the intra-frame gap exceeded t1.5."""

    kind = FRAME_INTERRUPTED

    def __init__(self, data, start_time, end_time, gap=None):
        super(FrameInterrupted, self).__init__(data, start_time, end_time)
        self.gap = gap


class FrameTruncated(RtuEvent):
    """Bytes still pending when the capture ended without a closing silence."""

    kind = FRAME_TRUNCATED


CRC_LITTLE_ENDIAN = "little"
CRC_BIG_ENDIAN = "big"


def _wire_to_register(pair, byte_order):
    """Interpret two on-wire CRC bytes as a register value."""
    if byte_order == CRC_LITTLE_ENDIAN:
        return pair[0] | (pair[1] << 8)
    return (pair[0] << 8) | pair[1]


def crc_wire_bytes(frame_without_crc, byte_order=CRC_LITTLE_ENDIAN):
    """Return the on-wire CRC bytes for a frame (without its CRC).

    The Modbus RTU standard sends the CRC low byte first (``"little"``).
    ``"big"`` is accepted to interoperate with stacks which historically emit
    the CRC high byte first.
    """
    crc = utils.calculate_crc(bytes(frame_without_crc))
    little = bytes((crc & 0xFF, (crc >> 8) & 0xFF))
    return little if byte_order == CRC_LITTLE_ENDIAN else little[::-1]


def append_crc(frame_without_crc, byte_order=CRC_LITTLE_ENDIAN):
    """Append a Modbus RTU CRC to a byte sequence."""
    data = bytes(frame_without_crc)
    return data + crc_wire_bytes(data, byte_order)


class RtuReplayer(object):
    """State machine turning timestamped byte arrivals into RTU frames.

    Times are entirely provided by the caller in microseconds (int or
    Fraction); no real time is measured and no sleep ever happens.
    """

    def __init__(self, baudrate=9600, bytesize=8, parity='N', stopbits=1,
                 inter_char_multiplier=1.5, inter_frame_multiplier=3.5,
                 timing=None, crc_byte_order=CRC_LITTLE_ENDIAN):
        if timing is None:
            timing = RtuTiming(
                baudrate, bytesize, parity, stopbits,
                inter_char_multiplier, inter_frame_multiplier
            )
        elif not isinstance(timing, RtuTiming):
            raise TypeError("timing must be an RtuTiming instance")
        if crc_byte_order not in (CRC_LITTLE_ENDIAN, CRC_BIG_ENDIAN):
            raise ValueError("crc_byte_order must be 'little' or 'big'")
        self.timing = timing
        self.crc_byte_order = crc_byte_order
        self._buffer = bytearray()
        self._frame_start = None
        self._last_time = None

    def reset(self):
        """Drop any pending bytes and restart the state machine."""
        self._buffer = bytearray()
        self._frame_start = None
        self._last_time = None

    def feed(self, timestamp, data=b''):
        """Feed one or more arrival events and return emitted events.

        ``timestamp`` is the arrival time in microseconds (int or Fraction).
        ``data`` is one byte (int 0-255) or an iterable of bytes; it may also
        be a single ``(timestamp, byte)`` pair, in which case ``data`` must be
        omitted.
        """
        events = []

        if isinstance(timestamp, tuple) and data == b'':
            timestamp, data = timestamp

        timestamp = _as_exact_time(timestamp, "timestamp")
        if self._last_time is not None and timestamp < self._last_time:
            raise ValueError("timestamps must be monotonic")

        if isinstance(data, int):
            if not 0 <= data <= 255:
                raise ValueError("byte value out of range: {0}".format(data))
            byte_values = (data,)
        else:
            byte_values = tuple(data)
            for byte_value in byte_values:
                if isinstance(byte_value, int) and 0 <= byte_value <= 255:
                    continue
                raise ValueError("data must contain bytes in 0..255")

        # Bytes delivered together share the same arrival instant: they are
        # contiguous characters (gap == 0), only the first byte closes a gap.
        for byte_value in byte_values:
            self._absorb(timestamp, byte_value, events)

        return events

    def _absorb(self, timestamp, byte_value, events):
        if self._buffer:
            gap = timestamp - self._last_time
            zone = self.timing.classify_gap(gap)
            if zone == 'frame':
                events.append(self._close_frame(self._frame_start, self._last_time))
            elif zone == 'interrupt':
                events.append(
                    FrameInterrupted(self._buffer, self._frame_start, self._last_time, gap)
                )
                self._buffer = bytearray()
            # zone == 'same': keep accumulating

        if not self._buffer:
            self._frame_start = timestamp
        self._buffer.append(byte_value)
        self._last_time = timestamp

    def _close_frame(self, start_time, end_time):
        frame = bytes(self._buffer)
        self._buffer = bytearray()
        return self._decode_frame(frame, start_time, end_time, self.crc_byte_order)

    @staticmethod
    def _decode_frame(frame, start_time, end_time,
                      byte_order=CRC_LITTLE_ENDIAN):
        if len(frame) < 4:
            return MalformedFrame(frame, start_time, end_time)

        expected = crc_wire_bytes(frame[:-2], byte_order)
        actual = bytes(frame[-2:])
        if actual != expected:
            return CrcError(
                frame, start_time, end_time,
                _wire_to_register(expected, byte_order),
                _wire_to_register(actual, byte_order)
            )

        pdu = frame[1:-2]
        exception_code = None
        if pdu and (pdu[0] & 0x80) and len(pdu) >= 2:
            exception_code = pdu[1]
        return FrameReceived(frame, start_time, end_time, pdu, exception_code)

    def flush(self, at_time=None):
        """End the capture explicitly and return events for the pending tail.

        ``at_time`` is the capture end time in microseconds (int or Fraction).
        The tail is judged from the silence observed after the last byte:

        * silence >= t3.5: the frame reached its boundary and is fully decoded;
        * t1.5 < silence < t3.5: the partial frame is reported as interrupted;
        * silence <= t1.5, or ``at_time`` omitted: the boundary was never
          reached and a :class:`FrameTruncated` event is emitted. The tail is
          therefore never silently promoted to a complete frame.
        """
        events = []
        if self._buffer:
            if at_time is None:
                events.append(FrameTruncated(self._buffer, self._frame_start, self._last_time))
            else:
                at_time = _as_exact_time(at_time, "at_time")
                if at_time < self._last_time:
                    raise ValueError("flush time must not be before the last byte")
                gap = at_time - self._last_time
                zone = self.timing.classify_gap(gap)
                if zone == 'frame':
                    events.append(self._close_frame(self._frame_start, self._last_time))
                elif zone == 'interrupt':
                    events.append(
                        FrameInterrupted(self._buffer, self._frame_start, self._last_time, gap)
                    )
                else:
                    events.append(
                        FrameTruncated(self._buffer, self._frame_start, self._last_time)
                    )
        self.reset()
        return events


def replay(events, baudrate=9600, bytesize=8, parity='N', stopbits=1,
           inter_char_multiplier=1.5, inter_frame_multiplier=3.5, timing=None,
           end_time=None, crc_byte_order=CRC_LITTLE_ENDIAN):
    """Replay an iterable of ``(timestamp_us, byte)`` events in one call.

    ``end_time`` is forwarded to :meth:`RtuReplayer.flush`; when omitted a
    final tail without an observed closing silence surfaces as
    :class:`FrameTruncated` instead of a complete frame.
    """
    replayer = RtuReplayer(
        baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits,
        inter_char_multiplier=inter_char_multiplier,
        inter_frame_multiplier=inter_frame_multiplier, timing=timing,
        crc_byte_order=crc_byte_order
    )
    result = []
    for event in events:
        result.extend(replayer.feed(event))
    result.extend(replayer.flush(end_time))
    return result
