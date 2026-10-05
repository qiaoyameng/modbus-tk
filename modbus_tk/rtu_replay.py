#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python

 (C)2009 - Luc Jean - luc.jean@gmail.com
 (C)2009 - Apidev - http://www.apidev.fr

 This is distributed under GNU LGPL license, see license.txt

Offline replayer for Modbus RTU captures.

When a serial link fails on site, the only evidence is often a log of
(timestamp, received bytes) pairs. This module turns such a log back into
frames without any serial hardware: the caller feeds the byte arrival events
and the serial line parameters, and the replayer applies the RTU timing rules
from the "MODBUS over serial line" specification:

* a silence of at least 3.5 character times (t3.5) closes a frame;
* a gap of more than 1.5 character times (t1.5) between two characters of the
  same frame aborts the partial frame;
* above 19200 baud the fixed limits of 1750 us and 750 us are used.

Time is always supplied by the caller, never measured and never slept on.
Timestamps are expressed in microseconds as integers or as exact rational
numbers (for example fractions.Fraction), so boundary cases can be checked
with exact arithmetic.
"""

from collections import namedtuple
from fractions import Fraction
import struct

from modbus_tk.utils import calculate_crc

#: Modbus broadcast slave address
BROADCAST_ADDRESS = 0

#: Above this baudrate the specification mandates fixed timing limits
HIGH_SPEED_BAUDRATE = 19200

#: Fixed t1.5 limit, in microseconds, above 19200 baud
HIGH_SPEED_T1_5_US = 750

#: Fixed t3.5 limit, in microseconds, above 19200 baud
HIGH_SPEED_T3_5_US = 1750

#: Character time used by the specification (1 start + 8 data + parity/stop)
SPEC_BITS_PER_CHAR = 11

MICROSECONDS_PER_SECOND = 1000000

#: Serial line parameters of the capture (pyserial-like)
SerialParams = namedtuple("SerialParams", ("baudrate", "bytesize", "parity", "stopbits"))

#: Exact RTU timing limits, in microseconds
RtuLimits = namedtuple("RtuLimits", ("char_time_us", "t1_5_us", "t3_5_us"))

#: A byte (or a batch of bytes) received at a given time, in microseconds
Arrival = namedtuple("Arrival", ("timestamp_us", "data"))


class RtuEvent(object):
    """Base class for events emitted while replaying an RTU capture"""

    #: short identifier of the event kind, overridden by subclasses
    event_type = "event"

    def __init__(self, timestamp, start_time=None, end_time=None):
        #: when the event was detected, in microseconds
        self.timestamp = timestamp
        #: timestamp of the first byte belonging to the event
        self.start_time = start_time if start_time is not None else timestamp
        #: timestamp of the last byte belonging to the event
        self.end_time = end_time if end_time is not None else timestamp

    def __repr__(self):
        return "{0}@{1}".format(self.__class__.__name__, self.timestamp)


class FrameReceived(RtuEvent):
    """A complete RTU frame with a valid CRC"""

    event_type = "frame"

    def __init__(self, timestamp, frame, address, function_code, pdu, crc,
                 broadcast, exception, exception_code=None,
                 start_time=None, end_time=None):
        super(FrameReceived, self).__init__(timestamp, start_time, end_time)
        #: full RTU ADU (address + PDU + CRC)
        self.frame = bytes(frame)
        #: slave address byte
        self.address = address
        #: function code byte (kept as received, including the exception flag)
        self.function_code = function_code
        #: PDU bytes, without address and CRC
        self.pdu = bytes(pdu)
        #: CRC value extracted from the frame
        self.crc = crc
        #: True when the frame targets address 0
        self.broadcast = broadcast
        #: True when the function code carries the exception flag (0x80)
        self.exception = exception
        #: exception code byte, when exception is True
        self.exception_code = exception_code

    def __repr__(self):
        return "FrameReceived@{0} addr={1} fc=0x{2:02X}{3}".format(
            self.timestamp, self.address, self.function_code,
            " broadcast" if self.broadcast else (" exception" if self.exception else "")
        )


class CrcError(RtuEvent):
    """A complete-looking frame whose CRC does not match"""

    event_type = "crc_error"

    def __init__(self, timestamp, frame, expected_crc, actual_crc,
                 start_time=None, end_time=None):
        super(CrcError, self).__init__(timestamp, start_time, end_time)
        #: received bytes, including the wrong CRC
        self.frame = bytes(frame)
        #: address byte, when at least one byte was available
        self.address = frame[0] if len(frame) else None
        #: CRC computed over the frame body
        self.expected_crc = expected_crc
        #: CRC carried by the frame
        self.actual_crc = actual_crc

    def __repr__(self):
        return "CrcError@{0} expected=0x{1:04X} got=0x{2:04X}".format(
            self.timestamp, self.expected_crc, self.actual_crc
        )


class FrameInterrupted(RtuEvent):
    """Partial frame dropped because two characters were more than t1.5 apart"""

    event_type = "frame_interrupted"

    def __init__(self, timestamp, partial, gap_us, limit_us,
                 start_time=None, end_time=None):
        super(FrameInterrupted, self).__init__(timestamp, start_time, end_time)
        #: bytes accumulated before the excessive gap
        self.partial = bytes(partial)
        #: observed gap, in microseconds
        self.gap_us = gap_us
        #: t1.5 limit that was exceeded, in microseconds
        self.limit_us = limit_us

    def __repr__(self):
        return "FrameInterrupted@{0} gap={1} > t1.5={2}".format(
            self.timestamp, self.gap_us, self.limit_us
        )


class MalformedFrame(RtuEvent):
    """A frame delimited by silence but too small to carry an RTU ADU"""

    event_type = "malformed_frame"

    def __init__(self, timestamp, frame, reason, start_time=None, end_time=None):
        super(MalformedFrame, self).__init__(timestamp, start_time, end_time)
        #: received bytes
        self.frame = bytes(frame)
        #: why the frame is rejected
        self.reason = reason

    def __repr__(self):
        return "MalformedFrame@{0} {1}".format(self.timestamp, self.reason)


class UnterminatedTail(RtuEvent):
    """Bytes left over at end of data without a t3.5 closing silence"""

    event_type = "unterminated_tail"

    def __init__(self, timestamp, data, start_time=None, end_time=None):
        super(UnterminatedTail, self).__init__(timestamp, start_time, end_time)
        #: bytes that must not be interpreted as a complete frame
        self.data = bytes(data)

    def __repr__(self):
        return "UnterminatedTail@{0} {1} byte(s)".format(
            self.timestamp, len(self.data)
        )


def calculate_rtu_limits(baudrate, interchar_multiplier=1.5,
                         interframe_multiplier=3.5):
    """Return the exact RTU timing limits, in microseconds.

    At 19200 baud and below a character time is 11 bit times, as mandated by
    the specification. Above 19200 baud the fixed limits of 750 us (t1.5) and
    1750 us (t3.5) apply. Results are fractions.Fraction values so that gaps
    landing exactly on a boundary compare with no floating point error.
    """
    if baudrate <= 0:
        raise ValueError("baudrate must be positive, got {0}".format(baudrate))

    interchar_multiplier = Fraction(interchar_multiplier)
    interframe_multiplier = Fraction(interframe_multiplier)

    if baudrate > HIGH_SPEED_BAUDRATE:
        char_time_us = Fraction(HIGH_SPEED_T3_5_US) / interframe_multiplier
        t1_5_us = Fraction(HIGH_SPEED_T1_5_US)
        t3_5_us = Fraction(HIGH_SPEED_T3_5_US)
    else:
        char_time_us = Fraction(
            SPEC_BITS_PER_CHAR * MICROSECONDS_PER_SECOND, baudrate
        )
        t1_5_us = char_time_us * interchar_multiplier
        t3_5_us = char_time_us * interframe_multiplier

    return RtuLimits(char_time_us, t1_5_us, t3_5_us)


def _serial_attr(serial, name, default):
    """Read a parameter from a pyserial-like object, with a fallback"""
    if serial is None:
        return default
    return getattr(serial, name, default)


class RtuReplayer(object):
    """Reassemble RTU frames from timestamped byte arrival events.

    Time is injected through the feed() method: the replayer never reads the
    system clock and never sleeps. Timestamps are in microseconds and may be
    integers or exact rational numbers (fractions.Fraction).
    """

    #: minimum length of an RTU ADU: address, function code, 2 CRC bytes
    MIN_FRAME_LEN = 4

    def __init__(self, baudrate=None, serial=None, bytesize=8, parity="N",
                 stopbits=1, interchar_multiplier=1.5,
                 interframe_multiplier=3.5):
        """
        baudrate: line speed in baud (mandatory unless serial provides it)
        serial: optional pyserial-like object exposing baudrate, bytesize,
                parity and stopbits attributes
        bytesize, parity, stopbits: serial line parameters (recorded with the
                frames; the specification character time is 11 bit times)
        interchar_multiplier, interframe_multiplier: t1.5/t3.5 multipliers
        """
        baudrate = baudrate if baudrate is not None else _serial_attr(serial, "baudrate", None)
        if not baudrate:
            raise ValueError("a baudrate or a serial object is required")

        bytesize = _serial_attr(serial, "bytesize", bytesize)
        parity = _serial_attr(serial, "parity", parity)
        stopbits = _serial_attr(serial, "stopbits", stopbits)

        self.serial_params = SerialParams(baudrate, bytesize, parity, stopbits)
        self.limits = calculate_rtu_limits(
            baudrate,
            interchar_multiplier=interchar_multiplier,
            interframe_multiplier=interframe_multiplier,
        )
        self.reset()

    def reset(self):
        """Drop every buffered byte and event, ready for a new capture"""
        self._buffer = bytearray()
        self._start_time = None
        self._last_time = None

    def feed(self, timestamp_us, data=b""):
        """Feed one byte or a batch of bytes sharing the same arrival time.

        Returns the list of events detected while processing the bytes.
        """
        if self._last_time is not None and timestamp_us < self._last_time:
            raise ValueError("timestamps must be increasing")

        if isinstance(data, int):
            data = bytes(bytearray((data,)))
        data = bytes(data)

        events = []
        for byte in data:
            events.extend(self._feed_byte(timestamp_us, byte))
        return events

    def flush(self, end_timestamp_us=None):
        """Close the capture.

        When the time elapsed since the last byte reaches t3.5 the buffered
        bytes are decoded as a frame. Otherwise an UnterminatedTail event is
        emitted: without the required inter-frame silence the tail must not be
        reported as a complete frame. This method must be called explicitly at
        the end of the data.
        """
        if not self._buffer:
            return []

        end_time = self._last_time if end_timestamp_us is None else end_timestamp_us
        if end_time < self._last_time:
            raise ValueError("end timestamp is before the last byte")

        if end_time - self._last_time >= self.limits.t3_5_us:
            events = self._emit_frame(end_time)
        else:
            events = [UnterminatedTail(
                end_time, self._buffer,
                start_time=self._start_time, end_time=self._last_time
            )]

        self._buffer = bytearray()
        self._start_time = None
        self._last_time = None
        return events

    def _feed_byte(self, timestamp_us, byte):
        events = []

        if self._buffer:
            gap = timestamp_us - self._last_time

            if gap >= self.limits.t3_5_us:
                events.extend(self._emit_frame(timestamp_us))
            elif gap > self.limits.t1_5_us:
                events.append(FrameInterrupted(
                    timestamp_us, self._buffer, gap, self.limits.t1_5_us,
                    start_time=self._start_time, end_time=self._last_time
                ))
                self._buffer = bytearray()
                self._start_time = None

        if not self._buffer:
            self._start_time = timestamp_us

        self._buffer.append(byte)
        self._last_time = timestamp_us
        return events

    def _emit_frame(self, detection_time):
        frame = bytes(self._buffer)
        start_time = self._start_time
        end_time = self._last_time

        self._buffer = bytearray()
        self._start_time = None
        self._last_time = None

        if len(frame) < self.MIN_FRAME_LEN:
            return [MalformedFrame(
                end_time, frame, "frame shorter than 4 bytes",
                start_time=start_time, end_time=end_time
            )]

        actual_crc = struct.unpack(">H", frame[-2:])[0]
        expected_crc = calculate_crc(frame[:-2])
        if actual_crc != expected_crc:
            return [CrcError(
                end_time, frame, expected_crc, actual_crc,
                start_time=start_time, end_time=end_time
            )]

        address = frame[0]
        function_code = frame[1]
        pdu = frame[1:-2]
        broadcast = address == BROADCAST_ADDRESS
        exception = bool(function_code & 0x80)
        exception_code = pdu[1] if exception and len(pdu) >= 2 else None

        return [FrameReceived(
            end_time, frame, address, function_code, pdu, expected_crc,
            broadcast, exception, exception_code,
            start_time=start_time, end_time=end_time
        )]


def replay(events, baudrate=None, serial=None, bytesize=8, parity="N",
           stopbits=1, interchar_multiplier=1.5, interframe_multiplier=3.5,
           end_timestamp_us=None):
    """Decode a full list of arrival events without touching a serial port.

    events: iterable of Arrival objects or (timestamp_us, data) pairs; data is
            an int, bytes or bytearray.
    baudrate/serial/bytesize/parity/stopbits: serial line parameters.
    end_timestamp_us: capture end time used for the mandatory final flush.

    Returns the ordered list of emitted RtuEvent objects.
    """
    replayer = RtuReplayer(
        baudrate=baudrate, serial=serial, bytesize=bytesize, parity=parity,
        stopbits=stopbits, interchar_multiplier=interchar_multiplier,
        interframe_multiplier=interframe_multiplier,
    )

    decoded = []
    for event in events:
        if isinstance(event, Arrival):
            timestamp_us, data = event
        else:
            timestamp_us, data = event
        decoded.extend(replayer.feed(timestamp_us, data))

    decoded.extend(replayer.flush(end_timestamp_us))
    return decoded
