#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python

 This is distributed under GNU LGPL license, see license.txt

Offline replay of a "(timestamp, bytes)" RTU capture. No serial port, no
sleep: the caller owns the time source and pushes microsecond arrival events.
"""

import struct

from modbus_tk.rtu_replay import replay, Arrival, FrameReceived
from modbus_tk.utils import calculate_crc


def make_frame(address, pdu):
    """Build a valid RTU ADU for the demo"""
    body = bytes(bytearray([address] + pdu))
    return body + struct.pack(">H", calculate_crc(body))


def main():
    first = make_frame(1, [3, 0, 10, 0, 2])           # read holding registers
    second = make_frame(0, [6, 0, 5, 0, 0xFF])        # broadcast write
    bad = bytearray(make_frame(2, [3, 0, 0, 0, 1]))
    bad[-1] ^= 0xFF                                   # corrupt the CRC

    events = [
        Arrival(0, first),
        Arrival(2000, second),
        Arrival(4000, bytes(bad)),
    ]

    for event in replay(events, baudrate=115200, end_timestamp_us=6000):
        if isinstance(event, FrameReceived):
            print("{0}: address={1} broadcast={2}".format(
                event.event_type, event.address, event.broadcast))
        else:
            print(event.event_type, event)


if __name__ == "__main__":
    main()
