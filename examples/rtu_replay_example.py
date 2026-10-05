#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Replay Modbus RTU frames from a timestamped byte capture without any serial
device. Each event in the capture is a (timestamp_us, byte) pair; the time
source is entirely supplied by the caller and no real sleep occurs.
"""

from modbus_tk.rtu_replay import replay, append_crc


def main():
    # Two valid frames with a 1750 us pause between them.
    first = append_crc(bytes([1, 3, 4, 0, 1, 0, 2]))
    second = append_crc(bytes([2, 3, 4, 0, 9, 0, 8]))

    capture = [(index * 10, byte_value) for index, byte_value in enumerate(first)]
    gap = 1750
    offset = (len(first) - 1) * 10 + gap
    capture += [(offset + index * 10, byte_value)
                for index, byte_value in enumerate(second)]

    # end_time is the instant the capture stops; a >= t3.5 silence is needed
    # for the last frame to be considered complete.
    for baudrate in (115200, 9600):
        print("baudrate={0}:".format(baudrate))
        for event in replay(capture, baudrate=baudrate,
                            end_time=capture[-1][0] + 10000):
            print("  ", event.kind, event.data.hex())


if __name__ == "__main__":
    main()
