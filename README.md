modbus-tk: Create Modbus app easily with Python
=================================================

[![Build Status](https://semaphoreci.com/api/v1/ljean/modbus-tk/branches/master/shields_badge.svg)](https://semaphoreci.com/ljean/modbus-tk)

Download / Install
------------------------------------
Current version is 1.1.5 It is available on PyPI https://pypi.python.org/pypi/modbus_tk

License
------------------------------------
This is distributed under GNU LGPL license

Description
------------------------------------
Make possible to write modbus TCP and RTU master and slave.

It can be used for testing purpose : It is shipped with slave simulator and a master with a web-based hmi (ok the hmi need to be improved :).

It can also be used to create any application which need to communicate over modbus. It is a full-stack implementation and is used on "real applications".

Thanks to Python and the incredible set of existing libraries, it can fit a lot of different needs : database logging, HMI, report generation ...

modbus-tk is different from pymodbus which is another implementation of the modbus stack in python.

modbus-tk tries to limit dependencies (even if it requires pyserial for Modbus RTU).

modbus-tk has no link with tkInter. tk stands for 'testkit' or 'toolkit' depending of the way you use it.

Discussion group
------------------------------------
Please join the modbus-tk discussion group to participate : https://groups.google.com/forum/?hl=fr#!forum/modbus-tk

Features
------------------------------------
* Modbus TCP support for writing masters and slaves
* Modbus RTU support for writing masters and slaves (requires pyserial)
* Modbus RTU over TCP support
* Can be customized with hook mechanism (simulate errors, timeouts...)
* ready-to use simulator with RPC interface
* Defines very easily your own memory blocks
* Set/Get values for any place in a memory block
* logging capability through python logging module
* Web-based HMI (experimental feature which requires bottle)

Offline RTU frame replay
------------------------------------
When the serial hardware is unavailable but a "timestamp + byte" capture is,
`modbus_tk.rtu_replay` splits the stream into RTU frames deterministically:

```python
from modbus_tk.rtu_replay import replay

# (timestamp_us, byte) arrival events; timestamps may also be fractions.Fraction
events = [(0, 0x01), (100, 0x03), ...]
for event in replay(events, baudrate=115200, end_time=12000):
    print(event.kind, bytes(event.data).hex())
```

It enforces the t1.5 intra-frame and t3.5 inter-frame timers (750/1750 us above
19200 bps), never sleeps, and emits distinct events for valid frames (including
broadcast and exception responses), CRC errors, mid-frame interruptions and
truncated tails left without a closing silence.

Feedback
------------------------------------
Feedback is welcomed! Please enter an issue for giving your feedback.

Follow modbus-tk
------------------------------------
Follow on Bluesky : https://bsky.app/profile/cmlj.bsky.social

Company website : <http://www.apidev.fr/>

Other links
------------------------------------
pymodslave http://sourceforge.net/projects/pymodslave/ : a gui app for simulation purpose built with Qt4 and modbus-tk

modbus-simulator https://github.com/dhoomakethu/kivy-modbus-simu : modbus simulator built with modbus-tk and kivy

MicroPython port by Sean Lannigan : https://gitlab.com/extel-open-source/micropython-modbus .So far it only supports RTU Master mode (i.e. no TCP and no slave mode), as this was all I needed in Sean's MicroPython application - the other modes probably wouldn't be too hard to port.
