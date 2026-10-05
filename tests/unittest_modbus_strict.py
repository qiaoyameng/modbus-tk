#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python

 (C)2009 - Luc Jean - luc.jean@gmail.com
 (C)2009 - Apidev - http://www.apidev.fr

 This is distributed under GNU LGPL license, see license.txt
"""

import struct
import unittest

import modbus_tk.defines as defines
import modbus_tk.utils as utils
from modbus_tk import hooks
from modbus_tk.exceptions import ModbusError, ModbusInvalidResponseError
from modbus_tk.modbus import Master
from modbus_tk.modbus_rtu import RtuQuery
from modbus_tk.modbus_tcp import TcpQuery, ModbusInvalidMbapError


def rtu_frame(slave, pdu):
    """Build a RTU frame (slave address + pdu + crc)"""
    data = struct.pack(">B", slave) + pdu
    return data + struct.pack(">H", utils.calculate_crc(data))


class FakeMaster(Master):
    """
    A master with an in-memory transport.
    Responses are either popped from queued_responses (used to inject
    stale or corrupted responses) or computed by the responder callable
    (used to simulate a real device answering the request it received).
    """

    def __init__(self, query_factory=RtuQuery, strict=True):
        super(FakeMaster, self).__init__(timeout_in_sec=1.0, strict=strict)
        self._query_factory = query_factory
        self.responder = None
        self.queued_responses = []
        self.sent_requests = []

    def _do_open(self):
        pass

    def _do_close(self):
        return True

    def _send(self, buf):
        self.sent_requests.append(buf)

    def _recv(self, expected_length=-1):
        if self.queued_responses:
            return self.queued_responses.pop(0)
        if self.responder is not None:
            return self.responder(self.sent_requests[-1])
        return b""

    def _make_query(self):
        return self._query_factory()


class FakeRtuDevice(object):
    """Simulates a slave device answering RTU requests"""

    def __init__(self, slave=1):
        self.slave = slave
        self.holding_registers = {}

    def handle(self, request):
        """Answer a RTU request like a real device"""
        (slave, ) = struct.unpack(">B", request[0:1])
        if slave != self.slave:
            # not for us: no answer, like a broadcast or another slave
            return b""
        pdu = request[1:-2]
        (function_code, ) = struct.unpack(">B", pdu[0:1])
        if function_code in (defines.READ_HOLDING_REGISTERS, defines.READ_INPUT_REGISTERS):
            (address, quantity) = struct.unpack(">HH", pdu[1:5])
            values = [self.holding_registers.get(address + i, 0) & 0xFFFF for i in range(quantity)]
            response_pdu = struct.pack(">BB", function_code, 2 * quantity)
            response_pdu += struct.pack(">{0}H".format(quantity), *values)
            return rtu_frame(slave, response_pdu)
        if function_code == defines.WRITE_SINGLE_REGISTER:
            (address, value) = struct.unpack(">HH", pdu[1:5])
            self.holding_registers[address] = value
            return rtu_frame(slave, pdu[0:5])
        if function_code == defines.WRITE_SINGLE_COIL:
            return rtu_frame(slave, pdu[0:5])
        if function_code == defines.WRITE_MULTIPLE_REGISTERS:
            (address, quantity) = struct.unpack(">HH", pdu[1:5])
            values = struct.unpack(">{0}H".format(quantity), pdu[5:5 + 2 * quantity])
            for i, value in enumerate(values):
                self.holding_registers[address + i] = value
            return rtu_frame(slave, pdu[0:5])
        if function_code == defines.WRITE_MULTIPLE_COILS:
            return rtu_frame(slave, pdu[0:5])
        raise ValueError("Unsupported function code {0}".format(function_code))


class TestStrictMaster(unittest.TestCase):
    """Check the strict response validation of the master"""

    def setUp(self):
        self.master = FakeMaster(strict=True)
        self.device = FakeRtuDevice(slave=1)
        self.master.responder = self.device.handle
        self.master.open()

    def tearDown(self):
        self.master.close()

    def testStrictIsEnabledByConstructorAndSetter(self):
        """strict mode can be enabled by the constructor and the setter"""
        self.assertTrue(self.master.get_strict())
        master = FakeMaster(strict=False)
        self.assertFalse(master.get_strict())
        master.set_strict(True)
        self.assertTrue(master.get_strict())

    def testReadHoldingRegistersSigned(self):
        """a normal signed register read goes through strict validation"""
        self.device.holding_registers[0] = 0xFFFE  # -2 as a signed value
        self.device.holding_registers[1] = 5
        result = self.master.execute(1, defines.READ_HOLDING_REGISTERS, 0, 2, data_format=">hh")
        self.assertEqual((-2, 5), result)

    def testWriteSingleRegisterOk(self):
        """a single write with a matching echo goes through strict validation"""
        result = self.master.execute(1, defines.WRITE_SINGLE_REGISTER, 10, output_value=5)
        self.assertEqual((10, 5), result)
        self.assertEqual(5, self.device.holding_registers[10])

    def testResponseWithWrongFunctionCode(self):
        """a response with another function code is a protocol mismatch"""
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BBH", 4, 2, 100)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.READ_HOLDING_REGISTERS, 0, 1
        )

    def testExceptionResponseMatchingRequest(self):
        """an exception response for the request function code is a device exception"""
        self.master.queued_responses.append(
            rtu_frame(1, struct.pack(">BB", defines.READ_HOLDING_REGISTERS | 0x80, defines.ILLEGAL_DATA_ADDRESS))
        )
        with self.assertRaises(ModbusError) as context:
            self.master.execute(1, defines.READ_HOLDING_REGISTERS, 0, 1)
        self.assertEqual(defines.ILLEGAL_DATA_ADDRESS, context.exception.get_exception_code())

    def testExceptionResponseWithWrongFunctionCode(self):
        """an exception response for another function code is a protocol mismatch"""
        self.master.queued_responses.append(
            rtu_frame(1, struct.pack(">BB", defines.READ_INPUT_REGISTERS | 0x80, defines.ILLEGAL_DATA_ADDRESS))
        )
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.READ_HOLDING_REGISTERS, 0, 1
        )

    def testSingleWriteEchoWithWrongAddress(self):
        """a single write echo with another address is a protocol mismatch"""
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BHH", 6, 11, 5)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.WRITE_SINGLE_REGISTER, 10, output_value=5
        )

    def testSingleWriteEchoWithWrongValue(self):
        """a single write echo with another value is a protocol mismatch"""
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BHH", 6, 10, 6)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.WRITE_SINGLE_REGISTER, 10, output_value=5
        )

    def testSingleCoilWriteEcho(self):
        """a single coil write echo must repeat the address and the value"""
        result = self.master.execute(1, defines.WRITE_SINGLE_COIL, 3, output_value=1)
        self.assertEqual((3, 0xFF00), result)
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BHH", 5, 3, 0x0000)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.WRITE_SINGLE_COIL, 3, output_value=1
        )

    def testMultipleWriteEchoOk(self):
        """a multiple write with a matching echo goes through strict validation"""
        result = self.master.execute(1, defines.WRITE_MULTIPLE_REGISTERS, 0, output_value=[1, 2, 3])
        self.assertEqual((0, 3), result)

    def testMultipleWriteEchoWithWrongAddress(self):
        """a multiple write echo with another address is a protocol mismatch"""
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BHH", 16, 1, 3)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.WRITE_MULTIPLE_REGISTERS, 0, output_value=[1, 2, 3]
        )

    def testMultipleWriteEchoWithWrongQuantity(self):
        """a multiple write echo with another quantity is a protocol mismatch"""
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BHH", 16, 0, 2)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.WRITE_MULTIPLE_REGISTERS, 0, output_value=[1, 2, 3]
        )

    def testMultipleCoilsWriteEchoWithWrongQuantity(self):
        """a multiple coils write echo with another quantity is a protocol mismatch"""
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BHH", 15, 0, 9)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.WRITE_MULTIPLE_COILS, 0, output_value=[1] * 10
        )

    def testReadRegistersWithWrongByteCount(self):
        """a read response whose byte count doesn't match the requested quantity is a mismatch"""
        # 2 registers requested, 3 registers returned: the byte count is
        # consistent with the response length but not with the request
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BBHHH", 3, 6, 1, 2, 3)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.READ_HOLDING_REGISTERS, 0, 2
        )

    def testReadCoilsWithWrongByteCount(self):
        """a coil read response whose byte count doesn't match the requested quantity is a mismatch"""
        # 9 coils requested: 2 bytes expected, 1 byte returned
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BBB", 1, 1, 0xFF)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.READ_COILS, 0, 9
        )

    def testReadCoilsOk(self):
        """a coil read with a matching byte count goes through strict validation"""
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BBBB", 1, 2, 0xFF, 0x01)))
        result = self.master.execute(1, defines.READ_COILS, 0, 9)
        self.assertEqual(tuple([1] * 9), result)

    def testReturnsRawDoesNotBypassValidation(self):
        """returns_raw doesn't bypass the strict validation"""
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BHH", 6, 10, 6)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.WRITE_SINGLE_REGISTER, 10, output_value=5, returns_raw=True
        )

    def testReturnsRawOk(self):
        """returns_raw still returns the raw data of a valid response"""
        self.device.holding_registers[0] = 0x1234
        result = self.master.execute(1, defines.READ_HOLDING_REGISTERS, 0, 1, returns_raw=True)
        self.assertEqual(struct.pack(">H", 0x1234), result)

    def testRawRequestWithCustomFunctionCode(self):
        """for a custom RAW request only the function code correspondence is checked"""
        # a valid echo of the custom function code: the content is user-defined
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BBBB", 0x42, 1, 2, 3)))
        result = self.master.execute(1, defines.RAW, 0, pdu=struct.pack(">B", 0x42), data_format=">BBB")
        self.assertEqual((1, 2, 3), result)
        # another function code is a protocol mismatch
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BB", 0x43, 0)))
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.RAW, 0, pdu=struct.pack(">B", 0x42), data_format=">B"
        )
        # an exception response for the custom function code is a device exception
        self.master.queued_responses.append(rtu_frame(1, struct.pack(">BB", 0x42 | 0x80, 4)))
        self.assertRaises(
            ModbusError,
            self.master.execute, 1, defines.RAW, 0, pdu=struct.pack(">B", 0x42), data_format=">B"
        )

    def testNonStrictModeIgnoresMismatch(self):
        """without strict mode a mismatched echo is returned as before"""
        master = FakeMaster(strict=False)
        master.queued_responses.append(rtu_frame(1, struct.pack(">BHH", 6, 11, 6)))
        result = master.execute(1, defines.WRITE_SINGLE_REGISTER, 10, output_value=5)
        self.assertEqual((11, 6), result)

    def testBroadcastReturnsNone(self):
        """a broadcast query doesn't expect any response and returns None"""
        result = self.master.execute(0, defines.WRITE_SINGLE_REGISTER, 10, output_value=5)
        self.assertEqual(None, result)
        self.assertEqual(1, len(self.master.sent_requests))
        result = self.master.execute(0, defines.READ_HOLDING_REGISTERS, 0, 2)
        self.assertEqual(None, result)

    def testHookModifiedRequest(self):
        """strict validation applies to the request actually sent (modified by a hook)"""
        self.device.holding_registers[5] = 1234

        def change_address_hook(args):
            # move the starting address of the request from 0 to 5
            (master, request) = args
            modified = request[:2] + struct.pack(">H", 5) + request[4:-2]
            return modified + struct.pack(">H", utils.calculate_crc(modified))

        hooks.install_hook("modbus.Master.before_send", change_address_hook)
        try:
            result = self.master.execute(1, defines.READ_HOLDING_REGISTERS, 0, 1)
            self.assertEqual((1234, ), result)
            result = self.master.execute(1, defines.WRITE_SINGLE_REGISTER, 0, output_value=7)
            self.assertEqual((5, 7), result)
            self.assertEqual(7, self.device.holding_registers[5])
        finally:
            hooks.uninstall_hook("modbus.Master.before_send", change_address_hook)


class TestStrictTcpMaster(unittest.TestCase):
    """Check the strict validation together with the modbus TCP layer"""

    def setUp(self):
        self.master = FakeMaster(query_factory=TcpQuery, strict=True)
        self.master.open()

    def tearDown(self):
        self.master.close()

    @staticmethod
    def _tcp_response(request, response_pdu, transaction_id_offset=0):
        (transaction_id, protocol_id, _, unit_id) = struct.unpack(">HHHB", request[:7])
        mbap = struct.pack(
            ">HHHB", transaction_id + transaction_id_offset, protocol_id, len(response_pdu) + 1, unit_id
        )
        return mbap + response_pdu

    def testValidTcpResponse(self):
        """a valid TCP response goes through strict validation"""
        def responder(request):
            return self._tcp_response(request, struct.pack(">BBH", 3, 2, 100))
        self.master.responder = responder
        result = self.master.execute(1, defines.READ_HOLDING_REGISTERS, 0, 1)
        self.assertEqual((100, ), result)

    def testTcpResponseWithWrongTransactionId(self):
        """a response with another transaction id is rejected by the TCP layer"""
        def responder(request):
            return self._tcp_response(request, struct.pack(">BBH", 3, 2, 100), transaction_id_offset=1)
        self.master.responder = responder
        self.assertRaises(
            ModbusInvalidMbapError,
            self.master.execute, 1, defines.READ_HOLDING_REGISTERS, 0, 1
        )

    def testTcpResponseWithWrongFunctionCode(self):
        """a response with another function code is a protocol mismatch"""
        def responder(request):
            return self._tcp_response(request, struct.pack(">BBH", 4, 2, 100))
        self.master.responder = responder
        self.assertRaises(
            ModbusInvalidResponseError,
            self.master.execute, 1, defines.READ_HOLDING_REGISTERS, 0, 1
        )


if __name__ == "__main__":
    unittest.main()
