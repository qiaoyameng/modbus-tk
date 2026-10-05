#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 Modbus TestKit: tests for the strict response validation of Master

 A fake in-memory transport is used (no socket/serial needed): the master
 sends a request and a responder callable builds the response PDU. Corrupted
 responses simulating a device returning the answer of a previous request can
 be injected field by field.
"""

import struct
import unittest

import modbus_tk.defines as defines
from modbus_tk.exceptions import ModbusError, ModbusInvalidResponseError
from modbus_tk.hooks import install_hook, uninstall_hook
from modbus_tk.modbus import Master
from modbus_tk.modbus_tcp import TcpQuery, ModbusInvalidMbapError


class FakeMaster(Master):
    """A Master with an in-memory TCP-like transport"""

    def __init__(self, strict=False):
        super(FakeMaster, self).__init__(1.0)
        self.set_strict_response_validation(strict)
        self._query = None
        self.responder = None
        self.last_request = None
        self.corrupt_transaction_id = False

    def _do_open(self):
        pass

    def _do_close(self):
        return True

    def _make_query(self):
        # keep a reference: execute builds the request with this query and
        # _recv needs it to parse back the MBAP
        self._query = TcpQuery()
        return self._query

    def _send(self, request):
        self.last_request = request

    def _recv(self, _expected_length=-1):
        (_, request_pdu) = self._query.parse_request(self.last_request)
        response_pdu = self.responder(request_pdu)
        if response_pdu is None:
            # broadcast: nothing comes back
            return b""
        (tid, pid, _length, uid) = struct.unpack(">HHHB", self.last_request[:7])
        if self.corrupt_transaction_id:
            tid += 1
        return struct.pack(">HHHB", tid, pid, len(response_pdu) + 1, uid) + response_pdu

    def execute_request(self, responder, *args, **kwargs):
        self.responder = responder
        return self.execute(1, *args, **kwargs)


def echo_responder(request_pdu):
    """A well-behaved slave echoing the request as the protocol requires"""
    (function_code, ) = struct.unpack(">B", request_pdu[0:1])
    if function_code in (defines.READ_COILS, defines.READ_DISCRETE_INPUTS):
        (_, quantity) = struct.unpack(">HH", request_pdu[1:5])
        byte_count = quantity // 8 + (1 if quantity % 8 else 0)
        data = b"\x00" * byte_count
        return struct.pack(">BB", function_code, byte_count) + data
    if function_code in (defines.READ_INPUT_REGISTERS, defines.READ_HOLDING_REGISTERS):
        (_, quantity) = struct.unpack(">HH", request_pdu[1:5])
        # return register values equal to their index, like the functional tests
        data = b"".join(struct.pack(">H", i % 65536) for i in range(quantity))
        return struct.pack(">BB", function_code, 2 * quantity) + data
    if function_code == defines.READ_WRITE_MULTIPLE_REGISTERS:
        (_, read_addr, read_quantity, _write_addr, _write_quantity, _bc) = struct.unpack(
            ">BHHHHB", request_pdu[0:10]
        )
        data = b"".join(struct.pack(">H", i % 65536) for i in range(read_quantity))
        return struct.pack(">BB", function_code, 2 * read_quantity) + data
    if function_code in (defines.WRITE_MULTIPLE_COILS, defines.WRITE_MULTIPLE_REGISTERS):
        # echo of function code, starting address and quantity only
        (address, quantity) = struct.unpack(">HH", request_pdu[1:5])
        return struct.pack(">BHH", function_code, address, quantity)
    # write single and mask write: full echo
    return request_pdu


class TestStrictResponseValidation(unittest.TestCase):
    """Check the response is matched against the request before returning data"""

    def setUp(self):
        self.master = FakeMaster(strict=True)

    def testNormalReadHoldingRegisters(self):
        """a correct read response passes strict validation"""
        result = self.master.execute_request(
            echo_responder, defines.READ_HOLDING_REGISTERS, 100, 10
        )
        self.assertEqual(tuple(range(10)), result)

    def testNormalSignedReadHoldingRegisters(self):
        """negative values are read correctly with the h format"""
        def responder(request_pdu):
            return struct.pack(">BB", defines.READ_HOLDING_REGISTERS, 4) + struct.pack(">hh", -1, -1234)

        result = self.master.execute_request(
            responder, defines.READ_HOLDING_REGISTERS, 0, 2, data_format=">hh"
        )
        self.assertEqual((-1, -1234), result)

    def testNormalReadCoilsUnaligned(self):
        """byte count of a coil read matches the requested quantity"""
        result = self.master.execute_request(
            echo_responder, defines.READ_COILS, 0, 11
        )
        self.assertEqual(tuple([0] * 11), result)

    def testNormalWriteSingleRegister(self):
        """a correct single write echo passes"""
        result = self.master.execute_request(
            echo_responder, defines.WRITE_SINGLE_REGISTER, 10, output_value=1234
        )
        self.assertEqual((10, 1234), result)

    def testNormalWriteMultipleRegisters(self):
        """a correct multiple write echo passes"""
        result = self.master.execute_request(
            echo_responder, defines.WRITE_MULTIPLE_REGISTERS, 100, output_value=tuple(range(5))
        )
        self.assertEqual((100, 5), result)

    def testNormalWriteMultipleCoils(self):
        """a correct multiple coils write echo passes"""
        result = self.master.execute_request(
            echo_responder, defines.WRITE_MULTIPLE_COILS, 0, output_value=tuple([1] * 10)
        )
        self.assertEqual((0, 10), result)

    def testNormalMaskWriteRegister(self):
        """a correct mask write echo passes"""
        result = self.master.execute_request(
            echo_responder, defines.MASK_WRITE_REGISTER, 10, and_mask=0x00FF, or_mask=0xFF00
        )
        self.assertEqual((10, 0x00FF, 0xFF00), result)

    def testNormalReadWriteMultipleRegisters(self):
        """a correct read-write response passes"""
        result = self.master.execute_request(
            echo_responder, defines.READ_WRITE_MULTIPLE_REGISTERS, 0, quantity_of_x=3,
            write_starting_address_fc23=10, output_value=tuple([9, 9, 9])
        )
        self.assertEqual((0, 1, 2), result)

    def testReturnsRawStillValidated(self):
        """returns_raw returns the raw data but does not bypass the checks"""
        result = self.master.execute_request(
            echo_responder, defines.READ_HOLDING_REGISTERS, 0, 2, returns_raw=True
        )
        self.assertEqual(struct.pack(">HH", 0, 1), result)

    def testDeviceExceptionIsModbusError(self):
        """a correct exception response raises ModbusError (device exception)"""
        def responder(request_pdu):
            return struct.pack(">BB", defines.READ_HOLDING_REGISTERS | 0x80, defines.ILLEGAL_DATA_ADDRESS)

        with self.assertRaises(ModbusError) as cm:
            self.master.execute_request(
                responder, defines.READ_HOLDING_REGISTERS, 0, 2
            )
        self.assertEqual(defines.ILLEGAL_DATA_ADDRESS, cm.exception.get_exception_code())

    def testExceptionWithWrongFunctionCodeIsInvalidResponse(self):
        """an exception response for another function is a protocol mismatch"""
        def responder(request_pdu):
            # answers with an exception of WRITE_SINGLE_REGISTER instead of a read
            return struct.pack(">BB", defines.WRITE_SINGLE_REGISTER | 0x80, defines.SLAVE_DEVICE_FAILURE)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.READ_HOLDING_REGISTERS, 0, 2
        )

    def testStaleReadResponseIsRejected(self):
        """a well-formed response to a previous read request is rejected"""
        def responder(request_pdu):
            # correct pdu for reading 2 registers, while this request reads 4
            return struct.pack(">BB", defines.READ_HOLDING_REGISTERS, 4) + b"\x00\x00\x00\x00"

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.READ_HOLDING_REGISTERS, 0, 4
        )

    def testReadRegisterWrongByteCount(self):
        """byte count not equal to 2*quantity is rejected"""
        def responder(request_pdu):
            return struct.pack(">BB", defines.READ_HOLDING_REGISTERS, 6) + b"\x00" * 6

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.READ_HOLDING_REGISTERS, 0, 4
        )

    def testReadCoilsWrongByteCount(self):
        """byte count not matching the quantity of coils is rejected"""
        def responder(request_pdu):
            # 11 coils need 2 bytes but the device returns only 1
            return struct.pack(">BB", defines.READ_COILS, 1) + b"\x00"

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.READ_COILS, 0, 11
        )

    def testResponseWithWrongFunctionCodeIsRejected(self):
        """a normal response with another function code is rejected"""
        def responder(request_pdu):
            return struct.pack(">BB", defines.READ_INPUT_REGISTERS, 4) + b"\x00" * 4

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.READ_HOLDING_REGISTERS, 0, 2
        )

    def testWriteSingleRegisterWrongAddress(self):
        """single write echo with wrong address is rejected"""
        def responder(request_pdu):
            return struct.pack(">BHH", defines.WRITE_SINGLE_REGISTER, 11, 1234)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.WRITE_SINGLE_REGISTER, 10, output_value=1234
        )

    def testWriteSingleRegisterWrongValue(self):
        """single write echo with wrong value is rejected"""
        def responder(request_pdu):
            # looks like a valid response of the previous request (other value)
            return struct.pack(">BHH", defines.WRITE_SINGLE_REGISTER, 10, 4321)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.WRITE_SINGLE_REGISTER, 10, output_value=1234
        )

    def testWriteSingleCoilWrongValue(self):
        """single coil echo with an invalid coil value is rejected"""
        def responder(request_pdu):
            return struct.pack(">BHH", defines.WRITE_SINGLE_COIL, 10, 0x0000)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.WRITE_SINGLE_COIL, 10, output_value=1
        )

    def testWriteMultipleRegistersWrongAddress(self):
        """multiple write echo with wrong address is rejected"""
        def responder(request_pdu):
            return struct.pack(">BHH", defines.WRITE_MULTIPLE_REGISTERS, 101, 5)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.WRITE_MULTIPLE_REGISTERS, 100, output_value=tuple(range(5))
        )

    def testWriteMultipleRegistersWrongQuantity(self):
        """multiple write echo with wrong quantity is rejected"""
        def responder(request_pdu):
            # address is right but quantity belongs to another request
            return struct.pack(">BHH", defines.WRITE_MULTIPLE_REGISTERS, 100, 4)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.WRITE_MULTIPLE_REGISTERS, 100, output_value=tuple(range(5))
        )

    def testWriteMultipleCoilsWrongQuantity(self):
        """multiple coils write echo with wrong quantity is rejected"""
        def responder(request_pdu):
            return struct.pack(">BHH", defines.WRITE_MULTIPLE_COILS, 0, 9)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.WRITE_MULTIPLE_COILS, 0, output_value=tuple([1] * 10)
        )

    def testMaskWriteRegisterWrongEcho(self):
        """mask write echo differing from the request is rejected"""
        def responder(request_pdu):
            return struct.pack(">BHHH", defines.MASK_WRITE_REGISTER, 10, 0x0000, 0xFF00)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.MASK_WRITE_REGISTER, 10, and_mask=0x00FF, or_mask=0xFF00
        )

    def testWriteSingleWrongEchoWithReturnsRaw(self):
        """returns_raw must not let a wrong single write echo through"""
        def responder(request_pdu):
            return struct.pack(">BHH", defines.WRITE_SINGLE_REGISTER, 10, 4321)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.WRITE_SINGLE_REGISTER, 10, output_value=1234, returns_raw=True
        )

    def testUnknownRawRequestOnlyFunctionCodeChecked(self):
        """for an unknown custom RAW request only MAC and function code apply"""
        custom_fc = 0x64

        def responder(request_pdu):
            # the payload is arbitrary and cannot be interpreted: it is accepted
            return struct.pack(">BBH", custom_fc, 0x01, 0x1234)

        result = self.master.execute_request(
            responder, defines.RAW, 0, pdu=struct.pack(">BH", custom_fc, 0xABCD),
            expected_length=7, data_format=">BH", returns_raw=True
        )
        self.assertEqual(struct.pack(">BH", 0x01, 0x1234), result)

    def testUnknownRawRequestWrongFunctionCode(self):
        """a custom RAW response with another function code is rejected"""
        def responder(request_pdu):
            return struct.pack(">B", 0x65)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.RAW, 0, pdu=struct.pack(">B", 0x64),
            expected_length=7
        )

    def testUnknownRawExceptionMustMatchFunction(self):
        """an exception of a custom RAW request must carry its function code"""
        custom_fc = 0x64

        def responder(request_pdu):
            return struct.pack(">BB", custom_fc | 0x80, defines.ILLEGAL_FUNCTION)

        with self.assertRaises(ModbusError) as cm:
            self.master.execute_request(
                responder, defines.RAW, 0, pdu=struct.pack(">B", custom_fc),
                expected_length=7
            )
        self.assertEqual(defines.ILLEGAL_FUNCTION, cm.exception.get_exception_code())

    def testTransactionIdMismatchIsInvalidResponse(self):
        """a wrong TCP transaction id is caught by the MAC layer checks"""
        self.master.corrupt_transaction_id = True
        self.assertRaises(
            ModbusInvalidMbapError, self.master.execute_request,
            echo_responder, defines.READ_HOLDING_REGISTERS, 0, 2
        )

    def testBroadcastHasNoResponse(self):
        """a broadcast write sends the request and returns None without recv"""
        sent = []
        original_send = self.master._send

        def send(request):
            sent.append(request)
            return original_send(request)

        self.master._send = send
        result = self.master.execute(
            0, defines.WRITE_MULTIPLE_REGISTERS, 5000, output_value=tuple(range(3))
        )
        self.assertIsNone(result)
        self.assertEqual(1, len(sent))
        # unit id of the broadcast request is 0
        self.assertEqual(0, struct.unpack(">B", sent[0][6:7])[0])

    def testBroadcastSingleWriteHasNoResponse(self):
        """a broadcast single write also returns None"""
        result = self.master.execute(
            0, defines.WRITE_SINGLE_REGISTER, 5030, output_value=12
        )
        self.assertIsNone(result)

    def testHookModifiedFinalRequestIsValidated(self):
        """strict validation is done against the final request after hooks"""
        def before_send(args):
            (_master, request) = args
            # the hook changes the value written in register 10 to 4321
            (tid, pid, length, uid) = struct.unpack(">HHHB", request[:7])
            new_pdu = struct.pack(">BHH", defines.WRITE_SINGLE_REGISTER, 10, 4321)
            return struct.pack(">HHHB", tid, pid, len(new_pdu) + 1, uid) + new_pdu

        install_hook("modbus.Master.before_send", before_send)
        try:
            result = self.master.execute_request(
                echo_responder, defines.WRITE_SINGLE_REGISTER, 10, output_value=1234
            )
        finally:
            uninstall_hook("modbus.Master.before_send", before_send)
        # the value echoed by the slave is 4321: the final (hooked) request
        self.assertEqual((10, 4321), result)

    def testHookModifiedRequestWithStaleEchoIsRejected(self):
        """an echo matching the pre-hook request is rejected after a hook change"""
        def before_send(args):
            (_master, request) = args
            (tid, pid, length, uid) = struct.unpack(">HHHB", request[:7])
            new_pdu = struct.pack(">BHH", defines.WRITE_SINGLE_REGISTER, 10, 4321)
            return struct.pack(">HHHB", tid, pid, len(new_pdu) + 1, uid) + new_pdu

        def old_value_responder(request_pdu):
            # returns an echo of the request as it was before the hook
            return struct.pack(">BHH", defines.WRITE_SINGLE_REGISTER, 10, 1234)

        install_hook("modbus.Master.before_send", before_send)
        try:
            self.assertRaises(
                ModbusInvalidResponseError, self.master.execute_request,
                old_value_responder, defines.WRITE_SINGLE_REGISTER, 10, output_value=1234
            )
        finally:
            uninstall_hook("modbus.Master.before_send", before_send)


class TestNonStrictMode(unittest.TestCase):
    """Without strict validation the historical behavior is preserved"""

    def setUp(self):
        self.master = FakeMaster(strict=False)

    def testWrongByteCountIsAcceptedWhenNotStrict(self):
        """a wrong byte count is silently processed when strict is off"""
        def responder(request_pdu):
            # device says 2 bytes and sends 2 bytes while 4 registers (8
            # bytes) were requested. Ask for a 1-register format so the short
            # payload can still be unpacked
            return struct.pack(">BB", defines.READ_HOLDING_REGISTERS, 2) + b"\x00\x00"

        result = self.master.execute_request(
            responder, defines.READ_HOLDING_REGISTERS, 0, 4, data_format=">H"
        )
        self.assertEqual((0,), result)

    def testWrongWriteEchoIsAcceptedWhenNotStrict(self):
        """a single write echo with another value is returned when strict is off"""
        def responder(request_pdu):
            return struct.pack(">BHH", defines.WRITE_SINGLE_REGISTER, 10, 4321)

        result = self.master.execute_request(
            responder, defines.WRITE_SINGLE_REGISTER, 10, output_value=1234
        )
        self.assertEqual((10, 4321), result)

    def testStrictCanBeEnabledAtRuntime(self):
        """set_strict_response_validation turns the checks on at any time"""
        self.master.set_strict_response_validation(True)

        def responder(request_pdu):
            return struct.pack(">BHH", defines.WRITE_SINGLE_REGISTER, 10, 4321)

        self.assertRaises(
            ModbusInvalidResponseError, self.master.execute_request,
            responder, defines.WRITE_SINGLE_REGISTER, 10, output_value=1234
        )

    def testDeviceExceptionStillRaisedWhenNotStrict(self):
        """device exceptions are raised regardless of the strict mode"""
        def responder(request_pdu):
            return struct.pack(">BB", defines.READ_HOLDING_REGISTERS | 0x80, defines.SLAVE_DEVICE_FAILURE)

        with self.assertRaises(ModbusError) as cm:
            self.master.execute_request(
                responder, defines.READ_HOLDING_REGISTERS, 0, 2
            )
        self.assertEqual(defines.SLAVE_DEVICE_FAILURE, cm.exception.get_exception_code())


if __name__ == '__main__':
    unittest.main()
