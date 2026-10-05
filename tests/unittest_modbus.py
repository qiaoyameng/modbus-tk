#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
 Modbus TestKit: Implementation of Modbus protocol in python

 (C)2009 - Luc Jean - luc.jean@gmail.com
 (C)2009 - Apidev - http://www.apidev.fr

 This is distributed under GNU LGPL license, see license.txt
"""

import unittest
import modbus_tk.modbus
import threading
import struct
import logging
import time
import sys
from modbus_tk.hooks import install_hook

LOGGER = modbus_tk.utils.create_logger("udp")


class TestSlaveRequestHandler(unittest.TestCase):
    def setUp(self):
        self._slave = modbus_tk.modbus.Slave(0)
        self._name = "toto"
    
    def tearDown(self):
        pass

    def testUnhandledFunction(self):
        """test that an error is sent back when using an unknown function"""
        func_code = 55
        response = struct.pack(">BB", 128+func_code, modbus_tk.defines.ILLEGAL_FUNCTION)
        self.assertEqual(response, self._slave.handle_request(struct.pack(">B", func_code)))
        self.assertEqual(response, self._slave.handle_request(struct.pack(">BHHHH", func_code, 1, 2, 3, 4)))

    def _read_digital_data(self, function, block_type):
        self._slave.add_block(self._name, block_type, 0, 128)
        list_of_coils = ((1, 0, 2, 1), (0, ), (1, ), [0, 1]*20, [1]*128, [1, 0, 1]*7, (1, 0, 0, 1), [1, 0]*20)
        starting_addresses = (0, 0, 127, 40, 0, 0, 124, 87)
        
        list_of_responses = (
            struct.pack(">BBB", function, 1, 13),
            struct.pack(">BBB", function, 1, 0),
            struct.pack(">BBB", function, 1, 1),
            struct.pack(">BBBBBBB", function, 5, 170, 170, 170, 170, 170),
            struct.pack(">BB", function, 16)+(struct.pack(">B", 255)*16),
            struct.pack(">BBBBB", function, 3, 109, 219, 22),
            struct.pack(">BBB", function, 1, 9),
            struct.pack(">BBBBBBB", function, 5, 85, 85, 85, 85, 85),
        )
        for i in range(len(list_of_coils)):
            self._slave.set_values(self._name, starting_addresses[i], list_of_coils[i])
            self.assertEqual(list_of_responses[i], self._slave.handle_request(
                struct.pack(">BHH", function, starting_addresses[i], len(list_of_coils[i])))
            )

    def _read_out_of_blocks(self, function, block_type):
        self._slave.add_block(self._name, block_type, 20, 80)
        list_of_ranges = ((200, 10), (0, 1), (100, 1), (100, 5), (60, 50), (10, 30))
        
        response = struct.pack(">BB", 128+function, modbus_tk.defines.ILLEGAL_DATA_ADDRESS)
        
        for r in list_of_ranges:
            self.assertEqual(response, self._slave.handle_request(struct.pack(">BHH", function, r[0], r[1])))

    def _read_continuous_blocks(self, function, block_type):
        self._slave.add_block(self._name+"1", block_type, 0, 20)
        self._slave.add_block(self._name+"2", block_type, 20, 80)
        self._slave.add_block(self._name+"3", block_type, 100, 20)
        
        list_of_ranges = ((0, 30), (10, 20), (0, 120), (80, 30))
        
        response = struct.pack(">BB", 128+function, modbus_tk.defines.ILLEGAL_DATA_ADDRESS)
        
        for r in list_of_ranges:
            self.assertEqual(response, self._slave.handle_request(struct.pack(">BHH", function, r[0], r[1])))

    def testHandleReadCoils(self):
        """test that the correct response pdu is sent when receiving a pdu for reading coils"""
        self._read_digital_data(modbus_tk.defines.READ_COILS, modbus_tk.defines.COILS)

    def testHandleReadDigitalInputs(self):
        """test that the correct response pdu is sent when receiving a pdu for reading discrete inputs"""
        self._read_digital_data(modbus_tk.defines.READ_DISCRETE_INPUTS, modbus_tk.defines.DISCRETE_INPUTS)

    def testHandleReadCoilsOutOfBlocks(self):
        """test that an error response pdu is sent when receiving a pdu for reading coils at an unknown addresses"""
        self._read_out_of_blocks(modbus_tk.defines.READ_COILS, modbus_tk.defines.COILS)

    def testHandleReadDiscreteInputsOutOfBlocks(self):
        """
        test that an error response pdu is sent when receiving a pdu
        for reading discrete inputs at an unknown addresses
        """
        self._read_out_of_blocks(modbus_tk.defines.READ_DISCRETE_INPUTS, modbus_tk.defines.DISCRETE_INPUTS)

    def testHandleReadCoilsOnContinuousBlocks(self):
        """
        test that an error response pdu is sent when receiving a pdu
        for reading coils at an address shared on distinct blocks
        """
        self._read_continuous_blocks(modbus_tk.defines.READ_COILS, modbus_tk.defines.COILS)

    def testHandleReadDiscreteInputsOnContinuousBlocks(self):
        """
        test that an error response pdu is sent when receiving a pdu
        for reading discrete inputs at an address shared on distinct blocks
        """
        self._read_continuous_blocks(modbus_tk.defines.READ_DISCRETE_INPUTS, modbus_tk.defines.DISCRETE_INPUTS)

    def _make_response(self, function, regs):
        response = struct.pack(">BB", function, 2*len(regs))
        for r in regs:
            response += struct.pack(">H", r)
        return response
        
    def _read_registers(self, function, block_type):
        self._slave.add_block(self._name, block_type, 0, 128)
        list_of_regs = ((20, 2, 19, 75, 42), (15, ), [11, 12]*20, tuple(range(125)), (27, ), (1, 2, 3, 4), tuple(range(10)))
        starting_addresses = (0, 0, 0, 0, 127, 123, 82)

        for i in range(len(list_of_regs)):
            self._slave.set_values(self._name, starting_addresses[i], list_of_regs[i])
            self.assertEqual(
                self._make_response(function, list_of_regs[i]),
                self._slave.handle_request(struct.pack(">BHH", function, starting_addresses[i], len(list_of_regs[i])))
            )

    def testHandleReadHoldingRegisters(self):
        """test that the correct response pdu is sent when receiving a pdu for reading holding registers"""
        self._read_registers(modbus_tk.defines.READ_HOLDING_REGISTERS, modbus_tk.defines.HOLDING_REGISTERS)

    def testHandleReadAnalogInputs(self):
        """test that the correct response pdu is sent when receiving a pdu for reading input registers"""
        self._read_registers(modbus_tk.defines.READ_INPUT_REGISTERS, modbus_tk.defines.ANALOG_INPUTS)

    def _read_too_many_registers(self, function, block_type):
        self._slave.add_block(self._name, block_type, 0, 128)
        self._slave.set_values(self._name, 0, tuple(range(128)))
        response = struct.pack(">BB", function+128, modbus_tk.defines.ILLEGAL_DATA_VALUE)
        self.assertEqual(response, self._slave.handle_request(struct.pack(">BHH", function, 0, 126)))

    def testHandleReadTooManyHoldingRegisters(self):
        """test that an error is returned when handling a pdu for reading more than 125 holding registers"""
        self._read_too_many_registers(modbus_tk.defines.READ_HOLDING_REGISTERS, modbus_tk.defines.HOLDING_REGISTERS)

    def testHandleReadTooManyAnalogInputs(self):
        """test that an error is returned when handling a pdu for reading more than 125 input registers"""
        self._read_too_many_registers(modbus_tk.defines.READ_INPUT_REGISTERS, modbus_tk.defines.ANALOG_INPUTS)

    def testHandleReadHoldingRegistersOutOfBlocks(self):
        """test that an error is returned when handling a pdu for reading out of blocks"""
        self._read_out_of_blocks(modbus_tk.defines.READ_HOLDING_REGISTERS, modbus_tk.defines.HOLDING_REGISTERS)

    def testHandleReadInputRegistersOutOfBlocks(self):
        """test that an error is returned when handling a pdu for reading reading out of blocks"""
        self._read_out_of_blocks(modbus_tk.defines.READ_INPUT_REGISTERS, modbus_tk.defines.ANALOG_INPUTS)

    def testHandleReadHoldingRegistersOnContinuousBlocks(self):
        """
        test that an error response pdu is sent when receiving a pdu
        for reading coils at an address shared on distinct blocks
        """
        self._read_continuous_blocks(modbus_tk.defines.READ_HOLDING_REGISTERS, modbus_tk.defines.HOLDING_REGISTERS)

    def testHandleReadInputregistersOnContinuousBlocks(self):
        """
        test that an error response pdu is sent when receiving a pdu
        for reading discrete inputs at an address shared on distinct blocks
        """
        self._read_continuous_blocks(modbus_tk.defines.READ_INPUT_REGISTERS, modbus_tk.defines.ANALOG_INPUTS)

    def testReadCoilsAcrossAdjacentBlocks(self):
        slave = modbus_tk.modbus.Slave(0, allow_cross_block_access=True)
        slave.add_block("coils_0", modbus_tk.defines.COILS, 0, 9)
        slave.add_block("coils_1", modbus_tk.defines.COILS, 9, 10)
        slave.add_block("coils_2", modbus_tk.defines.COILS, 19, 9)
        slave.set_values("coils_0", 7, (1, 1))
        slave.set_values("coils_1", 9, (1, 0, 0, 0, 1, 1, 0, 1, 0, 1))
        slave.set_values("coils_2", 19, (1, 1))

        response = slave.handle_request(struct.pack(">BHH", modbus_tk.defines.READ_COILS, 7, 14))
        self.assertEqual(response, struct.pack(">BBBB", modbus_tk.defines.READ_COILS, 2, 0xc7, 0x3a))

    def testReadCoilsAcrossAdjacentBlocksRejectsGaps(self):
        slave = modbus_tk.modbus.Slave(0, allow_cross_block_access=True)
        slave.add_block("coils_0", modbus_tk.defines.COILS, 0, 9)
        slave.add_block("coils_1", modbus_tk.defines.COILS, 10, 10)

        response = slave.handle_request(struct.pack(">BHH", modbus_tk.defines.READ_COILS, 7, 5))
        self.assertEqual(response, struct.pack(
            ">BB", modbus_tk.defines.READ_COILS + 128, modbus_tk.defines.ILLEGAL_DATA_ADDRESS
        ))

    def testWriteCoilsAcrossAdjacentBlocksIsAtomic(self):
        slave = modbus_tk.modbus.Slave(0, allow_cross_block_access=True)
        slave.add_block("coils_0", modbus_tk.defines.COILS, 0, 9)
        slave.add_block("coils_1", modbus_tk.defines.COILS, 9, 10)
        slave.add_block("holding_2", modbus_tk.defines.HOLDING_REGISTERS, 19, 9)
        self._assertCoilWriteAcrossAdjacentBlocksIsAtomic(slave)

    def testWriteCoilsAcrossAdjacentBlocksRejectsOutOfRange(self):
        slave = modbus_tk.modbus.Slave(0, allow_cross_block_access=True)
        slave.add_block("coils_0", modbus_tk.defines.COILS, 0, 9)
        slave.add_block("coils_1", modbus_tk.defines.COILS, 9, 10)
        slave.add_block("coils_2", modbus_tk.defines.COILS, 19, 8)
        slave.set_values("coils_0", 0, [0] * 9)
        slave.set_values("coils_1", 9, [0] * 10)

        response = slave.handle_request(
            struct.pack(">BHHBBBB", modbus_tk.defines.WRITE_MULTIPLE_COILS, 7, 21, 3, 0x67, 0x2a, 0)
        )
        self.assertEqual(response, struct.pack(
            ">BB", modbus_tk.defines.WRITE_MULTIPLE_COILS + 128, modbus_tk.defines.ILLEGAL_DATA_ADDRESS
        ))
        self.assertEqual(slave.get_values("coils_0", 0, 9), tuple([0] * 9))
        self.assertEqual(slave.get_values("coils_1", 9, 10), tuple([0] * 10))

    def testWriteCoilsAcrossAdjacentBlocks(self):
        slave = modbus_tk.modbus.Slave(0, allow_cross_block_access=True)
        slave.add_block("coils_0", modbus_tk.defines.COILS, 0, 9)
        slave.add_block("coils_1", modbus_tk.defines.COILS, 9, 10)
        slave.add_block("coils_2", modbus_tk.defines.COILS, 19, 9)

        response = slave.handle_request(
            struct.pack(">BHHBBB", modbus_tk.defines.WRITE_MULTIPLE_COILS, 7, 14, 2, 0xc7, 0x3a)
        )
        self.assertEqual(response, struct.pack(
            ">BHH", modbus_tk.defines.WRITE_MULTIPLE_COILS, 7, 14
        ))
        self.assertEqual(slave.get_values("coils_0", 7, 2), (1, 1))
        self.assertEqual(slave.get_values("coils_1", 9, 10), (1, 0, 0, 0, 1, 1, 0, 1, 0, 1))
        self.assertEqual(slave.get_values("coils_2", 19, 2), (1, 1))

    def _assertCoilWriteAcrossAdjacentBlocksIsAtomic(self, slave):
        slave.set_values("coils_0", 0, [0] * 9)
        slave.set_values("coils_1", 9, [0] * 10)

        response = slave.handle_request(
            struct.pack(">BHHBBB", modbus_tk.defines.WRITE_MULTIPLE_COILS, 7, 14, 2, 0x67, 0x2a)
        )
        self.assertEqual(response, struct.pack(
            ">BB", modbus_tk.defines.WRITE_MULTIPLE_COILS + 128, modbus_tk.defines.ILLEGAL_DATA_ADDRESS
        ))
        self.assertEqual(slave.get_values("coils_0", 0, 9), tuple([0] * 9))
        self.assertEqual(slave.get_values("coils_1", 9, 10), tuple([0] * 10))

    def testReadAndWriteHoldingRegistersAcrossAdjacentBlocks(self):
        slave = modbus_tk.modbus.Slave(0, allow_cross_block_access=True)
        slave.add_block("hr_0", modbus_tk.defines.HOLDING_REGISTERS, 0, 4)
        slave.add_block("hr_1", modbus_tk.defines.HOLDING_REGISTERS, 4, 4)
        slave.add_block("hr_2", modbus_tk.defines.HOLDING_REGISTERS, 8, 4)

        values = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
        request = struct.pack(">BHHB", modbus_tk.defines.WRITE_MULTIPLE_REGISTERS, 1, 10, 20)
        for value in values:
            request += struct.pack(">H", value)
        response = slave.handle_request(request)
        self.assertEqual(response, struct.pack(
            ">BHH", modbus_tk.defines.WRITE_MULTIPLE_REGISTERS, 1, 10
        ))

        response = slave.handle_request(struct.pack(">BHH", modbus_tk.defines.READ_HOLDING_REGISTERS, 1, 10))
        expected = struct.pack(">BB", modbus_tk.defines.READ_HOLDING_REGISTERS, 20)
        for value in values:
            expected += struct.pack(">H", value)
        self.assertEqual(response, expected)
        self.assertEqual(slave.get_values("hr_0", 1, 3), (1, 2, 3))
        self.assertEqual(slave.get_values("hr_1", 4, 4), (4, 5, 6, 7))
        self.assertEqual(slave.get_values("hr_2", 8, 3), (8, 9, 10))

    def testWriteHoldingRegistersAcrossAdjacentBlocksIsAtomic(self):
        slave = modbus_tk.modbus.Slave(0, allow_cross_block_access=True)
        slave.add_block("hr_0", modbus_tk.defines.HOLDING_REGISTERS, 0, 4)
        slave.add_block("hr_1", modbus_tk.defines.HOLDING_REGISTERS, 4, 4)
        slave.add_block("coils_2", modbus_tk.defines.COILS, 8, 4)
        slave.set_values("hr_0", 0, tuple(range(10, 14)))
        slave.set_values("hr_1", 4, tuple(range(20, 24)))

        request = struct.pack(">BHHB", modbus_tk.defines.WRITE_MULTIPLE_REGISTERS, 1, 10, 20)
        for value in range(1, 11):
            request += struct.pack(">H", value)
        response = slave.handle_request(request)
        self.assertEqual(response, struct.pack(
            ">BB", modbus_tk.defines.WRITE_MULTIPLE_REGISTERS + 128,
            modbus_tk.defines.ILLEGAL_DATA_ADDRESS
        ))
        self.assertEqual(slave.get_values("hr_0", 0, 4), (10, 11, 12, 13))
        self.assertEqual(slave.get_values("hr_1", 4, 4), (20, 21, 22, 23))


class TestSlaveBlocks(unittest.TestCase):
    def setUp(self):
        self._slave = modbus_tk.modbus.Slave(0)
        self._name = "toto"
        self._block_types = (
            modbus_tk.defines.COILS,
            modbus_tk.defines.DISCRETE_INPUTS,
            modbus_tk.defines.HOLDING_REGISTERS,
            modbus_tk.defines.ANALOG_INPUTS,
        )
    
    def tearDown(self):
        pass

    def testShareData(self):
        """Add a block with shared memory"""
        shared_list = []
        memory = {
            modbus_tk.defines.COILS: shared_list,
            modbus_tk.defines.DISCRETE_INPUTS: shared_list,
            modbus_tk.defines.HOLDING_REGISTERS: shared_list,
            modbus_tk.defines.ANALOG_INPUTS: shared_list,
        }

        slave = modbus_tk.modbus.Slave(0, True, memory)
        slave.add_block(self._name, modbus_tk.defines.COILS, 0, 100)
        self.assertTrue(slave._get_block(self._name))
        self.assertEqual(slave.get_values(self._name, 10, 1), (0, ))
        slave.set_values(self._name, 10, 2)
        self.assertEqual(slave.get_values(self._name, 10, 1), (2, ))

    def testAddBlock(self):
        """Add a block and check that it is added"""
        self._slave.add_block(self._name, modbus_tk.defines.COILS, 0, 100)
        self.assertTrue(self._slave._get_block(self._name))
        
    def testRemoveBlock(self):
        """Add a block and remove it and make sure that it is not registred anymore"""
        self._slave.add_block(self._name, modbus_tk.defines.COILS, 0, 100)
        self.assertTrue(self._slave._get_block(self._name))
        self._slave.remove_block(self._name)
        self.assertRaises(modbus_tk.modbus.MissingKeyError, self._slave._get_block, (self._name))
            
    def testAddBlockWithSameName(self):
        """Add a block and make sure that adding another block with teh same name fails"""
        self._slave.add_block(self._name, modbus_tk.defines.COILS, 0, 100)
        self.assertRaises(modbus_tk.modbus.DuplicatedKeyError, self._slave.add_block, self._name, modbus_tk.defines.COILS, 100, 100)

    def testAddAndRemoveBlocks(self):
        """Add 30 blocks and remove them"""
        count = 30
        for i in range(count):
            self._slave.add_block(self._name+str(i), modbus_tk.defines.COILS, 100*i, 100)
        
        for i in range(count):
            name = self._name+str(i)
            self.assertTrue(self._slave._get_block(name))
            self._slave.remove_block(name)
            self.assertRaises(modbus_tk.modbus.MissingKeyError, self._slave._get_block, name)
    
    def testAddBlocksOfType(self):
        """Add a block of each type and remove them"""
        for i in self._block_types:
            self._slave.add_block(self._name+str(i), i, 0, 100)
        
        for i in self._block_types:
            name = self._name+str(i)
            self.assertTrue(self._slave._get_block(name))
            self._slave.remove_block(name)
            self.assertRaises(modbus_tk.modbus.MissingKeyError, self._slave._get_block, name)
        
    def testAddUnsupportedBlock(self):
        """Add a block with a wrong type"""
        self.assertTrue(5 not in self._block_types)
        self.assertRaises(modbus_tk.modbus.InvalidModbusBlockError, self._slave.add_block, self._name, 5, 100, 100)
        self.assertRaises(modbus_tk.modbus.MissingKeyError, self._slave._get_block, (self._name))
                
    def testAddWrongAddress(self):
        """Add a block with a wrong addresss"""
        for i in self._block_types:
            self.assertRaises(modbus_tk.modbus.InvalidArgumentError, self._slave.add_block, self._name, i, -5, 100)
            self.assertRaises(modbus_tk.modbus.MissingKeyError, self._slave._get_block, (self._name))
    
    def testAddWrongSize(self):
        """Add a block with a wrong size"""
        for i in self._block_types:
            self.assertRaises(modbus_tk.modbus.InvalidArgumentError, self._slave.add_block, self._name, i, 0, 0)
            self.assertRaises(modbus_tk.modbus.MissingKeyError, self._slave._get_block, (self._name))
            self.assertRaises(modbus_tk.modbus.InvalidArgumentError, self._slave.add_block, self._name, i, 0, -10)
            self.assertRaises(modbus_tk.modbus.MissingKeyError, self._slave._get_block, (self._name))
    
    def testOverlappedBlocks(self):
        """Add 2 blocks with overlapped ranges and check that the 2nd one is not added"""
        for i in self._block_types:
            self._slave.add_block(self._name, i, 0, 100)
            for j in range(100):
                self.assertRaises(
                    modbus_tk.modbus.OverlapModbusBlockError, self._slave.add_block, self._name+"_", i, j, 100
                )
            self._slave.remove_block(self._name)
            
    def testAddContinuousBlock(self):
        """Add 2 continuous blocks and check that it is ok"""
        for i in self._block_types:
            self._slave.add_block(self._name, i, 0, 100)
            self._slave.add_block(self._name+"_", i, 100, 100)
            self._slave.remove_block(self._name)
            self._slave.remove_block(self._name+"_")
        
    def testMultiThreadedAccess(self):
        """test mutual access"""
        def add_blocks(slave, name, starting_address):
            slave.add_block(name, modbus_tk.defines.COILS, starting_address, 10)
        threads = []
        for i in range(10):
            threads.append(threading.Thread(target=add_blocks, args=(self._slave, self._name+str(i), i*10)))
            threads[i].start()
        
        for t in threads:
            t.join()
        
        for i in range(10):
            self._slave.remove_block(self._name+str(i))
            
    def testSetAndGetRegister(self):
        """change the value of a register and check that it is properly set"""
        self._slave.add_block(self._name, modbus_tk.defines.HOLDING_REGISTERS, 0, 100)
        self.assertEqual(self._slave.get_values(self._name, 10, 1), (0, ))
        self._slave.set_values(self._name, 10, 2)
        self.assertEqual(self._slave.get_values(self._name, 10, 1), (2, ))
        
    def testSetAndGetSeveralRegisters(self):
        """change the value of several registers and check that it is properly set"""
        self._slave.add_block(self._name, modbus_tk.defines.HOLDING_REGISTERS, 0, 100)
        self.assertEqual(self._slave.get_values(self._name, 10, 10), tuple([0]*10))
        self._slave.set_values(self._name, 10, tuple(range(0, 10)))
        self.assertEqual(self._slave.get_values(self._name, 10, 10), tuple(range(10)))
    
    def testSetAndGetSeveralCoils(self):
        """change the value of several coils and check that it is properly set"""
        self._slave.add_block(self._name, modbus_tk.defines.COILS, 0, 100)
        self.assertEqual(self._slave.get_values(self._name, 10, 10), tuple([0]*10))
        self._slave.set_values(self._name, 10, [1]*5)
        self.assertEqual(self._slave.get_values(self._name, 10, 10), tuple([1]*5+[0]*5))
    
    def testSetRegisterOutOfBounds(self):
        """change the value of a register out of a block and check that error are raised"""
        self._slave.add_block(self._name, modbus_tk.defines.HOLDING_REGISTERS, 20, 80)
        self.assertRaises(modbus_tk.modbus.OutOfModbusBlockError, self._slave.set_values, self._name, 100, 2)
        self.assertRaises(modbus_tk.modbus.OutOfModbusBlockError, self._slave.set_values, self._name, 105, 2)
        self.assertRaises(modbus_tk.modbus.OutOfModbusBlockError, self._slave.set_values, self._name, 95, [1]*10)
        self.assertRaises(modbus_tk.modbus.OutOfModbusBlockError, self._slave.set_values, self._name, 0, [1]*10)
        self.assertRaises(modbus_tk.modbus.OutOfModbusBlockError, self._slave.set_values, self._name, 15, [1]*10)
             
    def testSetRegisterAtTheBounds(self):
        """change the values on limits of the block and check that it is properly set"""
        self._slave.add_block(self._name, modbus_tk.defines.HOLDING_REGISTERS, 20, 80)
        self._slave.set_values(self._name, 20, 2)
        self._slave.set_values(self._name, 99, 2)
    
    def testSetRegisterOnContinuousBlocks(self):
        """create 2 continuous blocks and check that an error is raised when accessing adress range on the 2 blocks"""
        self._slave.add_block(self._name, modbus_tk.defines.HOLDING_REGISTERS, 20, 80)
        self._slave.add_block(self._name+"_", modbus_tk.defines.HOLDING_REGISTERS, 0, 20)
        self.assertRaises(modbus_tk.modbus.OutOfModbusBlockError, self._slave.set_values, self._name, 15, [1]*10)
        
    def testMultiThreadedSetValues(self):
        """test that set and get values is thread safe"""
        self._slave.add_block(self._name, modbus_tk.defines.HOLDING_REGISTERS, 0, 20)
        
        all_values = []
        count = 20
        nb_of_vals = 2
        
        def change_val(slave, name, count, nb_of_vals):
            for i in range(0, count, nb_of_vals):
                vals = tuple(range(i, i+nb_of_vals))
                slave.set_values(name, 0, vals)
                time.sleep(0.02)
            
        def get_val(slave, name, count, nb_of_vals, all_values):
            for i in range(0, count, nb_of_vals):
                for j in range(2):
                    vals = slave.get_values(name, 0, nb_of_vals)
                    all_values.append(vals)
                    time.sleep(0.01)    
        
        threads = []
        threads.append(threading.Thread(target=change_val, args=(self._slave, self._name, count, nb_of_vals)))
        threads.append(threading.Thread(target=get_val, args=(self._slave, self._name, count, nb_of_vals, all_values)))

        for t in threads:
            t.start()
        
        for t in threads:
            t.join()

        vals = []
        expected_values = []
        for i in range(0, count, nb_of_vals):
            expected_values.append(tuple(range(i, i+nb_of_vals)))

        def avoid_duplicates(x):
            if x not in vals:
                vals.append(x)
                return True
            return False

        vals = [value for value in all_values if avoid_duplicates(value)]

        self.assertEqual(vals, expected_values)


class TestServer(unittest.TestCase):
    def setUp(self):
        self.server = modbus_tk.modbus.Server()
    
    def tearDown(self):
        pass
        
    def testInvalidSlaveId(self):
        """Check that an error is raised when adding a slave with a wrong id"""
        slaves = (-5, 0, "", 256, 5600)
        for s in slaves:
            self.assertRaises(Exception, self.server.add_slave, s)

    def testAddSlave(self):
        """Check that a slave is added correctly"""
        slaves = range(1, 256)
        for id in slaves:
            s = self.server.add_slave(id)
            self.assertTrue(str(s).find("modbus_tk.modbus.Slave")>0)

    def testAddAndGetSlave(self):
        """Check that a slave can be retrieved by id after added"""
        slaves = range(1, 248)
        d = {}
        for id in slaves:
            d[id] = self.server.add_slave(id)
        for id in slaves:
            s = self.server.get_slave(id)
            self.assertTrue(s is d[id])

    def testErrorOnRemoveUnknownSlave(self):
        """Check that an error is raised when removing a slave with a wrong id"""
        slaves = range(0, 249)
        for id in slaves:
            self.assertRaises(Exception, self.server.remove_slave, id)

    def testAddAndRemove(self):
        """Add a slave, remove it and make sure it is not there anymore"""
        slaves = range(1, 248)
        for id in slaves:
            self.server.add_slave(id)
        for id in slaves:
            self.server.remove_slave(id)
        for id in slaves:
            self.assertRaises(Exception, self.server.get_slave, id)

    def testRemoveAllSlaves(self):
        """Add somes slave, remove all and make sure it there is nothing anymore"""
        slaves = range(1, 248)
        for id in slaves:
            self.server.add_slave(id)
        self.server.remove_all_slaves()
        for id in slaves:
            self.assertRaises(Exception, self.server.get_slave, id)
            
    def testHookOnSetBlockData(self):
        slave = self.server.add_slave(22)
        def setblock_hook(args):
            (block, slice, values) = args 
            setblock_hook.calls += 1
        setblock_hook.calls = 0
        install_hook('modbus.ModbusBlock.setitem', setblock_hook)

        block_types = (
            modbus_tk.defines.COILS,
            modbus_tk.defines.DISCRETE_INPUTS,
            modbus_tk.defines.HOLDING_REGISTERS,
            modbus_tk.defines.ANALOG_INPUTS
        )
        
        for block_type in block_types:
            slave.add_block(str(block_type), block_type, 0, 20)
            slave.set_values(str(block_type), 0, 1)
            slave.set_values(str(block_type), 5, (1, 0, 1))
        
        self.assertEqual(setblock_hook.calls, 8)
        
if __name__ == '__main__':
    unittest.main(argv=sys.argv)
