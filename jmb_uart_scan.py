#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
file: jmb_sens_scan.py
version: 1.1
date: 5.1.2022
"""

import ubluetooth
import struct
import micropython
import ubinascii
import utime
from lib.ble_advertising import decode_services, decode_name
from micropython import const

T_WAIT_FOR_IRQ_TERMINATED_MS = 100

_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)

_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)
_ADV_SCAN_IND = const(0x02)
_ADV_NONCONN_IND = const(0x03)

_UART_SERVICE_UUID = ubluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_RX_CHAR_UUID = ubluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX_CHAR_UUID = ubluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")


class BleJmbScan:
    def __init__(self, ble):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._scan_done = False
        self._central_list = list()

        self._reset()

    def _reset(self):
        # Cached name and address from a successful scan.
        self._name = None
        self._addr_type = None
        self._addr = None
        self._rssi = None

        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._scan_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None
        self._tx_handle = None
        self._rx_handle = None
        self._scan_done = False
        
    def _irq(self, event, data):
        
#         self._irq_list.append(event)
        
        if event == _IRQ_SCAN_RESULT: #5
            addr_type, addr, adv_type, rssi, adv_data = data
            if (adv_type in (_ADV_IND, _ADV_DIRECT_IND)
#                 and _UART_SERVICE_UUID in decode_services(adv_data)
                and decode_name(adv_data)[:4] == 'jmb_'
                ):
                # Found serve with name begining with "jmb_".
                in_list = False
                for r in self._central_list:
                    if r[1] == addr:
                        in_list = True
                if not in_list:        
                    self._central_list.append([addr_type, bytes(addr), adv_type, rssi, decode_name(adv_data)])

        elif event == _IRQ_SCAN_DONE: #6
            self._scan_done = True
            if self._scan_callback:
                if self._addr:
                    # Found a device during the scan (and the scan was explicitly stopped).
                    self._scan_callback(self._addr_type, self._addr, self._name)
                    self._scan_callback = None
                    self._scan_done = True
                else:
                    # Scan timed out.
                    self._scan_callback(None, None, None)
                
    def config_write_conn_info(self, data):
        with open ('config.txt', 'w') as f:
            for d in data:
                if type(d) == int:
                    f.write(str(d) + '\n')
                else:
                    f.write(d + '\n')

    # Find a device advertising the environmental sensor service.
    def scan(self, callback=None):
        self._addr_type = None
        self._addr = None
        self._scan_callback = callback
        self._ble.gap_scan(2000, 30000, 30000)



def main():
    
    print('initializing bluetooth')
    print('--------------------------')
    # instatiation of bluetooth.BLE
    ble = ubluetooth.BLE()
    central_scan = BleJmbScan(ble)
    
    # scan for central servers
    print('scanning')
    central_scan.scan()
    while not central_scan._scan_done:
        utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
    print('found ' + str(len(central_scan._central_list)) + ' central\n')
    
    # choose the one with the higher rssi
    nearest_index = -1
    nearest_level = -1000
    for nb, c in enumerate(central_scan._central_list):
        rssi = c[3]
        if rssi > nearest_level:
            nearest_level = rssi
            nearest_index = nb
    
    # displa the list of central servers
    for nb, c in enumerate(central_scan._central_list):
        # [addr_type, bytes(addr), adv_type, rssi, decode_name(adv_data)]
        msg = str(nb) + ' --> ' + c[4] + ' - ' + str(c[1]) + ' - ' + 'rssi:' + str(c[3])
        print(msg)
        
    # ask for client central choice
    print('\nto witch central one do you want to transfer yur mesures ?')
    v_choice = int(input('Enter the central number (default=' +
                   str(nearest_index) + ' rssi:' + str(nearest_level) + ') :')
                   or str(nearest_index))
    
    # writing the choice in the config.txt file
    print('writing config.txt --> central:' + str(v_choice))
    addr_type = central_scan._central_list[v_choice][0]
    addr = ubinascii.hexlify(bytes(central_scan._central_list[v_choice][1])).decode('utf-8')
    name = central_scan._central_list[v_choice][4]
    central_scan.config_write_conn_info([addr_type, addr, name])        
    
        
if __name__ == "__main__":
    main()
