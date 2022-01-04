#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# This example finds and connects to a peripheral running the
# UART service (e.g. ble_simple_peripheral.py).

import ubluetooth
import random
import struct
import micropython
import machine
import ubinascii
import utime

# import lib.wifi_esp32 as wifi_esp32
# import lib.rtc_esp32 as rtc_esp32
# import lib.adc1_cal as adc1_cal
# import lib.bme280 as bme280
# import lib.bme680 as bme680
# from lib.blink import blink_internal_blue_led
from ble_advertising import decode_services, decode_name

from micropython import const

T_DEEPSLEEP_MS = 10000
T_BEFORE_DEEPSLEEP_MS = 100
T_WAIT_FOR_IRQ_TERMINATED_MS = 100

WIFI_CONNECT = False
RTC_SYNC = False

BM_VCC_PIN = 17
BM_GND_PIN = 16
BM_SDA_PIN = 21
BM_SCL_pin = 22
BM_VCC_PIN = machine.Pin(BM_VCC_PIN, machine.Pin.OUT)
BM_GND_PIN = machine.Pin(BM_GND_PIN, machine.Pin.OUT)
BM_VCC_PIN.on()
BM_GND_PIN.off()

_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)

_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)
_ADV_SCAN_IND = const(0x02)
_ADV_NONCONN_IND = const(0x03)

_UART_SERVICE_UUID = ubluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_RX_CHAR_UUID = ubluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX_CHAR_UUID = ubluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")

class BleJmbSensor:
    def __init__(self, ble):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        
        self._scan_done = False
        self._gattc_service_done = None
        self._gattc_characteristic_done = None

        self._reset()

    def _reset(self):
        # Cached name and address from a successful scan.
        self._name = None
        self._addr_type = None
        self._addr = None

        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._scan_callback = None
        self._conn_callback = None
        self._read_callback = None

        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None
        self._tx_handle = None
        self._rx_handle = None
        
        self._scan_done = False
        self._connect_status = False
        self._uart_central_found = False
        self._gattc_service_done = False
        self._gattc_characteristic_done = False
        self._irq_peripheral_connect = False
        self._irq_peripheral_disconnect = False
        
        self._irq_list = []
        
    def _irq(self, event, data):
        
        self._irq_list.append(event)
        
        if event == _IRQ_PERIPHERAL_CONNECT: #7
            conn_handle, addr_type, addr = data
            print('---> 7_IRQ_PERIPHERAL_CONNECT', conn_handle, addr_type, self.bytes_to_asc(bytes(addr)))
            # Connect successful.
            if addr_type == self._addr_type and addr == self._addr:
                self._conn_handle = conn_handle
                self._ble.gattc_discover_services(self._conn_handle)
                self._irq_peripheral_connect = True

        elif event == _IRQ_PERIPHERAL_DISCONNECT: #8
            print('---> 8_IRQ_PERIPHERAL_DISCONNECT')
            # Disconnect (either initiated by us or the remote end).
            conn_handle, _, _ = data
            self._irq_peripheral_disconnect = True
            if conn_handle == self._conn_handle:
                # If it was initiated by us, it'll already be reset.
                self._reset()

        elif event == _IRQ_GATTC_SERVICE_RESULT: #9
            print('---> 9_IRQ_GATTC_SERVICE_RESULT')
            # Connected device returned a service.
            conn_handle, start_handle, end_handle, uuid = data
            if conn_handle == self._conn_handle and uuid == _UART_SERVICE_UUID:
                self._start_handle, self._end_handle = start_handle, end_handle

        elif event == _IRQ_GATTC_SERVICE_DONE: #10
            print('---> 10_IRQ_GATTC_SERVICE_DONE')
            # Service query complete.
            if self._start_handle and self._end_handle:
                self._ble.gattc_discover_characteristics(self._conn_handle, self._start_handle, self._end_handle)
                self._gattc_service_done = True
            else:
                print("Failed to find uart service.")

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT: #11
            print('---> 11_IRQ_GATTC_CHARACTERISTIC_RESULT')
            # Connected device returned a characteristic.
            conn_handle, def_handle, value_handle, properties, uuid = data
            if conn_handle == self._conn_handle and uuid == _UART_RX_CHAR_UUID:
                self._rx_handle = value_handle
            if conn_handle == self._conn_handle and uuid == _UART_TX_CHAR_UUID:
                self._tx_handle = value_handle

        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE: #12
            print('---> 12_IRQ_GATTC_CHARACTERISTIC_DONE')
            # Characteristic query complete.
            if self._tx_handle is not None and self._rx_handle is not None:
                # We've finished connecting and discovering device, fire the connect callback.
                if self._conn_callback:
                    self._conn_callback()
                self._gattc_characteristic_done = True
            else:
                print("Failed to find uart rx characteristic.")
                
                addr_asc = ubinascii.hexlify(bytes(self._addr)).decode('utf-8')
                addr_bin = ubinascii.unhexlify((addr_asc))


    def bytes_to_asc(self, v_bytes):
        return ubinascii.hexlify(bytes(v_bytes)).decode('utf-8')

    def asc_to_bytes(self, v_ascii):
        return ubinascii.unhexlify((v_ascii))
                
    def config_write_conn_info(self, data):
        with open ('config.txt', 'w') as f:
            for d in data:
                if type(d) == int:
                    f.write(str(d) + '\n')
                else:
                    f.write(d + '\n')

    def config_read_conn_info(self):
        with open ('config.txt', 'r') as f:
            data = f.readlines()
            for i, l in enumerate(data):
                if i == 0:
                    self._addr_type = int(l)
                elif i == 1:
                    self._addr = self.asc_to_bytes(l.replace('\n',''))
                elif i == 2:
                    self._name = l

    # Returns true if we've successfully connected and discovered characteristics.
    def is_connected(self):
        return self._irq_peripheral_connect

    # Find a device advertising the environmental sensor service.
    def scan(self, callback=None):
        self._addr_type = None
        self._addr = None
        self._scan_callback = callback
        self._ble.gap_scan(2000, 30000, 30000)

    # Connect to the specified device (otherwise use cached address from a scan).
    def connect(self, addr_type=None, addr=None, callback=None):
        print('---> connect')
        self._addr_type = addr_type or self._addr_type
        self._addr = addr or self._addr
        self._conn_callback = callback
        if self._addr_type is None or self._addr is None:
            return False
        self._ble.gap_connect(self._addr_type, self._addr, 5000)
        return True

    # Disconnect from current device.
    def disconnect(self):
        self._ble.gap_disconnect(self._conn_handle)
        self._reset()

    # Send data over the UART
    def write(self, v, response=False):
        if not self.is_connected():
            return
        self._ble.gattc_write(self._conn_handle, self._rx_handle, v, 1 if response else 0)

    # Set handler for when data is received over the UART.
    def on_notify(self, callback):
        self._notify_callback = callback

def restart_ESP32(i, err_msg):
    msg = str(i) + ' - restart_ESP32: ' + err_msg
    print(msg)
    with open('error.txt' , 'a') as f:
        f.write(msg+'\n')
    utime.sleep_ms(1000)
    machine.reset()


def main():
    while True:
        # mesure time for a single pass
        t_start_total = utime.ticks_ms()
        # load the pass counter value from file
        try:
            with open ('index.txt', 'r') as f:
                i = int(f.readline()) + 1
        except:
            i = 1
        with open ('index.txt', 'w') as f:
            f.write(str(i))
        
        # instatiation of bluetooth.BLE
        ble = ubluetooth.BLE()
        sensor = BleJmbSensor(ble)
        sensor._irq_list = []
        
        # read sensor config from file
        sensor.config_read_conn_info()
        #connect to the central
        connect_status = sensor.connect()
        while not connect_status:
            utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
            connect_status = sensor.connect()
        
        while (not sensor.is_connected()
               or not sensor._gattc_service_done
               or not sensor._gattc_characteristic_done):
            utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
        
        temp68 = 23 # int(bme68.temperature*100)/100 #bme68.temperature
        hum68 = 60 #int(bme68.humidity) #bme68.humidity
        pres68 = 950 #int(bme68.pressure) #bme68.pressure
        gas68 = 50 #int(bme68.gas / 1000)
        gaspc =  75 # int(100*gas68/75)
        alt68 = 750 #int(bme68.altitude)

        print('temperature -->', temp68, '°C') #, 'temp 280 -->', temp28, '°C')
        print('humidite ----->', hum68, '%') #, 'hum 280 -->', hum28, '%')
        print('pression ----->', pres68, 'hPa') #, 'pres 280 -->', pres28, 'hPa')
        print('gaz ---------->', gas68, 'Kohms', '=^=', gaspc, '%')
        print('altitude ----->', alt68, 'm')

        
        msg = 'jmb ' + str(int(temp68)) + ' ' + str(hum68) + ' ' + str(pres68) + ' ' + str(gaspc)
        sensor.write(msg)
            
        elapsed = utime.ticks_ms() - t_start_total
        print('pass:', i, '-->',  str((utime.ticks_ms() - t_start_total)/1000) + 's', )
        print('going to sleep for ' + str(int((T_BEFORE_DEEPSLEEP_MS + T_DEEPSLEEP_MS - elapsed)/1000)) + 's')
        
        print(sensor._irq_list)
        
        print('============================================================================')
        utime.sleep_ms(T_BEFORE_DEEPSLEEP_MS + T_DEEPSLEEP_MS - elapsed)
       

if __name__ == "__main__":
    main()
