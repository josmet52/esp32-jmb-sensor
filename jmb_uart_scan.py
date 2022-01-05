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
#         self._gattc_service_done = None
#         self._gattc_characteristic_done = None
        
#         self._irq_list = list()
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
#         self._conn_callback = None
#         self._read_callback = None

        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None
        self._tx_handle = None
        self._rx_handle = None
        
        self._scan_done = False
#         self._connect_status = False
#         self._uart_central_found = False
#         self._gattc_service_done = False
#         self._gattc_characteristic_done = False
#         self._irq_peripheral_connect = False
#         self._irq_peripheral_disconnect = False
        
    def _irq(self, event, data):
        
#         self._irq_list.append(event)
        
        if event == _IRQ_SCAN_RESULT: #5
            addr_type, addr, adv_type, rssi, adv_data = data
#             print('---> _IRQ_SCAN_RESULT', addr_type, bytes(addr), adv_type, rssi, decode_name(adv_data))
#             print('adv_type in (_ADV_IND, _ADV_DIRECT_IND)', adv_type in (_ADV_IND, _ADV_DIRECT_IND))
#             print('_UART_SERVICE_UUID in decode_services(adv_data)', _UART_SERVICE_UUID in decode_services(adv_data))
#             print('decode_name(adv_data)[:4] == jmb_', decode_name(adv_data)[:4] == 'jmb_')
#             print('---->', _UART_SERVICE_UUID, decode_services(adv_data))
#             print()
            if (adv_type in (_ADV_IND, _ADV_DIRECT_IND)
#                 and _UART_SERVICE_UUID in decode_services(adv_data)
                and decode_name(adv_data)[:4] == 'jmb_'
                ):
                # Found git_uPython_ble.
                in_list = False
                for r in self._central_list:
                    if r[1] == addr:
                        in_list = True
                if not in_list:        
                    self._central_list.append([addr_type, bytes(addr), adv_type, rssi, decode_name(adv_data)])
                
#                 self.config_write_conn_info([self._addr_type, ubinascii.hexlify(bytes(self._addr)).decode('utf-8'), self._name])
                

        elif event == _IRQ_SCAN_DONE: #6
#             print('---> _IRQ_SCAN_DONE')
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

#         if event == _IRQ_PERIPHERAL_CONNECT: #7
#             conn_handle, addr_type, addr = data
#             print('---> 7_IRQ_PERIPHERAL_CONNECT', conn_handle, addr_type, self.bytes_to_asc(bytes(addr)))
#             # Connect successful.
#             if addr_type == self._addr_type and addr == self._addr:
#                 self._conn_handle = conn_handle
#                 self._ble.gattc_discover_services(self._conn_handle)
#                 self._irq_peripheral_connect = True
#                 print('_IRQ_PERIPHERAL_CONNECT done')
# 
#         elif event == _IRQ_PERIPHERAL_DISCONNECT: #8
#             print('---> 8_IRQ_PERIPHERAL_DISCONNECT')
#             # Disconnect (either initiated by us or the remote end).
#             conn_handle, _, _ = data
#             self._irq_peripheral_disconnect = True
#             if conn_handle == self._conn_handle:
#                 # If it was initiated by us, it'll already be reset.
#                 self._reset()
# 
#         elif event == _IRQ_GATTC_SERVICE_RESULT: #9
#             print('---> 9_IRQ_GATTC_SERVICE_RESULT')
#             # Connected device returned a service.
#             conn_handle, start_handle, end_handle, uuid = data
#             if conn_handle == self._conn_handle and uuid == _UART_SERVICE_UUID:
#                 self._start_handle, self._end_handle = start_handle, end_handle
# 
#         elif event == _IRQ_GATTC_SERVICE_DONE: #10
#             print('---> 10_IRQ_GATTC_SERVICE_DONE')
#             # Service query complete.
#             if self._start_handle and self._end_handle:
#                 self._ble.gattc_discover_characteristics(self._conn_handle, self._start_handle, self._end_handle)
#                 self._gattc_service_done = True
#             else:
#                 print("Failed to find uart service.")
# 
#         elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT: #11
#             print('---> 11_IRQ_GATTC_CHARACTERISTIC_RESULT')
#             # Connected device returned a characteristic.
#             conn_handle, def_handle, value_handle, properties, uuid = data
#             if conn_handle == self._conn_handle and uuid == _UART_RX_CHAR_UUID[0]:
#                 self._rx_handle = value_handle
#             if conn_handle == self._conn_handle and uuid == _UART_TX_CHAR_UUID[0]:
#                 self._tx_handle = value_handle
# 
#         elif event == _IRQ_GATTC_CHARACTERISTIC_DONE: #12
#             print('---> 12_IRQ_GATTC_CHARACTERISTIC_DONE')
#             # Characteristic query complete.
#             if self._tx_handle is not None and self._rx_handle is not None:
#                 # We've finished connecting and discovering device, fire the connect callback.
#                 if self._conn_callback:
#                     self._conn_callback()
#                 self._gattc_characteristic_done = True
#             else:
#                 print("Failed to find uart rx characteristic.")

#     def bytes_to_asc(self, v_bytes):
#         return ubinascii.hexlify(bytes(v_bytes)).decode('utf-8')
# 
#     def asc_to_bytes(self, v_ascii):
#         return ubinascii.unhexlify((v_ascii))
                
    def config_write_conn_info(self, data):
        with open ('config.txt', 'w') as f:
            for d in data:
                if type(d) == int:
                    f.write(str(d) + '\n')
                else:
                    f.write(d + '\n')

#     def config_read_conn_info(self):
#         with open ('config.txt', 'r') as f:
#             data = f.readlines()
#             for i, l in enumerate(data):
#                 if i == 0:
#                     self._addr_type = int(l)
#                 elif i == 1:
#                     self._addr = self.asc_to_bytes(l.replace('\n',''))
#                 elif i == 2:
#                     self._name = l.replace('\n','')

    # Returns true if we've successfully connected and discovered characteristics.
#     def is_connected(self):
#         return self._irq_peripheral_connect

    # Find a device advertising the environmental sensor service.
    def scan(self, callback=None):
        self._addr_type = None
        self._addr = None
        self._scan_callback = callback
        self._ble.gap_scan(2000, 30000, 30000)

    # Connect to the specified device (otherwise use cached address from a scan).
#     def connect(self, addr_type=None, addr=None, callback=None):
#         print('---> connect', addr_type, addr)
#         self._addr_type = addr_type or self._addr_type
#         self._addr = addr or self._addr
#         self._conn_callback = callback
#         if self._addr_type is None or self._addr is None:
#             return False
#         self._ble.gap_connect(self._addr_type, self._addr)
#         return True

    # Disconnect from current device.
#     def disconnect(self):
#         self._ble.gap_disconnect(self._conn_handle)
#         self._reset()
# 
#     # Send data over the UART
#     def write(self, v, response=False):
#         if not self.is_connected():
#             return
#         self._ble.gattc_write(self._conn_handle, self._rx_handle, v, 1 if response else 0)

    # Set handler for when data is received over the UART.
#     def on_notify(self, callback):
#         self._notify_callback = callback

# def restart_ESP32(i, err_msg):
#     msg = str(i) + ' - restart_ESP32: ' + err_msg
#     print(msg)
#     with open('error.txt' , 'a') as f:
#         f.write(msg+'\n')
#     utime.sleep_ms(1000)
#     machine.reset()


def main():
#     try:
        print('initializing bluetooth')
        print('----------------------------------------------------------------------------')
        # instanciation of bme280, bme680 - Pin assignment
#         i2c = machine.SoftI2C(scl=machine.Pin(BM_SCL_pin), sda=machine.Pin(BM_SDA_PIN), freq=10000)
# #         bme28 = bme280.BME280(i2c=i2c)
#         bme68 = bme680.BME680_I2C(i2c=i2c)
        # instatiation of bluetooth.BLE
        ble = ubluetooth.BLE()
        central_scan = BleJmbScan(ble)
        # initialize the pass counter to 0
#         with open ('index.txt', 'w') as f:
#             f.write(str(0))
#         # read and initialise variable for sensor from config from file
#         sensor.config_read_conn_info()
#         addr_type = sensor._addr_type
#         addr = sensor._addr
        print('scanning')
        central_scan.scan()
        while not central_scan._scan_done:
            utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
#         print('scan done')
        
        print('found ' + str(len(central_scan._central_list)) + ' central\n')
        
        nearest_index = -1
        nearest_level = -1000
        for nb, c in enumerate(central_scan._central_list):
            rssi = c[3]
            if rssi > nearest_level:
                nearest_level = rssi
                nearest_index = nb
            
        for nb, c in enumerate(central_scan._central_list):
            # [addr_type, bytes(addr), adv_type, rssi, decode_name(adv_data)]
            msg = str(nb) + ' --> ' + c[4] + ' - ' + str(c[1]) + ' - ' + 'rssi:' + str(c[3])
            print(msg)
        print('\nto witch central one do you want to transfer aour mesures ?')
        v_choice = int(input('Enter the central number (default=' +
                       str(nearest_index) + ' rssi:' + str(nearest_level) + ') :')
                       or str(nearest_index))
        print('writing config.txt --> central:' + str(v_choice))
        
        addr_type = central_scan._central_list[v_choice][0]
        addr = ubinascii.hexlify(bytes(central_scan._central_list[v_choice][1])).decode('utf-8')
        name = central_scan._central_list[v_choice][4]
#         print(addr_type, addr, name)
        
        central_scan.config_write_conn_info([addr_type, addr, name])        
    
        
#         if not sensor._uart_central_found:
#             print('No uart central found. Active it and then restart this program')
#             import sys
#             sys.exit()
#             print('server found:',str((utime.ticks_ms()-t_start)/1000) + 's --> ',
#                   'sensor._addr_type:', sensor._addr_type,
#                   'sensor._name:', sensor._name,
#                   'sensor:_addr:', ubinascii.hexlify(bytes(sensor._addr)).decode('utf-8'))
#         while True:
#             # mesure time for a single pass
#             t_start_total = utime.ticks_ms()
#             # blink the blue led
#             blink_internal_blue_led(100, 100, 1, 2)
#             # load the pass counter value from file
#             try:
#                 with open ('index.txt', 'r') as f:
#                     i = int(f.readline()) + 1
#             except:
#                 i = 1
#             with open ('index.txt', 'w') as f:
#                 f.write(str(i))
#             
#             
#             #connect to the central
# #             v_nc = 1
#             sensor._addr_type, sensor._addr = addr_type, addr
#             connect_status = sensor.connect(sensor._addr_type, sensor._addr)
# #             print('connect demand done')
#             while not connect_status:
#                 utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
#                 connect_status = sensor.connect(sensor._addr_type, sensor._addr)
# #                 print(v_nc, "::::::::::")
# #                 v_nc += 1
# #             print('connect status ok')
#             
#             
#             while (not sensor.is_connected()
#                    or not sensor._gattc_service_done
#                    or not sensor._gattc_characteristic_done):
#                 utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
#             print('connect ok')
#                 
#     #         while not sensor.is_connected():
#     #             utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
#     #         print("Connected")
#     # 
#     #         while not sensor._gattc_service_done:
#     #             utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
#     #      
#     #         while not sensor._gattc_characteristic_done:
#     #             utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
# 
# 
#         #         if WIFI_CONNECT:
#         #             my_wifi = wifi_esp32.WifiEsp32('jmb-guest', 'pravidondaz')  # initialize the class
#         #             my_wifi.connect_wifi()  # connect to the wifi network
#         #             
#         #             if RTC_SYNC:
#         #                 my_rtc = rtc_esp32.RtcEsp32()  # initialize the class
#         #                 my_rtc.rtc_init()  # initialize the rtc with local date and time
#                 
#         #         # extract the date and time values in str format and print it
#         #         if WIFI_CONNECT and RTC_SYNC:
#         #             now = my_rtc.rtc_now()  # get date and time
#         #             datetime_formated = my_rtc.format_datetime(now)
#         #             print("now date and time :", datetime_formated)
# 
#         #         temp28 = bme28.temperature
#         #         hum28 = bme28.humidity
#         #         pres28 = bme28.pressure
#             
#             temp68 = int(bme68.temperature*100)/100 #bme68.temperature
#             hum68 = int(bme68.humidity) #bme68.humidity
#             pres68 = int(bme68.pressure) #bme68.pressure
#             gas68 = int(bme68.gas / 1000)
#             gaspc = int(100*gas68/75)
#             alt68 = int(bme68.altitude)
# 
#             print('temperature -->', temp68, '°C') #, 'temp 280 -->', temp28, '°C')
#             print('humidite ----->', hum68, '%') #, 'hum 280 -->', hum28, '%')
#             print('pression ----->', pres68, 'hPa') #, 'pres 280 -->', pres28, 'hPa')
#             print('gaz ---------->', gas68, 'Kohms', '=^=', gaspc, '%')
#             print('altitude ----->', alt68, 'm')
#             print('bat ---------->', '{:.2f}'.format(ubatt.voltage / 1000), 'V')
# 
#             
#             msg = 'jmb ' + str(temp68) + ' ' + str(hum68) + ' ' + str(pres68) + ' ' + str(i)
#             sensor.write(msg)
# #             print('msg sended to -->', sensor._name.replace('\n', '')) # , ubinascii.hexlify(bytes(sensor._addr)).decode('utf-8'))
#                 
#             elapsed = utime.ticks_ms() - t_start_total
#             print('pass:', i, '-->',  str((utime.ticks_ms() - t_start_total)/1000) + 's', )
#             print('going to sleep for ' + str(int((T_BEFORE_DEEPSLEEP_MS + T_DEEPSLEEP_MS - elapsed)/1000)) + 's')
#             
# 
#             sensor.disconnect()
#             while not sensor._irq_peripheral_disconnect:
#                 utime.sleep_ms(T_WAIT_FOR_IRQ_TERMINATED_MS)
#                 
#         print(central_scan._irq_list)
#             sensor._irq_list = []
#             
#             print('============================================================================')
#             utime.sleep_ms(T_BEFORE_DEEPSLEEP_MS + T_DEEPSLEEP_MS - elapsed)
#         
# #         print('going to deepsleep for: ' + str(int((T_BEFORE_DEEPSLEEP_MS + T_DEEPSLEEP_MS - elapsed)/1000)) + 's')
# #         utime.sleep_ms(T_BEFORE_DEEPSLEEP_MS)
# #         machine.deepsleep(T_DEEPSLEEP_MS - elapsed)
#         
# #     except:
# #         try:
# #             with open ('index.txt', 'r') as f:
# #                 i = int(f.readline())
# #         except:
# #             i = 1
# # #         restart_ESP32(i, msg)
# #         msg = str(i) + ' - restart_ESP32: ' 
# #         print(msg)
# #         with open('error.txt' , 'a') as f:
# #             f.write(msg+'\n')
# #         utime.sleep_ms(10000)
# #         machine.reset()
#         

if __name__ == "__main__":
    main()
