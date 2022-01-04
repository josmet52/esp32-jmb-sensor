from machine import Pin, Timer, SoftI2C
from time import sleep_ms
import ubluetooth
# from esp32 import raw_temperature
# from hdc1080 import HDC1080

class BLE():
    def __init__(self, name):   
#         print('---> def __init__')
        self.name = name
        self.ble = ubluetooth.BLE()
        self.ble.active(True)

        self.led = Pin(2, Pin.OUT)
        self.timer1 = Timer(0)
        self.timer2 = Timer(1)
        
        self.disconnected()
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()

    def connected(self):        
#         print('---> def connected')
        self.timer1.deinit()
        self.timer2.deinit()

    def disconnected(self):        
#         print('---> def disconnected')
        self.timer1.init(period=1000, mode=Timer.PERIODIC, callback=lambda t: self.led(1))
        sleep_ms(200)
        self.timer2.init(period=1000, mode=Timer.PERIODIC, callback=lambda t: self.led(0))

    def ble_irq(self, event, data):
#         print('---> def ble_irq', 'event', event, 'data', data)
        if event == 1:
            '''Central disconnected'''
            self.connected()
            self.led(1)
        
        elif event == 2:
            '''Central disconnected'''
            self.advertiser()
            self.disconnected()
        
        elif event == 3:
            '''New message received'''            
            buffer = self.ble.gatts_read(self.rx)
            message = buffer.decode('UTF-8').strip()
            if message[:3] == 'jmb':
#                 print(message)
                data = message.split(' ')
#                 print(data)
                msg = 'temp:' + data[1] + '°C hum:' + data[2] + '% pres:' + data[3] + 'hPa air:' + data[4] +'%'
                print(msg, '\n')
                
            if message == 'blue_led':
                blue_led.value(not blue_led.value())
                print('blue_led', blue_led.value())
                ble.send('blue_led' + str(blue_led.value()))
            if message == 'read_temp':
#                 print(sensor.read_temperature(True))
#                 ble.send(str(sensor.read_temperature(True)))
                print('--temperature--val--')
                ble.send('--temperature--val--')
            if message == 'read_hum':
#                 print(sensor.read_humidity())
#                 ble.send(str(sensor.read_humidity()))
                print('--humidity--val--')
                ble.send('--humidity--val--')
           
    def register(self):        
#         print('---> def register')
        # Nordic UART Service (NUS)
        NUS_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
        RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
        TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
            
        BLE_NUS = ubluetooth.UUID(NUS_UUID)
        BLE_RX = (ubluetooth.UUID(RX_UUID), ubluetooth.FLAG_WRITE)
        BLE_TX = (ubluetooth.UUID(TX_UUID), ubluetooth.FLAG_NOTIFY)
            
        BLE_UART = (BLE_NUS, (BLE_TX, BLE_RX,))
        SERVICES = (BLE_UART, )
        ((self.tx, self.rx,), ) = self.ble.gatts_register_services(SERVICES)

    def send(self, data):
#         print('---> def send', 'data', data)
        self.ble.gatts_notify(0, self.tx, data + '\n')

    def advertiser(self):
        print('---> def advertiser')
        name = bytes(self.name, 'UTF-8')
        self.ble.gap_advertise(100, bytearray('\x02\x01\x02') + bytearray((len(name) + 1, 0x09)) + name)
        
# test
# i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
# sensor = HDC1080(i2c)
blue_led = Pin(2, Pin.OUT)
ble = BLE("jmb_central_sens")

