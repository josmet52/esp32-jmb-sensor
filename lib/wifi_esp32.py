#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
project : wifi_esp32.py
----------------------------
Class to connect on WLAN

CLASS : WifiEsp32
PROCESSOR : ESP32
OS : micropython and loboris_micropython
uC : WEMOS MINI D1
VERSION : 1.0.0
DATE : 12.06.2021
jmb52.dev@gmail.com
"""
import network
import utime

class WifiEsp32:
    
    def __init__(self, ssid, pw):
        
        self.WIFI_SSID = ssid
        self.WIFI_PW = pw
        
    def connect_wifi(self):
        ssid = self.WIFI_SSID 
        password = self.WIFI_PW
        station = network.WLAN(network.STA_IF)
        station.active(True)
        station.connect(ssid, password)

        # check connection
        tmo = 100
        while not station.isconnected():
           utime.sleep_ms(100)
           tmo -= 1
           if tmo == 0:
               print('wifi connection breaked')
               break
        print('wifi connected on ' + self.WIFI_SSID + ' -> ' + str(station.isconnected()))
        print('network config:', station.ifconfig())
        
    
# demo prg for this class
if __name__ == '__main__':
    
    print('---------------------------------------------------------')
    my_wifi = WifiEsp32('jmb-guest', 'pravidondaz')  # initialize the class
    my_wifi.connect_wifi()  # connect to the wifi network
    print('---------------------------------------------------------')

