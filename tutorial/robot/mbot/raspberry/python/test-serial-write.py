#!/usr/bin/env python

# Rapberry connected to a laptop

import time
import serial

ser = serial.Serial(
              
  port='/dev/ttyAMA0',
  baudrate = 9600,
)

x = ser.write('hello')
ser.close()

