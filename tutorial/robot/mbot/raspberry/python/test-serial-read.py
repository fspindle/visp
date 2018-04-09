#!/usr/bin/env python

# Rapberry connected to a laptop

import time
import serial

ser = serial.Serial(
  port='/dev/ttyUSB0',
  baudrate = 9600,
  timeout=0
)
          
while True:
  data = ser.read(9999)
  if len(data) > 0:
    print 'Got:', len(data), ' data:', data

  time.sleep(0.5)
  print 'not blocked'

ser.close()
