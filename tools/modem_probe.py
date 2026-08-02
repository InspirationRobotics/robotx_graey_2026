#!/usr/bin/env python3
"""Standalone health check for the Delphis/Nanomodem-style acoustic modem.

Sends the '$?' status query (safe in air - it does NOT drive the acoustic
transducer) and decodes the reply: modem address + supply voltage.
Protocol per the RoboSub team's auv/device/modems/modems_api.py: 9600 8N1.

usage: python3 modem_probe.py [port]
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else \
    '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

print(f'opening {PORT} @ 9600')
with serial.Serial(PORT, 9600, parity=serial.PARITY_NONE,
                   stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,
                   timeout=1) as ser:
    time.sleep(0.3)
    ser.reset_input_buffer()
    for attempt in range(3):
        ser.write(b'$?')
        time.sleep(0.5)
        out = ser.read(ser.in_waiting or 32)
        print(f'attempt {attempt+1}: raw = {out!r}')
        if out:
            try:
                d = out.decode('ascii', 'replace')
                addr = d[2:5]
                volts = round(int(d[6:11]) * 15 / 65536, 2)
                print(f'  --> MODEM ADDRESS: {addr}   VOLTAGE: {volts} V')
            except (ValueError, IndexError):
                print('  --> replied, but could not parse (still proves it is alive)')
            break
    else:
        print('no response - modem silent')
