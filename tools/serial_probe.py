#!/usr/bin/env python3
"""Probe a serial port: listen, optionally poke, sweep bauds.
usage: python3 serial_probe.py /dev/ttyUSB1 [baud|sweep] [seconds]"""
import sys, time, serial

port = sys.argv[1]
mode = sys.argv[2] if len(sys.argv) > 2 else "sweep"
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
bauds = [4800, 9600, 19200, 38400, 57600, 115200] if mode == "sweep" else [int(mode)]

for b in bauds:
    print(f"\n--- {port} @ {b} ---", flush=True)
    try:
        with serial.Serial(port, b, timeout=0.3) as s:
            time.sleep(2.0)                      # let an Arduino finish resetting
            s.reset_input_buffer()
            s.write(b"\r\n?\r\n")                # harmless poke
            end, got = time.time() + secs, False
            while time.time() < end:
                d = s.read(256)
                if d:
                    got = True
                    sys.stdout.write(d.decode("ascii", "replace"))
                    sys.stdout.flush()
            if not got:
                print("(silence)")
    except Exception as e:
        print(f"(error: {e})")
