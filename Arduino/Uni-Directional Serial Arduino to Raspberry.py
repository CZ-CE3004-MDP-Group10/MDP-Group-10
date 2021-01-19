#!/usr/bin/env python3
import serial

if __name__ == '__main__':
    # Baud rate may need to increase to 115200.
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)