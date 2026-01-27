import serial
import time

PORT = 'COM12'
BAUD = 115200

def checksum(data):
    return (~sum(data)) & 0xFF

ser = serial.Serial(PORT, BAUD, bytesize=8, stopbits=2, parity='N', timeout=1)
time.sleep(0.1)

print("Reading TVGAIN6 (0x1A)...")

cmd = [0x55, 0x09, 0x1A]     # Read register
cmd.append(checksum(cmd[1:]))

ser.write(bytes(cmd))
time.sleep(0.1)
resp = ser.read(10)

print("Response:", resp)
ser.close()
