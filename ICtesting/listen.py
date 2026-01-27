import serial
import time

ser = serial.Serial(
    port='COM5',
    baudrate=115200,
    bytesize=8,
    stopbits=2,
    parity='N',
    timeout=1
)

# LISTEN command (example)
listen_cmd = bytes([0x55, 0x09, 0x00, 0xF6])
ser.write(listen_cmd)

time.sleep(0.1)
print("Listening for echoes...")

ser.close()
