import serial

ser = serial.Serial(
    port='COM12',
    baudrate=115200,
    bytesize=8,
    stopbits=2,
    parity='N',
    timeout=1
)

# Device status / communication test
cmd = bytes([0x55, 0x00, 0x00, 0xAB])

ser.write(cmd)
print("Status command sent")

response = ser.read(16)
print("Response:", response)

ser.close()
