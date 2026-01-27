import serial

ser = serial.Serial(
    port='COM12',
    baudrate=115200,
    bytesize=8,
    stopbits=2,
    parity='N',
    timeout=1
)

# Command to read echo RAM (example)
read_cmd = bytes([0x55, 0x0A, 0x00, 0xF5])
ser.write(read_cmd)

data = ser.read(128)
print("Received bytes:", len(data))

echo = list(data)
ser.close()

print(echo)
