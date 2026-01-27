import serial

ser = serial.Serial(
    port='COM12',       # change to your COM port
    baudrate=115200,
    bytesize=8,
    stopbits=2,
    parity='N',
    timeout=1
)

print("Connected to PGA460")
ser.close()
