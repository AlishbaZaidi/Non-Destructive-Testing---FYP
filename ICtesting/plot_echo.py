import serial
import matplotlib.pyplot as plt

ser = serial.Serial(
    port='COM12',
    baudrate=115200,
    bytesize=8,
    stopbits=2,
    parity='N',
    timeout=1
)

# Tell PGA460 to listen
listen_cmd = bytes([0x55, 0x09, 0x00, 0xF6])
ser.write(listen_cmd)

# Read echo memory
read_cmd = bytes([0x55, 0x0A, 0x00, 0xF5])
ser.write(read_cmd)

data = ser.read(128)
ser.close()

echo = list(data)

# Plot
plt.figure(figsize=(10,5))
plt.plot(echo)
plt.title("Ultrasonic Signal from PGA460 (300 kHz)")
plt.xlabel("Sample Number (Time)")
plt.ylabel("Amplitude")
plt.grid(True)
plt.show()
