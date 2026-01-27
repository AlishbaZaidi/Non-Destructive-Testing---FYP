import serial, time
import matplotlib.pyplot as plt
import numpy as np

PORT = 'COM12'
BAUD = 115200

def checksum(data):
    return (~sum(data)) & 0xFF

def send(ser, cmd, delay=0.05):
    ser.reset_input_buffer()
    ser.write(bytes(cmd))
    ser.flush()
    time.sleep(delay)
    return ser.read(ser.in_waiting)

ser = serial.Serial(PORT, BAUD, bytesize=8, stopbits=2, parity='N', timeout=1)
time.sleep(0.1)

print("Listening...")
listen = [0x55, 0x02, 0x01]
listen.append(checksum(listen[1:]))
send(ser, listen, delay=0.3)

print("Reading echo RAM...")
read = [0x55, 0x0B, 0x00, 0x80]  # 128 samples
read.append(checksum(read[1:]))
resp = send(ser, read, delay=0.1)

ser.close()

echo = np.array(list(resp[1:-1]))
print("Samples:", len(echo))

plt.figure()
plt.plot(echo)
plt.title("Ultrasonic Echo @ 300 kHz")
plt.xlabel("Sample Index")
plt.ylabel("Amplitude")
plt.grid(True)
plt.show()
