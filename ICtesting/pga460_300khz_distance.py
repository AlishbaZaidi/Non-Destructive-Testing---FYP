import serial, time
import numpy as np
import matplotlib.pyplot as plt

PORT = 'COM12'
BAUD = 115200
SAMPLE_PERIOD = 8e-6      # 8 µs per sample (PGA460 echo RAM)
SPEED_OF_SOUND = 343     # m/s

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

listen = [0x55, 0x02, 0x01]
listen.append(checksum(listen[1:]))
send(ser, listen, delay=0.3)

read = [0x55, 0x0B, 0x00, 0x80]
read.append(checksum(read[1:]))
resp = send(ser, read, delay=0.1)

ser.close()

echo = np.array(list(resp[1:-1]))
peak_idx = np.argmax(echo)

tof = peak_idx * SAMPLE_PERIOD
distance_m = (tof * SPEED_OF_SOUND) / 2
distance_cm = distance_m * 100

print(f"Peak sample: {peak_idx}")
print(f"Estimated distance: {distance_cm:.2f} cm")

plt.figure()
plt.plot(echo)
plt.axvline(peak_idx, linestyle='--')
plt.title("300 kHz Echo with Detected Peak")
plt.xlabel("Sample Index")
plt.ylabel("Amplitude")
plt.grid(True)
plt.show()
