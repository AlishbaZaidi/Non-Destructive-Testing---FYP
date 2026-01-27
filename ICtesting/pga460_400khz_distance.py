import serial
import time
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

# =========================
# OPEN SERIAL PORT
# =========================
print("Opening serial port...")
ser = serial.Serial(PORT, BAUD, bytesize=8, stopbits=2, parity='N', timeout=1)
time.sleep(0.1)
print("Serial port opened.\n")

# =========================
# LISTEN
# =========================
print("Listening...")
listen_cmd = [0x55, 0x02, 0x01]
listen_cmd.append(checksum(listen_cmd[1:]))
send(ser, listen_cmd, delay=0.3)

# =========================
# READ ECHO DATA
# =========================
print("Reading echo data...")
read_cmd = [0x55, 0x0B, 0x00, 0x80]
read_cmd.append(checksum(read_cmd[1:]))

response = send(ser, read_cmd, delay=0.1)
ser.close()

if len(response) < 5:
    print("❌ No echo data received.")
    exit()

# Remove diagnostic + checksum
echo = np.array(list(response[1:-1]))

print(f"Received {len(echo)} samples")

# =========================
# FIND STRONGEST ECHO
# =========================
peak_index = np.argmax(echo)
peak_value = echo[peak_index]

print(f"Peak at sample {peak_index} with amplitude {peak_value}")

# =========================
# DISTANCE CALCULATION
# =========================
SAMPLE_PERIOD = 8e-6   # 8 microseconds per sample
SPEED_OF_SOUND = 343   # m/s

time_of_flight = peak_index * SAMPLE_PERIOD
distance_m = (time_of_flight * SPEED_OF_SOUND) / 2
distance_cm = distance_m * 100

print(f"Estimated distance: {distance_cm:.2f} cm")

# =========================
# PLOT
# =========================
plt.figure(figsize=(10,5))
plt.plot(echo, label="Echo Signal")
plt.axvline(peak_index, color='r', linestyle='--', label="Detected Echo")
plt.title("PGA460 Ultrasonic Echo @ 400 kHz")
plt.xlabel("Sample Index")
plt.ylabel("Amplitude")
plt.legend()
plt.grid(True)
plt.show()
