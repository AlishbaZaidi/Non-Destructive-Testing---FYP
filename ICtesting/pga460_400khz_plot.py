import serial
import time
import matplotlib.pyplot as plt

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
# STEP 4: LISTEN
# =========================
print("=== STEP 4: LISTEN FOR ULTRASONIC SIGNAL ===")

listen_cmd = [0x55, 0x02, 0x01]   # Listen Only, Preset 1
listen_cmd.append(checksum(listen_cmd[1:]))

send(ser, listen_cmd, delay=0.3)
print("Listen command sent.\n")

# =========================
# STEP 5: READ ECHO DATA
# =========================
print("=== STEP 5: READ ECHO DATA ===")

# Read 128 bytes from Echo Data RAM starting at 0x00
read_cmd = [0x55, 0x0B, 0x00, 0x80]
read_cmd.append(checksum(read_cmd[1:]))

response = send(ser, read_cmd, delay=0.1)
print(f"Raw response: {response}")

ser.close()

# =========================
# PROCESS DATA
# =========================
if len(response) < 5:
    print("\n❌ No echo data received.")
    exit()

# Remove diagnostic byte (first) and checksum (last)
echo_data = list(response[1:-1])

print(f"\nReceived {len(echo_data)} echo samples.")

# =========================
# PLOT DATA
# =========================
plt.figure(figsize=(10,5))
plt.plot(echo_data, linewidth=1)
plt.title("PGA460 Ultrasonic Echo Signal @ 400 kHz")
plt.xlabel("Sample Index")
plt.ylabel("Amplitude")
plt.grid(True)
plt.show()
