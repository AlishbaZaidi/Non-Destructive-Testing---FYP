import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ---------------- USER SETTINGS ----------------
PORT = "COM11"          
BAUDRATE = 9600
TIMEOUT = 1            # seconds
NUM_SAMPLES = 128      
# ------------------------------------------------

# Open serial port
ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=TIMEOUT
)

time.sleep(2)
ser.reset_input_buffer()

def checksum(bytes_list):
    """PGA460 checksum: inverted byte sum with carry"""
    s = sum(bytes_list) & 0xFF
    return (~s) & 0xFF

# ---------------- COMMAND 2: LISTEN ONLY (Preset 1) ----------------
# Command format: 55 | CMD | DATA | CRC
cmd_listen = [0x55, 0x02, 0x00]
cmd_listen.append(checksum(cmd_listen[1:]))

ser.write(bytearray(cmd_listen))
time.sleep(0.05)  # wait for listening window

# ---------------- COMMAND 7: ECHO DATA DUMP ----------------
cmd_echo = [0x55, 0x07]
cmd_echo.append(checksum(cmd_echo[1:]))

ser.write(bytearray(cmd_echo))

# ---------------- READ RESPONSE ----------------
# Expected:
# 1 byte  - Diagnostic
# 128 bytes - Echo data
# 1 byte  - Checksum
expected_bytes = 1 + NUM_SAMPLES + 1
raw = ser.read(expected_bytes)
print(raw)
if len(raw) != expected_bytes:
    print("Error: Incomplete data received")
    ser.close()
    exit()

diag = raw[0]
echo_data = raw[1:-1]
crc = raw[-1]

print(f"Diagnostic Byte: 0x{diag:02X}")
print("First 10 Echo Samples:", list(echo_data[:10]))

# ---------------- PLOT ----------------
samples = np.array(echo_data)

plt.figure(figsize=(10, 4))
plt.plot(samples)
plt.title("PGA460 Echo Data (80 kHz)")
plt.xlabel("Sample Index (1 µs per sample)")
plt.ylabel("Amplitude (ADC units)")
plt.grid(True)
plt.tight_layout()
plt.show()

ser.close()