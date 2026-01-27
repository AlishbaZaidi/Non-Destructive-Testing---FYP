import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ================= USER SETTINGS =================
COM_PORT = "COM11"
BAUDRATE = 9600
NUM_SAMPLES = 128
# =================================================

def checksum(data):
    return (~(sum(data) & 0xFF)) & 0xFF

# ---------------- UART ----------------
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def write_register(addr, value):
    pkt = [0x55, 0x0A, addr, value & 0xFF]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.01)

# =================================================
# STEP 1: HIGH FREQUENCY MODE ENABLE
# =================================================
print("Enabling High Frequency Mode (200 kHz)...")
write_register(0x1A, 0x80)  # TVGAIN6 → FREQ_SHIFT = 1

# =================================================
# STEP 2: SET FREQUENCY
# 200 kHz / 6 ≈ 33
# =================================================
write_register(0x16, 33)

# =================================================
# STEP 3: FIR BANDPASS COEFFICIENTS (YOUR VALUES)
# =================================================
print("Writing FIR Band-Pass Coefficients...")

fir_coeffs = [
    0x03, 0x05, 0xFC, 0xE6, 0xF5, 0x22,
    0x22, 0xF5, 0xE6, 0xFC, 0x05, 0x03
]

for i, val in enumerate(fir_coeffs):
    write_register(0x41 + i, val)

time.sleep(0.05)

# =================================================
# STEP 4: LISTEN ONLY COMMAND
# =================================================
print("Listening...")
cmd_listen = [0x55, 0x01, 0x00]
cmd_listen.append(checksum(cmd_listen[1:]))
ser.write(bytearray(cmd_listen))
time.sleep(0.05)

# =================================================
# STEP 5: READ ECHO DATA
# =================================================
print("Reading echo data...")
cmd_echo = [0x55, 0x07]
cmd_echo.append(checksum(cmd_echo[1:]))
ser.write(bytearray(cmd_echo))

raw = ser.read(1 + NUM_SAMPLES + 1)
ser.close()

# =================================================
# STEP 6: PARSE DATA
# =================================================
diag = raw[0]
echo = np.frombuffer(raw[1:-1], dtype=np.uint8)
echo_signed = echo.astype(np.int16) - 128
time_us = np.arange(NUM_SAMPLES)

print(f"Diagnostic Byte: 0x{diag:02X}")
print("Raw samples:", echo)

# =================================================
# STEP 7: TIME DOMAIN PLOT
# =================================================
plt.figure(figsize=(10,4))
plt.plot(time_us, echo_signed)
plt.title("PGA460 Echo Waveform – 200 kHz (HF Mode)")
plt.xlabel("Time (µs)")
plt.ylabel("Amplitude")
plt.grid(True)
plt.show()

# =================================================
# STEP 8: FFT
# =================================================
fft = np.fft.fft(echo_signed)
freq = np.fft.fftfreq(len(fft), d=1e-6)
print("FFT Samples: ", freq)

plt.figure(figsize=(10,4))
plt.plot(freq[:len(freq)//2], np.abs(fft[:len(freq)//2]))
plt.title("Frequency Spectrum")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.show()
