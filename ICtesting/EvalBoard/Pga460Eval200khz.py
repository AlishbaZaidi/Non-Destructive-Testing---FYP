import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ================= USER SETTINGS =================
COM_PORT = "COM11"       # CHANGE THIS
BAUDRATE = 9600
NUM_SAMPLES = 128
# =================================================

# ---------------- CHECKSUM FUNCTION ----------------
def checksum(data):
    return (~(sum(data) & 0xFF)) & 0xFF

# ---------------- UART SETUP ----------------
ser = serial.Serial(
    port=COM_PORT,
    baudrate=BAUDRATE,
    timeout=1
)
time.sleep(2)
ser.reset_input_buffer()

# ---------------- REGISTER WRITE FUNCTION ----------------
def write_register(addr, value):
    pkt = [0x55, 0x0A, addr, value]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.01)

# =========================================================
# STEP 1: ENABLE HIGH FREQUENCY MODE (FREQ_SHIFT = 1)
# TVGAIN6 = 0x1A
# Bit7 = FREQ_SHIFT
# =========================================================
print("Enabling High Frequency Mode...")
write_register(0x1A, 0x80)   # FREQ_SHIFT = 1, gain unchanged

# =========================================================
# STEP 2: SET FREQUENCY REGISTER
# 200 kHz / 6 ≈ 33.3 kHz → FREQUENCY ≈ 33
# FREQUENCY register = 0x16
# =========================================================
print("Setting frequency register...")
write_register(0x16, 33)

# =========================================================
# STEP 3: BAND-PASS FILTER COEFFICIENTS (MANDATORY)
# Registers: 0x41 – 0x46
# Known-good starting values for ~200 kHz
# =========================================================
print("Writing band-pass filter coefficients...")
bp_coeffs = {
    0x41: 0xD1, # 65, 209
    0x42: 0x4E, # 66, 78
    0x43: 0xF9, # 67, 249
    0x44: 0xA5, # 68, 165
    0x45: 0x03, # 69, 03
    0x46: 0x2D  # 70, 45
}

for addr, val in bp_coeffs.items():
    write_register(addr, val)

time.sleep(0.05)

# =========================================================
# STEP 4: Burst + LISTEN ONLY (COMMAND 1)
# =========================================================
print("Listening...")
cmd_listen = [0x55, 0x01, 0x00]
cmd_listen.append(checksum(cmd_listen[1:]))
ser.write(bytearray(cmd_listen))
time.sleep(0.05)

# =========================================================
# STEP 5: ECHO DATA DUMP (COMMAND 7)
# =========================================================
print("Reading echo data...")
cmd_echo = [0x55, 0x07]
cmd_echo.append(checksum(cmd_echo[1:]))
ser.write(bytearray(cmd_echo))

raw = ser.read(1 + NUM_SAMPLES + 1)
ser.close()

# =========================================================
# STEP 6: PARSE DATA
# =========================================================
diag = raw[0]
echo = np.frombuffer(raw[1:-1], dtype=np.uint8)
echo_signed = echo.astype(np.int16) - 128
time_us = np.arange(NUM_SAMPLES)

print(f"Diagnostic Byte: 0x{diag:02X}")
print("Raw samples:", echo)

# =========================================================
# STEP 7: TIME DOMAIN PLOT
# =========================================================
plt.figure(figsize=(10,4))
plt.plot(time_us, echo_signed)
plt.title("PGA460 Echo Waveform – 200 kHz")
plt.xlabel("Time (µs)")
plt.ylabel("Amplitude (Centered ADC)")
plt.grid(True)
plt.show()

# =========================================================
# STEP 8: FFT (FREQUENCY CONFIRMATION)
# =========================================================
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
