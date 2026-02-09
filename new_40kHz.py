import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# USER SETTINGS
# ============================================================
COM_PORT = "/dev/cu.usbserial-14140"
BAUDRATE = 115200

NUM_SAMPLES = 128

# IMPORTANT:
# Echo Data Dump is DSP output (not raw ADC)
# Effective sample rate ≈ 125 kHz
FS = 125_000

# ============================================================
# HELPER FUNCTIONS
# ============================================================
def checksum(data):
    """
    PGA460 checksum:
    8-bit one's complement of the sum of all bytes (excluding 0x55)
    """
    return (~(sum(data) & 0xFF)) & 0xFF


def write_register(addr, value):
    """
    Write a single PGA460 register using UART.
    EEPROM is NOT programmed — this is volatile only.
    """
    pkt = [0x55, 0x0A, addr, value]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.01)


def burst_and_read(pulses):
    """
    Execute Burst + Listen, then read Echo Data Dump.
    pulses = number of excitation pulses (1 for time-domain, 16 for FFT)
    """
    # BURST + LISTEN Preset1
    cmd = [0x55, 0x00, pulses]
    cmd.append(checksum(cmd[1:]))
    ser.write(bytearray(cmd))

    time.sleep(0.12)  # allow DSP + echo capture to complete

    # Echo Data Dump command
    cmd_echo = [0x55, 0x07]
    cmd_echo.append(checksum(cmd_echo[1:]))
    ser.write(bytearray(cmd_echo))

    raw = ser.read(130)  # 1 diag + 128 samples + checksum
    return raw


# ============================================================
# SERIAL INIT
# ============================================================
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
time.sleep(2)

# ============================================================
# PGA460 CONFIGURATION
# ============================================================
print("Configuring PGA460 for 40 kHz operation...")

# ------------------------------------------------------------
# 1) Set burst frequency to EXACTLY 40 kHz
#
# Formula:
# f = 30 kHz + (FREQ × 0.2 kHz)
# FREQ = (40 - 30) / 0.2 = 50 decimal = 0x32
# ------------------------------------------------------------
write_register(0x16, 0x32)

# ------------------------------------------------------------
# 2) Set initial analog gain
# We keep default BPF auto-calculation (DO NOT touch BPF coeffs)
# ------------------------------------------------------------
write_register(0x1B, 0x40)  # ~58 dB gain

# ------------------------------------------------------------
# 3) Do NOT write BPF_A2 / BPF_A3 / BPF_B1
# PGA460 will auto-compute correct Butterworth BPF
# ------------------------------------------------------------

# ============================================================
# TIME-DOMAIN VIEW (1 PULSE)
# ============================================================
print("Capturing time-domain envelope (1 pulse)...")

raw = burst_and_read(pulses=1)

if len(raw) < 130:
    print("Error: insufficient data received.")
    ser.close()
    exit()

echo_td = np.frombuffer(raw[1:-1], dtype=np.uint8).astype(np.int16) - 128

# ============================================================
# FFT VIEW (16 PULSES)
# ============================================================
print("Capturing FFT envelope (16 pulses)...")

raw = burst_and_read(pulses=16)

if len(raw) < 130:
    print("Error: insufficient data received.")
    ser.close()
    exit()

echo_fft = np.frombuffer(raw[1:-1], dtype=np.uint8).astype(np.int16) - 128

ser.close()

# ============================================================
# FFT CALCULATION
# ============================================================
fft_vals = np.fft.fft(echo_fft * np.hanning(NUM_SAMPLES))
freqs = np.fft.fftfreq(NUM_SAMPLES, 1 / FS)

# ============================================================
# PLOTTING
# ============================================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8))

# ------------------------------------------------------------
# Time Domain (Envelope)
# ------------------------------------------------------------
ax1.plot(echo_td, marker='o')
ax1.set_title("Time Domain – Echo Envelope (1 Pulse)")
ax1.set_xlabel("Sample Index (~8 µs per sample)")
ax1.set_ylabel("Amplitude")
ax1.grid(True)

# ------------------------------------------------------------
# Frequency Domain (Envelope Spectrum)
# ------------------------------------------------------------
ax2.plot(freqs[:NUM_SAMPLES // 2] / 1000,
         np.abs(fft_vals[:NUM_SAMPLES // 2]))
ax2.set_title("Frequency Domain – Envelope Spectrum (16 Pulses)")
ax2.set_xlabel("Frequency (kHz)")
ax2.set_ylabel("Magnitude")
ax2.set_xlim(0, 20)  # envelope lives at low frequency
ax2.grid(True)

plt.tight_layout()
plt.show()