import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ================= USER SETTINGS =================
COM_PORT = "COM11"
BAUDRATE = 9600
NUM_SAMPLES = 128
FS = 1_000_000   # 1 MHz ADC sampling rate
# =================================================

def checksum(data):
    return (~(sum(data) & 0xFF)) & 0xFF

ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def write_register(addr, value):
    pkt = [0x55, 0x0A, addr, value]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.01)

# ================= CONFIGURATION =================
print("Enabling High Frequency Mode...")
write_register(0x1A, 0x80)   # FREQ_SHIFT = 1

print("Setting frequency register for 300 kHz...")
write_register(0x16, 50)     # 300 kHz / 6

print("Writing 300 kHz band-pass FIR coefficients...")
fir_coeffs = [ 0xFF, 0xFF, 0x01, 0x00, 0xFD, 0x02, 0x03, 0xFA,
  0x00, 0x09, 0xF9, 0xF8, 0x0E, 0x00, 0xF0, 0x0A,
  0x0A, 0xF0, 0x00, 0x0E, 0xF8, 0xF9, 0x09, 0x00,
  0xFA, 0x03, 0x02, 0xFD, 0x00, 0x01, 0xFF, 0xFF ]

for i, val in enumerate(fir_coeffs):
    write_register(0x41 + i, val)

time.sleep(0.05)

# ================= LISTEN =================
print("Listening...")
cmd = [0x55, 0x01, 0x00]
cmd.append(checksum(cmd[1:]))
ser.write(bytearray(cmd))
time.sleep(0.05)

# ================= READ ECHO =================
print("Reading echo data...")
cmd = [0x55, 0x07]
cmd.append(checksum(cmd[1:]))
ser.write(bytearray(cmd))

raw = ser.read(1 + NUM_SAMPLES + 1)
ser.close()

diag = raw[0]
echo = np.frombuffer(raw[1:-1], dtype=np.uint8)
signal = echo.astype(np.int16) - 128
print("Raw samples:", echo)
t = np.arange(NUM_SAMPLES) / FS  # seconds

# ================= METRICS =================
rms_energy = np.sqrt(np.mean(signal ** 2))
ptp = np.ptp(signal)

envelope = np.abs(signal)
decay = envelope / np.max(envelope)

print(f"\nDiagnostic Byte: 0x{diag:02X}")
print(f"RMS Energy: {rms_energy:.2f}")
print(f"Peak-to-Peak: {ptp}")

# ================= TIME DOMAIN =================
plt.figure(figsize=(10,4))
plt.plot(t * 1e6, signal)
plt.title("PGA460 Echo Waveform (HF Mode, Carrier = 300 kHz)")
plt.xlabel("Time (µs)")
plt.ylabel("Amplitude (ADC centered)")
plt.grid(True)
plt.show()

# ================= ECHO DECAY =================
plt.figure(figsize=(10,4))
plt.plot(t * 1e6, decay)
plt.title("Echo Decay vs Time (300 kHz)")
plt.xlabel("Time (µs)")
plt.ylabel("Normalized Amplitude")
plt.grid(True)
plt.show()

# ================= FFT =================
fft = np.fft.fft(signal)
freq = np.fft.fftfreq(len(fft), d=1/FS)
print("FFT Samples: ", freq)
mag = np.abs(fft)

half = len(freq)//2
peak_freq = freq[:half][np.argmax(mag[:half])]

print(f"Dominant Frequency: {peak_freq/1000:.1f} kHz")

plt.figure(figsize=(10,4))
plt.plot(freq[:half] / 1000, mag[:half])
plt.axvline(300, linestyle='--', label="Expected 300 kHz")
plt.title("FFT of Echo Signal (300 kHz)")
plt.xlabel("Frequency (kHz)")
plt.ylabel("Magnitude")
plt.legend()
plt.grid(True)
plt.show()
