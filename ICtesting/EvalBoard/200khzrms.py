import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ================= USER SETTINGS =================
COM_PORT = "COM11"
BAUDRATE = 9600
NUM_SAMPLES = 128
FS = 1_000_000  # 1 MHz ADC sampling rate
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
write_register(0x1A, 0x80)

print("Setting frequency register...")
write_register(0x16, 33)

print("Writing band-pass filter coefficients...")
fir_coeffs = [
    0x03, 0x05, 0xFC, 0xE6, 0xF5, 0x22,
    0x22, 0xF5, 0xE6, 0xFC, 0x05, 0x03
]

for i, val in enumerate(fir_coeffs):
    write_register(0x41 + i, val)


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
echo_signed = echo.astype(np.int16) - 128
print("Raw samples:", echo)


t = np.arange(NUM_SAMPLES) / FS  # seconds

# ================= METRICS =================
rms_energy = np.sqrt(np.mean(echo_signed ** 2))
ptp = np.ptp(echo_signed)

envelope = np.abs(echo_signed)
decay = envelope / np.max(envelope)

print(f"Diagnostic Byte: 0x{diag:02X}")
print(f"RMS Energy: {rms_energy:.2f}")
print(f"Peak-to-Peak: {ptp}")

# ================= TIME DOMAIN =================
plt.figure(figsize=(10,4))
plt.plot(t * 1e6, echo_signed)
plt.title("PGA460 Echo Envelope (HF Mode, Carrier = 200 kHz)")
plt.xlabel("Time (µs)")
plt.ylabel("Amplitude (ADC centered)")
plt.grid(True)
plt.show()

# ================= ECHO DECAY =================
plt.figure(figsize=(10,4))
plt.plot(t * 1e6, decay)
plt.title("Echo Decay vs Time")
plt.xlabel("Time (µs)")
plt.ylabel("Normalized Amplitude")
plt.grid(True)
plt.show()

# ================= FFT =================
fft = np.fft.fft(echo_signed)
freq = np.fft.fftfreq(len(fft), d=1/FS)
print("FFT Samples: ", freq)

plt.figure(figsize=(10,4))
plt.plot(freq[:len(freq)//2] / 1000, np.abs(fft[:len(freq)//2]))
plt.title("FFT of Echo (Down-Converted Band around 200 kHz)")
plt.xlabel("Frequency (kHz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.show()
