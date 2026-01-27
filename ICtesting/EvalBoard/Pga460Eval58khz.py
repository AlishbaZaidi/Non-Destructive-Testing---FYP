import serial
import time
import numpy as np
import matplotlib.pyplot as plt

PORT = "COM11"          
BAUDRATE = 9600
NUM_SAMPLES = 128

ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def checksum(data):
    return (~(sum(data) & 0xFF)) & 0xFF

# ---- LISTEN ONLY ----
cmd_listen = [0x55, 0x02, 0x00]
cmd_listen.append(checksum(cmd_listen[1:]))
ser.write(bytearray(cmd_listen))
time.sleep(0.05)

# ---- ECHO DATA DUMP ----
cmd_echo = [0x55, 0x07]
cmd_echo.append(checksum(cmd_echo[1:]))
ser.write(bytearray(cmd_echo))

raw = ser.read(1 + NUM_SAMPLES + 1)
ser.close()

diag = raw[0]
echo = np.frombuffer(raw[1:-1], dtype=np.uint8)

print(f"Diagnostic Byte: 0x{diag:02X}")
print("Echo Samples:", echo)

# ---- CENTER ADC (IMPORTANT) ----
echo_signed = echo.astype(np.int16) - 128
time_us = np.arange(NUM_SAMPLES)



fft = np.fft.fft(echo_signed)
freq = np.fft.fftfreq(len(fft), d=1e-6)
print("FFT Samples: ", fft)

# ---- PLOT ----
plt.figure(figsize=(10, 4))
plt.plot(time_us, echo_signed)
plt.title("PGA460 Echo Waveform (80 kHz)")
plt.xlabel("Time (µs)")
plt.ylabel("Amplitude (Centered ADC)")
plt.grid(True)
plt.show()

plt.figure(figsize=(8,4))
plt.plot(freq[:len(freq)//2], np.abs(fft[:len(fft)//2]))
plt.title("Frequency Spectrum")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.show()
