import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ================= SETTINGS =================
COM_PORT = "/dev/cu.usbserial-14130" # Updated from your terminal
BAUDRATE = 115200
NUM_SAMPLES = 300
FS = 1_000_000 # The PGA460 internal ADC sample rate is 1MHz

def checksum(data):
    return (~(sum(data) & 0xFF)) & 0xFF

ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
time.sleep(2)

def write_register(addr, value):
    pkt = [0x55, 0x0A, addr, value]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.01)

# ================= CONFIGURATION =================
print("Configuring for 40 kHz Single Pulse...")
# 1. Disable High Frequency Mode [cite: 137, 155] 
# 0x1A is TV_GAIN_6 register with value set to 0
write_register(0x1A, 0x00) 
# 2. Set Frequency to 40kHz (approx 0x23)
# 0x16 is REC_LENGTH register with value set 35 ( it is echo data record period)
write_register(0x16, 0x23)
# 3. Set Pulse Count to 1 (Single Pulse)
# 0x1B is INIT Gain register with value set to 1 ( AFE initiual gain configuration )
# value is 59 db according to datasheet
write_register(0x1B, 0x01)

# ================= BURST + LISTEN =================
print("Transmitting and Listening...")
cmd = [0x55, 0x00, 0x01] 
cmd.append(checksum(cmd[1:]))
ser.write(bytearray(cmd))
time.sleep(0.1) 

# ================= READ ECHO DATA = [cite: 30] =================
print("Reading Echo Data Dump...")
cmd_echo = [0x55, 0x07]
cmd_echo.append(checksum(cmd_echo[1:]))
ser.write(bytearray(cmd_echo))

raw = ser.read(1 + NUM_SAMPLES + 1)
ser.close()

if len(raw) < (NUM_SAMPLES + 2):
    print("Error: No data received. Check 6V power and GND connections.")
else:
    # Remove diagnostic byte and checksum [cite: 30]
    echo = np.frombuffer(raw[1:-1], dtype=np.uint8)
    echo_signed = echo.astype(np.int16) - 128

    # --- FFT Analysis ---
    fft_vals = np.fft.fft(echo_signed)
    freqs = np.fft.fftfreq(NUM_SAMPLES, 1/FS)
    
    # Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    ax1.plot(echo_signed)
    ax1.set_title("Time Domain: 40 kHz Single Pulse")
    ax1.set_xlabel("Sample Index (1 sample = 1µs)")
    ax1.set_ylabel("Amplitude")
    
    ax2.plot(freqs[:NUM_SAMPLES//2]/1000, np.abs(fft_vals[:NUM_SAMPLES//2]))
    ax2.set_title("Frequency Domain (FFT)")
    ax2.set_xlabel("Frequency (kHz)")
    ax2.set_ylabel("Magnitude")
    ax2.set_xlim(0, 100) # Focus on 0-100kHz range
    
    plt.tight_layout()
    plt.show()