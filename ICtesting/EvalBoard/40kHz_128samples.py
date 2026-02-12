import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

# ============================================================
# USER SETTINGS
# ============================================================
COM_PORT = "/dev/cu.usbserial-14130"
BAUDRATE = 115200

NUM_SAMPLES = 128

# IMPORTANT:
# Echo Data Dump is DSP output (not raw ADC)
# Effective sample rate ≈ 125 kHz
FS = 125_000

# File names for saving data - using timestamp to avoid overwriting
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
DATA_FILENAME = f"pga460_samples_{timestamp}.txt"
PLOT_FILENAME = f"pga460_time_domain_plot_{timestamp}.png"

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
    # Clear any existing data in the serial buffer
    ser.reset_input_buffer()
    
    # BURST + LISTEN Preset1
    cmd = [0x55, 0x00, pulses]
    cmd.append(checksum(cmd[1:]))
    ser.write(bytearray(cmd))

    time.sleep(0.15)  # Increased from 0.12 to allow more time for DSP + echo capture

    # Echo Data Dump command
    cmd_echo = [0x55, 0x07]
    cmd_echo.append(checksum(cmd_echo[1:]))
    ser.write(bytearray(cmd_echo))

    # Read exactly 130 bytes (1 diag + 128 samples + 1 checksum)
    # Use longer timeout to ensure we get all data
    raw = ser.read(130)
    
    # If we didn't get all bytes, try reading more
    if len(raw) < 130:
        print(f"Warning: Only received {len(raw)} bytes, expected 130")
        # Try to read remaining bytes
        remaining = 130 - len(raw)
        raw += ser.read(remaining)
        print(f"Read additional {remaining} bytes, total: {len(raw)}")
    
    return raw


def save_samples_to_file(samples, filename):
    """
    Save the samples to a text file with metadata
    """
    with open(filename, 'w') as f:
        # Write header with metadata
        f.write(f"# PGA460 Time-Domain Samples\n")
        f.write(f"# Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# Number of samples: {len(samples)}\n")
        f.write(f"# Sample rate: {FS} Hz\n")
        f.write(f"# Sample period: {1/FS*1e6:.2f} µs\n")
        f.write(f"# Pulse count: 1\n")
        f.write(f"# Format: Sample_Index, Amplitude\n")
        f.write("# ============================================\n")
        
        # Write data
        for i, sample in enumerate(samples):
            f.write(f"{i}, {sample}\n")
    
    print(f"Samples saved to {filename}")
    print(f"Total samples written: {len(samples)}")


def save_plot_to_file(fig, filename):
    """
    Save the plot to a file
    """
    fig.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved to {filename}")


# ============================================================
# SERIAL INIT
# ============================================================
print(f"Connecting to {COM_PORT} at {BAUDRATE} baud...")
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=2)  # Increased timeout to 2 seconds
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
write_register(0x1B, 0x7F)  # ~58 dB gain

# ------------------------------------------------------------
# 3) Do NOT write BPF_A2 / BPF_A3 / BPF_B1
# PGA460 will auto-compute correct Butterworth BPF
# ------------------------------------------------------------

# ============================================================
# TIME-DOMAIN VIEW (1 PULSE)
# ============================================================
print("Capturing time-domain envelope (1 pulse)...")

raw = burst_and_read(pulses=1)

print(f"Received {len(raw)} bytes total")

if len(raw) < 130:
    print(f"Error: insufficient data received. Got {len(raw)} bytes, expected 130")
    ser.close()
    exit()

# Extract samples (bytes 1 to 129, skipping byte 0 which is diagnostic)
echo_td = np.frombuffer(raw[1:129], dtype=np.uint8).astype(np.int16) - 128

print(f"Processed {len(echo_td)} samples")
print(f"Sample range: {echo_td.min()} to {echo_td.max()}")
print(f"First 10 samples: {echo_td[:10]}")

ser.close()

# ============================================================
# SAVE SAMPLES TO FILE
# ============================================================
save_samples_to_file(echo_td, DATA_FILENAME)

# ============================================================
# PLOTTING
# ============================================================
fig, ax = plt.subplots(1, 1, figsize=(11, 6))

# ------------------------------------------------------------
# Time Domain (Envelope)
# ------------------------------------------------------------
ax.plot(echo_td, marker='o', linewidth=2, markersize=4)
ax.set_title(f"Time Domain – Echo Envelope (1 Pulse)")
ax.set_xlabel("Sample Index (0-127)")
ax.set_ylabel("Amplitude")
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 127)  # Explicitly set x-axis to show all 128 samples

# # Add text box with metadata
# metadata_text = f"Sample Rate: {FS} Hz\nSamples: {len(echo_td)}\nPulses: 1\nFile: {DATA_FILENAME}"
# ax.text(0.02, 0.98, metadata_text, transform=ax.transAxes,
#         fontsize=10, verticalalignment='top',
#         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# Annotate the plot with sample count
ax.annotate(f'Total samples: {len(echo_td)}', xy=(0.98, 0.02), 
            xycoords='axes fraction', fontsize=10,
            horizontalalignment='right', verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

plt.tight_layout()

# ============================================================
# SAVE PLOT TO FILE
# ============================================================
save_plot_to_file(fig, PLOT_FILENAME)

# ============================================================
# DISPLAY PLOT
# ============================================================
plt.show()