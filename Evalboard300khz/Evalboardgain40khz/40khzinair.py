import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# ============================================================
# SETTINGS
# ============================================================
COM_PORT       = "COM12"
BAUDRATE       =  9600            #115200
SPEED_OF_SOUND = 343.0
N_AVERAGES     = 16    # number of bursts to average — reduces random noise
NOISE_FLOOR    = 10

# ============================================================
# HELPERS
# ============================================================
def checksum(data):
    return (~(sum(data) & 0xFF)) & 0xFF

def write_reg(addr, value):
    pkt = [0x55, 0x0A, addr, value]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.02)

def read_reg(addr):
    ser.reset_input_buffer()
    pkt = [0x55, 0x09, addr]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.02)
    raw = ser.read(3)
    return raw[1] if len(raw) >= 3 else None

def burst_and_edd(wait_s):
    # Command 0x00 = Burst+Listen Preset1. Source: Table 7-3
    # Command 0x07 = EDD read. Source: Table 7-3
    ser.reset_input_buffer()
    cmd = [0x55, 0x00, 1]
    cmd.append(checksum(cmd[1:]))
    ser.write(bytearray(cmd))
    time.sleep(wait_s)
    edd = [0x55, 0x07]
    edd.append(checksum(edd[1:]))
    ser.write(bytearray(edd))
    raw = ser.read(130)
    if len(raw) < 130:
        raw += ser.read(130 - len(raw))
    return np.frombuffer(raw[1:129], dtype=np.uint8).astype(np.int16) - 128

# ============================================================
# CONNECT
# ============================================================
print(f"Connecting to {COM_PORT}...")
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=2)
time.sleep(2)

# Frequency = 40 kHz. Source: Section 7.6.3.29
write_reg(0x1C, 0x32)
time.sleep(0.1)  # wait for BPF auto-recalc. Source: Section 7.3.4.1

# P1_REC=1 -> record=8.192ms, sample_period=64us
# With DECPL_T=0, blanking=4096us=64 samples
# Usable samples after blanking: 64-127 (64 samples)
# P1_REC=0 gives 0 usable samples (blanking fills entire buffer)
# Source: Table 7-43, Section 7.6.3.35
write_reg(0x22, 0x1C)  # P1_REC=1, P2_REC=12 (keep EEPROM value)

# AFE_GAIN_RNG=11 (32-64dB range), DECPL_T=0 (blanking=4096us)
# Source: Table 7-47, Section 7.6.3.39
write_reg(0x26, 0xC0)

# GAIN_INIT=0, BPF_BW=3 (keep EEPROM value)
# Gain = 0.5*(0+1) + 32 = 32.5 dB
# Source: Table 7-36, Section 7.6.3.28
write_reg(0x1B, 0xC0)

# DATADUMP_EN=1. Source: Table 7-53, Section 7.6.3.45
write_reg(0x40, 0x80)
time.sleep(0.05)

# ============================================================
# READ AND VERIFY REGISTERS
# ============================================================
rec              = read_reg(0x22)
P1_REC           = (rec >> 4) & 0x0F
record_time_ms   = 4.096 * (P1_REC + 1)
sample_period_us = record_time_ms * 1000.0 / 128.0
burst_wait_s     = record_time_ms / 1000.0 + 0.05
blank_us         = 4096   # DECPL_T=0
blank_samples    = int(blank_us / sample_period_us)
usable_start     = blank_samples

print(f"\nP1_REC={P1_REC}  record={record_time_ms:.3f} ms  sample_period={sample_period_us:.1f} us")
print(f"Blanking={blank_us} us = {blank_samples} samples")
print(f"Usable EDD range: samples {usable_start} to 127")
print(f"Max measurable range: {127 * sample_period_us * 1e-6 * SPEED_OF_SOUND / 2 * 100:.1f} cm")

if blank_samples >= 128:
    print("ERROR: blanking >= entire buffer. Increase P1_REC.")
    ser.close()
    exit(1)

# ============================================================
# CAPTURE — average N_AVERAGES bursts
# ============================================================
print(f"\nCapturing {N_AVERAGES} bursts and averaging...")
accumulator = np.zeros(128, dtype=np.float64)

for i in range(N_AVERAGES):
    samp = burst_and_edd(burst_wait_s)
    accumulator += samp.astype(np.float64)
    print(f"  burst {i+1:2d}/{N_AVERAGES}  RMS={np.sqrt(np.mean(samp.astype(float)**2)):.1f}", end='\r')

ser.close()
print()

averaged = accumulator / N_AVERAGES

# Only look at usable samples (after blanking window)
usable = averaged[usable_start:]
rms_usable = np.sqrt(np.mean(usable**2))
print(f"\nAveraged RMS (samples {usable_start}-127): {rms_usable:.2f}")
print(f"Max amplitude in usable region: {np.max(np.abs(usable)):.1f}")

# Find ring-down end in usable region
rd_end = None
for i in range(len(usable) - 2):
    if (abs(usable[i]) <= NOISE_FLOOR and
        abs(usable[i+1]) <= NOISE_FLOOR and
        abs(usable[i+2]) <= NOISE_FLOOR):
        rd_end = usable_start + i
        break

if rd_end is not None:
    rd_us = rd_end * sample_period_us
    rd_cm = rd_us * 1e-6 * SPEED_OF_SOUND / 2 * 100
    print(f"\nRing-down ends: sample {rd_end}  ({rd_us:.0f} us)  min range = {rd_cm:.1f} cm")
    print(f"Use BLANK_SAMPLES = {rd_end} in echo detection script.")
else:
    print(f"\nRing-down did not settle below NOISE_FLOOR={NOISE_FLOOR} in usable region.")
    print(f"Usable region mean amplitude = {np.mean(np.abs(usable)):.1f}")
    if np.mean(np.abs(usable)) < 5:
        print("Signal very weak — transducer may not be connected or frequency mismatch.")
    else:
        print("Persistent signal — likely 40kHz environmental interference.")
        print("Try moving board away from switching power supplies / laptop charger.")

# ============================================================
# PLOT
# ============================================================
x = np.arange(128)
x_us = x * sample_period_us

fig, ax = plt.subplots(figsize=(13, 5))

# Show blanked region
ax.axvspan(0, blank_samples, color='red', alpha=0.10, label=f'HW blanking ({blank_us} us)')

# Plot averaged signal
ax.plot(x, averaged, color='steelblue', linewidth=1.5, marker='o', markersize=2.5,
        label=f'EDD average ({N_AVERAGES} bursts)')

# Noise floor band
ax.axhspan(-NOISE_FLOOR, NOISE_FLOOR, color='orange', alpha=0.15, label=f'Noise floor ±{NOISE_FLOOR}')
ax.axhline(0, color='gray', linewidth=0.5)

# Vertical separator at end of blanking
ax.axvline(blank_samples, color='red', linewidth=1.0, linestyle=':', alpha=0.6)
ax.text(blank_samples + 1, 110, f'blanking ends\nsample {blank_samples}', fontsize=8, color='darkred')

# Ring-down end marker
if rd_end is not None:
    ax.axvline(rd_end, color='green', linewidth=1.8, linestyle='--')
    ax.text(rd_end + 1, 70,
            f"ring-down ends\nsample {rd_end}\n({rd_cm:.1f} cm min range)",
            fontsize=9, color='darkgreen',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.85))

# Secondary x-axis: time in µs
ax2 = ax.twiny()
ax2.set_xlim(ax.get_xlim())
time_ticks_us = np.arange(0, 128 * sample_period_us + 1, 500)
samp_ticks    = time_ticks_us / sample_period_us
mask = (samp_ticks >= 0) & (samp_ticks <= 127)
ax2.set_xticks(samp_ticks[mask])
ax2.set_xticklabels([f"{t:.0f}" for t in time_ticks_us[mask]], fontsize=8)
ax2.set_xlabel("Time (µs)", fontsize=9)

ax.set_xlabel("Sample index")
ax.set_ylabel("Amplitude")
ax.set_xlim(0, 127)
ax.set_ylim(-135, 135)
ax.set_title(
    f"40 kHz ring-down in air (no target)  |  "
    f"32.5 dB gain  |  {sample_period_us:.0f} µs/sample  |  "
    f"averaged over {N_AVERAGES} bursts"
)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.2)

plt.tight_layout()
fname = f"ringdown_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
plt.savefig(fname, dpi=300, bbox_inches='tight')
print(f"Saved: {fname}")
plt.show()