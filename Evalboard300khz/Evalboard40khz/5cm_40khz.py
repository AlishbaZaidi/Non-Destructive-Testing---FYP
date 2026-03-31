import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# ============================================================
# USER SETTINGS
# ============================================================
COM_PORT       = "COM12"
BAUDRATE       = 115200
SPEED_OF_SOUND = 343.0      # m/s at ~20 deg C

# ============================================================
# BLANKING WINDOW
# ============================================================
# The UTR-1440K-TT-R driven through a step-up transformer rings
# down for a long time. Measured ring-down extends to at least
# sample 20 (amplitude 124/128 = still saturated).
#
# With P1_REC=0, sample_period = 32 us:
#   sample 25 = 800 us -> min detectable distance = 13.7 cm
#   sample 30 = 960 us -> min detectable distance = 16.5 cm
#
# Set BLANK_SAMPLES high enough that all samples below it are
# confirmed ring-down. 25 is a safe starting point for this EVM.
# Use the ring-down diagnostic printed at runtime to tune this.
BLANK_SAMPLES = 16

# ============================================================
# ECHO DETECTION THRESHOLD (absolute, 0-127 scale)
# ============================================================
# Use a FIXED absolute threshold rather than a fraction of
# the global max. Global max is always ~128 due to ring-down
# saturation, so a relative threshold like 0.25*max = 32
# always fires on ring-down regardless of blank window.
#
# A real echo at 20 cm typically has amplitude 20-60 depending
# on gain. Start at 15, raise if noise causes false detections,
# lower if real echoes are being missed.
ECHO_THRESHOLD = 15

# ============================================================
# FILE NAMES
# ============================================================
timestamp     = datetime.now().strftime("%Y%m%d_%H%M%S")
DATA_FILENAME = f"pga460_samples_{timestamp}.txt"
PLOT_FILENAME = f"pga460_plot_{timestamp}.png"


# ============================================================
# HELPER FUNCTIONS
# ============================================================
def checksum(data):
    """8-bit one's complement sum of all bytes (excl. 0x55 sync byte)."""
    return (~(sum(data) & 0xFF)) & 0xFF


def write_register(addr, value):
    """Volatile register write (not stored to EEPROM)."""
    pkt = [0x55, 0x0A, addr, value]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.02)


def read_register(addr):
    """
    Read a single PGA460 register.
    Returns register byte value, or None on failure.
    """
    ser.reset_input_buffer()
    pkt = [0x55, 0x0B, addr]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.02)
    raw = ser.read(3)   # diag + value + checksum
    if len(raw) < 3:
        print(f"  Warning: read_register(0x{addr:02X}) returned {len(raw)} bytes")
        return None
    return raw[1]


def read_rec_length():
    """
    Read REC_LENGTH register (0x22) and derive EDD timing parameters.

    From datasheet Table 7-43 (Section 7.6.3.35):
        Record time = 4.096 x (P1_REC + 1)  [ms]

    From datasheet Section 7.3.7.1:
        EDD stores 128 peak-hold samples across the full record interval.
        sample_period_us = record_time_us / 128
                         = 32 x (P1_REC + 1)   [us]
        FS_EDD           = 31250 / (P1_REC + 1) [Hz]

    Returns (P1_REC, record_time_ms, sample_period_us, fs_edd_hz)
    """
    reg = read_register(0x22)
    if reg is None:
        print("  Could not read REC_LENGTH -- using default P1_REC=0")
        reg = 0x00

    P1_REC           = (reg >> 4) & 0x0F
    P2_REC           =  reg       & 0x0F
    record_time_ms   = 4.096 * (P1_REC + 1)
    record_time_us   = record_time_ms * 1000.0
    sample_period_us = record_time_us / 128.0
    fs_edd_hz        = 1_000_000.0 / sample_period_us

    print(f"  REC_LENGTH register = 0x{reg:02X}")
    print(f"  P1_REC = {P1_REC}  ->  record time = {record_time_ms:.3f} ms")
    print(f"  P2_REC = {P2_REC}")
    print(f"  EDD sample period  = {sample_period_us:.1f} us")
    print(f"  FS_EDD             = {fs_edd_hz:.1f} Hz")
    print(f"  Min detectable dist (blank={BLANK_SAMPLES}): "
          f"{sample_to_distance_m(BLANK_SAMPLES, sample_period_us)*100:.1f} cm")
    return P1_REC, record_time_ms, sample_period_us, fs_edd_hz


def burst_and_read(pulses=1, wait_s=0.15):
    """
    Burst + Listen (Preset 1), then read Echo Data Dump.
    Returns raw 130-byte response.
    """
    ser.reset_input_buffer()
    cmd = [0x55, 0x00, pulses]
    cmd.append(checksum(cmd[1:]))
    ser.write(bytearray(cmd))
    time.sleep(wait_s)

    edd = [0x55, 0x07]
    edd.append(checksum(edd[1:]))
    ser.write(bytearray(edd))

    raw = ser.read(130)
    if len(raw) < 130:
        print(f"  Warning: received {len(raw)}/130 bytes -- reading remainder...")
        raw += ser.read(130 - len(raw))
    return raw


def samples_from_raw(raw):
    """
    Extract 128 signed amplitude samples from the 130-byte EDD response.
    Byte 0:      diagnostic byte  (skip)
    Bytes 1-128: uint8 samples centred at 128
    Byte 129:    checksum         (skip)
    Returns int16 array in range [-128, 127].
    """
    return np.frombuffer(raw[1:129], dtype=np.uint8).astype(np.int16) - 128


def find_first_echo(samples, blank=BLANK_SAMPLES, threshold=ECHO_THRESHOLD):
    """
    Locate the first real echo after the blanking window.

    Uses a FIXED absolute threshold (not relative to global max).
    The global max is always ~128 due to ring-down saturation, so
    a relative threshold always fires inside the ring-down zone.

    Args:
        samples   : signed int16 array (128 elements)
        blank     : first N samples to skip unconditionally (ring-down zone)
        threshold : minimum absolute amplitude to count as detection (0-127)

    Returns (peak_index, peak_amplitude) or (None, None).
    """
    search = samples[blank:]
    above  = np.where(np.abs(search) > threshold)[0]

    if len(above) == 0:
        return None, None

    first_above = above[0] + blank          # map back to full array index
    lo = max(first_above - 3, blank)
    hi = min(first_above + 6, len(samples))
    peak_idx = lo + int(np.argmax(np.abs(samples[lo:hi])))
    return peak_idx, int(samples[peak_idx])


def sample_to_distance_m(sample_idx, sample_period_us):
    """
    Convert sample index to one-way distance in metres.
    TOF = sample_idx * sample_period_us  (round-trip time)
    distance = TOF * v / 2               (one-way)
    """
    tof_s = sample_idx * sample_period_us * 1e-6
    return tof_s * SPEED_OF_SOUND / 2


def print_ringdown_diagnostic(samples, blank, sample_period_us):
    """
    Print amplitudes around the blank boundary so you can see
    exactly where ring-down ends and tune BLANK_SAMPLES.
    """
    print()
    print("  -- Ring-down boundary diagnostic --")
    print(f"  {'Idx':>4}  {'Amp':>6}  {'|Amp|':>6}  {'Dist cm':>8}  note")
    lo = max(0, blank - 6)
    hi = min(len(samples), blank + 16)
    for i in range(lo, hi):
        d = sample_to_distance_m(i, sample_period_us) * 100
        if i < blank:
            note = "blanked"
        elif i == blank:
            note = "<-- search starts here"
        else:
            note = "RING-DOWN" if abs(samples[i]) > ECHO_THRESHOLD else "ok"
        print(f"  {i:>4}  {samples[i]:>6}  {abs(samples[i]):>6}  {d:>8.1f}  {note}")
    print()
    # find where ring-down truly ends (first sample after blank with |amp| <= threshold)
    end_idx = None
    for i in range(0, len(samples)):
        if abs(samples[i]) <= ECHO_THRESHOLD:
            end_idx = i
            break
    if end_idx is not None:
        end_d = sample_to_distance_m(end_idx, sample_period_us) * 100
        print(f"  First sample with |amp| <= {ECHO_THRESHOLD}: sample {end_idx} ({end_d:.1f} cm)")
        if end_idx >= blank:
            print(f"  Ring-down appears to end before the blank window. Settings look good.")
        else:
            print(f"  WARNING: ring-down extends past blank window (sample {blank})!")
            print(f"  Increase BLANK_SAMPLES to at least {end_idx + 2} to avoid false detections.")
    print()


def save_samples(samples, filename, sample_period_us, fs_edd,
                 P1_REC, record_time_ms, peak_idx, distance_m):
    dist_str = f"{distance_m*100:.1f} cm" if distance_m is not None else "N/A"
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("# PGA460 Echo Data Dump\n")
        f.write(f"# Date           : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# P1_REC         : {P1_REC}\n")
        f.write(f"# Record time    : {record_time_ms:.3f} ms\n")
        f.write(f"# FS_EDD         : {fs_edd:.1f} Hz\n")
        f.write(f"# Sample period  : {sample_period_us:.2f} us\n")
        f.write(f"# Speed of sound : {SPEED_OF_SOUND} m/s\n")
        f.write(f"# BLANK_SAMPLES  : {BLANK_SAMPLES}\n")
        f.write(f"# ECHO_THRESHOLD : {ECHO_THRESHOLD}\n")
        f.write(f"# First echo     : sample {peak_idx} -> {dist_str}\n")
        f.write("# Index, Amplitude\n")
        for i, s in enumerate(samples):
            f.write(f"{i}, {s}\n")
    print(f"Saved: {filename}")


# ============================================================
# SERIAL INIT
# ============================================================
print(f"Connecting to {COM_PORT} @ {BAUDRATE} baud...")
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=2)
time.sleep(2)

# ============================================================
# PGA460 CONFIGURATION
# ============================================================
print("Configuring PGA460 for 40 kHz...")

# Burst frequency: f = 30 + (FREQ_reg x 0.2) kHz -> 40 kHz: FREQ=50=0x32
write_register(0x16, 0x32)

# Analog gain (~58 dB)
write_register(0x1B, 0x7F)

# ============================================================
# READ REC_LENGTH -- derive FS_EDD from datasheet formula
# ============================================================
print("Reading REC_LENGTH register (0x22)...")
P1_REC, record_time_ms, sample_period_us, FS_EDD = read_rec_length()

burst_wait_s = (record_time_ms / 1000.0) + 0.05

# ============================================================
# CAPTURE
# ============================================================
print(f"Capturing EDD (1 pulse, waiting {burst_wait_s*1000:.0f} ms)...")
raw = burst_and_read(pulses=1, wait_s=burst_wait_s)

if len(raw) < 130:
    print(f"Error: only {len(raw)} bytes received.")
    ser.close()
    exit(1)

samples = samples_from_raw(raw)
print(f"Samples : {len(samples)}  range [{samples.min()}, {samples.max()}]")
print(f"First 10: {samples[:10].tolist()}")
ser.close()

# ============================================================
# RING-DOWN DIAGNOSTIC -- always printed to help tune settings
# ============================================================
print_ringdown_diagnostic(samples, BLANK_SAMPLES, sample_period_us)

# ============================================================
# DISTANCE CALCULATION
# ============================================================
peak_idx, peak_amp = find_first_echo(samples)

if peak_idx is not None:
    distance_m  = sample_to_distance_m(peak_idx, sample_period_us)
    distance_cm = distance_m * 100.0
    tof_us      = peak_idx * sample_period_us
    print("-- Echo detected ------------------------------------------")
    print(f"  Peak sample    : {peak_idx}")
    print(f"  Amplitude      : {peak_amp}")
    print(f"  Time of flight : {tof_us:.1f} us  (round trip)")
    print(f"  Distance       : {distance_cm:.1f} cm  ({distance_m*1000:.1f} mm)  [one way]")
    print("-----------------------------------------------------------")
else:
    distance_m  = None
    distance_cm = None
    print(f"No echo detected above threshold ({ECHO_THRESHOLD}) "
          f"after blank window (sample {BLANK_SAMPLES} = "
          f"{sample_to_distance_m(BLANK_SAMPLES, sample_period_us)*100:.1f} cm).")
    print("  -> Lower ECHO_THRESHOLD if reflector is present but weak.")
    print("  -> Lower BLANK_SAMPLES if reflector is closer than min range.")

save_samples(samples, DATA_FILENAME, sample_period_us, FS_EDD,
             P1_REC, record_time_ms, peak_idx, distance_m)

# ============================================================
# PLOTTING
# ============================================================
fig, ax1 = plt.subplots(figsize=(13, 5))
x = np.arange(len(samples))

ax1.plot(x, samples, color='steelblue', linewidth=1.5,
         marker='o', markersize=2.5, label='EDD amplitude', zorder=3)

# Ring-down zone
ax1.axvspan(0, BLANK_SAMPLES - 0.5, color='tomato', alpha=0.13,
            label=f'Blanked ring-down (0-{BLANK_SAMPLES-1})')

# Threshold lines
ax1.axhline( ECHO_THRESHOLD, color='orange', linewidth=0.8,
             linestyle='--', alpha=0.8)
ax1.axhline(-ECHO_THRESHOLD, color='orange', linewidth=0.8,
             linestyle='--', alpha=0.8, label=f'Threshold +/-{ECHO_THRESHOLD}')
ax1.axhline(0, color='gray', linewidth=0.5)

# Detected peak
if peak_idx is not None:
    ax1.axvline(peak_idx, color='green', linewidth=1.2, linestyle='--', alpha=0.8)
    xt = min(peak_idx + 6, 108)
    ax1.annotate(
        f"Peak @ sample {peak_idx}\n-> {distance_cm:.1f} cm",
        xy=(peak_idx, peak_amp),
        xytext=(xt, peak_amp * 0.6),
        fontsize=9,
        arrowprops=dict(arrowstyle='->', color='green', lw=1.2),
        color='darkgreen',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.85)
    )

ax1.set_xlabel("Sample index")
ax1.set_ylabel("Amplitude (signed, centred at 0)")
ax1.set_xlim(0, 127)
ax1.set_ylim(-135, 135)
ax1.set_title(
    f"PGA460 EDD | 40 kHz | P1_REC={P1_REC} | "
    f"{sample_period_us:.0f} us/sample | blank={BLANK_SAMPLES} | "
    + (f"Distance: {distance_cm:.1f} cm" if distance_cm is not None
       else "No echo detected")
)
ax1.grid(True, alpha=0.2)

# Secondary x-axis: distance in cm
ax2 = ax1.twiny()
ax2.set_xlim(ax1.get_xlim())
max_dist_cm   = sample_to_distance_m(127, sample_period_us) * 100.0
dist_ticks_cm = np.arange(0, max_dist_cm + 1, 5)
sample_ticks  = ((dist_ticks_cm * 1e-2) / SPEED_OF_SOUND
                 * 2.0 / (sample_period_us * 1e-6))
mask = (sample_ticks >= 0) & (sample_ticks <= 127)
ax2.set_xticks(sample_ticks[mask])
ax2.set_xticklabels([f"{d:.0f}" for d in dist_ticks_cm[mask]], fontsize=8)
ax2.set_xlabel("One-way distance (cm)", fontsize=9)

ax1.legend(loc='lower right', fontsize=8)

min_range_cm = sample_to_distance_m(BLANK_SAMPLES, sample_period_us) * 100
info = (
    f"P1_REC         = {P1_REC}\n"
    f"Record time    = {record_time_ms:.3f} ms\n"
    f"Sample period  = {sample_period_us:.0f} us\n"
    f"FS_EDD         = {FS_EDD:.0f} Hz\n"
    f"v_sound        = {SPEED_OF_SOUND} m/s\n"
    f"Blank samples  = {BLANK_SAMPLES}  ({BLANK_SAMPLES*sample_period_us:.0f} us)\n"
    f"Min range      = {min_range_cm:.1f} cm\n"
    f"Threshold      = +/-{ECHO_THRESHOLD}"
)
ax1.text(0.01, 0.98, info, transform=ax1.transAxes, fontsize=7.5,
         verticalalignment='bottom', family='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.85))

plt.tight_layout()
fig.savefig(PLOT_FILENAME, dpi=300, bbox_inches='tight')
print(f"Plot saved: {PLOT_FILENAME}")
plt.show()