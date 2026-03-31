"""
PGA460-Q1 PSM-EVM — 40 kHz DEFAULT mode, 5 cm target, air medium
Transducer: UTR-1440K-TT-R (40 kHz closed-top, on-board)
Interface:  UART via J2 connector, USB-to-serial adapter

Every register value and formula is cited to SLASEC8C (PGA460-Q1 datasheet).

KEY DIFFERENCES FROM 300 kHz SETUP:
  - FREQ_SHIFT = 0  (do NOT write TVGAIN6)
  - BPF coefficients auto-calculated by chip (do NOT write 0x41-0x46)
  - FREQUENCY = 0x32 (50 decimal) for 40 kHz
  - DEADTIME = 0x00 (transformer mode, no dead-time needed)
  - GAIN_INIT = 0   (start at minimum 58.5 dB — increase if signal is weak)
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# ============================================================
# USER SETTINGS
# ============================================================
COM_PORT     = "COM12"
BAUDRATE     = 115200
NUM_AVERAGES = 8        # bursts to average — more = better SNR
SOUND_SPEED  = 343.0   # m/s at ~20 deg C

# ============================================================
# STEP-BY-STEP TIMING DERIVATION
# All formulas from datasheet. No invented values.
# ============================================================

# ----------------------------------------------------------
# STEP 1 — Round-trip time of flight for 5 cm target
# ----------------------------------------------------------
# Physics: sound travels to target and back.
#   TOF = (2 x distance) / speed_of_sound
#       = (2 x 0.05 m)   / 343.0 m/s
#       = 0.10 / 343.0
#       = 0.00029155 s
#       = 291.5 us
TARGET_M = 0.05
TOF_S    = (2 * TARGET_M) / SOUND_SPEED   # 0.00029155 s
TOF_US   = TOF_S * 1e6                    # 291.5 us

# ----------------------------------------------------------
# STEP 2 — Record window duration
# ----------------------------------------------------------
# SOURCE: Table 7-43, p.70:
#   Record time = 4.096 x (P1_REC + 1)  [ms]
#
# P1_REC = 0 gives the MINIMUM window (4.096 ms).
# Smaller window = each EDD slot covers less time = echo lands
# at a HIGHER sample index = more separation from ring-down.
#
#   T_rec = 4.096 x (0 + 1) = 4.096 ms = 4096 us
P1_REC    = 0
T_REC_MS  = 4.096 * (P1_REC + 1)   # 4.096 ms   (Table 7-43 p.70)
T_REC_US  = T_REC_MS * 1000.0      # 4096 us

# ----------------------------------------------------------
# STEP 3 — Total raw DSP samples inside the window
# ----------------------------------------------------------
# SOURCE: p.42, Section 7.3.7.1:
#   "the output rate of the digital data path is 1 us/sample"
#
#   total_DSP = 4096 us / 1 us/sample = 4096 samples
DSP_RATE_US  = 1.0
TOTAL_DSP    = int(T_REC_US / DSP_RATE_US)   # 4096

# ----------------------------------------------------------
# STEP 4 — Raw DSP samples per EDD slot
# ----------------------------------------------------------
# SOURCE: p.42, Section 7.3.7.1:
#   "8192 / 128 = 64 samples per slot"  (their P1_REC=1 example)
#   Same formula: total_DSP / 128
#
#   DSP_per_slot = 4096 / 128 = 32
EDD_SLOTS    = 128
DSP_PER_SLOT = TOTAL_DSP / EDD_SLOTS         # 32.0

# ----------------------------------------------------------
# STEP 5 — Microseconds per EDD slot
# ----------------------------------------------------------
#   us_per_slot = 32 samples x 1 us/sample = 32 us
US_PER_SLOT  = DSP_PER_SLOT * DSP_RATE_US    # 32.0 us

# ----------------------------------------------------------
# STEP 6 — Which EDD slot contains the 5 cm echo
# ----------------------------------------------------------
#   EDD_slot = TOF_us / us_per_slot
#            = 291.5  / 32.0
#            = 9.11  → index 9
#
# Slot 9 covers time 288 us to 319 us.
# 291.5 us falls inside this range.
ECHO_SLOT = TOF_US / US_PER_SLOT   # 9.11 -> sample 9

# ----------------------------------------------------------
# STEP 7 — Centimetres per EDD slot
# ----------------------------------------------------------
#   cm_per_slot = (32e-6 s x 343 m/s) / 2 x 100 cm/m
#               = 0.5488 cm
CM_PER_SLOT = (US_PER_SLOT * 1e-6 * SOUND_SPEED / 2) * 100   # 0.5488 cm

# ----------------------------------------------------------
# Ring-down blanking
# ----------------------------------------------------------
# At 40 kHz, transducer ring-down is longer than at 300 kHz
# because Q-factor is typically higher for closed-top transducers.
# Blank samples 0-11 (covers 0 to 384 us of ring-down).
# The echo at sample 9 is unfortunately inside this range.
#
# This is a FUNDAMENTAL constraint of the PGA460 EDD at short range.
# The minimum window gives 32 us/slot. At 5 cm the echo is at sample 9.
# Ring-down at 40 kHz can last 10-15 samples.
# To see the echo clearly you need a target at 15-20 cm minimum.
# We blank 0-8 and SEARCH from sample 9 onward, understanding
# the first few valid samples may still have ring-down contamination.
RINGDOWN_BLANK = 9

print("=" * 60)
print("40 kHz default mode — timing derivation")
print("=" * 60)
print(f"  TOF (Step 1)       = {TOF_US:.2f} us")
print(f"  T_rec (Step 2)     = {T_REC_US:.0f} us  [4.096x(P1_REC+1), Table 7-43 p.70]")
print(f"  Total DSP (Step 3) = {TOTAL_DSP}  [1 us/sample, p.42]")
print(f"  DSP/slot (Step 4)  = {DSP_PER_SLOT:.0f}  [total/128, p.42]")
print(f"  us/slot  (Step 5)  = {US_PER_SLOT:.0f} us")
print(f"  Echo slot (Step 6) = {ECHO_SLOT:.2f} -> index {int(ECHO_SLOT)}")
print(f"  cm/slot  (Step 7)  = {CM_PER_SLOT:.4f} cm")
print(f"  Ring-down blank    = samples 0-{RINGDOWN_BLANK-1}")
print()

# ============================================================
# REGISTER VALUES — each one derived from datasheet
# ============================================================

# --- EE_CNTRL 0x40 = 0x80 ---
# SOURCE: Table 7-53 p.75, bit7 = DATADUMP_EN = 1
# SOURCE: p.42: required for EDD to populate memory
# 0x80 = 1000 0000
EE_CNTRL_VAL = 0x80

# --- FREQUENCY 0x1C = 0x32 ---
# SOURCE: Table 7-37 p.67:
#   Frequency = 0.2 x FREQ + 30  [kHz]   (FREQ_SHIFT=0)
#   For 40 kHz: 40 = 0.2 x FREQ + 30  ->  FREQ = 50 = 0x32
#   Verify: 0.2 x 50 + 30 = 40.0 kHz  ✓
FREQUENCY_VAL = 50    # = 0x32

# --- INIT_GAIN 0x1B = 0x40 ---
# SOURCE: Table 7-36 p.67:
#   bits[7:6] = BPF_BW: BandWidth = 2 x (BPF_BW + 1)  [kHz]
#     BPF_BW=1 -> 2x(1+1) = 4 kHz  (appropriate for 40 kHz transducer)
#   bits[5:0] = GAIN_INIT (start at 0 = minimum, increase if weak)
#     Init_Gain = 0.5 x (GAIN_INIT+1) + 58  dB  (AFE_GAIN_RNG=0 default)
#     GAIN_INIT=0  -> 58.5 dB  (start here)
#     GAIN_INIT=20 -> 68.5 dB  (try if signal is weak)
#     GAIN_INIT=40 -> 78.5 dB  (try if still weak)
#     GAIN_INIT=63 -> 90.0 dB  (maximum — will saturate at close range)
#
# byte = (BPF_BW << 6) | GAIN_INIT
#      = (1 << 6)      | 0
#      = 0x40
BPF_BW_VAL    = 1     # 4 kHz bandwidth
GAIN_INIT_VAL = 0     # start at minimum, increase if needed
INIT_GAIN_VAL = (BPF_BW_VAL << 6) | GAIN_INIT_VAL   # = 0x40

# --- DEADTIME 0x1D = 0x00 ---
# SOURCE: Table 7-38 p.68:
#   bits[7:4] = THR_CMP_DEGLTCH = 0 (no deglitch)
#   bits[3:0] = PULSE_DT: DeadTime = 0.0625 x PULSE_DT  [us]
# SOURCE: Section 7.3.2.1 p.12:
#   Board uses center-tap transformer with complementary FETs.
#   This mode does not require dead-time (no shoot-through risk).
#   Constraint (p.14): max DT <= t/8 = (1/40kHz)/8 = 3.125 us
#   -> PULSE_DT=0 (no dead-time), byte = 0x00
DEADTIME_VAL  = 0x00

# --- PULSE_P1 0x1E = 0x08 ---
# SOURCE: Table 7-39 p.68:
#   bits[7:5] = IO_IF_SEL=0, UART_DIAG=0, IO_DIS=0
#   bits[4:0] = P1_PULSE: 0h means 1 pulse on OUTA only
#                          8 = 8 full pulse pairs on OUTA+OUTB
# 8 pulses x (1/40 kHz) = 200 us of burst energy — good for 5 cm
# byte = (0<<7)|(0<<6)|(0<<5)|8 = 0x08
PULSE_P1_VAL  = 0x08

# --- REC_LENGTH 0x22 = 0x0C ---
# SOURCE: Table 7-43 p.70:
#   bits[7:4] = P1_REC: Record time = 4.096 x (P1_REC+1)  [ms]
#   bits[3:0] = P2_REC: keep factory default = 12
# SOURCE: Table 7-6 p.53: REC_LENGTH factory default = 0x1C
#   -> P1_REC=1 (default), P2_REC=12
#   We change P1_REC to 0 (minimum window), keep P2_REC=12
# byte = (0<<4)|12 = 0x0C
P2_REC_DEFAULT = 12
REC_LENGTH_VAL = (P1_REC << 4) | P2_REC_DEFAULT   # = 0x0C

# NOTE: TVGAIN6 (0x1A) is NOT written.
#   Factory default = 0xFC = FREQ_SHIFT=0.
#   FREQ_SHIFT=0 is correct for 40 kHz (30-80 kHz range).
#   Writing it is unnecessary and risks triggering side effects.

# NOTE: BPF registers 0x41-0x46 are NOT written.
#   SOURCE: p.16: "On power up, the PGA460-Q1 device calculates
#   the coefficients and places them in BPF_A2_xSB, BPF_A3_xSB,
#   and BPF_B1_xSB registers."
#   "if the FREQ or BPF_BW bit is changed, the calculation is rerun"
#   FREQ_SHIFT=0 -> chip auto-calculates after we write FREQUENCY
#   and INIT_GAIN. We do nothing else.

print("Register values")
print("-" * 60)
print(f"  0x40 = 0x{EE_CNTRL_VAL:02X}  EE_CNTRL   DATADUMP_EN=1  (Table 7-53 p.75)")
print(f"  0x1B = 0x{INIT_GAIN_VAL:02X}  INIT_GAIN  BPF_BW=4kHz GAIN_INIT=0  (Table 7-36 p.67)")
print(f"  0x1C = 0x{FREQUENCY_VAL:02X}  FREQUENCY  FREQ=50 -> 40kHz  (Table 7-37 p.67)")
print(f"  0x1D = 0x{DEADTIME_VAL:02X}  DEADTIME   PULSE_DT=0  (Table 7-38 p.68)")
print(f"  0x1E = 0x{PULSE_P1_VAL:02X}  PULSE_P1   P1_PULSE=8 pairs  (Table 7-39 p.68)")
print(f"  0x22 = 0x{REC_LENGTH_VAL:02X}  REC_LENGTH P1_REC=0 P2_REC=12  (Table 7-43 p.70)")
print(f"  0x1A       NOT written (FREQ_SHIFT stays 0, Table 7-35 p.67)")
print(f"  0x41-0x46  NOT written (BPF auto-calculated, Section 7.3.4.1 p.16)")
print("-" * 60)
print()

# ============================================================
# LOW-LEVEL HELPERS
# ============================================================

def checksum(data):
    """
    SOURCE: p.34, Section 7.3.6.2.1.4:
      'inverted byte sum WITH CARRY operation over all data fields
       and the command field. The sync field (0x55) is not included.'

    'With carry' means: if sum > 255, add the overflow back in
    before inverting. This is a one's complement sum, NOT truncation.
    Truncating (~(sum & 0xFF)) gives the wrong result when sum > 255.

    Verification — datasheet Example 2 p.52:
      Write 0x40=0x80: sum = 0x0A+0x40+0x80 = 0xCA (no carry)
      CHK = ~0xCA & 0xFF = 0x35  (datasheet confirms 0x35)
    """
    s = sum(data)
    while s > 0xFF:                # add carry back in
        s = (s & 0xFF) + (s >> 8)
    return (~s) & 0xFF


def write_register(addr, value):
    """
    Command 10 = 0x0A — register write
    SOURCE: Table 7-3 p.34, Example 2 p.52:
      Send:    0x55, 0x0A, ADDR, DATA, CHK
      CHK    = checksum([0x0A, ADDR, DATA])
      Response: none
    """
    pkt = [0x55, 0x0A, addr, value]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.015)


def read_register(addr):
    """
    Command 9 = 0x09 — register read
    SOURCE: Table 7-3 p.34, Example 1 p.52:
      Send:    0x55, 0x09, ADDR, CHK
      Receive: DIAG(1), DATA(1), CHK(1) = 3 bytes
      DATA is at response index [1]
    """
    pkt = [0x55, 0x09, addr]
    pkt.append(checksum(pkt[1:]))
    ser.reset_input_buffer()
    ser.write(bytearray(pkt))
    time.sleep(0.015)
    resp = ser.read(3)
    if len(resp) >= 2:
        return resp[1]
    return None


def burst_and_capture():
    """
    Fire Burst+Listen Preset1 then read Echo Data Dump.

    Burst command (cmd 0):
    SOURCE: Table 7-3 p.34, Example 3 p.52:
      Send:    0x55, 0x00, N_OBJECTS, CHK
               N_OBJECTS = 1 (detect 1 object)
               CHK = checksum([0x00, 0x01]) = ~(0x01)&0xFF = 0xFE
      Response: none

    EDD command (cmd 7):
    SOURCE: Table 7-3 p.34:
      Send:    0x55, 0x07, CHK    CHK = ~(0x07)&0xFF = 0xF8
      Receive: DIAG(1) + data x 128 + CHK(1) = 130 bytes

    Wait time:
      Record window = 4.096 ms.
      Add generous margin for DSP processing + UART latency = 120 ms total.
    """
    ser.reset_input_buffer()

    # Burst + Listen Preset 1
    n_obj     = 1
    burst_pkt = [0x55, 0x00, n_obj]
    burst_pkt.append(checksum(burst_pkt[1:]))   # = 0xFE
    ser.write(bytearray(burst_pkt))

    time.sleep(0.12)   # 4.096 ms record + 115.9 ms margin

    # Echo Data Dump
    edd_pkt = [0x55, 0x07]
    edd_pkt.append(checksum(edd_pkt[1:]))       # = 0xF8
    ser.write(bytearray(edd_pkt))

    raw = ser.read(130)
    if len(raw) < 130:
        raw = raw + ser.read(130 - len(raw))
    return raw


def listen_only_capture():
    """
    Command 2 = Listen Only Preset 1 (NO burst fired).
    SOURCE: Table 7-3 p.34:
      Send: 0x55, 0x02, N_OBJECTS, CHK
    Use this to check ambient noise floor.
    If noise appears here too, it is ambient or board-generated.
    If noise disappears vs burst capture, the burst itself is the source.
    """
    ser.reset_input_buffer()
    n_obj = 1
    pkt   = [0x55, 0x02, n_obj]
    pkt.append(checksum(pkt[1:]))
    ser.write(bytearray(pkt))
    time.sleep(0.12)

    edd_pkt = [0x55, 0x07]
    edd_pkt.append(checksum(edd_pkt[1:]))
    ser.write(bytearray(edd_pkt))

    raw = ser.read(130)
    if len(raw) < 130:
        raw = raw + ser.read(130 - len(raw))
    return raw


# ============================================================
# CONNECT
# ============================================================
print(f"Connecting to {COM_PORT} at {BAUDRATE} baud ...")
ser = serial.Serial(COM_PORT, BAUDRATE, timeout=2)
time.sleep(2)
print("Connected.\n")

# ============================================================
# WRITE REGISTERS
# ============================================================
# Write order matters:
#   1. Registers that have no auto-calc side effects first.
#   2. FREQUENCY triggers BPF auto-calculation — write it and wait.
#   3. INIT_GAIN (BPF_BW change) triggers BPF auto-calc again — wait.
#   After step 3 the chip has computed correct BPF coefficients for
#   40 kHz automatically. We write nothing to 0x41-0x46.
# ============================================================
print("Writing registers ...")

write_register(0x40, EE_CNTRL_VAL)   # DATADUMP_EN=1     (Table 7-53 p.75)
write_register(0x1D, DEADTIME_VAL)   # PULSE_DT=0        (Table 7-38 p.68)
write_register(0x1E, PULSE_P1_VAL)   # P1_PULSE=8        (Table 7-39 p.68)
write_register(0x22, REC_LENGTH_VAL) # P1_REC=0          (Table 7-43 p.70)

# FREQUENCY: triggers chip BPF auto-calculation
write_register(0x1C, FREQUENCY_VAL)  # FREQ=50 -> 40kHz  (Table 7-37 p.67)
time.sleep(0.05)                      # wait for auto-calc

# INIT_GAIN: BPF_BW change triggers another auto-calculation
write_register(0x1B, INIT_GAIN_VAL)  # BPF_BW=4kHz       (Table 7-36 p.67)
time.sleep(0.05)                      # wait for auto-calc
# Chip has now written correct BPF coefficients automatically.

print("Done.\n")

# ============================================================
# VERIFY REGISTERS
# ============================================================
print("Verifying registers (cmd 0x09, Table 7-3 p.34) ...")
verify = {
    0x40: (EE_CNTRL_VAL,   "EE_CNTRL   DATADUMP_EN=1"),
    0x1B: (INIT_GAIN_VAL,  "INIT_GAIN  BPF_BW=1(4kHz) GAIN_INIT=0"),
    0x1C: (FREQUENCY_VAL,  "FREQUENCY  FREQ=50 -> 40kHz"),
    0x1D: (DEADTIME_VAL,   "DEADTIME   PULSE_DT=0"),
    0x1E: (PULSE_P1_VAL,   "PULSE_P1   P1_PULSE=8"),
    0x22: (REC_LENGTH_VAL, "REC_LENGTH P1_REC=0 P2_REC=12"),
}
all_ok = True
for addr, (expected, label) in verify.items():
    val = read_register(addr)
    if val is None:
        print(f"  0x{addr:02X}  {label:42s}  NO RESPONSE")
        all_ok = False
    elif val == expected:
        print(f"  0x{addr:02X}  {label:42s}  0x{val:02X} OK")
    else:
        print(f"  0x{addr:02X}  {label:42s}  read=0x{val:02X} expected=0x{expected:02X} MISMATCH")
        all_ok = False

if not all_ok:
    print("\n  One or more mismatches — check Table 7-7 p.54 of SLASEC8C.")
else:
    print("\n  All registers OK.\n")

# ============================================================
# NOISE FLOOR CHECK (Listen Only — no burst)
# ============================================================
print("Noise floor check (Listen Only, no burst) ...")
raw_lo   = listen_only_capture()
samp_lo  = np.frombuffer(raw_lo[1:129], dtype=np.uint8).astype(np.float64)
lo_min, lo_max, lo_mean = int(samp_lo.min()), int(samp_lo.max()), samp_lo.mean()
print(f"  min={lo_min}  max={lo_max}  mean={lo_mean:.1f}")

if lo_max > 50:
    print("  WARNING: High ambient noise floor (max > 50).")
    print("  Check for nearby vibration sources or EMI.")
else:
    print("  Noise floor OK.")
print()

# ============================================================
# BURST CAPTURE — averaged
# ============================================================
print(f"Capturing {NUM_AVERAGES} bursts ...")
accum = np.zeros(128, dtype=np.float64)
good  = 0

for i in range(NUM_AVERAGES):
    raw  = burst_and_capture()
    if len(raw) < 130:
        print(f"  [{i+1}/{NUM_AVERAGES}] Short read ({len(raw)} bytes) — skipped")
        continue
    # raw[0]     = diagnostic byte
    # raw[1:129] = 128 EDD samples, unsigned 8-bit (0-255)
    # raw[129]   = checksum
    # SOURCE: Table 7-3 p.34: "Byte1-Byte128: Echo data dump"
    samp   = np.frombuffer(raw[1:129], dtype=np.uint8).astype(np.float64)
    accum += samp
    good  += 1
    print(f"  [{i+1}/{NUM_AVERAGES}]  diag=0x{raw[0]:02X}  "
          f"min={int(samp.min()):3d}  max={int(samp.max()):3d}  "
          f"mean={samp.mean():.1f}")

if good == 0:
    print("ERROR: zero valid captures. Check COM port, wiring, DATADUMP_EN.")
    ser.close()
    exit(1)

echo = accum / good
print(f"\n  {good}/{NUM_AVERAGES} bursts averaged.\n")
ser.close()

# ============================================================
# DETECTION
# ============================================================
search = echo.copy()
search[:RINGDOWN_BLANK] = 0.0   # blank ring-down region

peak_idx  = int(np.argmax(search))
peak_amp  = echo[peak_idx]
peak_dist = peak_idx * CM_PER_SLOT   # Step 7
peak_us   = peak_idx * US_PER_SLOT   # Step 5

print("=" * 60)
print("Detection result")
print("=" * 60)
print(f"  Noise floor (Listen Only) : max={lo_max}  mean={lo_mean:.1f}")
print(f"  Ring-down blanked         : samples 0-{RINGDOWN_BLANK-1}")
print(f"  Peak sample index         : {peak_idx}")
print(f"  Peak amplitude            : {peak_amp:.1f}  (0-255)")
print(f"  Measured TOF              : {peak_us:.1f} us  (={peak_idx}x{US_PER_SLOT:.0f} us/slot)")
print(f"  Measured distance         : {peak_dist:.2f} cm  (={peak_idx}x{CM_PER_SLOT:.4f} cm/slot)")
print(f"  Expected 5 cm echo        : sample {ECHO_SLOT:.1f}  TOF={TOF_US:.1f} us")

# Interpretation
print()
if peak_amp > 200:
    print("  SATURATED — signal is clipping.")
    print(f"  Increase GAIN_INIT in steps of 10.")
    print(f"  Current GAIN_INIT={GAIN_INIT_VAL}. Try changing to 0 if not already.")
    print(f"  If already 0, add write_register(0x26, 0x60) to shift gain range down.")
elif peak_amp < 20:
    print("  WEAK SIGNAL — try increasing GAIN_INIT.")
    print(f"  Change GAIN_INIT_VAL from {GAIN_INIT_VAL} to 20, then 40, then 63.")
elif abs(peak_dist - 5.0) < 2.0:
    print("  Echo detected near expected position.")
else:
    print(f"  Echo at {peak_dist:.1f} cm, expected 5.0 cm.")
    print("  If signal is clean (not saturated, not noisy), this may be accurate.")
    print("  Check physical target distance.")

# ============================================================
# PLOT
# ============================================================
dist_axis  = np.arange(128) * CM_PER_SLOT   # cm (Step 7)
time_axis  = np.arange(128) * US_PER_SLOT   # us (Step 5)

fig, axes = plt.subplots(2, 1, figsize=(13, 9))
fig.suptitle(
    f"PGA460  40 kHz default mode  |  {good}x averaged  |  "
    f"P1_REC={P1_REC} -> {T_REC_MS:.3f} ms  |  "
    f"{US_PER_SLOT:.0f} us/slot  |  {CM_PER_SLOT:.3f} cm/slot  |  "
    f"GAIN_INIT={GAIN_INIT_VAL} ({0.5*(GAIN_INIT_VAL+1)+58:.1f} dB)",
    fontsize=10
)

for ax, x_axis, xlabel, xunit, exp_val in [
    (axes[0], dist_axis, "Distance", "cm", 5.0),
    (axes[1], time_axis, "Time",     "us", TOF_US),
]:
    # Noise floor band
    ax.axhspan(0, lo_max, alpha=0.08, color='gray', label=f'Noise floor (listen-only max={lo_max})')
    ax.plot(x_axis, echo, color='steelblue', linewidth=2, label='EDD envelope (averaged)')
    ax.axvspan(x_axis[0], x_axis[RINGDOWN_BLANK - 1],
               alpha=0.2, color='orange',
               label=f'Ring-down blanked (0-{RINGDOWN_BLANK-1})')
    ax.axvline(exp_val, color='red', linestyle='--', linewidth=1.5,
               label=f'Expected 5 cm ({exp_val:.1f} {xunit})')
    if peak_idx >= RINGDOWN_BLANK:
        ax.axvline(x_axis[peak_idx], color='green', linestyle=':', linewidth=2,
                   label=f'Peak: sample {peak_idx} ({x_axis[peak_idx]:.2f} {xunit})')
    ax.set_xlabel(f'{xlabel} ({xunit})', fontsize=11)
    ax.set_ylabel('Amplitude (0-255)', fontsize=11)
    ax.set_xlim(x_axis[0], x_axis[-1])
    ax.set_ylim(-5, 265)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
fname = f"pga460_40kHz_5cm_{ts}.png"
plt.savefig(fname, dpi=200, bbox_inches='tight')
print(f"\nPlot saved: {fname}")
plt.show()