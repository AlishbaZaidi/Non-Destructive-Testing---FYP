"""
PGA460-Q1 PSM-EVM — 40 kHz DEFAULT mode, 5 cm target, air medium
Transducer: UTR-1440K-TT-R (40 kHz closed-top, on-board)
Interface:  UART via J2 connector, USB-to-serial adapter

Every register value and formula is cited to SLASEC8C (PGA460-Q1 datasheet).

GAIN STRATEGY (start here, tune if needed):
  AFE_GAIN_RNG = 3  -> gain offset = 32 dB  (lowest possible range)
  GAIN_INIT    = 0  -> Init_Gain = 0.5*(0+1) + 32 = 32.5 dB  (minimum)
  If signal is too weak (max < 20): increase GAIN_INIT in steps of 10.
  If signal is still saturated:     check for ambient noise sources.
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# ============================================================
# USER SETTINGS  — only change these
# ============================================================
COM_PORT     = "COM12"
BAUDRATE     = 115200
NUM_AVERAGES = 8        # number of bursts to average
SOUND_SPEED  = 343.0   # m/s at ~20 deg C

# ============================================================
# STEP-BY-STEP TIMING DERIVATION
# All formulas from datasheet. No invented values.
# ============================================================

# ----------------------------------------------------------
# STEP 1 — Round-trip time of flight for 5 cm target
# ----------------------------------------------------------
# Physics: sound travels TO target and BACK.
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
# P1_REC = 0  ->  minimum window = 4.096 ms.
# Minimum window means each EDD slot covers the least time,
# so the echo lands at the HIGHEST possible sample index,
# maximising separation from ring-down at sample 0.
#
#   T_rec = 4.096 x (0 + 1) = 4.096 ms = 4096 us
P1_REC   = 0
T_REC_MS = 4.096 * (P1_REC + 1)   # 4.096 ms
T_REC_US = T_REC_MS * 1000.0      # 4096 us

# ----------------------------------------------------------
# STEP 3 — Total raw DSP samples inside the window
# ----------------------------------------------------------
# SOURCE: p.42, Section 7.3.7.1:
#   "the output rate of the digital data path is 1 us/sample"
#
#   total_DSP = 4096 us / 1 us = 4096 raw samples
DSP_RATE_US  = 1.0
TOTAL_DSP    = int(T_REC_US / DSP_RATE_US)   # 4096

# ----------------------------------------------------------
# STEP 4 — Raw DSP samples per EDD slot
# ----------------------------------------------------------
# SOURCE: p.42, Section 7.3.7.1 (their exact example):
#   "8192 / 128 = 64 samples per slot"  (for P1_REC=1)
#   General formula: total_DSP / 128
#
#   DSP_per_slot = 4096 / 128 = 32
EDD_SLOTS    = 128
DSP_PER_SLOT = TOTAL_DSP / EDD_SLOTS   # 32.0

# ----------------------------------------------------------
# STEP 5 — Microseconds per EDD slot
# ----------------------------------------------------------
#   us_per_slot = 32 samples x 1 us/sample = 32 us
US_PER_SLOT = DSP_PER_SLOT * DSP_RATE_US   # 32.0 us

# ----------------------------------------------------------
# STEP 6 — Which EDD slot contains the 5 cm echo
# ----------------------------------------------------------
#   EDD_slot = TOF_us / us_per_slot
#            = 291.5  / 32.0
#            = 9.11  -> index 9
#
# Slot 9 covers the time range 288 us to 319 us.
# 291.5 us falls inside this range -> sample index 9.
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
# At 40 kHz with 8 pulse pairs, ring-down lasts longer than
# at 300 kHz. Blank samples 0-8. The echo is at sample 9,
# right at the boundary — it may still have some ring-down
# contamination. For a cleaner measurement move the target
# to 15-20 cm (echo at sample 27-36, well clear of ring-down).
RINGDOWN_BLANK = 9

print("=" * 60)
print("40 kHz default mode — timing derivation")
print("=" * 60)
print(f"  TOF (Step 1)       = {TOF_US:.2f} us")
print(f"  T_rec (Step 2)     = {T_REC_US:.0f} us  [4.096x(P1_REC+1) ms, Table 7-43 p.70]")
print(f"  Total DSP (Step 3) = {TOTAL_DSP}  [1 us/sample, p.42]")
print(f"  DSP/slot  (Step 4) = {DSP_PER_SLOT:.0f}  [total/128, p.42]")
print(f"  us/slot   (Step 5) = {US_PER_SLOT:.0f} us")
print(f"  Echo slot (Step 6) = {ECHO_SLOT:.2f} -> index {int(ECHO_SLOT)}")
print(f"  cm/slot   (Step 7) = {CM_PER_SLOT:.4f} cm")
print(f"  Ring-down blank    = samples 0-{RINGDOWN_BLANK-1}")
print()

# ============================================================
# REGISTER VALUES — every one derived from datasheet
# ============================================================

# --- EE_CNTRL 0x40 = 0x80 ---
# SOURCE: Table 7-53 p.75: bit7 = DATADUMP_EN = 1
# SOURCE: p.42: required for EDD memory to be written at all.
# 0x80 = 1000 0000
EE_CNTRL_VAL = 0x80

# --- DECPL_TEMP 0x26 = 0x0B ---
# SOURCE: Table 7-7 p.54: register 0x26 = DECPL_TEMP
# SOURCE: Table 7-6 p.53: factory default = 0x0A = 0000 1010
#   bits[7:6] = DECPL_TEMP_SEL = 0  (keep)
#   bits[5:2] = DECPL_T        = 2  (keep)
#   bits[1:0] = AFE_GAIN_RNG   = 2  (factory) -> change to 3
#
# SOURCE: Section 6.7 p.7 (receiver characteristics table):
#   AFE_GAIN_RNG=0 -> gain range  58-90 dB  (factory)
#   AFE_GAIN_RNG=1 -> gain range  52-84 dB
#   AFE_GAIN_RNG=2 -> gain range  46-78 dB  (previous default in use)
#   AFE_GAIN_RNG=3 -> gain range  32-64 dB  <- lowest possible
#
# With the previous setting (AFE_GAIN_RNG=2) and GAIN_INIT=0:
#   Init_Gain = 0.5*(0+1) + 46 = 46.5 dB
#   Listen-only max=255 -> even 46.5 dB saturates on ambient noise.
#
# New value: AFE_GAIN_RNG=3, GAIN_INIT=0:
#   Init_Gain = 0.5*(0+1) + 32 = 32.5 dB  (absolute minimum)
#
# Preserve bits[7:2], change only bits[1:0]:
#   new = (0x0A & 0xFC) | 0x03 = 0x08 | 0x03 = 0x0B

# SOURCE: Table 7-47 p.72: bits[7:6] = AFE_GAIN_RNG
# 11b = 32 to 64 dB (lowest range)
# Preserve bits[5:0] from factory default 0x0A:
# new = (3<<6) | (0x0A & 0x3F) = 0xC0 | 0x0A = 0xCA
DECPL_TEMP_VAL = 0xCA

# --- INIT_GAIN 0x1B = 0x40 ---
# SOURCE: Table 7-36 p.67:
#   bits[7:6] = BPF_BW: BandWidth = 2 x (BPF_BW + 1)  [kHz]
#     BPF_BW=1 -> 2x(1+1) = 4 kHz  (good for 40 kHz transducer)
#   bits[5:0] = GAIN_INIT
#     Init_Gain = 0.5 x (GAIN_INIT + 1) + value(AFE_GAIN_RNG)  [dB]
#     With AFE_GAIN_RNG=3 (offset=32 dB):
#       GAIN_INIT=0  -> 32.5 dB  <- start here
#       GAIN_INIT=10 -> 37.5 dB
#       GAIN_INIT=20 -> 42.5 dB
#       GAIN_INIT=40 -> 52.5 dB
#       GAIN_INIT=63 -> 64.0 dB  (max in this range)
#
# byte = (BPF_BW << 6) | GAIN_INIT
#      = (1 << 6)      | 0
#      = 0x40
BPF_BW_VAL    = 1
GAIN_INIT_VAL = 0
INIT_GAIN_VAL = (BPF_BW_VAL << 6) | GAIN_INIT_VAL   # = 0x40

# Compute actual gain for display
AFE_GAIN_OFFSET = 32   # offset for AFE_GAIN_RNG=3
ACTUAL_GAIN_DB  = 0.5 * (GAIN_INIT_VAL + 1) + AFE_GAIN_OFFSET

# --- FREQUENCY 0x1C = 0x32 ---
# SOURCE: Table 7-37 p.67:
#   Frequency = 0.2 x FREQ + 30  [kHz]  (FREQ_SHIFT=0)
#   For 40 kHz: 40 = 0.2 x FREQ + 30  ->  FREQ = (40-30)/0.2 = 50 = 0x32
#   Verify: 0.2 x 50 + 30 = 40.0 kHz  ✓
FREQUENCY_VAL = 50   # = 0x32

# --- DEADTIME 0x1D = 0x00 ---
# SOURCE: Table 7-38 p.68:
#   bits[7:4] = THR_CMP_DEGLTCH = 0 (no deglitch)
#   bits[3:0] = PULSE_DT: DeadTime = 0.0625 x PULSE_DT  [us]
# SOURCE: Section 7.3.2.1 p.12:
#   Board uses center-tap transformer with complementary FETs.
#   No dead-time is needed in transformer drive mode.
#   Constraint (p.14): max DT <= t/8 = 25/8 = 3.125 us  (at 40 kHz)
#   -> PULSE_DT=0  byte = 0x00
DEADTIME_VAL  = 0x00

# --- PULSE_P1 0x1E = 0x08 ---
# SOURCE: Table 7-39 p.68:
#   bits[7:5] = IO_IF_SEL=0, UART_DIAG=0, IO_DIS=0
#   bits[4:0] = P1_PULSE: 0h = 1 pulse on OUTA only
#                          8 = 8 full pulse PAIRS on OUTA+OUTB
# 8 pulses x (1/40 kHz) = 200 us burst — adequate for 5 cm
# byte = 0x08
PULSE_P1_VAL  = 0x08

# --- REC_LENGTH 0x22 = 0x0C ---
# SOURCE: Table 7-43 p.70:
#   bits[7:4] = P1_REC = 0  -> 4.096 ms minimum window
#   bits[3:0] = P2_REC = 12 -> keep factory default
# SOURCE: Table 7-6 p.53: factory REC_LENGTH = 0x1C -> P2_REC=12
# byte = (P1_REC << 4) | P2_REC = (0<<4)|12 = 0x0C
P2_REC_DEFAULT = 12
REC_LENGTH_VAL = (P1_REC << 4) | P2_REC_DEFAULT   # = 0x0C

# NOTE: TVGAIN6 (0x1A) is NOT written.
#   Factory default 0xFC already has FREQ_SHIFT=0 (30-80 kHz range).
#
# NOTE: BPF registers 0x41-0x46 are NOT written.
#   SOURCE: p.16: when FREQ_SHIFT=0, the chip auto-calculates BPF
#   coefficients after FREQUENCY or BPF_BW changes.
#   We write FREQUENCY then INIT_GAIN and wait — chip does the rest.

print("Register values")
print("-" * 60)
print(f"  0x40 = 0x{EE_CNTRL_VAL:02X}  EE_CNTRL    DATADUMP_EN=1             (Table 7-53 p.75)")
print(f"  0x26 = 0x{DECPL_TEMP_VAL:02X}  DECPL_TEMP  AFE_GAIN_RNG=3 (32dB base) (Sec 6.7 p.7)")
print(f"  0x1B = 0x{INIT_GAIN_VAL:02X}  INIT_GAIN   BPF_BW=4kHz GAIN_INIT=0   (Table 7-36 p.67)")
print(f"         -> Init_Gain = 0.5x(0+1)+32 = {ACTUAL_GAIN_DB:.1f} dB")
print(f"  0x1C = 0x{FREQUENCY_VAL:02X}  FREQUENCY   FREQ=50 -> 40kHz          (Table 7-37 p.67)")
print(f"  0x1D = 0x{DEADTIME_VAL:02X}  DEADTIME    PULSE_DT=0                 (Table 7-38 p.68)")
print(f"  0x1E = 0x{PULSE_P1_VAL:02X}  PULSE_P1    P1_PULSE=8 pairs            (Table 7-39 p.68)")
print(f"  0x22 = 0x{REC_LENGTH_VAL:02X}  REC_LENGTH  P1_REC=0 P2_REC=12         (Table 7-43 p.70)")
print(f"  0x1A       NOT written  (FREQ_SHIFT stays 0, Table 7-35 p.67)")
print(f"  0x41-0x46  NOT written  (BPF auto-calculated, Section 7.3.4.1 p.16)")
print("-" * 60)
print()

# ============================================================
# LOW-LEVEL HELPERS
# ============================================================

def checksum(data):
    """
    SOURCE: p.34, Section 7.3.6.2.1.4:
      'inverted byte sum WITH CARRY operation'
      Sync byte 0x55 is excluded from checksum.

    'With carry' means: if sum > 255, the overflow is added back
    before inverting. This is a one's complement sum.
    Simple truncation (~(sum & 0xFF)) gives wrong results when
    sum > 255 — this was the root cause of TVGAIN6 silently failing.

    Verification — datasheet Example 2 p.52:
      Write 0x40=0x80: sum = 0x0A+0x40+0x80 = 0xCA (no carry needed)
      CHK = ~0xCA & 0xFF = 0x35  (datasheet confirms 0x35) ✓
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
      Response: none (device is silent)
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
      Receive: DIAG(1 byte), DATA(1 byte), CHK(1 byte) = 3 bytes
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
               N_OBJECTS = 1
               CHK = checksum([0x00, 0x01]) = 0xFE
      Response: none

    EDD command (cmd 7):
    SOURCE: Table 7-3 p.34:
      Send:    0x55, 0x07, CHK   where CHK = 0xF8
      Receive: DIAG(1) + data x 128 + CHK(1) = 130 bytes
    """
    ser.reset_input_buffer()

    # Burst + Listen Preset 1
    # n_obj     = 1
    # burst_pkt = [0x55, 0x00, n_obj]
    # burst_pkt.append(checksum(burst_pkt[1:]))   # = 0xFE
    n_obj = 0                                        # was 1
    burst_pkt = [0x55, 0x00, n_obj]
    burst_pkt.append(checksum(burst_pkt[1:]))   
    ser.write(bytearray(burst_pkt))

    # Wait: record window (4.096 ms) + DSP settle + margin = 120 ms
    time.sleep(0.12)

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
    Command 2 = Listen Only Preset 1 — NO burst is fired.
    SOURCE: Table 7-3 p.34:
      Send: 0x55, 0x02, N_OBJECTS, CHK
    Used to measure the ambient noise floor.
    If max is high here, noise is ambient (not from the burst).
    If max is low here but high during burst, noise is burst-generated.
    """
    ser.reset_input_buffer()
    n_obj = 0
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
# Write order:
#   1. Registers with no auto-calc side effects.
#   2. FREQUENCY  -> triggers chip BPF auto-calculation (wait 50ms).
#   3. INIT_GAIN  -> BPF_BW change triggers another auto-calc (wait 50ms).
# After step 3 the chip has computed correct 40kHz BPF coefficients.
# We never write 0x41-0x46 — the chip handles them automatically.
# ============================================================
print("Writing registers ...")

write_register(0x40, EE_CNTRL_VAL)    # DATADUMP_EN=1         (Table 7-53 p.75)
write_register(0x26, DECPL_TEMP_VAL)  # AFE_GAIN_RNG=3        (Section 6.7 p.7)
write_register(0x1D, DEADTIME_VAL)    # PULSE_DT=0            (Table 7-38 p.68)
write_register(0x1E, PULSE_P1_VAL)    # P1_PULSE=8            (Table 7-39 p.68)
write_register(0x22, REC_LENGTH_VAL)  # P1_REC=0              (Table 7-43 p.70)

# FREQUENCY triggers BPF auto-calculation
write_register(0x1C, FREQUENCY_VAL)   # FREQ=50 -> 40kHz      (Table 7-37 p.67)
time.sleep(0.05)                       # wait for auto-calc

# INIT_GAIN (BPF_BW change) triggers another auto-calculation
write_register(0x1B, INIT_GAIN_VAL)   # BPF_BW=4kHz GAIN=0    (Table 7-36 p.67)
time.sleep(0.05)                       # wait for auto-calc
# Chip has now written correct BPF coefficients for 40 kHz.

print("Done.\n")

# ============================================================
# VERIFY REGISTERS
# ============================================================
print("Verifying registers (cmd 0x09, Table 7-3 p.34) ...")
verify = {
    0x40: (EE_CNTRL_VAL,   "EE_CNTRL    DATADUMP_EN=1"),
    0x26: (0xCA, "DECPL_TEMP  AFE_GAIN_RNG=3 bits[7:6]=11"),
    0x1B: (INIT_GAIN_VAL,  "INIT_GAIN   BPF_BW=1(4kHz) GAIN_INIT=0"),
    0x1C: (FREQUENCY_VAL,  "FREQUENCY   FREQ=50 -> 40kHz"),
    0x1D: (DEADTIME_VAL,   "DEADTIME    PULSE_DT=0"),
    0x1E: (PULSE_P1_VAL,   "PULSE_P1    P1_PULSE=8"),
    0x22: (REC_LENGTH_VAL, "REC_LENGTH  P1_REC=0 P2_REC=12"),
}
all_ok = True
for addr, (expected, label) in verify.items():
    val = read_register(addr)
    if val is None:
        print(f"  0x{addr:02X}  {label:44s}  NO RESPONSE")
        all_ok = False
    elif val == expected:
        print(f"  0x{addr:02X}  {label:44s}  0x{val:02X} OK")
    else:
        print(f"  0x{addr:02X}  {label:44s}  read=0x{val:02X} expected=0x{expected:02X} MISMATCH")
        all_ok = False

if not all_ok:
    print("\n  One or more mismatches. Cross-check Table 7-7 p.54 of SLASEC8C.")
else:
    print("\n  All registers OK.\n")

# ============================================================
# NOISE FLOOR CHECK — Listen Only (no burst)
# ============================================================
print("Noise floor check (Listen Only — no burst fired) ...")
raw_lo  = listen_only_capture()
samp_lo = np.frombuffer(raw_lo[1:129], dtype=np.uint8).astype(np.float64)
lo_min  = int(samp_lo.min())
lo_max  = int(samp_lo.max())
lo_mean = samp_lo.mean()
print(f"  min={lo_min}  max={lo_max}  mean={lo_mean:.1f}")

if lo_max > 100:
    print("  HIGH NOISE FLOOR (max > 100) even with no burst.")
    print("  Possible causes:")
    print("    - Strong ambient ultrasound (fans, AC unit, machinery)")
    print("    - Move board away from any motors or air vents")
    print("    - If max=255 persists, try shielding INP/INN traces")
elif lo_max > 30:
    print("  MODERATE noise floor. Echo detection may still work.")
else:
    print("  Noise floor OK — proceed with burst capture.")
print()

# ============================================================
# BURST CAPTURE — averaged
# ============================================================
print(f"Capturing {NUM_AVERAGES} bursts ...")
accum = np.zeros(128, dtype=np.float64)
good  = 0

for i in range(NUM_AVERAGES):
    raw = burst_and_capture()
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
search[:RINGDOWN_BLANK] = 0.0   # zero out ring-down region (samples 0-8)

peak_idx  = int(np.argmax(search))
peak_amp  = echo[peak_idx]
peak_dist = peak_idx * CM_PER_SLOT   # from Step 7
peak_us   = peak_idx * US_PER_SLOT   # from Step 5

print("=" * 60)
print("Detection result")
print("=" * 60)
print(f"  Gain setting              : GAIN_INIT={GAIN_INIT_VAL}, "
      f"AFE_GAIN_RNG=3 -> {ACTUAL_GAIN_DB:.1f} dB")
print(f"  Noise floor (listen-only) : max={lo_max}  mean={lo_mean:.1f}")
print(f"  Ring-down blanked         : samples 0-{RINGDOWN_BLANK-1}")
print(f"  Peak sample index         : {peak_idx}")
print(f"  Peak amplitude            : {peak_amp:.1f}  (0-255)")
print(f"  Measured TOF              : {peak_us:.1f} us  "
      f"(= {peak_idx} x {US_PER_SLOT:.0f} us/slot,  Step 5)")
print(f"  Measured distance         : {peak_dist:.2f} cm  "
      f"(= {peak_idx} x {CM_PER_SLOT:.4f} cm/slot,  Step 7)")
print(f"  Expected 5 cm echo        : sample {ECHO_SLOT:.1f},  TOF={TOF_US:.1f} us")
print()

# Interpretation and tuning hints
if lo_max > 100:
    print("  ACTION: Noise floor still high. Check for ambient noise sources.")
    print("          Try running in a quieter location or add acoustic shielding.")
elif peak_amp > 240:
    print("  ACTION: Signal saturating (max > 240).")
    print("          GAIN_INIT is already 0. Reduce further via DECPL_TEMP if needed.")
    print("          But first check if noise floor is the true cause.")
elif peak_amp < 20:
    print("  ACTION: Weak signal (max < 20).")
    new_g = GAIN_INIT_VAL + 10
    new_val = (BPF_BW_VAL << 6) | min(new_g, 63)
    print(f"          Increase GAIN_INIT from {GAIN_INIT_VAL} to {min(new_g,63)}.")
    print(f"          Change: GAIN_INIT_VAL = {min(new_g,63)}  (byte = 0x{new_val:02X})")
elif abs(peak_dist - 5.0) < CM_PER_SLOT * 2:
    print(f"  Echo detected at {peak_dist:.2f} cm — within 2 slots of expected 5 cm.")
else:
    print(f"  Echo detected at {peak_dist:.2f} cm (expected 5.0 cm).")
    print("  If waveform looks clean (single peak, low noise floor), verify")
    print("  the physical distance to your target with a ruler.")

# ============================================================
# PLOT
# ============================================================
dist_axis = np.arange(128) * CM_PER_SLOT   # cm  (Step 7)
time_axis = np.arange(128) * US_PER_SLOT   # us  (Step 5)

fig, axes = plt.subplots(2, 1, figsize=(13, 9))
fig.suptitle(
    f"PGA460  40 kHz default  |  {good}x avg  |  "
    f"P1_REC={P1_REC} -> {T_REC_MS:.3f} ms  |  "
    f"{US_PER_SLOT:.0f} us/slot  |  {CM_PER_SLOT:.3f} cm/slot  |  "
    f"GAIN={ACTUAL_GAIN_DB:.1f} dB  (GAIN_INIT={GAIN_INIT_VAL}, RNG=3)",
    fontsize=10
)

for ax, x_axis, xlabel, xunit, exp_val in [
    (axes[0], dist_axis, "Distance", "cm", 5.0),
    (axes[1], time_axis, "Time",     "us", TOF_US),
]:
    # Noise floor band (listen-only)
    ax.axhspan(0, lo_max,
               alpha=0.10, color='gray',
               label=f'Noise floor — listen-only max={lo_max}')
    # EDD envelope
    ax.plot(x_axis, echo,
            color='steelblue', linewidth=2,
            label='EDD envelope (averaged)')
    # Ring-down region
    ax.axvspan(x_axis[0], x_axis[RINGDOWN_BLANK - 1],
               alpha=0.20, color='orange',
               label=f'Ring-down blanked (0-{RINGDOWN_BLANK-1})')
    # Expected echo position
    ax.axvline(exp_val,
               color='red', linestyle='--', linewidth=1.5,
               label=f'Expected 5 cm ({exp_val:.1f} {xunit})')
    # Detected peak
    if peak_idx >= RINGDOWN_BLANK:
        ax.axvline(x_axis[peak_idx],
                   color='green', linestyle=':', linewidth=2,
                   label=f'Peak: sample {peak_idx}  ({x_axis[peak_idx]:.2f} {xunit})')

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
print(f"Plot saved: {fname}")
plt.show()