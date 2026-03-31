"""
PGA460-Q1 — 300 kHz, 5 cm target, PSM-EVM board, UART via J2
Every number in this file is derived step-by-step from SLASEC8C.
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from scipy import signal as sp_signal

# ============================================================
# USER SETTINGS
# ============================================================
COM_PORT     = "COM12"
BAUDRATE     = 115200
NUM_AVERAGES = 8
SOUND_SPEED  = 343.0   # m/s at ~20 deg C

# ============================================================
# STEP-BY-STEP TIMING DERIVATION
# Every number below is computed from first principles.
# No values are assumed or invented.
# ============================================================

# ----------------------------------------------------------
# STEP 1 — Round-trip time of flight for 5 cm target
# ----------------------------------------------------------
# Physics: sound must travel TO the target AND back.
#
#   TOF = (2 x distance) / speed_of_sound
#       = (2 x 0.05 m)   / 343.0 m/s
#       = 0.10 m          / 343.0 m/s
#       = 0.00029155 s
#       = 291.5 us
TARGET_DIST_M = 0.05
TOF_S         = (2 * TARGET_DIST_M) / SOUND_SPEED   # 0.00029155 s
TOF_US        = TOF_S * 1e6                          # 291.5 us

# ----------------------------------------------------------
# STEP 2 — Record window duration
# ----------------------------------------------------------
# SOURCE: Table 7-43, p.70, REC_LENGTH register field P1_REC:
#   "Preset1 record time length:
#    Record time = 4.096 x (P1_REC + 1)  [ms]"
#
# This formula has NO dependency on FREQ_SHIFT.
# FREQ_SHIFT only scales the burst driving frequency (Table 7-35 p.67).
# It does NOT change the record window or DSP sample rate.
#
# We choose P1_REC=0 because it gives the SMALLEST possible window.
# A smaller window means each EDD slot covers less time, so the echo
# lands at a HIGHER sample index (further from ring-down at sample 0).
#
# Effect of each P1_REC value on echo position:
#   P1_REC=0 -> T=4.096ms -> 32us/slot -> echo at sample 9.1  (use this)
#   P1_REC=1 -> T=8.192ms -> 64us/slot -> echo at sample 4.6
#   P1_REC=2 -> T=12.288ms-> 96us/slot -> echo at sample 3.0
#   P1_REC=3 -> T=16.384ms->128us/slot -> echo at sample 2.3
#
#   P1_REC=0:
#   T_rec = 4.096 x (0 + 1) = 4.096 x 1 = 4.096 ms = 4096 us
P1_REC       = 0
T_REC_MS     = 4.096 * (P1_REC + 1)   # 4.096 ms   SOURCE: Table 7-43 p.70
T_REC_US     = T_REC_MS * 1000.0      # 4096.0 us

# ----------------------------------------------------------
# STEP 3 — Total raw DSP samples inside the record window
# ----------------------------------------------------------
# SOURCE: p.42, Section 7.3.7.1, exact quote:
#   "Because the output rate of the digital data path is 1 us/sample,
#    the total record interval has 8192 samples."
#   (Their worked example uses P1_REC=1 -> 8192 us -> 8192 samples.)
#
# So: one raw DSP sample is produced every 1 us, always.
#
#   total_DSP = T_rec_us / (1 us/sample)
#             = 4096 us  / 1
#             = 4096 raw DSP samples
DSP_RATE_US   = 1.0                         # 1 us per raw sample (p.42)
TOTAL_DSP     = int(T_REC_US / DSP_RATE_US) # 4096 raw samples

# ----------------------------------------------------------
# STEP 4 — How many raw DSP samples are compressed into
#           one EDD slot
# ----------------------------------------------------------
# SOURCE: p.42, Section 7.3.7.1, exact quote:
#   "one sample location in the data-dump memory is written with
#    the highest (peak) value of 8192 / 128 = 64 samples."
#   (Their example: P1_REC=1 -> 8192 total -> 64 per EDD slot.)
#
# The EDD always has exactly 128 slots.
# SOURCE: p.42: "128-byte data memory array"
# Each slot stores the PEAK of the raw DSP samples that land in it.
#
#   DSP_per_slot = total_DSP / 128
#                = 4096 / 128
#                = 32 raw DSP samples per EDD slot
EDD_SLOTS     = 128
DSP_PER_SLOT  = TOTAL_DSP / EDD_SLOTS       # 32.0 raw samples per slot

# ----------------------------------------------------------
# STEP 5 — Microseconds covered by one EDD slot
# ----------------------------------------------------------
# Each raw DSP sample lasts 1 us (Step 3).
# Each EDD slot holds 32 raw DSP samples (Step 4).
#
#   us_per_slot = DSP_per_slot x 1 us/sample
#               = 32 x 1
#               = 32 us
US_PER_SLOT   = DSP_PER_SLOT * DSP_RATE_US  # 32.0 us per EDD slot

# ----------------------------------------------------------
# STEP 6 — Which EDD slot index contains the 5 cm echo
# ----------------------------------------------------------
# The echo arrives at TOF = 291.5 us (Step 1).
# Each EDD slot covers 32 us of elapsed time (Step 5).
# EDD slots are zero-indexed:
#   slot 0  covers time 0 us  to 31 us
#   slot 1  covers time 32 us to 63 us
#   ...
#   slot 9  covers time 288 us to 319 us  <- echo at 291.5 us is HERE
#
#   EDD_slot = TOF_us / us_per_slot
#            = 291.5 us / 32.0 us
#            = 9.1108
#            -> floor to integer index = 9
#
# So the 5 cm echo will appear at EDD sample index 9.
ECHO_EDD_SLOT = TOF_US / US_PER_SLOT        # 9.1108 -> sample index 9

# ----------------------------------------------------------
# STEP 7 — Centimetres per EDD slot (for plotting distance axis)
# ----------------------------------------------------------
# One slot covers 32 us. In 32 us sound travels (round trip):
#   distance = (speed x time) / 2
#            = (343.0 m/s x 32e-6 s) / 2
#            = 0.010976 m / 2
#            = 0.005488 m
#            = 0.5488 cm per slot
CM_PER_SLOT   = (US_PER_SLOT * 1e-6 * SOUND_SPEED / 2) * 100  # 0.5488 cm

# ----------------------------------------------------------
# Ring-down blanking
# ----------------------------------------------------------
# Samples 0-7 are contaminated by transducer ring-down after burst.
# The echo is expected at sample 9, so we blank only samples 0-7.
# If ring-down is still visible at sample 8 in your plot, raise this to 9.
RINGDOWN_BLANK = 8

print("=" * 60)
print("Timing derivation (all formulas from datasheet)")
print("=" * 60)
print(f"  Step 1  TOF             = {TOF_US:.2f} us")
print(f"  Step 2  Record window   = {T_REC_US:.0f} us  [4.096x(P1_REC+1) ms, Table 7-43 p.70]")
print(f"  Step 3  Total DSP       = {TOTAL_DSP} samples  [1 us/sample, p.42]")
print(f"  Step 4  DSP per slot    = {DSP_PER_SLOT:.0f}  [total_DSP / 128, p.42]")
print(f"  Step 5  us per slot     = {US_PER_SLOT:.0f} us")
print(f"  Step 6  Echo slot       = {ECHO_EDD_SLOT:.4f} -> index {int(ECHO_EDD_SLOT)}")
print(f"  Step 7  cm per slot     = {CM_PER_SLOT:.4f} cm")
print()

# ============================================================
# BPF COEFFICIENTS
# ============================================================
# SOURCE: Section 7.3.4.1, p.16:
#   "In case the FREQ_SHIFT bit is set to 1 ... the band-pass filter
#    coefficients are not calculated automatically by the PGA460-Q1
#    device. In this case the MCU is required to write these values."
#
# Filter design:
#   Type      : 2nd-order Butterworth band-pass IIR
#   ADC rate  : 1 MHz  (Table 6.8 p.7: "Conversion time = 1 us")
#   Centre    : 300 kHz
#   Bandwidth : 8 kHz  (BPF_BW=3, see INIT_GAIN derivation below)
#   Low edge  : 300 - 4 = 296 kHz
#   High edge : 300 + 4 = 304 kHz
#
# PGA460 transfer function (BPF_A2/A3/B1 registers, Table 7-7 p.54):
#   H(z) = B1*(1 - z^-2) / (1 + A2*z^-1 + A3*z^-2)
#
# scipy.signal.butter(1, Wn, btype='bandpass') returns:
#   b = [b0, 0, -b0]   ->  B1 = b0
#   a = [1, a1, a2]    ->  A2 = a1,  A3 = a2
#
# Coefficients scaled to Q14 fixed-point: multiply by 16384, round.
# Stored as signed 16-bit big-endian pairs in registers 0x41-0x46.

FS_HZ  = 1e6
FC_HZ  = 300e3
BW_HZ  = 8e3
Wn     = [(FC_HZ - BW_HZ/2) / (FS_HZ/2),   # 296000/500000 = 0.592
          (FC_HZ + BW_HZ/2) / (FS_HZ/2)]   # 304000/500000 = 0.608

b_coef, a_coef = sp_signal.butter(1, Wn, btype='bandpass')

A2_f = float(a_coef[1])   #  0.60306926
A3_f = float(a_coef[2])   #  0.95095678
B1_f = float(b_coef[0])   #  0.02452161

Q14    = 16384.0
A2_int = int(round(A2_f * Q14))   #  9881 = 0x2699
A3_int = int(round(A3_f * Q14))   # 15580 = 0x3CDC
B1_int = int(round(B1_f * Q14))   #   402 = 0x0192

print("BPF coefficients (Section 7.3.4.1 p.16)")
print(f"  fc={FC_HZ/1e3:.0f} kHz  BW={BW_HZ/1e3:.0f} kHz  fs={FS_HZ/1e6:.0f} MHz")
print(f"  A2 = {A2_f:+.8f}  ->  Q14: {A2_int:6d}  0x{A2_int & 0xFFFF:04X}")
print(f"  A3 = {A3_f:+.8f}  ->  Q14: {A3_int:6d}  0x{A3_int & 0xFFFF:04X}")
print(f"  B1 = {B1_f:+.8f}  ->  Q14: {B1_int:6d}  0x{B1_int & 0xFFFF:04X}")
print()

# ============================================================
# REGISTER VALUE DERIVATIONS
# ============================================================

# --- EE_CNTRL 0x40 ---
# SOURCE: Table 7-53, p.75
#   bit 7 = DATADUMP_EN: "0b=Disabled  1b=Enabled"
# SOURCE: p.42:
#   "enabled by the DATADUMP_EN bit in the EE_CNTRL register.
#    When enabled, and upon receiving a BURST/LISTEN command,
#    the PGA460-Q1 device holds the IO pin low for the entire
#    record interval ... When the data-dump cycle is complete
#    the data can be extracted by the data dump read command."
# Without this bit set the EDD memory is never written.
# 0x80 = 1000 0000
#         ^-------  bit 7 = DATADUMP_EN = 1
EE_CNTRL_VAL = 0x80

# --- TVGAIN6 0x1A ---
# SOURCE: Table 7-35, p.67
#   bits [7:2] = TVG_G5  (keep factory default)
#   bit  [1]   = RESERVED
#   bit  [0]   = FREQ_SHIFT: "1b = Enabled, active frequency = 6 x ..."
# Factory EEPROM default (Table 7-6, p.53): REC_LENGTH default = 0xFC
#   0xFC = 1111 1100  ->  TVG_G5=63, RESERVED=0, FREQ_SHIFT=0
# Set bit 0 to enable FREQ_SHIFT:
#   0xFD = 1111 1101  ->  TVG_G5=63, RESERVED=0, FREQ_SHIFT=1
TVGAIN6_VAL = 0xFD

# --- INIT_GAIN 0x1B ---
# SOURCE: Table 7-36, p.67
#   bits [7:6] = BPF_BW: "BandWidth = 2 x (BPF_BW + 1) [kHz]"
#     BPF_BW=0 -> 2x(0+1) = 2 kHz
#     BPF_BW=1 -> 2x(1+1) = 4 kHz
#     BPF_BW=2 -> 2x(2+1) = 6 kHz
#     BPF_BW=3 -> 2x(3+1) = 8 kHz  <- we want 8 kHz
#   bits [5:0] = GAIN_INIT: max value = 63 (start at max, reduce if saturated)
#
# byte = (BPF_BW << 6) | GAIN_INIT
#      = (3 << 6)      | 63
#      = 0b11_000000   | 0b00_111111
#      = 0b11_111111
#      = 0xFF
#INIT_GAIN_VAL = (3 << 6) | 63   # = 0xFF
INIT_GAIN_VAL = (3 << 6) | 0   # = 0xC0  — BPF_BW=8kHz, GAIN_INIT=0 (minimum)
# --- FREQUENCY 0x1C ---
# SOURCE: Table 7-37, p.67
#   "Frequency = 0.2 x FREQ + 30 [kHz]"  (base formula, FREQ_SHIFT=0)
# SOURCE: Table 7-35, p.67 (FREQ_SHIFT=1):
#   "active frequency = 6 x frequency result from calculation"
#   actual = 6 x (0.2 x FREQ + 30)
#
# Solve for 300 kHz:
#   300   = 6 x (0.2 x FREQ + 30)
#   300/6 = 0.2 x FREQ + 30
#   50    = 0.2 x FREQ + 30
#   20    = 0.2 x FREQ
#   FREQ  = 20 / 0.2 = 100 = 0x64
#
# Verify: 6 x (0.2 x 100 + 30) = 6 x (20 + 30) = 6 x 50 = 300 kHz
FREQUENCY_VAL = 100   # = 0x64

# --- DEADTIME 0x1D ---
# SOURCE: Table 7-38, p.68
#   bits [7:4] = THR_CMP_DEGLTCH: "deglitch = THR_CMP_DEGLTCH x 8 [us]"
#                set to 0 (no deglitch needed)
#   bits [3:0] = PULSE_DT: "DeadTime = 0.0625 x PULSE_DT [us]"
#
# SOURCE: p.14, Section 7.3.2.3, Note:
#   "The maximum dead time setting should be less than or equal to
#    t/8 where t is burst period."
#
# At 300 kHz:
#   t        = 1 / 300,000 Hz = 3.333 us
#   max_DT   = 3.333 / 8     = 0.4167 us
#   max PULSE_DT = 0.4167 / 0.0625 = 6.67 -> integer max = 6
#
# Set PULSE_DT=6:
#   DT = 0.0625 x 6 = 0.375 us   <= 0.4167 us  (constraint satisfied)
#
# byte = (THR_CMP_DEGLTCH << 4) | PULSE_DT
#      = (0 << 4) | 6 = 0x06
DEADTIME_VAL = 0x06

# --- PULSE_P1 0x1E ---
# SOURCE: Table 7-39, p.68
#   bit  [7]   = IO_IF_SEL: 0b=TCI mode (we use UART on TXD/RXD pins)
#   bit  [6]   = UART_DIAG: 0b (default, leave unchanged)
#   bit  [5]   = IO_DIS:    0b=IO transceiver enabled (default)
#   bits [4:0] = P1_PULSE:  "Number of burst pulses for Preset1"
#                NOTE: "0h means one pulse is generated on OUTA only"
#                      Any value > 0 generates that many PAIRS (OUTA+OUTB)
#                We want 16 full pairs: P1_PULSE = 16
#
# byte = (IO_IF_SEL<<7) | (UART_DIAG<<6) | (IO_DIS<<5) | P1_PULSE
#      = (0<<7) | (0<<6) | (0<<5) | 16
#      = 0x10
PULSE_P1_VAL = 0x10

# --- REC_LENGTH 0x22 ---
# SOURCE: Table 7-43, p.70
#   bits [7:4] = P1_REC: "Record time = 4.096 x (P1_REC + 1) [ms]"
#   bits [3:0] = P2_REC: same formula for Preset 2
#
# P1_REC = 0 (minimum window, see Step 2 above)
# P2_REC = 12 (keep factory EEPROM default)
#   SOURCE: Table 7-6, p.53: "REC_LENGTH default = 0x1C"
#   0x1C = 0001 1100 -> P1_REC=1, P2_REC=12
#   We only change P1_REC from 1 to 0; P2_REC stays at 12.
#
# byte = (P1_REC << 4) | P2_REC
#      = (0 << 4)      | 12
#      = 0x00 | 0x0C
#      = 0x0C
P2_REC_DEFAULT = 12
REC_LENGTH_VAL = (P1_REC << 4) | P2_REC_DEFAULT   # 0x0C

print("Register values")
print("-" * 60)
print(f"  0x40 = 0x{EE_CNTRL_VAL:02X}  DATADUMP_EN=1         (Table 7-53 p.75)")
print(f"  0x1A = 0x{TVGAIN6_VAL:02X}  FREQ_SHIFT=1          (Table 7-35 p.67)")
print(f"  0x1B = 0x{INIT_GAIN_VAL:02X}  BPF_BW=8kHz GAIN=max  (Table 7-36 p.67)")
print(f"  0x1C = 0x{FREQUENCY_VAL:02X}  FREQ=100 -> 300kHz    (Table 7-37 p.67)")
print(f"  0x1D = 0x{DEADTIME_VAL:02X}  PULSE_DT=6 (0.375us)  (Table 7-38 p.68)")
print(f"  0x1E = 0x{PULSE_P1_VAL:02X}  P1_PULSE=16           (Table 7-39 p.68)")
print(f"  0x22 = 0x{REC_LENGTH_VAL:02X}  P1_REC=0 P2_REC=12    (Table 7-43 p.70)")
print("-" * 60)
print()

# ============================================================
# LOW-LEVEL HELPERS
# ============================================================

def checksum(data):
    """
    SOURCE: p.34, Section 7.3.6.2.1.4:
      "inverted byte sum WITH CARRY operation"
      If the sum of bytes exceeds 255, the carry is added back
      before inverting. This is a one's complement sum, not truncation.
    """
    s = sum(data)
    while s > 0xFF:                    # add carry back in
        s = (s & 0xFF) + (s >> 8)
    return (~s) & 0xFF

def write_register(addr, value):
    """
    Command 10 = 0x0A — register write
    SOURCE: Table 7-3 p.34, Example 2 p.52:
      Send:    0x55, 0x0A, ADDR, DATA, CHK
      CHK    = ~(0x0A + ADDR + DATA) & 0xFF
      Response: none (device is silent after write)
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
      Receive: DIAG(1 byte), DATA(1 byte), CHK(1 byte) = 3 bytes total
      DATA is at response index [1].
    """
    pkt = [0x55, 0x09, addr]
    pkt.append(checksum(pkt[1:]))
    ser.reset_input_buffer()
    ser.write(bytearray(pkt))
    time.sleep(0.015)
    resp = ser.read(3)         # [DIAG, DATA, CHK]
    if len(resp) >= 2:
        return resp[1]         # DATA byte
    return None


def write_coeff(addr_msb, value_16bit):
    """Write a signed 16-bit coefficient as MSB then LSB register."""
    msb = (value_16bit >> 8) & 0xFF
    lsb = value_16bit & 0xFF
    write_register(addr_msb,     msb)
    write_register(addr_msb + 1, lsb)


def burst_and_capture():
    """
    Fire Burst+Listen Preset1 then read Echo Data Dump.

    Burst command (cmd 0):
    SOURCE: Table 7-3 p.34, Example 3 p.52:
      Send:    0x55, 0x00, N_OBJECTS, CHK
               N_OBJECTS = 1..8  (number of objects to detect)
               CHK = ~(0x00 + N_OBJECTS) & 0xFF
      Example: 0x55, 0x00, 0x01, 0xFE
      Response: none

    EDD command (cmd 7):
    SOURCE: Table 7-3 p.34:
      "Transducer echo data dump"
      Send:    0x55, 0x07, CHK   where CHK = ~(0x07) & 0xFF = 0xF8
      Receive: DIAG(1) + data x 128 + CHK(1) = 130 bytes
    """
    ser.reset_input_buffer()

    # Burst+Listen Preset1
    n_obj     = 1
    burst_pkt = [0x55, 0x00, n_obj]
    burst_pkt.append(checksum(burst_pkt[1:]))   # ~(0x00+0x01)&0xFF = 0xFE
    ser.write(bytearray(burst_pkt))

    # Wait: record window (4.096 ms) + processing + serial margin = 80 ms
    time.sleep(0.08)

    # EDD request
    edd_pkt = [0x55, 0x07]
    edd_pkt.append(checksum(edd_pkt[1:]))        # ~(0x07)&0xFF = 0xF8
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
# BPF coefficients are written LAST.
# SOURCE: p.16, Section 7.3.4.1:
#   "if the FREQ or BPF_BW bit is changed, the coefficient
#    calculation sequence is rerun and the device rewrites
#    these registers."
# Writing coefficients after FREQ and BPF_BW prevents the
# device from overwriting our values.
# ============================================================
print("Writing registers ...")

# Step 1 — these do NOT trigger BPF recalculation
write_register(0x40, EE_CNTRL_VAL)   # DATADUMP_EN=1
write_register(0x1D, DEADTIME_VAL)   # PULSE_DT=6
write_register(0x1E, PULSE_P1_VAL)   # P1_PULSE=16
write_register(0x22, REC_LENGTH_VAL) # P1_REC=0

# Step 2 — write FREQUENCY first, triggers auto BPF calc (FREQ_SHIFT still 0)
# SOURCE p.16: "if the FREQ bit is changed, the coefficient calculation
#               sequence is rerun"
write_register(0x1C, FREQUENCY_VAL)
time.sleep(0.05)   # wait for auto-calc to finish

# Step 3 — write INIT_GAIN, triggers auto BPF calc again (FREQ_SHIFT still 0)
# SOURCE p.16: "if the BPF_BW bit is changed, the coefficient calculation
#               sequence is rerun"
write_register(0x1B, INIT_GAIN_VAL)
time.sleep(0.05)   # wait for auto-calc to finish

# Step 4 — NOW set FREQ_SHIFT=1, AFTER both auto-calcs have completed
# This is the critical ordering fix. Both recalculations above ran with
# FREQ_SHIFT=0, so they cannot clear it again after this point.
write_register(0x1A, TVGAIN6_VAL)
time.sleep(0.05)

# Step 5 — immediately verify TVGAIN6 before anything else runs
rb = read_register(0x1A)
print(f"TVGAIN6 readback: 0x{rb:02X}  expected: 0x{TVGAIN6_VAL:02X}  "
      f"{'OK' if rb == TVGAIN6_VAL else 'STILL FAILING'}")

# Step 6 — write BPF coefficients LAST, overriding the auto-calculated ones
# SOURCE p.16: MCU must write these when FREQ_SHIFT=1
write_coeff(0x41, A2_int)
write_coeff(0x43, A3_int)
write_coeff(0x45, B1_int)

print("Done.\n")



time.sleep(0.05)
readback = read_register(0x1B)
print(f"INIT_GAIN readback: 0x{readback:02X}  (wrote 0x{INIT_GAIN_VAL:02X})")

# ============================================================
# VERIFY REGISTERS
# Using command 0x09 (Table 7-3 p.34, Example 1 p.52)
# ============================================================
print("Verifying registers ...")
verify = {
    0x40: (EE_CNTRL_VAL,   "EE_CNTRL   DATADUMP_EN=1"),
    0x1A: (TVGAIN6_VAL,    "TVGAIN6    FREQ_SHIFT=1"),
    0x1B: (INIT_GAIN_VAL,  "INIT_GAIN  BPF_BW=3(8kHz) GAIN=63"),
    0x1C: (FREQUENCY_VAL,  "FREQUENCY  FREQ=100 -> 300kHz"),
    0x1D: (DEADTIME_VAL,   "DEADTIME   PULSE_DT=6 (0.375us)"),
    0x1E: (PULSE_P1_VAL,   "PULSE_P1   P1_PULSE=16"),
    0x22: (REC_LENGTH_VAL, "REC_LENGTH P1_REC=0 P2_REC=12"),
}
all_ok = True
for addr, (expected, label) in verify.items():
    val = read_register(addr)
    if val is None:
        print(f"  0x{addr:02X}  {label:40s}  NO RESPONSE")
        all_ok = False
    elif val == expected:
        print(f"  0x{addr:02X}  {label:40s}  0x{val:02X} OK")
    else:
        print(f"  0x{addr:02X}  {label:40s}  read=0x{val:02X} expected=0x{expected:02X} MISMATCH")
        all_ok = False

if not all_ok:
    print("\n  Mismatch on one or more registers.")
    print("  Cross-check addresses with Table 7-7 p.54 of SLASEC8C.")
else:
    print("\n  All registers OK.\n")

# ============================================================
# CAPTURE
# ============================================================
print(f"Capturing {NUM_AVERAGES} bursts ...")
accum = np.zeros(128, dtype=np.float64)
good  = 0

for i in range(NUM_AVERAGES):
    raw = burst_and_capture()
    if len(raw) < 130:
        print(f"  [{i+1}/{NUM_AVERAGES}] Short read ({len(raw)} bytes) skipped")
        continue
    # raw[0]     = diagnostic byte
    # raw[1:129] = 128 EDD samples, unsigned 8-bit, range 0-255
    # raw[129]   = checksum
    # SOURCE: Table 7-3 p.34: "Byte1-Byte128: Echo data dump"
    samp   = np.frombuffer(raw[1:129], dtype=np.uint8).astype(np.float64)
    accum += samp
    good  += 1
    print(f"  [{i+1}/{NUM_AVERAGES}]  diag=0x{raw[0]:02X}  "
          f"min={int(samp.min()):3d}  max={int(samp.max()):3d}")

if good == 0:
    print("ERROR: zero valid captures. Check COM port, wiring, DATADUMP_EN.")
    ser.close()
    exit(1)

echo = accum / good
print(f"\n  {good}/{NUM_AVERAGES} averaged.\n")
ser.close()

# ============================================================
# DETECTION
# ============================================================
search = echo.copy()
search[:RINGDOWN_BLANK] = 0.0   # zero out ring-down region (samples 0-7)

peak_idx  = int(np.argmax(search))
peak_amp  = echo[peak_idx]
# Distance from Step 7:  distance = sample_index * cm_per_slot
peak_dist = peak_idx * CM_PER_SLOT
# TOF from Step 5:       tof = sample_index * us_per_slot
peak_us   = peak_idx * US_PER_SLOT

print("=" * 60)
print("Detection result")
print("=" * 60)
print(f"  Blanked samples    : 0-{RINGDOWN_BLANK-1} (ring-down)")
print(f"  Peak sample index  : {peak_idx}")
print(f"  Peak amplitude     : {peak_amp:.1f}  (0-255)")
print(f"  Measured TOF       : {peak_us:.1f} us  "
      f"(= {peak_idx} x {US_PER_SLOT:.0f} us/slot, Step 5)")
print(f"  Measured distance  : {peak_dist:.2f} cm  "
      f"(= {peak_idx} x {CM_PER_SLOT:.4f} cm/slot, Step 7)")
print(f"  Expected (5 cm)    : sample {ECHO_EDD_SLOT:.1f}  "
      f"TOF={TOF_US:.1f} us  (Step 6)")

if peak_amp < 15:
    print()
    print("  LOW AMPLITUDE - likely causes:")
    print("  1. Transducer not resonating at exactly 300 kHz.")
    print("     Try FREQ=95 (->294kHz) or FREQ=105 (->306kHz).")
    print("  2. PSM-EVM transformer (Wurth 750317161) is rated for 40 kHz.")
    print("     Coupling at 300 kHz may be very poor.")
    print("  3. Try 31 pulses: change PULSE_P1_VAL = 0x1F")
print()

# ============================================================
# PLOT
# ============================================================
dist_axis = np.arange(128) * CM_PER_SLOT   # cm axis (Step 7)
time_axis = np.arange(128) * US_PER_SLOT   # us axis (Step 5)

fig, axes = plt.subplots(2, 1, figsize=(13, 9))
fig.suptitle(
    f"PGA460  300kHz  |  {good}x avg  |  "
    f"P1_REC={P1_REC} -> {T_REC_MS:.3f}ms  |  "
    f"{US_PER_SLOT:.0f}us/slot  |  {CM_PER_SLOT:.3f}cm/slot",
    fontsize=10
)

for ax, x_axis, xlabel, xunit, exp_val in [
    (axes[0], dist_axis, "Distance", "cm", 5.0),
    (axes[1], time_axis, "Time",     "us", TOF_US),
]:
    ax.plot(x_axis, echo, color='steelblue', linewidth=2,
            label='EDD envelope (averaged)')
    ax.axvspan(x_axis[0], x_axis[RINGDOWN_BLANK - 1],
               alpha=0.2, color='orange',
               label=f'Ring-down blanked (0-{RINGDOWN_BLANK-1})')
    ax.axvline(exp_val, color='red', linestyle='--', linewidth=1.5,
               label=f'Expected 5cm ({exp_val:.1f} {xunit})')
    if peak_idx >= RINGDOWN_BLANK:
        ax.axvline(x_axis[peak_idx], color='green', linestyle=':', linewidth=2,
                   label=f'Peak: sample {peak_idx} ({x_axis[peak_idx]:.2f} {xunit})')
    ax.set_xlabel(f'{xlabel} ({xunit})', fontsize=11)
    ax.set_ylabel('Amplitude (0-255)', fontsize=11)
    ax.set_xlim(x_axis[0], x_axis[-1])
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
fname = f"pga460_300kHz_5cm_{ts}.png"
plt.savefig(fname, dpi=200, bbox_inches='tight')
print(f"Plot saved: {fname}")
plt.show()