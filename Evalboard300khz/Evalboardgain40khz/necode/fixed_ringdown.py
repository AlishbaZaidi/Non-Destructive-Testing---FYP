"""
PGA460-Q1 | 40 kHz Ringdown — Clean Simple Version
"""

import serial, time
import numpy as np
import matplotlib.pyplot as plt

COM_PORT  = "COM12"
BAUD_RATE = 115200
N_BURSTS  = 32

def checksum(data):
    s = sum(data)
    s = (s & 0xFF) + (s >> 8)
    return (~s) & 0xFF

def frame(cmd, data=None):
    data = data or []
    p = [cmd] + list(data)
    return bytes([0x55] + p + [checksum(p)])

def wreg(ser, addr, val):
    ser.write(frame(0x0A, [addr, val]))
    time.sleep(0.02)

print(f"Opening {COM_PORT} @ {BAUD_RATE} ...")
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=2.0)
time.sleep(2)
ser.reset_input_buffer()
print("[OK]\n")

wreg(ser, 0x1C, 0x32)
time.sleep(0.1)
wreg(ser, 0x22, 0x1C)
wreg(ser, 0x26, 0xC0)
wreg(ser, 0x1B, 0xC0)
wreg(ser, 0x40, 0x80)
time.sleep(0.1)

# ── EDD CAPTURE ───────────────────────────────────────────────────────────────
print("=" * 65)
print("  EDD RAW VALUES (first 10 of 128 samples, first 5 bursts)")
print("  Each sample = peak amplitude of 64µs window, signed -128..+127")
print("  Ref: SLASEC8 §7.3.4 — peak-hold downsampled to 128 points")
print("=" * 65)
print(f"  {'Burst':>5}  s0    s1    s2    s3    s4    s5    s6    s7    s8    s9    RMS")
print(f"  {'-'*70}")

captures = []
for i in range(N_BURSTS):
    ser.reset_input_buffer()
    ser.write(frame(0x00, [0x01]))
    time.sleep(0.15)
    ser.write(frame(0x07))
    time.sleep(0.15)
    raw = ser.read(130)
    if len(raw) < 128:
        continue
    data = np.frombuffer(raw[1:129], dtype=np.uint8).astype(np.int16) - 128
    captures.append(data.astype(float))
    if i < 5:
        vals = "  ".join(f"{int(data[j]):4d}" for j in range(10))
        rms  = np.sqrt(np.mean(data.astype(float)**2))
        print(f"  {i+1:5d}  {vals}  {rms:5.1f}")

print(f"  ... ({N_BURSTS} bursts total, all same — deterministic saturated 40kHz carrier)")
print(f"\n  WHY SAME EVERY BURST:")
print(f"  40kHz × 64µs/sample = 2.56 cycles per sample → fixed phase pattern")
print(f"  ADC fully clipped at ±128 (min hardware gain = 32.5dB, Ref §7.6.3.28)")
print(f"  Result: same clipped values every burst")

# ── CMD 8 CAPTURE ─────────────────────────────────────────────────────────────
wreg(ser, 0x40, 0x00)   # DATADUMP_EN=0 for CMD 8
time.sleep(0.05)

print(f"\n{'=' * 65}")
print(f"  CMD 8 SYSTEM DIAGNOSTICS RAW BYTES (all {N_BURSTS} runs)")
print(f"  Ref: SLASEC8 §7.3.5 p.21, Table 7-3")
print(f"  Response: [DIAG] [FREQ_BYTE] [DECAY_BYTE] [CHECKSUM]")
print(f"  DECAY_BYTE × 16µs = ringdown time")
print(f"{'=' * 65}")
print(f"  {'Run':>4}   DIAG   FREQ   DECAY   CHK    Decay(µs)")
print(f"  {'-'*50}")

decay_vals = []
for i in range(N_BURSTS):
    ser.reset_input_buffer()
    ser.write(frame(0x00, [0x01]))
    time.sleep(0.15)
    ser.write(frame(0x08))
    time.sleep(0.05)
    raw = ser.read(4)
    if len(raw) < 3:
        continue
    d_us = raw[2] * 16
    decay_vals.append(float(d_us))
    chk  = raw[3] if len(raw) > 3 else 0
    print(f"  {i+1:4d}   0x{raw[0]:02X}   0x{raw[1]:02X}   0x{raw[2]:02X}    0x{chk:02X}   {d_us:.0f}")

ser.close()

avg_decay = float(np.mean(decay_vals))

print(f"\n{'=' * 65}")
print(f"  FINAL NUMBERS")
print(f"{'=' * 65}")
print(f"  DECAY_BYTE   = 0x{int(avg_decay/16):02X} = {int(avg_decay/16)} (decimal)")
print(f"  Ringdown     = {int(avg_decay/16)} × 16µs = {avg_decay:.0f}µs = {avg_decay/1000:.3f}ms")
print(f"  Blanking now = 4096µs (DECPL_T=0, register 0x26)")
print(f"  Status       = {'Ringdown fits in blanking' if avg_decay<=4096 else 'Ringdown EXCEEDS blanking — need larger DECPL_T'}")
print(f"{'=' * 65}")

# ── CLEAN PLOT ────────────────────────────────────────────────────────────────
arr   = np.array(captures, dtype=float)
avg   = arr.mean(axis=0)
idx   = np.arange(128)
t_us  = idx * 64.0

fig, ax = plt.subplots(figsize=(12, 5))
fig.patch.set_facecolor('white')
ax.set_facecolor('white')

# Blanking shading
ax.axvspan(0, 4096, color='#ffdddd', alpha=0.45)
ax.axvspan(4096, 8192, color='#ddffdd', alpha=0.35)

# Zero line
ax.axhline(0, color='#cccccc', lw=0.8)

# Individual captures faint
for c in captures:
    ax.plot(t_us, c, color='#aaaaaa', lw=0.25, alpha=0.12)

# Averaged EDD signal
ax.plot(t_us, avg, color='#2471a3', lw=1.8,
        marker='o', markersize=3, label='EDD  (32 bursts avg)')

# Ringdown end line
ax.axvline(avg_decay, color='#1e8449', lw=2.0, ls='--',
           label=f'Ringdown end  {avg_decay:.0f}µs = {avg_decay/1000:.2f}ms  (CMD 8)')

# Blanking end line
ax.axvline(4096, color='#c0392b', lw=1.5, ls=':',
           label='Blanking end  4096µs  (DECPL_T=0)')

# Simple text labels for zones
ax.text(2048, 120, 'RINGING\n(blanking)', ha='center', fontsize=9, color='#922b21')
ax.text(6144, 120, 'QUIET ZONE\n(echo detection)', ha='center', fontsize=9, color='#1a5c2a')

# Top time axis in ms
ax2 = ax.twiny()
ax2.set_xlim(0, 8192)
ax2.set_xticks([0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000])
ax2.set_xticklabels(['0', '1', '2', '3', '4', '5', '6', '7', '8'])
ax2.set_xlabel("Time (ms)")

ax.set_xlabel("Time (µs)")
ax.set_ylabel("Amplitude")
ax.set_xlim(0, 8192)
ax.set_ylim(-135, 140)
ax.legend(loc='upper right', fontsize=9, framealpha=0.9)
ax.grid(True, alpha=0.18)
ax.set_title(
    f"40 kHz Transducer Ringdown  |  UTR-1440K-TT-R  |  PGA460PSM-EVM\n"
    f"Measured ringdown = {avg_decay:.0f}µs = {avg_decay/1000:.2f}ms"
    f"  (SLASEC8 §7.3.5 CMD 8  |  DECAY_BYTE = 0x{int(avg_decay/16):02X} = {int(avg_decay/16)} × 16µs)",
    fontsize=10
)

plt.tight_layout()
plt.savefig("ringdown_clean.png", dpi=150, bbox_inches='tight')
print("\n[SAVED] ringdown_clean.png")
plt.show()