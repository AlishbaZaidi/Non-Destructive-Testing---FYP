"""
PGA460-Q1 | Diagnostic + Real Ringdown
=======================================
Fixes:
  1. Baud rate 9600  (SLASEC8 §7.5.1 — default UART baud)
  2. No reset_input_buffer() before reading EDD response
  3. Longer timeouts
  4. Step-by-step diagnostic prints
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import serial

# ── CONFIG — sirf port badlo ──────────────────────────────────────────────────
COM_PORT   = "COM12"
BAUD_RATE  = 9600       # <-- 9600 hai, 115200 NAHI  (Ref: SLASEC8 §7.5.1)
NUM_BURSTS = 16
# ─────────────────────────────────────────────────────────────────────────────

SYNC     = 0x55
CMD_P1BL = 0x00    # Preset-1 Burst + Listen  (Ref: SLASEC8 Table 7-19)
CMD_TEDD = 0x04    # Echo Data Dump            (Ref: SLASEC8 §7.3.4)
CMD_SRR  = 0x06    # Single Register Read      (Ref: SLASEC8 §7.5.3)


def checksum(payload):
    """XOR of all bytes after SYNC  (Ref: SLASEC8 §7.5.1)"""
    chk = 0
    for b in payload:
        chk ^= b
    return chk & 0xFF


def make_frame(cmd, data=None):
    """SYNC | CMD | DATA | CHECKSUM  (Ref: SLASEC8 §7.5.1)"""
    data = data or []
    p = [cmd] + data
    return bytes([SYNC] + p + [checksum(p)])


# ─────────────────────────────────────────────────────────────────────────────
# STEP 1 — open port
# ─────────────────────────────────────────────────────────────────────────────
print("=" * 55)
print("  PGA460-Q1 Diagnostic")
print(f"  Port: {COM_PORT}  Baud: {BAUD_RATE}")
print("=" * 55)

try:
    ser = serial.Serial(
        port      = COM_PORT,
        baudrate  = BAUD_RATE,       # 9600 — default UART  (SLASEC8 §7.5.1)
        bytesize  = serial.EIGHTBITS,
        parity    = serial.PARITY_NONE,
        stopbits  = serial.STOPBITS_ONE,
        timeout   = 2.0,
    )
    print(f"[OK] Port opened: {COM_PORT}")
except Exception as e:
    raise SystemExit(f"[ERR] Cannot open port: {e}")

time.sleep(0.5)                      # board settle hone do
ser.reset_input_buffer()
ser.reset_output_buffer()

# ─────────────────────────────────────────────────────────────────────────────
# STEP 2 — register read test (address 0x00 = DEV_STAT0)
#   Agar yeh kaam kare toh communication OK hai
#   Ref: SLASEC8 §7.5.3 — Single Register Read
# ─────────────────────────────────────────────────────────────────────────────
print("\n[TEST] Sending Single Register Read (addr 0x00) ...")
reg_frame = make_frame(CMD_SRR, [0x00])
print(f"       TX bytes: {[hex(b) for b in reg_frame]}")
ser.write(reg_frame)
time.sleep(0.1)
resp = ser.read(3)    # 1 data byte + 1 checksum + possible echo
print(f"       RX bytes: {[hex(b) for b in resp]} (got {len(resp)} bytes)")

if len(resp) == 0:
    print("[WARN] No response to register read.")
    print("       Possible reasons:")
    print("         - TX and RX wires swapped (swap them and retry)")
    print("         - Board not powered (check 6V supply)")
    print("         - Wrong COM port (check Device Manager)")
    print("       Continuing anyway to try burst+EDD ...\n")
else:
    print(f"[OK]  Got response — communication working!\n")

# ─────────────────────────────────────────────────────────────────────────────
# STEP 3 — burst + listen + EDD
# ─────────────────────────────────────────────────────────────────────────────
print(f"[CAPTURE] Starting {NUM_BURSTS} burst-listen-EDD cycles ...")
captures = []

for i in range(NUM_BURSTS):

    # --- Send Burst + Listen ---
    ser.reset_input_buffer()
    burst_frame = make_frame(CMD_P1BL)
    ser.write(burst_frame)

    # Wait for burst (8 cycles × 25µs = 200µs)
    # + listen window (8192µs default RecLength)
    # + margin
    # Ref: SLASEC8 §7.3
    time.sleep(0.12)

    # --- Send EDD request ---
    # NOTE: do NOT reset_input_buffer() here — we are reading new data
    edd_frame = make_frame(CMD_TEDD)
    ser.write(edd_frame)
    time.sleep(0.10)    # give IC time to send all 129 bytes at 9600 baud
                        # 129 bytes × 10 bits / 9600 = ~134 ms, so 100ms is ok
                        # at 9600 baud each byte ~1.04 ms

    # --- Read 128 data + 1 checksum = 129 bytes ---
    raw = ser.read(129)

    if len(raw) < 128:
        print(f"  [{i+1:02d}/{NUM_BURSTS}] FAILED — got {len(raw)}/129 bytes")
        if i == 0:
            print("         [HINT] If consistently 0 bytes: check TX/RX wiring")
            print("         [HINT] If partial bytes: increase sleep time above")
        continue

    # Verify checksum  (Ref: SLASEC8 §7.5.1)
    rx_chk   = raw[128]
    calc_chk = checksum(list(raw[:128]))
    chk_ok   = "CHK-OK" if rx_chk == calc_chk else f"CHK-BAD(got {rx_chk:#04x} exp {calc_chk:#04x})"

    # Convert 0-255 unsigned → signed -128..127
    # So ringing oscillates around zero in the plot
    data = [b if b < 128 else b - 256 for b in raw[:128]]
    peak = max(abs(x) for x in data)

    captures.append(data)
    print(f"  [{i+1:02d}/{NUM_BURSTS}] OK — peak={peak:3d}  {chk_ok}")

ser.close()
print(f"\n[DONE] Port closed. Successful captures: {len(captures)}/{NUM_BURSTS}")

if not captures:
    print("\n[ERR] Zero captures succeeded.")
    print("  Check:")
    print("  1. Baud rate — should be 9600 (not 115200)")
    print("  2. TX/RX wiring — board TX -> adapter RX, board RX -> adapter TX")
    print("  3. Board powered at 6V")
    print("  4. COM port correct")
    raise SystemExit()

# ─────────────────────────────────────────────────────────────────────────────
# STEP 4 — plot
# ─────────────────────────────────────────────────────────────────────────────
arr       = np.array(captures, dtype=float)
avg       = arr.mean(axis=0)
noise_std = float(np.std(avg[-20:]))    # noise from quiet far-field region

sample_idx = np.arange(128)
BLANK_END  = 64                         # hardware blanking = 4096 µs

fig, ax = plt.subplots(figsize=(13, 5))
fig.patch.set_facecolor('white')
ax.set_facecolor('white')

# Blanking zone
ax.axvspan(0, BLANK_END, color='#ffcccc', alpha=0.55,
           label='HW blanking (4096 µs)')

# Noise floor
nf = noise_std * 10
ax.axhspan(-nf, nf, color='#ffd580', alpha=0.4,
           label=f'Noise floor ±{nf:.0f}')

# Individual captures (faint grey)
for cap in captures:
    ax.plot(sample_idx, cap, color='#999999', linewidth=0.4, alpha=0.2)

# Averaged signal on top
ax.plot(sample_idx, avg, color='#1f6fb2', linewidth=1.6,
        marker='o', markersize=2.5,
        label=f'EDD average ({len(captures)} bursts)',
        zorder=5)

# Blanking end marker
ax.axvline(BLANK_END, color='#cc0000', linestyle='--', linewidth=1.2, zorder=6)

ymax = max(float(np.max(np.abs(avg))) * 1.2, 20)
ax.text(BLANK_END + 0.8, ymax * 0.88,
        f'blanking ends\nsample {BLANK_END}',
        color='#cc0000', fontsize=8.5, va='top')

# Top time axis (µs)
ax2 = ax.twiny()
ax2.set_xlim(ax.get_xlim())
tick_s = list(range(0, 128, 8))
ax2.set_xticks(tick_s)
ax2.set_xticklabels([str(s * 64) for s in tick_s], fontsize=7.5)
ax2.set_xlabel("Time (µs)", fontsize=9)

ax.set_xlabel("Sample index", fontsize=10)
ax.set_ylabel("Amplitude", fontsize=10)
ax.set_xlim(0, 127)
ax.set_ylim(-ymax, ymax)
ax.set_title(f"40 kHz ring-down in air (no target)  |  64 µs/sample  |  "
             f"averaged over {len(captures)} bursts  |  REAL DATA", fontsize=10)
ax.legend(loc='upper right', fontsize=8.5, framealpha=0.9)
ax.grid(True, alpha=0.3, linewidth=0.5)

plt.tight_layout()
plt.savefig("ringdown_real.png", dpi=150, bbox_inches='tight')
print("[SAVED] ringdown_real.png")
plt.show()