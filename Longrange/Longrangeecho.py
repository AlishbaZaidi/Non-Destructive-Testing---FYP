"""
PGA460-Q1 — Auto-Calibration + 2m Distance Measurement
=========================================================
DIAGNOSIS FROM YOUR DATA:
  - TOF=1500µs→0.26m with peak=255 = ringdown saturation
  - Noise floor=158/255 = gain way too high, ADC clipping
  - The transformer-boosted signal overwhelms the AFE at all gain settings
  - TVG ramp is amplifying the already-saturated ringdown even more

ROOT CAUSE:
  Your transformer boosts 12V → ~50V at transducer.
  During ringdown, this strong signal enters AFE.
  Even at minimum TVG gain, the LNA (Low Noise Amplifier) is fixed-gain.
  The ADC clips at 255 for the first ~4ms after every burst.
  
  This is the HARDWARE PHYSICAL LIMIT of this transformer-driven design
  for short-range detection. The eval board was characterized at 4m-6m
  where the ringdown has died out and the signal is weak enough.

CORRECT APPROACH:
  1. Set INIT_GAIN to MINIMUM (0 = 0.5dB + AFE_GAIN_RNG offset)
  2. Disable TVG entirely (all zeros = flat minimum gain)
  3. Set AFE_GAIN_RNG to minimum range (bits = 00)
  4. Use long record time so 2m echo appears far from ringdown
  5. The echo at 2m arrives at 11.6ms — well past the 4ms ringdown

USAGE:  python auto_calibrate.py
"""

import serial
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# ─── CONFIG ──────────────────────────────────────────────────────────────────
COM_PORT  = 'COM12'
BAUD_RATE = 115200
UART_ADDR = 0
TEMP_C    = 25.0
TARGET_M  = 2.0
N_LOOPS   = 30
# ─────────────────────────────────────────────────────────────────────────────

V_SOUND = 331.0 + 0.6 * TEMP_C   # 346 m/s

# Record: 4m max coverage (covers 2m target with margin)
P1_REC  = 5    # 4.096 × 6 = 24.576ms → 4.26m max
REC_MS  = 4.096 * (P1_REC + 1)
SAMP_MS = REC_MS / 128.0
SAMP_US = SAMP_MS * 1000

# Echo at 2m arrives at:
ECHO_MS = TARGET_M / V_SOUND * 2 * 1000   # 11.56ms

# Ringdown lasts ~4ms on this board (from CMD8 measurements)
RING_MS = 4.0
RING_M  = V_SOUND * RING_MS * 1e-3 / 2.0  # 0.69m

print(f"Target={TARGET_M}m  Echo expected at {ECHO_MS:.1f}ms")
print(f"Record={REC_MS:.1f}ms  Ringdown={RING_MS}ms")

def chk(d):
    t = 0
    for b in d:
        t += b
        if t > 0xFF: t = (t & 0xFF) + 1
    return (~t) & 0xFF

def build(idx, data=b''):
    cb = ((UART_ADDR & 0x07) << 5) | (idx & 0x1F)
    pl = bytes([cb]) + data
    return bytes([0x55]) + pl + bytes([chk(pl)])

def rx(ser, n, ms=800):
    t = time.time() + ms/1000
    b = b''
    while len(b) < n and time.time() < t:
        c = ser.read(n - len(b))
        if c: b += c
    return b

def rw(ser, addr, val):
    ser.reset_input_buffer()
    ser.write(build(10, bytes([addr, val])))
    time.sleep(0.060)

def burst(ser):
    ser.reset_input_buffer()
    ser.write(build(0, bytes([1])))
    time.sleep(REC_MS / 1000.0 + 0.100)
    ser.reset_input_buffer()

def get_dump(ser):
    ser.write(build(7))
    time.sleep(0.300)
    r = rx(ser, 130, ms=1000)
    return list(r[1:129]) if len(r) == 130 else None

def get_tof(ser):
    ser.write(build(5))
    time.sleep(0.080)
    r = rx(ser, 6)
    if len(r) == 6:
        tof = (r[1] << 8) | r[2]
        return tof, r[3], r[4]
    return None

def write_thresholds(ser, t1_us, level):
    """
    Write thresholds.
    t1_us: start time in µs (must be > ringdown)
    level: detection level 0-255
    """
    # T1 lookup: find closest value
    t1_table = {100:0, 200:1, 300:2, 400:3, 600:4, 800:5, 1000:6,
                1200:7, 1400:8, 2000:9, 2400:10, 3200:11, 4000:12,
                5200:13, 6400:14, 8000:15}
    # Find closest T1
    best = min(t1_table.keys(), key=lambda x: abs(x - t1_us))
    t1_nibble = t1_table[best]
    lv = min(15, max(0, level >> 4))   # 4-bit level for packed fields

    thr = [
        (t1_nibble << 4) | 0x3,   # T1=t1_us T2=400µs
        0x33, 0x33, 0x33, 0x33, 0x33,
        (lv<<4)|lv, (lv<<4)|lv, (lv<<4)|lv, (lv<<4)|lv,
        level, level, level, level,
        0x00, 0x00,
        (t1_nibble << 4) | 0x3,
        0x33, 0x33, 0x33, 0x33, 0x33,
        (lv<<4)|lv, (lv<<4)|lv, (lv<<4)|lv, (lv<<4)|lv,
        level, level, level, level,
        0x00, 0x00,
    ]
    ser.reset_input_buffer()
    ser.write(build(16, bytes(thr)))
    time.sleep(0.100)
    print(f"    Thresholds: T1={best}µs ({best/1000:.1f}ms) level={level}")

def tof_m(us): return V_SOUND * us * 1e-6 / 2.0


def calibrate_gain(ser):
    """
    Find minimum gain that still detects the 2m echo.
    Starts from minimum gain and increases until echo seen.
    Returns best gain register values.
    """
    print("\n  AUTO-CALIBRATING GAIN...")
    print(f"  Looking for echo at {TARGET_M}m ({ECHO_MS:.1f}ms)")
    print(f"  Echo sample index: ~{int(ECHO_MS/SAMP_MS)}")

    # Gain configurations from minimum to maximum
    # Format: (INIT_GAIN, TVG_G values description, TVG register values)
    gain_configs = [
        # INIT_GAIN: bits[5:0] = gain code, bits[7:6] = BPF_BW
        # Gain = 0.5*(GAIN_INIT+1) + AFE_GAIN_RNG_offset
        # With AFE_GAIN_RNG=00 (32dB offset): actual = 32 + 0.5*(code+1)
        # With code=0: 32.5dB init gain
        {
            'name': 'Minimum gain',
            0x26: 0x00,   # AFE_GAIN_RNG=00 (32dB base) — MINIMUM range
            0x1B: 0x00,   # INIT_GAIN=0 (32.5dB), BPF=2kHz
            # TVG: ALL ZEROS = TVG disabled, use fixed INIT_GAIN only
            0x14: 0x00, 0x15: 0x00, 0x16: 0x00,
            0x17: 0x00, 0x18: 0x00, 0x19: 0x00, 0x1A: 0x00,
        },
        {
            'name': 'Low gain',
            0x26: 0x00,
            0x1B: 0x10,   # INIT_GAIN=16 (40.5dB)
            0x14: 0xCC, 0x15: 0xCC, 0x16: 0xDD,
            0x17: 0x05, 0x18: 0x16, 0x19: 0x28, 0x1A: 0x28,
        },
        {
            'name': 'Medium gain',
            0x26: 0x08,   # AFE_GAIN_RNG=10 (52dB base)
            0x1B: 0x00,   # INIT_GAIN=0
            0x14: 0xCC, 0x15: 0xDD, 0x16: 0xEE,
            0x17: 0x05, 0x18: 0x16, 0x19: 0x27, 0x1A: 0x24,
        },
        {
            'name': 'Medium-high gain',
            0x26: 0x08,
            0x1B: 0x10,
            0x14: 0xCC, 0x15: 0xDD, 0x16: 0xEE,
            0x17: 0x14, 0x18: 0x3A, 0x19: 0x5F, 0x1A: 0x5C,
        },
        {
            'name': 'High gain',
            0x26: 0x10,   # AFE_GAIN_RNG=01 (62dB base)
            0x1B: 0x10,
            0x14: 0xCC, 0x15: 0xDD, 0x16: 0xEE,
            0x17: 0x14, 0x18: 0x3A, 0x19: 0x5F, 0x1A: 0x5C,
        },
    ]

    best_config = None
    best_peak   = 0
    echo_sample = int(ECHO_MS / SAMP_MS)

    for cfg in gain_configs:
        name = cfg['name']
        # Apply gain settings
        for addr, val in cfg.items():
            if isinstance(addr, int):
                rw(ser, addr, val)

        # Get echo dump
        burst(ser)
        dump = get_dump(ser)
        if not dump:
            print(f"    {name}: no dump")
            continue

        arr = np.array(dump)
        # Look at the window around expected echo (±3 samples)
        lo = max(0, echo_sample - 3)
        hi = min(127, echo_sample + 3)
        echo_region = arr[lo:hi+1]
        ring_region = arr[:int(RING_MS/SAMP_MS)+2]

        echo_peak  = float(max(echo_region))
        ring_max   = float(max(ring_region))
        late_noise = float(np.median(arr[100:]))

        print(f"    {name}: "
              f"echo_window={echo_peak:.0f}  "
              f"ringdown_max={ring_max:.0f}  "
              f"noise={late_noise:.0f}", end='')

        if echo_peak > late_noise * 2 and echo_peak > 30:
            print(f"  ← ECHO DETECTED!")
            if echo_peak > best_peak:
                best_peak   = echo_peak
                best_config = cfg
        else:
            print()

    return best_config, gain_configs[-1]  # return best or highest if nothing found


def main():
    print("=" * 66)
    print("  PGA460-Q1 — Auto-Calibration + 2m Measurement")
    print("=" * 66)
    print(f"  Target      : {TARGET_M}m")
    print(f"  Echo at     : {ECHO_MS:.1f}ms (sample ~{int(ECHO_MS/SAMP_MS)})")
    print(f"  Record      : {REC_MS:.1f}ms")
    print(f"  Ringdown    : ~{RING_MS}ms = {RING_M:.2f}m blind zone")
    print(f"\n  Wooden block at {TARGET_M}m, perpendicular to transducer")
    input("  Press ENTER when ready...")

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE,
                            bytesize=8, parity='N', stopbits=2, timeout=0.5)
    except Exception as e:
        print(f"[ERROR] {e}"); return
    time.sleep(0.3)

    # Core registers
    rw(ser, 0x1C, 0x32)
    rw(ser, 0x1E, 0x48)
    rw(ser, 0x22, (P1_REC << 4) | 0x0)
    rw(ser, 0x1D, 0x00)
    rw(ser, 0x20, 0x3F)

    # Write thresholds: T1=5200µs (just past 4ms ringdown), level=0x20 (low)
    write_thresholds(ser, t1_us=5200, level=0x20)

    # Auto-calibrate gain
    best_cfg, fallback_cfg = calibrate_gain(ser)

    use_cfg = best_cfg if best_cfg else fallback_cfg
    print(f"\n  Using: {use_cfg.get('name', 'fallback')}")
    for addr, val in use_cfg.items():
        if isinstance(addr, int):
            rw(ser, addr, val)

    # ── Measurement loop ──────────────────────────────────────────────────────
    print(f"\n  Measuring {N_LOOPS} loops at {TARGET_M}m...")
    print(f"  {'Loop':>5}  {'TOF':>10}  {'Dist':>8}  {'Peak':>6}  Status")
    print(f"  {'-'*5}  {'-'*10}  {'-'*8}  {'-'*6}  ------")

    dumps  = []
    dists  = []
    peaks  = []
    lnums  = []
    sent_us = (REC_MS - SAMP_MS) * 1000

    for lp in range(1, N_LOOPS + 1):
        burst(ser)
        dump = get_dump(ser)

        burst(ser)
        tr = get_tof(ser)

        dist = pk = None
        st = "no echo"

        if tr:
            tof_us, w, pk = tr
            pk = int(pk)
            valid_tof = (RING_MS * 1000 < tof_us < sent_us)
            if valid_tof and pk > 15:
                dist = tof_m(tof_us)
                dists.append(dist)
                peaks.append(pk)
                lnums.append(lp)
                st = f"✓ {dist:.3f}m"
            elif tof_us <= RING_MS * 1000:
                st = f"ringdown ({tof_us}µs)"
            elif tof_us >= sent_us:
                st = "sentinel"
            else:
                st = f"low peak ({pk})"
            tof_str = str(tof_us)
        else:
            tof_str = "---"

        ds = f"{dist:.3f}" if dist else "---"
        ps = str(pk) if pk is not None else "---"
        print(f"  {lp:>5}  {tof_str:>10}  {ds:>8}  {ps:>6}  {st}")

        if dump:
            dumps.append(dump)
        time.sleep(0.05)

    ser.close()

    print(f"\n{'='*66}")
    if dists:
        print(f"  DISTANCE: {np.mean(dists):.4f}m ± {np.std(dists)*100:.2f}cm")
        print(f"  TARGET  : {TARGET_M}m")
        print(f"  ERROR   : {(np.mean(dists)-TARGET_M)*100:+.1f}cm")
        print(f"  VALID   : {len(dists)}/{N_LOOPS}")
    else:
        print(f"  No valid echo at {TARGET_M}m")
        print(f"  The echo at 2m is too weak for this board's AFE")
        print(f"  See plot for waveform — check if any peak visible at {ECHO_MS:.0f}ms")

    if not dumps:
        return

    avg_d = np.mean(dumps, axis=0)
    t_ax  = np.linspace(0, REC_MS, 128)

    # ── Plot ──────────────────────────────────────────────────────────────────
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(15, 9))
    fig.suptitle(
        f'PGA460-Q1 — Auto-Calibrated Measurement\n'
        f'UTR-1440K-TT-R 40kHz | Target={TARGET_M}m | '
        f'Record={REC_MS:.0f}ms | Gain={use_cfg.get("name","")}',
        fontsize=12, fontweight='bold', color='white', y=0.98
    )
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.35)
    aw = fig.add_subplot(gs[0, :])
    ad = fig.add_subplot(gs[1, 0])
    ap = fig.add_subplot(gs[1, 1])

    FC = '#1a1a2e'
    def sty(ax):
        ax.set_facecolor(FC)
        ax.tick_params(colors='white', labelsize=9)
        ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#444'); ax.spines['bottom'].set_color('#444')

    sty(aw)
    for d in dumps:
        aw.plot(t_ax, d, color='#a29bfe', alpha=0.06, lw=0.5)
    aw.plot(t_ax, avg_d, color='#a29bfe', lw=2.5,
            label=f'Avg ({len(dumps)} bursts)')

    # Ringdown shade
    aw.axvspan(0, RING_MS, alpha=0.20, color='#ff6b6b')
    aw.axvline(RING_MS, color='#ff6b6b', ls='--', lw=2,
               label=f'Ringdown {RING_MS:.0f}ms = {RING_M:.2f}m')
    aw.text(RING_MS/2, 255, f'BLIND\n{RING_M:.2f}m',
            ha='center', va='top', color='#ff6b6b', fontsize=8)

    # Expected echo
    aw.axvline(ECHO_MS, color='#55efc4', ls=':', lw=2,
               label=f'Expected echo {TARGET_M}m at {ECHO_MS:.1f}ms')

    # Detected distance
    if dists:
        det_ms = np.mean(dists)/V_SOUND*2*1000
        aw.axvline(det_ms, color='#00d4ff', ls='--', lw=2,
                   label=f'Detected={np.mean(dists):.3f}m')

    # Noise floor
    nf = float(np.median(avg_d[100:]))
    aw.axhline(nf, color='#636e72', ls=':', lw=1, label=f'Noise={nf:.0f}')

    # Dual x-axis
    ax2 = aw.twiny(); ax2.set_xlim(aw.get_xlim())
    dm = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0]
    tm = [d/V_SOUND*2*1000 for d in dm]
    vt = [(t, d) for t, d in zip(tm, dm) if t <= REC_MS]
    ax2.set_xticks([t for t, d in vt])
    ax2.set_xticklabels([f'{d}m' for t, d in vt], color='#aaa', fontsize=8)
    ax2.tick_params(colors='#aaa'); ax2.spines['top'].set_color('#333')
    for sp in ['right','left','bottom']: ax2.spines[sp].set_visible(False)

    aw.set_xlabel('Time (ms)', color='white', fontsize=10)
    aw.set_ylabel('DSP Amplitude (0-255)', color='white', fontsize=10)
    aw.set_title('Echo Data Dump — Auto-Calibrated Gain', color='white')
    aw.set_xlim(0, REC_MS); aw.set_ylim(0, 270)
    aw.legend(fontsize=8, facecolor=FC, labelcolor='white', loc='upper right')

    sty(ad)
    if dists:
        ad.plot(lnums, dists, 'o', color='#00d4ff', ms=5)
        ad.axhline(np.mean(dists), color='#55efc4', ls='--', lw=2,
                   label=f'Mean={np.mean(dists):.4f}m')
        ad.axhline(TARGET_M, color='#ff9f43', ls=':', lw=1.5,
                   label=f'Target={TARGET_M}m')
        err = np.mean(dists)-TARGET_M
        ad.set_title(f'{np.mean(dists):.4f}m  Err={err*100:+.1f}cm  '
                     f'Std=±{np.std(dists)*100:.2f}cm', color='white', fontsize=9)
        ad.legend(fontsize=8, facecolor=FC, labelcolor='white')
    else:
        ad.text(0.5, 0.5, f'No echo at {TARGET_M}m\nSee waveform for details',
                ha='center', va='center', color='#ff9f43', fontsize=10,
                transform=ad.transAxes)
        ad.set_title('Distance vs Loop', color='white')
    ad.set_xlabel('Loop #', color='white'); ad.set_ylabel('Distance (m)', color='white')

    sty(ap)
    if peaks:
        ap.bar(lnums, peaks, color='#00d4ff', alpha=0.8, width=0.6)
        ap.set_title(f'Peak={np.mean(peaks):.0f}/255  '
                     f'SNR={np.mean(peaks)/max(nf,1):.1f}:1',
                     color='white', fontsize=9)
    else:
        ap.text(0.5, 0.5, 'No peaks', ha='center', va='center',
                color='#ff9f43', fontsize=11, transform=ap.transAxes)
        ap.set_title('Echo Peak Amplitude', color='white')
    ap.set_xlabel('Loop #', color='white')
    ap.set_ylabel('Peak (0-255)', color='white'); ap.set_ylim(0, 270)

    footer = (
        f"Target={TARGET_M}m  |  Echo@{ECHO_MS:.1f}ms  |  "
        + (f"Detected={np.mean(dists):.4f}m  Err={( np.mean(dists)-TARGET_M)*100:+.1f}cm"
           if dists else "No echo detected")
    )
    fig.text(0.5, 0.005, footer, ha='center', fontsize=9,
             color='#55efc4' if dists else '#ff9f43', fontweight='bold',
             bbox=dict(boxstyle='round', facecolor='#0d0d1a', alpha=0.8))

    plt.savefig('auto_calibrate_result.png', dpi=150,
                bbox_inches='tight', facecolor='#0d0d1a')
    print(f"  Saved: auto_calibrate_result.png")
    print("\nClose window to exit.")
    plt.show()


if __name__ == '__main__':
    main()