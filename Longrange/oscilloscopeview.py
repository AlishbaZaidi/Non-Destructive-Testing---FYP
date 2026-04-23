"""
PGA460-Q1 — Oscilloscope-Style Transducer Waveform Plot
=========================================================
Recreates the oscilloscope view from real CMD7 echo dump data.

The oscilloscope image shows:
  LEFT  : Many burst pulses → long ringdown decay envelope
  RIGHT : Fewer burst pulses → shorter ringdown, cleaner signal

Our CMD7 data is the DSP output (rectified + peak-held envelope),
which corresponds to the ENVELOPE of the oscilloscope signal.
We reconstruct the oscilloscope-style view by:
  1. Modulating the envelope with a 40kHz sine carrier
  2. Plotting the raw envelope alongside for comparison
  3. Showing burst zone, ringdown zone, echo zone clearly

USAGE:
  python oscilloscope_view.py
  Place object at 1-5m for best results.
"""

import serial
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# ─── CONFIG ──────────────────────────────────────────────────────────────────
COM_PORT     = 'COM12'
BAUD_RATE    = 115200
UART_ADDR    = 0
TEMP_C       = 25.0
N_CAPTURES   = 5      # number of bursts to capture and show
# ─────────────────────────────────────────────────────────────────────────────

V_SOUND   = 331.0 + 0.6 * TEMP_C
FREQ_HZ   = 40000.0    # 40kHz transducer

# Record length settings — must match what is in the IC right now
# P1_REC=8 → 36.864ms (set by long_range_echo.py)
# P1_REC=0 → 4.096ms  (factory default)
# Script will read this from register and auto-detect
RECORD_MS_DEFAULT = 36.864   # will be overridden by register read

# ── UART helpers ──────────────────────────────────────────────────────────────
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

def rx(ser, n, ms=600):
    t = time.time() + ms/1000
    b = b''
    while len(b) < n and time.time() < t:
        c = ser.read(n - len(b))
        if c: b += c
    return b

def reg_read(ser, addr):
    ser.reset_input_buffer()
    ser.write(build(9, bytes([addr])))
    time.sleep(0.060)
    r = rx(ser, 3)
    return r[1] if len(r) >= 3 else None

def burst_listen(ser, wait_ms):
    ser.reset_input_buffer()
    ser.write(build(0, bytes([1])))
    time.sleep(wait_ms / 1000.0 + 0.050)
    ser.reset_input_buffer()

def get_echo_dump(ser):
    ser.write(build(7))
    time.sleep(0.200)
    r = rx(ser, 130, ms=800)
    return list(r[1:129]) if len(r) == 130 else None

def get_tof(ser):
    ser.write(build(5))
    time.sleep(0.060)
    r = rx(ser, 6)
    if len(r) == 6:
        tof = (r[1] << 8) | r[2]
        return tof, r[3], r[4]
    return None

def get_decay(ser):
    ser.write(build(8))
    time.sleep(0.060)
    r = rx(ser, 4)
    return r[2] if len(r) == 4 else None

def reconstruct_oscilloscope(envelope_samples, record_ms, freq_hz=40000.0,
                               pulse_count=10):
    """
    Take the DSP envelope (128 samples) and reconstruct
    an oscilloscope-style waveform by modulating with 40kHz carrier.

    The burst region is simulated as full-amplitude sine.
    The ringdown + echo regions use the envelope as amplitude modulation.

    Returns:
      t_us   : time axis in µs
      signal : reconstructed voltage waveform (normalized -1 to 1)
      envelope_interp : interpolated envelope
    """
    n_samples = 128
    sample_us = record_ms * 1000 / n_samples

    # High-resolution time axis (1000 points per sample)
    pts_per_sample = 200
    total_pts = n_samples * pts_per_sample
    t_us = np.linspace(0, record_ms * 1000, total_pts)

    # Interpolate envelope to full resolution
    env_x = np.arange(n_samples) * sample_us + sample_us / 2
    env_y = np.array(envelope_samples, dtype=float)
    # Normalize envelope 0-1
    env_norm = env_y / 255.0
    env_interp = np.interp(t_us, env_x, env_norm)

    # Generate 40kHz carrier
    carrier = np.sin(2 * np.pi * freq_hz * t_us * 1e-6)

    # Burst region: estimate from pulse count
    # At 40kHz, one period = 25µs. pulse_count periods = pulse_count*25µs
    burst_duration_us = pulse_count * (1e6 / freq_hz)
    burst_mask = (t_us <= burst_duration_us).astype(float)

    # Blend: burst region = full amplitude carrier, after = envelope-modulated
    burst_signal   = carrier * burst_mask
    echo_signal    = carrier * env_interp * (1 - burst_mask)
    signal = burst_signal + echo_signal

    return t_us, signal, env_interp, burst_duration_us


def find_ringdown_end(samples, noise_margin=2.5):
    arr = np.array(samples, dtype=float)
    noise = float(np.median(np.sort(arr)[:20]))
    thr = max(noise * noise_margin, 15.0)
    end_idx = 0
    for i in range(len(arr)):
        if arr[i] > thr:
            end_idx = i
        elif i > end_idx + 5:
            break
    return end_idx, thr, noise


def main():
    print("=" * 66)
    print("  PGA460-Q1 — Oscilloscope-Style Waveform from CMD7 Data")
    print("=" * 66)

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE,
                            bytesize=8, parity='N', stopbits=2, timeout=0.5)
    except Exception as e:
        print(f"[ERROR] {e}"); return
    time.sleep(0.3)

    # Read current config
    rec_reg  = reg_read(ser, 0x22)
    pulse_reg = reg_read(ser, 0x1E)
    freq_reg  = reg_read(ser, 0x1C)

    rec_reg   = rec_reg   if rec_reg   is not None else 0x80
    pulse_reg = pulse_reg if pulse_reg is not None else 0x48
    freq_reg  = freq_reg  if freq_reg  is not None else 0x32

    p1_rec      = (rec_reg >> 4) & 0x0F
    record_ms   = 4.096 * (p1_rec + 1)
    pulse_count = ((pulse_reg >> 3) & 0x1F) + 1
    freq_khz    = 0.2 * freq_reg + 30.0
    sample_us   = record_ms * 1000 / 128

    print(f"\n  Config read from IC:")
    print(f"    Record time  : {record_ms:.2f}ms  (P1_REC={p1_rec})")
    print(f"    Pulse count  : {pulse_count}")
    print(f"    Frequency    : {freq_khz:.1f} kHz")
    print(f"    Per sample   : {sample_us:.1f}µs")
    print(f"\n  Collecting {N_CAPTURES} echo dumps...")

    # Collect captures
    captures = []
    tof_readings = []

    for i in range(N_CAPTURES):
        burst_listen(ser, record_ms)
        dump = get_echo_dump(ser)

        burst_listen(ser, record_ms)
        tof_r = get_tof(ser)

        if dump:
            captures.append(dump)
            tof_str = "no object"
            if tof_r:
                tof_us, w, pk = tof_r
                if 0 < tof_us < 0x9E00:
                    d = V_SOUND * tof_us * 1e-6 / 2.0
                    tof_str = f"{d:.3f}m"
                    tof_readings.append(d)
            print(f"  Capture {i+1}: max={max(dump)}  min={min(dump)}  dist={tof_str}")
        time.sleep(0.05)

    ser.close()

    if not captures:
        print("[ERROR] No data received."); return

    avg_dump = np.mean(captures, axis=0)
    ring_end_idx, ring_thr, ring_noise = find_ringdown_end(avg_dump.tolist())
    ring_end_us  = (ring_end_idx + 1) * sample_us
    ring_end_ms  = ring_end_us / 1000
    ring_end_m   = V_SOUND * ring_end_us * 1e-6 / 2.0

    # Detected distance
    dist_m = np.mean(tof_readings) if tof_readings else None
    dist_us = dist_m / V_SOUND * 2 * 1e6 if dist_m else None

    print(f"\n  Analysis:")
    print(f"    Ringdown ends : {ring_end_ms:.2f}ms → {ring_end_m*100:.0f}cm blind zone")
    if dist_m:
        print(f"    Object at     : {dist_m:.3f}m")

    # ── RECONSTRUCT WAVEFORM ──────────────────────────────────────────────────
    t_us, signal, env_norm, burst_us = reconstruct_oscilloscope(
        avg_dump.tolist(), record_ms, freq_khz * 1000, pulse_count)

    # Also create a "before tuning" simulation — more pulses
    more_pulses = min(pulse_count * 3, 31)
    _, signal_heavy, env_heavy, burst_us_heavy = reconstruct_oscilloscope(
        avg_dump.tolist(), record_ms, freq_khz * 1000, more_pulses)

    # ── PLOT ─────────────────────────────────────────────────────────────────
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 11))
    fig.suptitle(
        'PGA460-Q1 — Oscilloscope-Style Transducer Waveform\n'
        f'UTR-1440K-TT-R 40kHz | Reconstructed from CMD7 Echo Data Dump',
        fontsize=13, fontweight='bold', color='white', y=0.98
    )
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.50, wspace=0.30)

    ax_osc_heavy = fig.add_subplot(gs[0, 0])   # oscilloscope view many pulses
    ax_osc_curr  = fig.add_subplot(gs[0, 1])   # oscilloscope view current pulses
    ax_raw_left  = fig.add_subplot(gs[1, 0])   # raw envelope many pulses
    ax_raw_right = fig.add_subplot(gs[1, 1])   # raw envelope current pulses
    ax_full      = fig.add_subplot(gs[2, :])   # full annotated echo dump

    FC    = '#0a0a0a'    # near-black like oscilloscope background
    BLUE  = '#4488ff'    # oscilloscope trace color

    def style_osc(ax, title):
        ax.set_facecolor(FC)
        ax.tick_params(colors='#666', labelsize=7)
        for spine in ax.spines.values():
            spine.set_color('#333')
        ax.set_title(title, color='white', fontsize=9, pad=4)
        ax.set_xlabel('Time (ms)', color='#888', fontsize=8)
        ax.set_ylabel('Amplitude', color='#888', fontsize=8)
        # Add oscilloscope-style grid
        ax.grid(True, color='#222', lw=0.5, alpha=0.8)
        ax.set_xlim(0, record_ms)
        ax.set_ylim(-1.15, 1.15)

    def style_env(ax, title):
        ax.set_facecolor('#1a1a2e')
        ax.tick_params(colors='white', labelsize=8)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#444')
        ax.spines['bottom'].set_color('#444')
        ax.set_title(title, color='white', fontsize=9, pad=4)
        ax.set_xlabel('Time (ms)', color='white', fontsize=8)
        ax.set_ylabel('DSP Amplitude (0-255)', color='white', fontsize=8)
        ax.set_xlim(0, record_ms)
        ax.set_ylim(0, 270)

    t_ms = t_us / 1000.0  # convert to ms for x-axis

    # ── Top-left: oscilloscope style, heavy pulses ────────────────────────────
    style_osc(ax_osc_heavy,
              f'Oscilloscope View — {more_pulses} pulses (long ringdown)')
    ax_osc_heavy.plot(t_ms, signal_heavy, color=BLUE, lw=0.5, alpha=0.9)
    # Shade burst region
    ax_osc_heavy.axvspan(0, burst_us_heavy/1000, alpha=0.08, color='yellow')
    ax_osc_heavy.axvline(burst_us_heavy/1000, color='yellow', lw=0.8, ls='--', alpha=0.5)
    ax_osc_heavy.axvline(ring_end_ms, color='#ff6b6b', lw=1, ls='--', alpha=0.7)
    ax_osc_heavy.text(burst_us_heavy/1000/2, 1.05, 'BURST',
                      ha='center', color='yellow', fontsize=7)
    ax_osc_heavy.text(ring_end_ms + record_ms*0.01, 1.05, f'RINGDOWN →',
                      color='#ff6b6b', fontsize=7)
    # Add horizontal center line like oscilloscope
    ax_osc_heavy.axhline(0, color='#333', lw=0.5)

    # ── Top-right: oscilloscope style, current pulses ─────────────────────────
    style_osc(ax_osc_curr,
              f'Oscilloscope View — {pulse_count} pulses (current setting)')
    ax_osc_curr.plot(t_ms, signal, color=BLUE, lw=0.5, alpha=0.9)
    ax_osc_curr.axvspan(0, burst_us/1000, alpha=0.08, color='yellow')
    ax_osc_curr.axvline(burst_us/1000, color='yellow', lw=0.8, ls='--', alpha=0.5)
    ax_osc_curr.axvline(ring_end_ms, color='#ff6b6b', lw=1.2, ls='--',
                         label=f'Ringdown end {ring_end_ms:.1f}ms')
    if dist_us:
        dist_ms_val = dist_us / 1000
        ax_osc_curr.axvline(dist_ms_val, color='#00d4ff', lw=1.2, ls='--',
                             label=f'Echo {dist_m:.2f}m')
        ax_osc_curr.text(dist_ms_val + record_ms*0.01, 1.05,
                         f'ECHO\n{dist_m:.2f}m', color='#00d4ff', fontsize=7)
    ax_osc_curr.axhline(0, color='#333', lw=0.5)
    ax_osc_curr.text(burst_us/1000/2, 1.05, 'BURST',
                     ha='center', color='yellow', fontsize=7)
    ax_osc_curr.legend(fontsize=7, facecolor='#111', labelcolor='white',
                       loc='upper right')

    # ── Middle-left: raw DSP envelope heavy ───────────────────────────────────
    style_env(ax_raw_left, f'CMD7 DSP Envelope — simulated {more_pulses} pulses')
    t_env = np.linspace(0, record_ms, 128)
    ax_raw_left.plot(t_env, avg_dump, color='#a29bfe', lw=1.5)
    ax_raw_left.axvspan(0, ring_end_ms, alpha=0.15, color='#ff6b6b')
    ax_raw_left.axhline(ring_noise, color='#636e72', ls=':', lw=1)
    ax_raw_left.axhline(ring_thr, color='#ff9f43', ls='--', lw=1)

    # ── Middle-right: raw DSP envelope current ────────────────────────────────
    style_env(ax_raw_right, f'CMD7 DSP Envelope — current ({pulse_count} pulses)')
    for d in captures:
        ax_raw_right.plot(t_env, d, color='#a29bfe', alpha=0.15, lw=0.8)
    ax_raw_right.plot(t_env, avg_dump, color='#a29bfe', lw=2,
                      label=f'Average ({N_CAPTURES} bursts)')
    ax_raw_right.axvspan(0, ring_end_ms, alpha=0.15, color='#ff6b6b',
                          label=f'Ringdown {ring_end_ms:.1f}ms')
    ax_raw_right.axvline(ring_end_ms, color='#ff6b6b', lw=1.5, ls='--')
    ax_raw_right.axhline(ring_noise, color='#636e72', ls=':', lw=1,
                          label=f'Noise floor {ring_noise:.0f}')
    ax_raw_right.axhline(ring_thr, color='#ff9f43', ls='--', lw=1,
                          label=f'Threshold {ring_thr:.0f}')
    if dist_us:
        ax_raw_right.axvline(dist_us/1000, color='#00d4ff', lw=1.5, ls='--',
                              label=f'Object {dist_m:.2f}m')
    ax_raw_right.legend(fontsize=7, facecolor='#1a1a2e', labelcolor='white',
                         loc='upper right')

    # ── Bottom: full annotated view ────────────────────────────────────────────
    ax_full.set_facecolor('#0a0a0a')
    ax_full.tick_params(colors='#888', labelsize=9)
    for spine in ax_full.spines.values():
        spine.set_color('#333')
    ax_full.grid(True, color='#1a1a1a', lw=0.5)

    # Plot oscilloscope signal
    ax_full.plot(t_ms, signal, color=BLUE, lw=0.6, alpha=0.9,
                 label='Reconstructed waveform (40kHz carrier × DSP envelope)')
    # Overlay envelope
    ax_full.plot(t_ms, env_norm, color='#ff9f43', lw=1.5, alpha=0.7,
                 label='Envelope (normalized)')
    ax_full.plot(t_ms, -env_norm, color='#ff9f43', lw=1.5, alpha=0.7)

    # Annotate regions
    ax_full.axvspan(0, burst_us/1000, alpha=0.12, color='yellow',
                    label=f'Burst ({pulse_count} pulses, {burst_us:.0f}µs)')
    ax_full.axvspan(burst_us/1000, ring_end_ms, alpha=0.08, color='#ff6b6b',
                    label=f'Ringdown ({ring_end_ms:.1f}ms → {ring_end_m*100:.0f}cm blind)')
    ax_full.axvline(ring_end_ms, color='#ff6b6b', lw=1.5, ls='--')
    ax_full.axhline(0, color='#333', lw=0.5)

    if dist_us:
        dist_ms_val = dist_us / 1000
        ax_full.axvline(dist_ms_val, color='#00d4ff', lw=2, ls='--',
                        label=f'Echo peak → {dist_m:.3f}m')
        # Annotate peak
        ax_full.annotate(
            f'ECHO\n{dist_m:.3f}m\n({dist_m*100:.0f}cm)',
            xy=(dist_ms_val, 0.7),
            xytext=(dist_ms_val + record_ms * 0.03, 0.85),
            color='#00d4ff', fontsize=9, fontweight='bold',
            arrowprops=dict(arrowstyle='->', color='#00d4ff'),
            bbox=dict(boxstyle='round', facecolor='#0a0a0a', alpha=0.8)
        )

    # Region labels
    ax_full.text(burst_us/2000, 1.05, f'BURST\n{pulse_count} pulses\n{burst_us:.0f}µs',
                 ha='center', color='yellow', fontsize=8, fontweight='bold')
    ax_full.text((burst_us/1000 + ring_end_ms) / 2, 1.05,
                 f'RINGDOWN\n{ring_end_ms:.1f}ms',
                 ha='center', color='#ff6b6b', fontsize=8, fontweight='bold')
    if ring_end_ms < record_ms * 0.8:
        ax_full.text((ring_end_ms + record_ms) / 2, 1.05,
                     f'DETECTION ZONE\n{ring_end_m:.2f}m → {V_SOUND*record_ms*1e-3/2:.1f}m',
                     ha='center', color='#55efc4', fontsize=8, fontweight='bold')

    ax_full.set_xlim(0, record_ms)
    ax_full.set_ylim(-1.25, 1.25)
    ax_full.set_xlabel('Time after burst end (ms)', color='white', fontsize=10)
    ax_full.set_ylabel('Amplitude (normalized)', color='white', fontsize=10)
    ax_full.set_title(
        f'Full Annotated Waveform — {freq_khz:.0f}kHz Transducer | '
        f'Burst→Ringdown→Echo Structure',
        color='white', fontsize=10)
    ax_full.legend(fontsize=8, facecolor='#111', labelcolor='white',
                   loc='upper right', ncol=2)

    # Footer
    footer = (
        f"UTR-1440K-TT-R 40kHz  |  "
        f"Record={record_ms:.0f}ms  |  "
        f"Pulses={pulse_count}  |  "
        f"Ringdown={ring_end_ms:.1f}ms → {ring_end_m*100:.0f}cm blind zone  |  "
        f"{'Object at ' + f'{dist_m:.3f}m' if dist_m else 'No object detected — place at 1-5m'}"
    )
    fig.text(0.5, 0.002, footer, ha='center', fontsize=8.5,
             color='#55efc4', fontweight='bold',
             bbox=dict(boxstyle='round', facecolor='#0d0d1a', alpha=0.8))

    plt.savefig('oscilloscope_waveform.png', dpi=150,
                bbox_inches='tight', facecolor='#050505')

    print(f"\n  Plot saved: oscilloscope_waveform.png")
    print(f"  Ringdown : {ring_end_ms:.2f}ms → {ring_end_m*100:.0f}cm")
    if dist_m:
        print(f"  Distance : {dist_m:.3f}m")
    print(f"\nClose window to exit.")
    plt.show()


if __name__ == '__main__':
    main()