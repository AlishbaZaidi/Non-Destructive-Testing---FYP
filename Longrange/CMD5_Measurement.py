"""
PGA460-Q1 — Correct Distance Measurement Using CMD5 TOF
=========================================================
KEY INSIGHT:
  CMD5 (Ultrasonic Measurement Result) = CORRECT measurement
    - Returns TOF of FIRST echo that crosses the threshold
    - Changes when you change the object → measures YOUR object
    - This is the actual designed distance measurement method

  CMD7 (Echo Data Dump) = waveform visualization only
    - Shows the strongest peak in the entire record window
    - Was stuck at 1.4781m = background wall/object in the room
    - Useful for seeing the signal shape, NOT for distance

  PGA460 datasheet Table 7-3 footnote (5) explicitly says:
  "distance (m) = [vsound × (MSB<<8 + LSB) ÷ 2 × 1µs]"
  This is CMD5, not CMD7.

WHY CMD5 WAS ONLY VALID 1/30 TIMES BEFORE:
  Threshold T1 start = 5200µs — good (past ringdown)
  But threshold LEVEL = 0x20 (32/255) — too low OR too high
  The comparator triggers inconsistently.

FIX:
  We will sweep the threshold level and find the value that
  gives consistent CMD5 detections for your object.

USAGE:
  Set TARGET_M below.
  Run:  python cmd5_measurement.py
"""

import serial, time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from datetime import datetime

# ─── CONFIG ──────────────────────────────────────────────────────────────────
COM_PORT  = 'COM12'
BAUD_RATE = 115200
UART_ADDR = 0
TEMP_C    = 25.0
TARGET_M  = 1.3  # ← set to your tape-measured distance
N_LOOPS   = 30
# ─────────────────────────────────────────────────────────────────────────────

V_SOUND = 331.0 + 0.6 * TEMP_C   # 346 m/s
P1_REC  = 5
REC_MS  = 4.096 * (P1_REC + 1)   # 24.576ms
SAMP_MS = REC_MS / 128.0
SENT_US = (REC_MS - SAMP_MS) * 1000
RING_US = 4000.0   # confirmed ringdown

ECHO_MS = TARGET_M / V_SOUND * 2 * 1000

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
    t = time.time() + ms/1000; b = b''
    while len(b) < n and time.time() < t:
        c = ser.read(n - len(b))
        if c: b += c
    return b

def rw(ser, a, v):
    ser.reset_input_buffer()
    ser.write(build(10, bytes([a, v])))
    time.sleep(0.06)

def burst(ser):
    ser.reset_input_buffer()
    ser.write(build(0, bytes([1])))
    time.sleep(REC_MS/1000 + 0.1)
    ser.reset_input_buffer()

def get_cmd5(ser):
    ser.write(build(5))
    time.sleep(0.08)
    r = rx(ser, 6)
    if len(r) == 6:
        tof = (r[1] << 8) | r[2]
        return tof, r[3], r[4]   # tof_us, width, peak
    return None

def get_cmd7(ser):
    ser.write(build(7))
    time.sleep(0.3)
    r = rx(ser, 130, ms=1000)
    return list(r[1:129]) if len(r) == 130 else None

def tof_to_m(us): return V_SOUND * us * 1e-6 / 2.0

def write_thresholds(ser, t1_code, level_byte):
    """
    t1_code: nibble for T1 absolute time (0x00-0x0F)
             0x08=1400µs, 0x09=2000µs, 0x0A=2400µs,
             0x0B=3200µs, 0x0C=4000µs, 0x0D=5200µs
    level_byte: 0x00-0xFF threshold level
    """
    lv = min(0xF, level_byte >> 4)
    thr = ([(t1_code<<4)|0x3] + [0x33]*5 +
           [(lv<<4)|lv]*4 +
           [level_byte]*4 + [0x00]*2) * 2
    ser.reset_input_buffer()
    ser.write(build(16, bytes(thr)))
    time.sleep(0.1)

def setup_ic(ser, level_byte=0x40):
    """Apply all registers + thresholds."""
    for a, v in [
        (0x1C, 0x32),
        (0x1E, 0x48),
        (0x22, (P1_REC<<4)|0x0),
        (0x1B, 0x10),
        (0x26, 0x10),
        (0x1D, 0x00),
        (0x20, 0x3F),
        (0x14, 0xCC),
        (0x15, 0xDD),
        (0x16, 0xEE),
        (0x17, 0x14),
        (0x18, 0x4F),
        (0x19, 0xBF),
        (0x1A, 0xBC),
    ]:
        rw(ser, a, v)
    # T1=0x0C=4000µs (just after ringdown), level=level_byte
    write_thresholds(ser, t1_code=0x0C, level_byte=level_byte)


def find_best_threshold(ser):
    """
    Sweep threshold level and find value giving most consistent CMD5.
    Tests levels: 0x08, 0x10, 0x20, 0x30, 0x40, 0x60, 0x80
    """
    print("\n  AUTO-FINDING BEST THRESHOLD LEVEL...")
    print(f"  (Looking for consistent TOF near {TARGET_M}m = {ECHO_MS:.1f}ms)")
    print(f"  {'Level':>8}  {'Valid':>6}  {'Avg dist':>10}  {'Std':>8}")
    print(f"  {'-'*8}  {'-'*6}  {'-'*10}  {'-'*8}")

    levels   = [0x08, 0x10, 0x18, 0x20, 0x30, 0x40, 0x60, 0x80, 0xA0]
    best_lv  = 0x40
    best_n   = 0

    for lv in levels:
        write_thresholds(ser, t1_code=0x0C, level_byte=lv)
        time.sleep(0.05)

        dists = []
        for _ in range(5):
            burst(ser)
            tr = get_cmd5(ser)
            if tr:
                tof, w, pk = tr
                if RING_US < tof < SENT_US and pk > 3:
                    dists.append(tof_to_m(tof))
            time.sleep(0.03)

        n   = len(dists)
        avg = np.mean(dists) if dists else 0
        std = np.std(dists) if len(dists) > 1 else 0
        print(f"  0x{lv:02X}      {n:>6}  {avg:>10.4f}  {std*100:>6.2f}cm")

        # Best = most valid readings, closest to target
        score = n * (1 - min(abs(avg - TARGET_M), 1.0))
        if n > best_n or (n == best_n and score > 0):
            best_n  = n
            best_lv = lv

    print(f"\n  Best level: 0x{best_lv:02X}")
    return best_lv


def main():
    print("=" * 62)
    print(f"  PGA460-Q1 — CMD5 TOF Distance Measurement")
    print(f"  Target = {TARGET_M}m | Echo @ {ECHO_MS:.2f}ms")
    print(f"  Record = {REC_MS:.1f}ms | Ringdown = 4ms = 0.69m")
    print("=" * 62)
    print(f"\n  Object at {TARGET_M}m from transducer face.")
    input("  Press ENTER to start...")

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, bytesize=8,
                            parity='N', stopbits=2, timeout=0.5)
    except Exception as e:
        print(f"[ERROR] {e}"); return
    time.sleep(0.3)

    setup_ic(ser, level_byte=0x40)

    # Auto-find best threshold
    best_lv = find_best_threshold(ser)

    # Apply best threshold and run measurement
    write_thresholds(ser, t1_code=0x0C, level_byte=best_lv)
    print(f"\n  Running {N_LOOPS} measurements with threshold=0x{best_lv:02X}...")
    print(f"\n  {'Lp':>4}  {'TOF(µs)':>10}  {'Distance':>10}  "
          f"{'Width':>7}  {'Peak':>6}  Status")
    print(f"  {'-'*4}  {'-'*10}  {'-'*10}  {'-'*7}  {'-'*6}  ------")

    dists=[]; peaks=[]; widths=[]; tofs=[]; lnums=[]
    dumps=[]

    for lp in range(1, N_LOOPS+1):
        # Get CMD7 for waveform
        burst(ser)
        dump = get_cmd7(ser)
        if dump: dumps.append(dump)

        # CMD5 = primary distance
        burst(ser)
        tr = get_cmd5(ser)

        dist=pk=w=None; st="no echo"
        if tr:
            tof_us, w, pk = tr
            pk=int(pk); w=int(w)
            if RING_US < tof_us < SENT_US and pk > 3:
                dist = tof_to_m(tof_us)
                dists.append(dist); peaks.append(pk)
                widths.append(w); tofs.append(tof_us)
                lnums.append(lp)
                st = f"✓ {dist:.4f}m"
            elif tof_us <= RING_US:
                st = f"ringdown (tof={tof_us}µs)"
            elif tof_us >= SENT_US:
                st = "sentinel — no echo"
            else:
                st = f"low peak (pk={pk})"
            ts = str(tof_us)
        else:
            ts = "---"

        ds = f"{dist:.4f}" if dist else "---"
        print(f"  {lp:>4}  {ts:>10}  {ds:>10}m  "
              f"{str(w) if w else '---':>7}  {str(pk) if pk else '---':>6}  {st}")
        time.sleep(0.05)

    ser.close()

    # ── Results ───────────────────────────────────────────────────────────────
    print(f"\n{'='*62}")
    print(f"  FINAL RESULTS — CMD5 TOF")
    print(f"{'='*62}")

    if dists:
        m   = np.mean(dists)
        s   = np.std(dists)
        err = (m - TARGET_M) * 100
        snr = np.mean(peaks)
        print(f"  Measured  : {m:.4f} m")
        print(f"  Target    : {TARGET_M} m")
        print(f"  Error     : {err:+.2f} cm")
        print(f"  Std dev   : ±{s*100:.3f} cm")
        print(f"  Valid     : {len(dists)}/{N_LOOPS}  ({len(dists)/N_LOOPS*100:.0f}%)")
        print(f"  Avg peak  : {np.mean(peaks):.0f}/255")
        print(f"  Avg width : {np.mean(widths)*4:.0f}µs")
        print(f"  Ringdown  : 4ms → 0.69m blind zone")
    else:
        print(f"  No valid echo from CMD5.")
        print(f"  Object may be at wrong distance or threshold still wrong.")

    if not dumps:
        return

    avg_dump = np.mean(dumps, axis=0)
    nf_f     = float(np.median(avg_dump[100:]))
    t_ax     = np.linspace(0, REC_MS, 128)

    # ── Plot ──────────────────────────────────────────────────────────────────
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        'PGA460-Q1 — Ultrasonic Distance Measurement\n'
        f'UTR-1440K-TT-R 40kHz | CMD5 TOF = Primary | Target={TARGET_M}m',
        fontsize=13, fontweight='bold', color='white', y=0.98
    )
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.50, wspace=0.38)
    aw  = fig.add_subplot(gs[0, :])
    ad  = fig.add_subplot(gs[1, 0])
    ah  = fig.add_subplot(gs[1, 1])
    atb = fig.add_subplot(gs[1, 2])

    FC = '#1a1a2e'
    def sty(ax):
        ax.set_facecolor(FC); ax.tick_params(colors='white', labelsize=9)
        ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#444'); ax.spines['bottom'].set_color('#444')

    # ── Waveform ──────────────────────────────────────────────────────────────
    sty(aw)
    for d in dumps:
        aw.plot(t_ax, d, color='#a29bfe', alpha=0.04, lw=0.5)
    aw.plot(t_ax, avg_dump, color='#a29bfe', lw=2.5,
            label=f'Avg CMD7 waveform ({len(dumps)} bursts)')
    aw.axvspan(0, RING_US/1000, alpha=0.22, color='#ff6b6b')
    aw.axvline(RING_US/1000, color='#ff6b6b', ls='--', lw=2,
               label=f'Ringdown {RING_US/1000:.0f}ms = {tof_to_m(RING_US):.2f}m blind zone')
    aw.axhline(nf_f, color='#636e72', ls=':', lw=1.5,
               label=f'Noise floor = {nf_f:.0f}')
    aw.text(RING_US/2000, 255, f'BLIND\nZONE\n{tof_to_m(RING_US):.2f}m',
            ha='center', va='top', color='#ff6b6b', fontsize=8.5,
            fontweight='bold', bbox=dict(boxstyle='round', facecolor=FC, alpha=0.8))

    # Expected echo line (target)
    aw.axvline(ECHO_MS, color='#55efc4', ls=':', lw=1.5,
               label=f'Target {TARGET_M}m @ {ECHO_MS:.2f}ms')

    if dists:
        m = np.mean(dists)
        det_ms = m / V_SOUND * 2 * 1000
        aw.axvline(det_ms, color='#00d4ff', ls='--', lw=2.5,
                   label=f'CMD5 detected: {m:.4f}m @ {det_ms:.2f}ms')
        # Draw a star at that time on the average waveform
        idx = int(det_ms / SAMP_MS)
        if 0 <= idx < 128:
            aw.plot(t_ax[idx], avg_dump[idx], '*',
                    color='#00d4ff', ms=16, zorder=6)
            aw.annotate(
                f'CMD5 ECHO\n{m:.4f}m\n{m*100:.1f}cm\npk≈{np.mean(peaks):.0f}',
                xy=(t_ax[idx], avg_dump[idx]),
                xytext=(t_ax[idx]+1.5, max(40, avg_dump[idx]-50)),
                color='#00d4ff', fontsize=9.5, fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='#00d4ff', lw=2),
                bbox=dict(boxstyle='round', facecolor='#0d1117',
                          alpha=0.95, edgecolor='#00d4ff', lw=1.5)
            )

    # Threshold level line
    aw.axhline(best_lv, color='#ff9f43', ls='--', lw=1.5,
               label=f'Threshold level = 0x{best_lv:02X} ({best_lv}/255)')

    ax2 = aw.twiny(); ax2.set_xlim(aw.get_xlim())
    dm2=[0,0.5,1.0,1.5,2.0,2.5,3.0,4.0]
    tm2=[d/V_SOUND*2*1000 for d in dm2]
    vt=[(t,d) for t,d in zip(tm2,dm2) if t<=REC_MS]
    ax2.set_xticks([t for t,d in vt])
    ax2.set_xticklabels([f'{d}m' for t,d in vt], color='#aaa', fontsize=8.5)
    ax2.tick_params(colors='#aaa'); ax2.spines['top'].set_color('#444')
    for sp in ['right','left','bottom']: ax2.spines[sp].set_visible(False)

    aw.set_xlabel('Time after burst end (ms)', color='white', fontsize=11)
    aw.set_ylabel('DSP Amplitude (8-bit, 0–255)', color='white', fontsize=11)
    aw.set_title('Echo Data Dump (CMD7) + CMD5 TOF Marker', color='white', fontsize=11)
    aw.set_xlim(0, REC_MS); aw.set_ylim(0, 270)
    aw.legend(fontsize=8.5, facecolor=FC, labelcolor='white',
              loc='upper right', ncol=2)

    # ── Distance over loops ───────────────────────────────────────────────────
    sty(ad)
    if dists:
        m, s = np.mean(dists), np.std(dists)
        ad.plot(lnums, dists, 'o', color='#00d4ff', ms=6, alpha=0.8, label='CMD5')
        ad.axhline(m,        color='#00d4ff', ls='--', lw=2.5,
                   label=f'Mean={m:.4f}m')
        ad.axhline(TARGET_M, color='#55efc4', ls=':', lw=1.5,
                   label=f'Target={TARGET_M}m')
        spread = max(abs(max(dists)-m), abs(min(dists)-m), 0.03)
        ad.set_ylim(m-spread*3, m+spread*3)
        ad.set_title(f'CMD5: {m:.4f}m ± {s*100:.3f}cm  '
                     f'({len(dists)}/{N_LOOPS} valid)',
                     color='white', fontsize=9)
        ad.legend(fontsize=7.5, facecolor=FC, labelcolor='white')
    else:
        ad.text(0.5,0.5,'No CMD5 echo',ha='center',va='center',
                color='#ff9f43',fontsize=11,transform=ad.transAxes)
    ad.set_xlabel('Loop #', color='white')
    ad.set_ylabel('Distance (m)', color='white')

    # ── Histogram ─────────────────────────────────────────────────────────────
    sty(ah)
    if dists and len(dists) > 1:
        m, s = np.mean(dists), np.std(dists)
        unique = len(set([round(x,4) for x in dists]))
        ah.hist(dists, bins=max(3,unique), color='#00d4ff', alpha=0.8,
                edgecolor='#0984e3')
        ah.axvline(m,        color='#55efc4', ls='--', lw=2.5,
                   label=f'Mean={m:.4f}m')
        ah.axvline(TARGET_M, color='#ff9f43', ls=':', lw=1.5,
                   label=f'Target={TARGET_M}m')
        ah.set_title(f'Distribution  Std=±{s*100:.3f}cm', color='white', fontsize=9)
        ah.legend(fontsize=7.5, facecolor=FC, labelcolor='white')
    else:
        ah.text(0.5,0.5,'Need more data',ha='center',va='center',
                color='#ff9f43',transform=ah.transAxes)
        ah.set_title('Distribution', color='white')
    ah.set_xlabel('Distance (m)', color='white')
    ah.set_ylabel('Count', color='white')

    # ── Summary table ─────────────────────────────────────────────────────────
    atb.set_facecolor(FC); atb.axis('off')
    atb.set_title('Measurement Summary', color='white', fontsize=10, pad=8)
    if dists:
        m, s = np.mean(dists), np.std(dists)
        rows = [
            ['Parameter', 'Value'],
            ['Measured',       f'{m:.4f} m'],
            ['Target',         f'{TARGET_M} m'],
            ['Error',          f'{(m-TARGET_M)*100:+.2f} cm'],
            ['Std Dev',        f'±{s*100:.3f} cm'],
            ['Valid / Total',  f'{len(dists)}/{N_LOOPS}'],
            ['Method',         'CMD5 TOF'],
            ['Threshold',      f'0x{best_lv:02X} ({best_lv}/255)'],
            ['Avg Peak',       f'{np.mean(peaks):.0f}/255'],
            ['Noise Floor',    f'{nf_f:.0f}'],
            ['SNR',            f'{np.mean(peaks)/max(nf_f,1):.1f}:1'],
            ['Ringdown',       f'4ms → 0.69m blind'],
            ['Frequency',      '40 kHz'],
            ['V_sound',        f'{V_SOUND:.0f} m/s @ {TEMP_C}°C'],
            ['Formula',        'v × TOF_µs×1e-6 / 2'],
        ]
        tbl = atb.table(cellText=rows[1:], colLabels=rows[0],
                        loc='center', cellLoc='center')
        tbl.auto_set_font_size(False); tbl.set_fontsize(8.5)
        for (r,c),cell in tbl.get_celld().items():
            fc = '#2d3436' if r==0 else ('#1e3a5f' if rows[r][0] in
                 ['Measured','Error','Std Dev'] else
                 '#252540' if r%2==0 else FC)
            cell.set_facecolor(fc)
            cell.set_text_props(color='white',
                                fontweight='bold' if r==0 else 'normal')
            cell.set_edgecolor('#444')
        tbl.scale(1, 1.35)

    footer = (
        f"Transducer: UTR-1440K-TT-R 40kHz | "
        f"Measured={np.mean(dists):.4f}m | "
        f"Std=±{np.std(dists)*100:.3f}cm | "
        f"Ringdown=4ms→0.69m | "
        f"Formula: dist=v_sound×TOF/2 (PGA460 datasheet Table 7-3 fn.5)"
        if dists else "No echo detected"
    )
    fig.text(0.5,0.002,footer,ha='center',fontsize=8.5,
             color='#55efc4',fontweight='bold',
             bbox=dict(boxstyle='round',facecolor='#0d0d1a',alpha=0.85))
    timestamp     = datetime.now().strftime("%Y%m%d_%H%M%S")

    plt.savefig('result_0.7m_4_{timestamp}.png',dpi=150,
                bbox_inches='tight',facecolor='#0d0d1a')
    print(f"\n  Saved: cmd5_measurement9_1.5m_2_{timestamp}.png")
    print(f"\n  ══════════════════════════════════════")
    if dists:
        m,s=np.mean(dists),np.std(dists)
        print(f"  ANSWER: {m:.4f}m ± {s*100:.3f}cm")
        print(f"  Error : {(m-TARGET_M)*100:+.2f}cm")
    print(f"  ══════════════════════════════════════")
    print("\nClose window to exit.")
    plt.show()

if __name__ == '__main__':
    main()