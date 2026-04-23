"""
PGA460-Q1 — Maximum Range Detection
=====================================
T1 OPTIONS (change T1_CODE below):
  0xE = 6400us -> min detectable = 346 * 0.0064 / 2 = 1.11m
  0xF = 8000us -> min detectable = 346 * 0.0080 / 2 = 1.38m
"""

import serial, time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

COM_PORT  = 'COM12'
BAUD_RATE = 115200
UART_ADDR = 0
TEMP_C    = 25.0
N_LOOPS   = 30

# CHANGE THIS LINE TO SWITCH WINDOWS
T1_CODE = 0xE   # 0xE = 6400us (1.11m min) | 0xF = 8000us (1.38m min)

V_SOUND = 331.0 + 0.6 * TEMP_C

# P1_REC=15 -> 4.096x16 = 65.536ms -> 11.3m max range
P1_REC  = 15
REC_MS  = 4.096 * (P1_REC + 1)
SAMP_MS = REC_MS / 128.0
SENT_US = (REC_MS - SAMP_MS) * 1000

T1_US  = {0xC:4000, 0xD:5200, 0xE:6400, 0xF:8000}[T1_CODE]
MIN_M  = V_SOUND * T1_US * 1e-6 / 2.0
MAX_M  = V_SOUND * REC_MS * 1e-3 / 2.0

print(f"T1=0x{T1_CODE:X} -> {T1_US}us -> min range={MIN_M:.2f}m")
print(f"Record={REC_MS:.0f}ms -> max range={MAX_M:.1f}m")

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

def rx(ser, n, ms=1000):
    t = time.time() + ms/1000; b = b''
    while len(b) < n and time.time() < t:
        c = ser.read(n - len(b))
        if c: b += c
    return b

def rw(ser, a, v):
    ser.reset_input_buffer()
    ser.write(build(10, bytes([a, v])))
    time.sleep(0.06)

def measure(ser):
    ser.reset_input_buffer()
    ser.write(build(0, bytes([1])))
    time.sleep(REC_MS/1000 + 0.15)
    ser.reset_input_buffer()

    ser.write(build(5)); time.sleep(0.08)
    r5 = rx(ser, 6)
    tof=w=pk=None
    if len(r5)==6:
        tof=(r5[1]<<8)|r5[2]; w=r5[3]; pk=r5[4]

    ser.write(build(7)); time.sleep(0.3)
    r7 = rx(ser, 130, ms=1000)
    dump = list(r7[1:129]) if len(r7)==130 else None

    return tof, w, pk, dump

def tof_m(us): return V_SOUND * us * 1e-6 / 2

def valid(tof, pk):
    if tof is None or pk is None: return False
    if tof <= T1_US:   return False
    if tof >= SENT_US: return False
    if pk < 5:         return False
    return True

def main():
    print("="*60)
    print(f"  PGA460-Q1 - Max Range Detection")
    print(f"  T1=0x{T1_CODE:X} ({T1_US}us) -> blind zone={MIN_M:.2f}m")
    print(f"  Record={REC_MS:.0f}ms -> max={MAX_M:.1f}m")
    print("="*60)
    input("\n  Press ENTER to start...")

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, bytesize=8,
                            parity='N', stopbits=2, timeout=0.5)
    except Exception as e:
        print(f"[ERROR] {e}"); return
    time.sleep(0.3)

    for a, v in [
        (0x1C, 0x32),             # FREQUENCY: 40kHz
        (0x1E, 0x48),             # PULSE_P1: 10 pulses
        (0x22, (P1_REC<<4)|0x0),  # REC_LENGTH: max record
        (0x1B, 0x10),             # INIT_GAIN: medium
        (0x26, 0x10),             # DECPL_TEMP: 32dB offset
        (0x1D, 0x00),             # DEADTIME: 0
        (0x20, 0x3F),             # CURR_LIM: 400mA
        (0x14, 0xCC),             # TVGAIN0: TVG start 4ms
        (0x15, 0xDD),             # TVGAIN1
        (0x16, 0xEE),             # TVGAIN2
        (0x17, 0x14),             # TVGAIN3: G1 G2
        (0x18, 0x4F),             # TVGAIN4: G3 G4
        (0x19, 0xBF),             # TVGAIN5
        (0x1A, 0xBC),             # TVGAIN6: max gain
    ]:
        rw(ser, a, v)

    # Threshold: T1_CODE sets blind zone, level=0x30
    lv  = 0x3
    thr = ([(T1_CODE<<4)|0x3] + [0x33]*5 +
           [(lv<<4)|lv]*4 + [0x30]*4 + [0x00]*2) * 2
    ser.reset_input_buffer()
    ser.write(build(16, bytes(thr)))
    time.sleep(0.1)
    print(f"  T1=0x{T1_CODE:X}={T1_US}us  level=0x30\n")

    print(f"  {'Lp':>4}  {'TOF(us)':>9}  {'Distance':>10}  {'Peak':>6}  Status")
    print(f"  {'-'*4}  {'-'*9}  {'-'*10}  {'-'*6}  ------")

    dumps=[]; dists=[]; peaks=[]; lnums=[]

    for lp in range(1, N_LOOPS+1):
        tof, w, pk, dump = measure(ser)
        if dump: dumps.append(dump)

        if valid(tof, pk):
            d=tof_m(tof)
            dists.append(d); peaks.append(pk); lnums.append(lp)
            st=f"OK {d:.3f}m"
        else:
            d=None
            st=("blind zone"               if tof and tof<=T1_US else
                "NO ECHO - max range here" if tof and tof>=SENT_US else
                f"weak pk={pk}"            if pk else "timeout")

        ts=str(tof) if tof else "---"
        ds=f"{d:.3f}" if d else "---"
        print(f"  {lp:>4}  {ts:>9}  {ds:>9}m  "
              f"{str(pk) if pk else '---':>6}  {st}")
        time.sleep(0.05)

    ser.close()

    print(f"\n{'='*60}")
    if dists:
        print(f"  Max detected : {max(dists):.3f}m")
        print(f"  Min detected : {min(dists):.3f}m")
        print(f"  Mean         : {np.mean(dists):.3f}m")
        print(f"  Valid        : {len(dists)}/{N_LOOPS}")
    else:
        print(f"  Nothing detected with T1=0x{T1_CODE:X}")

    if not dumps: return

    avg_d = np.mean(dumps, axis=0)
    nf    = float(np.median(avg_d[100:]))
    t_ax  = np.linspace(0, REC_MS, 128)

    plt.style.use('dark_background')
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle(
        f'PGA460-Q1 Max Range | T1=0x{T1_CODE:X} ({T1_US}us) | '
        f'Blind={MIN_M:.2f}m | Record={REC_MS:.0f}ms',
        fontsize=12, fontweight='bold', color='white'
    )
    gs = gridspec.GridSpec(2, 1, figure=fig, hspace=0.4)
    aw = fig.add_subplot(gs[0])
    ad = fig.add_subplot(gs[1])

    FC='#1a1a2e'
    def sty(ax):
        ax.set_facecolor(FC); ax.tick_params(colors='white', labelsize=9)
        ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#444'); ax.spines['bottom'].set_color('#444')

    sty(aw)
    for d in dumps: aw.plot(t_ax, d, color='#a29bfe', alpha=0.04, lw=0.5)
    aw.plot(t_ax, avg_d, color='#a29bfe', lw=2, label=f'Avg ({len(dumps)} bursts)')
    aw.axvspan(0, T1_US/1000, alpha=0.25, color='#ff6b6b')
    aw.axvline(T1_US/1000, color='#ff6b6b', ls='--', lw=2,
               label=f'T1={T1_US}us = {MIN_M:.2f}m blind zone')
    aw.text(T1_US/2000, 255, f'BLIND\n{MIN_M:.2f}m',
            ha='center', va='top', color='#ff6b6b', fontsize=9,
            fontweight='bold', bbox=dict(boxstyle='round', facecolor=FC, alpha=0.8))
    aw.axhline(nf,   color='#636e72', ls=':', lw=1, label=f'Noise={nf:.0f}')
    aw.axhline(0x30, color='#ff9f43', ls='--', lw=1.5, label='Threshold=48/255')
    if dists:
        aw.axvline(max(dists)/V_SOUND*2*1000, color='#00d4ff', ls='--', lw=2,
                   label=f'Max detected {max(dists):.3f}m')

    ax2=aw.twiny(); ax2.set_xlim(aw.get_xlim())
    dm2=list(range(0,12)); tm2=[d/V_SOUND*2*1000 for d in dm2]
    vt=[(t,d) for t,d in zip(tm2,dm2) if t<=REC_MS]
    ax2.set_xticks([t for t,d in vt])
    ax2.set_xticklabels([f'{d}m' for t,d in vt], color='#aaa', fontsize=8)
    ax2.tick_params(colors='#aaa'); ax2.spines['top'].set_color('#444')
    for sp in ['right','left','bottom']: ax2.spines[sp].set_visible(False)

    aw.set_xlabel('Time after burst (ms)', color='white')
    aw.set_ylabel('DSP Amplitude (0-255)', color='white')
    aw.set_xlim(0, REC_MS); aw.set_ylim(0, 270)
    aw.legend(fontsize=8, facecolor=FC, labelcolor='white', loc='upper right')

    sty(ad)
    if dists:
        ad.plot(lnums, dists, 'o', color='#00d4ff', ms=6, alpha=0.8)
        ad.axhline(np.mean(dists), color='#00d4ff', ls='--', lw=2,
                   label=f'Mean={np.mean(dists):.3f}m')
        ad.axhline(MIN_M, color='#ff6b6b', ls=':', lw=1.5,
                   label=f'Blind zone={MIN_M:.2f}m')
        ad.set_title(f'Max={max(dists):.3f}m  Mean={np.mean(dists):.3f}m  '
                     f'Valid={len(dists)}/{N_LOOPS}  T1=0x{T1_CODE:X}',
                     color='white', fontsize=9)
        ad.legend(fontsize=8, facecolor=FC, labelcolor='white')
    else:
        ad.text(0.5, 0.5, 'No detections', ha='center', va='center',
                color='#ff9f43', fontsize=12, transform=ad.transAxes)
    ad.set_xlabel('Loop #', color='white')
    ad.set_ylabel('Distance (m)', color='white')

    fname = f'max_range_T1_{T1_CODE:X}.png'
    plt.savefig(fname, dpi=150, bbox_inches='tight', facecolor='#0d0d1a')
    print(f"\n  Saved: {fname}")
    print("\nClose window to exit.")
    plt.show()

if __name__ == '__main__':
    main()