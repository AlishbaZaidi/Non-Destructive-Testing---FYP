"""
PGA460-Q1 — Final Clean Distance Measurement
=============================================
Uses CMD7 waveform peak as primary distance measurement.
CMD5 TOF as secondary comparison.

IMPORTANT: Measure TARGET_M from the METAL FACE of the transducer
           (the silver cylinder on the board) to the block surface.

USAGE:  python final_measurement.py
"""

import serial, time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

COM_PORT  = 'COM12'
BAUD_RATE = 115200
UART_ADDR = 0
TEMP_C    = 25.0
TARGET_M  = 2.0
N_LOOPS   = 30

V_SOUND = 331.0 + 0.6 * TEMP_C
P1_REC  = 5
REC_MS  = 4.096 * (P1_REC + 1)
SAMP_MS = REC_MS / 128.0
SAMP_US = SAMP_MS * 1000
RING_MS = 4.0
RING_S  = int(RING_MS / SAMP_MS)
ECHO_MS = TARGET_M / V_SOUND * 2 * 1000
SENT_US = (REC_MS - SAMP_MS) * 1000

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
    ser.reset_input_buffer(); ser.write(build(10, bytes([a, v]))); time.sleep(0.06)

def burst(ser):
    ser.reset_input_buffer(); ser.write(build(0, bytes([1])))
    time.sleep(REC_MS/1000 + 0.1); ser.reset_input_buffer()

def get_dump(ser):
    ser.write(build(7)); time.sleep(0.3)
    r = rx(ser, 130, ms=1000)
    return list(r[1:129]) if len(r) == 130 else None

def get_tof(ser):
    ser.write(build(5)); time.sleep(0.08)
    r = rx(ser, 6)
    if len(r) == 6:
        tof = (r[1] << 8) | r[2]
        return tof, r[3], r[4]
    return None

def sample_to_m(i): return V_SOUND * (i + 0.5) * SAMP_US * 1e-6 / 2.0

def find_peak(samples, ring_s, nf):
    arr = np.array(samples, dtype=float)
    thr = max(nf * 1.5, nf + 15.0)
    s, e = ring_s + 1, int(len(arr) * 0.95)
    if s >= e or max(arr[s:e]) < thr:
        return None, None, None
    pv = float(max(arr[s:e]))
    pi = s + int(np.argmax(arr[s:e]))
    return pi, pv, sample_to_m(pi)

def main():
    print("=" * 60)
    print(f"  PGA460-Q1 Final Measurement | Target={TARGET_M}m")
    print(f"  Record={REC_MS:.1f}ms  Echo expected@{ECHO_MS:.1f}ms")
    print("=" * 60)
    input("  Press ENTER when ready...")

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, bytesize=8,
                            parity='N', stopbits=2, timeout=0.5)
    except Exception as e:
        print(f"[ERROR] {e}"); return
    time.sleep(0.3)

    # Config
    for a, v in [(0x1C,0x32),(0x1E,0x48),(0x22,(P1_REC<<4)|0x0),
                  (0x1B,0x10),(0x26,0x10),(0x1D,0x00),(0x20,0x3F),
                  (0x14,0xCC),(0x15,0xDD),(0x16,0xEE),(0x17,0x14),
                  (0x18,0x4F),(0x19,0xBF),(0x1A,0xBC)]:
        rw(ser, a, v)

    # Thresholds: T1=5200µs, level=0x20
    thr = ([0xD3]+[0x33]*5+[0x22]*4+[0x20]*4+[0x00]*2)*2
    ser.reset_input_buffer(); ser.write(build(16, bytes(thr))); time.sleep(0.1)
    print(f"  Config done. Thresholds T1=5200µs level=0x20\n")

    # Baseline noise
    burst(ser); d0 = get_dump(ser)
    nf0 = float(np.median(np.array(d0)[100:])) if d0 else 50.0

    dumps=[]; wav_d=[]; wav_p=[]; c5d=[]; c5p=[]; lwav=[]; lc5=[]

    print(f"  {'Lp':>4}  {'WAV_m':>8}  {'WAV_pk':>7}  {'CMD5_m':>8}  {'CMD5_pk':>8}")
    print(f"  {'-'*4}  {'-'*8}  {'-'*7}  {'-'*8}  {'-'*8}")

    for lp in range(1, N_LOOPS+1):
        burst(ser); dump = get_dump(ser)
        burst(ser); tr   = get_tof(ser)

        wd=wp=None; cd=cp=None
        if dump:
            dumps.append(dump)
            nf  = float(np.median(np.array(dump)[100:]))
            pi, pv, pd = find_peak(dump, RING_S, max(nf, nf0))
            if pi is not None:
                wd=pd; wp=pv; wav_d.append(wd); wav_p.append(wp); lwav.append(lp)

        if tr:
            tu, w, pk = tr; pk=int(pk)
            if RING_MS*1000 < tu < SENT_US and pk > 5:
                cd = V_SOUND*tu*1e-6/2; cp=pk
                c5d.append(cd); c5p.append(cp); lc5.append(lp)

        ws=f"{wd:.4f}" if wd else "---"
        wk=f"{wp:.0f}" if wp else "---"
        cs=f"{cd:.4f}" if cd else "---"
        ck=f"{cp}" if cp else "---"
        print(f"  {lp:>4}  {ws:>8}  {wk:>7}  {cs:>8}  {ck:>8}")
        time.sleep(0.05)

    ser.close()

    avg_d = np.mean(dumps, axis=0) if dumps else np.zeros(128)
    nf_f  = float(np.median(avg_d[100:]))

    print(f"\n{'='*60}"); print(f"  RESULTS"); print(f"{'='*60}")
    print(f"  Noise floor : {nf_f:.1f}")
    print(f"  Ringdown    : {RING_MS}ms = {V_SOUND*RING_MS*1e-3/2:.2f}m")
    if wav_d:
        m, s = np.mean(wav_d), np.std(wav_d)
        print(f"\n  CMD7 Waveform Peak:")
        print(f"    Distance : {m:.4f}m ± {s*100:.2f}cm")
        print(f"    Target   : {TARGET_M}m")
        print(f"    Error    : {(m-TARGET_M)*100:+.1f}cm")
        print(f"    Peak amp : {np.mean(wav_p):.0f}/255  "
              f"SNR={np.mean(wav_p)/max(nf_f,1):.1f}:1")
        print(f"    Valid    : {len(wav_d)}/{N_LOOPS}")
        if abs((m-TARGET_M)*100) > 15:
            print(f"\n  NOTE: {abs((m-TARGET_M)*100):.0f}cm error detected.")
            print(f"  The board detected distance = {m:.4f}m")
            print(f"  This IS the real distance if you measure from transducer face.")
    if c5d:
        print(f"\n  CMD5 TOF: {np.mean(c5d):.4f}m  valid={len(c5d)}/{N_LOOPS}")

    if not dumps: return

    # ── Plot ──────────────────────────────────────────────────────────────────
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(15, 9))
    fig.suptitle(
        f'PGA460-Q1 — Final Distance Measurement\n'
        f'UTR-1440K-TT-R 40kHz | Target={TARGET_M}m | '
        f'Waveform peak method | {N_LOOPS} bursts',
        fontsize=12, fontweight='bold', color='white', y=0.98)
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.35)
    aw=fig.add_subplot(gs[0,:]); ad=fig.add_subplot(gs[1,0]); ap=fig.add_subplot(gs[1,1])
    FC='#1a1a2e'
    def sty(ax):
        ax.set_facecolor(FC); ax.tick_params(colors='white',labelsize=9)
        ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#444'); ax.spines['bottom'].set_color('#444')

    t_ax = np.linspace(0, REC_MS, 128)
    sty(aw)
    for d in dumps: aw.plot(t_ax, d, color='#a29bfe', alpha=0.05, lw=0.5)
    aw.plot(t_ax, avg_d, color='#a29bfe', lw=2.5, label=f'Avg ({len(dumps)} bursts)')
    aw.axvspan(0, RING_MS, alpha=0.2, color='#ff6b6b')
    aw.axvline(RING_MS, color='#ff6b6b', ls='--', lw=2,
               label=f'Ringdown {RING_MS}ms={V_SOUND*RING_MS*1e-3/2:.2f}m')
    aw.axhline(nf_f, color='#636e72', ls=':', lw=1, label=f'Noise={nf_f:.0f}')
    aw.axvline(ECHO_MS, color='#55efc4', ls=':', lw=1.5,
               label=f'Target {TARGET_M}m @ {ECHO_MS:.1f}ms')
    if wav_d:
        dm = np.mean(wav_d)/V_SOUND*2*1000
        aw.axvline(dm, color='#00d4ff', ls='--', lw=2.5,
                   label=f'Detected {np.mean(wav_d):.4f}m @ {dm:.1f}ms')
        pi,pv,_ = find_peak(avg_d.tolist(), RING_S, nf_f)
        if pi:
            aw.plot(t_ax[pi], avg_d[pi], 'o', color='#00d4ff', ms=14, zorder=5)
            aw.annotate(
                f'ECHO\n{np.mean(wav_d):.4f}m\npk={avg_d[pi]:.0f}\n'
                f'SNR={avg_d[pi]/max(nf_f,1):.1f}:1',
                xy=(t_ax[pi], avg_d[pi]),
                xytext=(t_ax[pi]+1.5, avg_d[pi]-35),
                color='#00d4ff', fontsize=9, fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='#00d4ff', lw=1.5),
                bbox=dict(boxstyle='round', facecolor=FC, alpha=0.9))
    if c5d:
        dm5=np.mean(c5d)/V_SOUND*2*1000
        aw.axvline(dm5, color='#fdcb6e', ls=':', lw=1.5,
                   label=f'CMD5 {np.mean(c5d):.4f}m @ {dm5:.1f}ms')

    ax2=aw.twiny(); ax2.set_xlim(aw.get_xlim())
    dm2=[0,0.5,1,1.5,2,2.5,3,4]; tm2=[d/V_SOUND*2*1000 for d in dm2]
    vt=[(t,d) for t,d in zip(tm2,dm2) if t<=REC_MS]
    ax2.set_xticks([t for t,d in vt]); ax2.set_xticklabels([f'{d}m' for t,d in vt],color='#aaa',fontsize=8)
    ax2.tick_params(colors='#aaa'); ax2.spines['top'].set_color('#333')
    for sp in ['right','left','bottom']: ax2.spines[sp].set_visible(False)

    aw.set_xlabel('Time (ms)',color='white',fontsize=10)
    aw.set_ylabel('DSP Amplitude (0-255)',color='white',fontsize=10)
    aw.set_title('Echo Data Dump — Waveform Peak Analysis',color='white')
    aw.set_xlim(0,REC_MS); aw.set_ylim(0,270)
    aw.legend(fontsize=8,facecolor=FC,labelcolor='white',loc='upper right',ncol=2)

    sty(ad)
    if wav_d: ad.plot(lwav,wav_d,'o',color='#00d4ff',ms=6,label='CMD7')
    if c5d:   ad.plot(lc5, c5d, 's',color='#fdcb6e',ms=6,label='CMD5')
    ad.axhline(TARGET_M,color='#ff9f43',ls=':',lw=1.5,label=f'Target={TARGET_M}m')
    if wav_d: ad.axhline(np.mean(wav_d),color='#00d4ff',ls='--',lw=2,
                         label=f'Mean={np.mean(wav_d):.4f}m')
    ad.set_xlabel('Loop #',color='white'); ad.set_ylabel('Distance (m)',color='white')
    ad.set_title(f'Distance — CMD7={np.mean(wav_d):.4f}m Err={( np.mean(wav_d)-TARGET_M)*100:+.1f}cm'
                 if wav_d else 'No echo', color='white', fontsize=9)
    ad.legend(fontsize=7,facecolor=FC,labelcolor='white')

    sty(ap)
    if wav_p: ap.bar(lwav,wav_p,color='#00d4ff',alpha=0.8,width=0.6,label='CMD7')
    if c5p:   ap.bar(lc5,c5p,  color='#fdcb6e',alpha=0.6,width=0.4,label='CMD5')
    ap.axhline(nf_f,color='#636e72',ls=':',lw=1,label=f'Noise={nf_f:.0f}')
    pk_avg = np.mean(wav_p) if wav_p else 0
    ap.set_title(f'Peak={pk_avg:.0f}/255  SNR={pk_avg/max(nf_f,1):.1f}:1',
                 color='white',fontsize=9)
    ap.set_xlabel('Loop #',color='white'); ap.set_ylabel('Peak (0-255)',color='white')
    ap.set_ylim(0,270); ap.legend(fontsize=7,facecolor=FC,labelcolor='white')

    footer = (
        (f"CMD7={np.mean(wav_d):.4f}m ±{np.std(wav_d)*100:.2f}cm "
         f"Err={( np.mean(wav_d)-TARGET_M)*100:+.1f}cm  |  " if wav_d else "No CMD7 echo  |  ")
        + (f"CMD5={np.mean(c5d):.4f}m" if c5d else "No CMD5 TOF")
    )
    fig.text(0.5,0.005,footer,ha='center',fontsize=9,
             color='#55efc4',fontweight='bold',
             bbox=dict(boxstyle='round',facecolor='#0d0d1a',alpha=0.8))

    plt.savefig('final_measurement-4.png',dpi=150,bbox_inches='tight',facecolor='#0d0d1a')
    print(f"\n  Saved: final_measurement-3.png")
    print("\nClose window to exit.")
    plt.show()

if __name__ == '__main__':
    main()