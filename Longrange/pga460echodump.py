"""
PGA460-Q1  ──  Echo Data Dump + Ringdown Visualiser
Transducer : PUI Audio UTR-1440K-TT-R  (40 kHz)
Interface  : UART (one-wire or RXD/TXD)  19200 baud
Author     : generated for UTR-1440K / PGA460 eval board

Two measurements per loop
  • Preset 1 – short window (4.1 ms)  → shows transducer ringdown
  • Preset 2 – long  window (53.2 ms) → shows long-range echo (up to ~9 m)

Usage
-----
1.  pip install pyserial matplotlib numpy
2.  Set PORT below (Windows "COM3", Linux "/dev/ttyUSB0")
3.  python pga460_echo_dump.py
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ─── USER SETTINGS ────────────────────────────────────────────────────────────
PORT     = "COM12"           # ← change to your port
BAUD     = 115200
N_LOOPS  = 15               # number of echo dumps to average
VSOUND   = 343.0            # m/s  (adjust for temperature if needed)

# Preset 1  (ringdown / short-range  window)
P1_PULSE = 8                # burst pulse count  (fewer = shorter ringdown)
P1_REC   = 0                # record length code  → 4.096*(0+1) = 4.096 ms

# Preset 2  (long-range window)
P2_PULSE = 18               # burst pulse count  (more = stronger echo)
P2_REC   = 12               # record length code  → 4.096*(12+1) = 53.25 ms

# Driver current limit for Preset 1 (CURR_LIM1 code)
# Formula: I = 7*code + 50 mA   →  code=50 gives ~400 mA
CURR_LIM1_CODE = 50         # ~400 mA
# ──────────────────────────────────────────────────────────────────────────────


def checksum(payload: list[int]) -> int:
    """
    PGA460 checksum: inverted byte-sum with carry, over cmd+data fields.
    Sync byte 0x55 is NOT included.
    """
    s = sum(payload)
    while s > 0xFF:
        s = (s & 0xFF) + (s >> 8)
    return (~s) & 0xFF


def send_frame(ser: serial.Serial, payload: list[int]) -> None:
    frame = bytes([0x55] + payload + [checksum(payload)])
    ser.write(frame)
    time.sleep(0.004)           # brief pause for device to process


def write_reg(ser: serial.Serial, addr: int, val: int) -> None:
    """UART command 0x0A – register write (no response)."""
    send_frame(ser, [0x0A, addr, val])
    ser.read(ser.in_waiting)    # discard idle 0xFF bytes


def burst_listen_p1(ser: serial.Serial, n_obj: int = 1) -> None:
    """Command 0x00 – Burst/Listen Preset 1."""
    send_frame(ser, [0x00, n_obj])


def burst_listen_p2(ser: serial.Serial, n_obj: int = 1) -> None:
    """Command 0x01 – Burst/Listen Preset 2."""
    send_frame(ser, [0x01, n_obj])


def read_echo_dump(ser: serial.Serial, wait_s: float = 0.12) -> list[int] | None:
    """
    Command 0x07 – echo data dump read.
    Returns 128 amplitude bytes or None on failure.
    Response frame: diag(1) + data(128) + checksum(1) = 130 bytes.
    """
    send_frame(ser, [0x07])
    time.sleep(wait_s)
    raw = ser.read(130)
    if len(raw) < 130:
        print(f"  ⚠  Short response: got {len(raw)} bytes (expected 130)")
        return None
    return list(raw[1:129])     # strip leading diag byte + trailing checksum


def configure_pga460(ser: serial.Serial) -> None:
    """
    Apply settings for UTR-1440K-TT-R 40 kHz transducer.

    Register map (key ones):
      0x1C  FREQUENCY   – burst frequency  (FREQ = (f_kHz - 30) / 0.2)
      0x1E  PULSE_P1    – Preset1 pulse count + IO/UART config
      0x1F  PULSE_P2    – Preset2 pulse count + UART address
      0x20  CURR_LIM_P1 – Preset1 driver current limit
      0x22  REC_LENGTH  – record time for P1 [7:4] and P2 [3:0]
    """
    print("Configuring PGA460 for 40 kHz …")

    # 40 kHz:  FREQ = (40 - 30) / 0.2 = 50 = 0x32
    write_reg(ser, 0x1C, 0x32)

    # PULSE_P1: IO_IF_SEL=0 | UART_DIAG=0 | IO_DIS=0 | P1_PULSE
    write_reg(ser, 0x1E, P1_PULSE & 0x1F)

    # PULSE_P2: UART_ADDR=0 | P2_PULSE
    write_reg(ser, 0x1F, P2_PULSE & 0x1F)

    # CURR_LIM_P1: DIS_CL=0 | CURR_LIM1 code (bits 5:0)
    write_reg(ser, 0x20, CURR_LIM1_CODE & 0x3F)

    # REC_LENGTH: P1_REC in [7:4], P2_REC in [3:0]
    write_reg(ser, 0x22, ((P1_REC & 0xF) << 4) | (P2_REC & 0xF))

    # BPF centre is auto-set from FREQUENCY register on PGA460 power-up.
    # LPF cutoff is in CURR_LIM_P2[7:6]; default factory = 4 kHz – leave it.

    time.sleep(0.05)
    print("  Done.\n")


def enable_data_dump(ser: serial.Serial) -> None:
    """Set DATADUMP_EN bit in EE_CNTRL register (0x40)."""
    write_reg(ser, 0x40, 0x80)
    time.sleep(0.02)


def disable_data_dump(ser: serial.Serial) -> None:
    write_reg(ser, 0x40, 0x00)


# ─── Time-axis helpers ────────────────────────────────────────────────────────

def record_time_ms(rec_code: int) -> float:
    """Return record window duration in ms for a given P_REC code."""
    return 4.096 * (rec_code + 1)


def time_axis_ms(rec_code: int, n: int = 128) -> np.ndarray:
    return np.linspace(0.0, record_time_ms(rec_code), n)


def dist_axis_m(rec_code: int, n: int = 128) -> np.ndarray:
    t = time_axis_ms(rec_code, n) * 1e-3          # seconds
    return VSOUND * t / 2.0                        # one-way distance


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.3)                                # let device settle

    configure_pga460(ser)
    enable_data_dump(ser)

    p1_traces: list[list[int]] = []
    p2_traces: list[list[int]] = []

    for i in range(N_LOOPS):
        print(f"Loop {i+1:2d}/{N_LOOPS}", end="  ")

        # ── Preset 1 – ringdown / short window ──────────────────────────────
        burst_listen_p1(ser, n_obj=1)
        wait_p1 = record_time_ms(P1_REC) / 1000.0 + 0.05   # window + margin
        d1 = read_echo_dump(ser, wait_s=wait_p1)
        if d1:
            p1_traces.append(d1)
            print("P1 ✓", end="  ")
        else:
            print("P1 ✗", end="  ")

        time.sleep(0.1)

        # ── Preset 2 – long-range ────────────────────────────────────────────
        burst_listen_p2(ser, n_obj=1)
        wait_p2 = record_time_ms(P2_REC) / 1000.0 + 0.05
        d2 = read_echo_dump(ser, wait_s=wait_p2)
        if d2:
            p2_traces.append(d2)
            print("P2 ✓")
        else:
            print("P2 ✗")

        time.sleep(0.15)

    disable_data_dump(ser)
    ser.close()
    print(f"\nCollected  P1: {len(p1_traces)}  P2: {len(p2_traces)}  traces")

    if not p1_traces and not p2_traces:
        print("No data – check PORT and wiring.")
        return

    # ─── Plot ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 10))
    gs  = gridspec.GridSpec(2, 1, hspace=0.45)

    # ── Subplot 1 : Ringdown (Preset 1, short window) ────────────────────────
    ax1 = fig.add_subplot(gs[0])
    t1  = time_axis_ms(P1_REC)        # x in ms (better resolution for ringdown)
    d1x = dist_axis_m(P1_REC)         # secondary x in m

    if p1_traces:
        arr = np.array(p1_traces)
        for row in arr:
            ax1.plot(t1 * 1000, row, alpha=0.25, linewidth=0.7, color='steelblue')
        avg1 = arr.mean(axis=0)
        ax1.plot(t1 * 1000, avg1, color='navy', linewidth=2.0, label=f'Average (n={len(arr)})')

    ax1.set_xlabel("Time after burst end (µs)")
    ax1.set_ylabel("Echo amplitude (8-bit)")
    ax1.set_title("Transducer ringdown  –  UTR-1440K-TT-R 40 kHz\n"
                  f"Preset 1 | window = {record_time_ms(P1_REC):.1f} ms | {P1_PULSE} pulses")
    ax1.legend(loc='upper right')
    ax1.grid(True, linewidth=0.4, alpha=0.6)
    ax1.set_xlim(0, record_time_ms(P1_REC) * 1000)

    # Secondary x-axis: distance in m
    ax1b = ax1.twiny()
    ax1b.set_xlim(ax1.get_xlim())
    dist_ticks = np.linspace(0, d1x[-1], 6)
    time_ticks = dist_ticks * 2 / VSOUND * 1e6    # µs
    ax1b.set_xticks(time_ticks)
    ax1b.set_xticklabels([f"{d:.2f} m" for d in dist_ticks])
    ax1b.set_xlabel("Distance (m)")

    # Annotate ringdown region
    ax1.axvspan(0, 1500, alpha=0.08, color='red', label='Typical ringdown zone')
    ax1.text(750, ax1.get_ylim()[1] * 0.95, "ringdown zone",
             ha='center', va='top', fontsize=8, color='darkred')

    # ── Subplot 2 : Long range (Preset 2) ────────────────────────────────────
    ax2  = fig.add_subplot(gs[1])
    dist2 = dist_axis_m(P2_REC)

    if p2_traces:
        arr2 = np.array(p2_traces)
        colors = plt.cm.tab10(np.linspace(0, 1, len(arr2)))
        for idx, row in enumerate(arr2):
            ax2.plot(dist2, row, alpha=0.3, linewidth=0.7, color=colors[idx])
        avg2 = arr2.mean(axis=0)
        ax2.plot(dist2, avg2, color='black', linewidth=2.0,
                 label=f'Average (n={len(arr2)})')

    ax2.set_xlabel("Distance (m)")
    ax2.set_ylabel("Echo amplitude (8-bit)")
    ax2.set_title("Long-range echo dump  –  UTR-1440K-TT-R 40 kHz\n"
                  f"Preset 2 | window = {record_time_ms(P2_REC):.1f} ms | {P2_PULSE} pulses"
                  f"  →  max range ≈ {dist_axis_m(P2_REC)[-1]:.1f} m")
    ax2.legend(loc='upper right')
    ax2.grid(True, linewidth=0.4, alpha=0.6)
    ax2.set_xlim(0, dist2[-1])

    # Reference lines at common distances
    for d_ref, lbl in [(1, "1 m"), (2, "2 m"), (3, "3 m"), (4, "4 m"),
                       (5, "5 m"), (6, "6 m")]:
        if d_ref <= dist2[-1]:
            ax2.axvline(d_ref, color='gray', linewidth=0.6, linestyle='--', alpha=0.5)
            ax2.text(d_ref + 0.05, ax2.get_ylim()[1] * 0.97, lbl,
                     fontsize=7, color='gray', va='top')

    plt.suptitle(f"PGA460-Q1  ·  UTR-1440K-TT-R 40 kHz  ·  UART {BAUD} baud",
                 fontsize=12, fontweight='bold')

    plt.savefig("pga460_echo_dump.png", dpi=150, bbox_inches='tight')
    print("Saved:  pga460_echo_dump.png")
    plt.show()


if __name__ == "__main__":
    main()