import serial, time

PORT = 'COM12'
BAUD = 115200

def checksum(data):
    return (~sum(data)) & 0xFF

def send(ser, cmd, delay=0.05):
    ser.reset_input_buffer()
    ser.write(bytes(cmd))
    ser.flush()
    time.sleep(delay)
    return ser.read(ser.in_waiting)

ser = serial.Serial(PORT, BAUD, bytesize=8, stopbits=2, parity='N', timeout=1)
time.sleep(0.1)

# ----------------------------
# STEP 1: Enable HF mode
# ----------------------------
print("Enabling High Frequency Mode...")
cmd = [0x55, 0x09, 0x1A]
cmd.append(checksum(cmd[1:]))
resp = send(ser, cmd)
current = resp[1]

new_val = current | 0x01  # set FREQ_SHIFT
cmd = [0x55, 0x0A, 0x1A, new_val]
cmd.append(checksum(cmd[1:]))
send(ser, cmd)
print(f"TVGAIN6 = 0x{new_val:02X}")

# ----------------------------
# STEP 2: Set 300 kHz
# ----------------------------
print("Setting frequency to 300 kHz...")
cmd = [0x55, 0x0A, 0x1C, 0x64]  # FREQ = 100
cmd.append(checksum(cmd[1:]))
send(ser, cmd)

cmd = [0x55, 0x09, 0x1C]
cmd.append(checksum(cmd[1:]))
print("FREQ readback:", send(ser, cmd))

# ----------------------------
# STEP 3: BPF for 300 kHz
# ----------------------------
print("Configuring Band-Pass Filter for 300 kHz...")

bpf = {
    0x41: 0x6A,
    0x42: 0xC0,
    0x43: 0x00,
    0x44: 0x00,
    0x45: 0x6A,
    0x46: 0xC0,
}

for addr, val in bpf.items():
    cmd = [0x55, 0x0A, addr, val]
    cmd.append(checksum(cmd[1:]))
    send(ser, cmd)
    print(f"BPF 0x{addr:02X} = 0x{val:02X}")

ser.close()
print("\n✅ 300 kHz configuration complete.")
