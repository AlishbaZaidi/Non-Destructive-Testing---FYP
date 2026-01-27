import serial
import time

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

# ==============================
# STEP 1: ENABLE HIGH FREQ MODE
# ==============================
print("=== STEP 1: Enable High Frequency Mode (FREQ_SHIFT) ===")

# Read TVGAIN6
cmd = [0x55, 0x09, 0x1A]
cmd.append(checksum(cmd[1:]))
resp = send(ser, cmd)

print("TVGAIN6 Read:", resp)

if len(resp) < 3:
    print("❌ No response, UART problem")
    ser.close()
    exit()

current = resp[1]
print(f"Current TVGAIN6 = 0x{current:02X}")

# Set FREQ_SHIFT bit (bit 0)
new_val = current | 0x01
cmd = [0x55, 0x0A, 0x1A, new_val]
cmd.append(checksum(cmd[1:]))

send(ser, cmd)
print(f"TVGAIN6 updated to 0x{new_val:02X} (High frequency mode enabled)")

# ==============================
# STEP 2: SET FREQUENCY = 400 kHz
# ==============================
print("\n=== STEP 2: Set Frequency to 400 kHz ===")

# FREQ register = 0x1C
# 400 kHz → FREQ = 183 = 0xB7
cmd = [0x55, 0x0A, 0x1C, 0xB7]
cmd.append(checksum(cmd[1:]))

send(ser, cmd)
print("FREQ register written: 0xB7")

# Verify
cmd = [0x55, 0x09, 0x1C]
cmd.append(checksum(cmd[1:]))
resp = send(ser, cmd)
print("FREQ Readback:", resp)

# ==============================
# STEP 3: CONFIGURE BAND-PASS FILTER
# ==============================
print("\n=== STEP 3: Configure Band-Pass Filter for 400 kHz ===")

bpf_values = {
    0x41: 0x7F,  # BPF_A2_MSB
    0x42: 0xFF,  # BPF_A2_LSB
    0x43: 0x00,  # BPF_A3_MSB
    0x44: 0x00,  # BPF_A3_LSB
    0x45: 0x7F,  # BPF_B1_MSB
    0x46: 0xFF,  # BPF_B1_LSB
}

for addr, val in bpf_values.items():
    cmd = [0x55, 0x0A, addr, val]
    cmd.append(checksum(cmd[1:]))
    send(ser, cmd)
    print(f"Wrote 0x{val:02X} to register 0x{addr:02X}")

ser.close()
print("\n✅ 400 kHz mode + BPF configuration complete")
