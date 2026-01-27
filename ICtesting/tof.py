import serial, time

PORT='COM12'
BAUD=115200

def cs(d): return (~sum(d)) & 0xFF

ser = serial.Serial(PORT, BAUD, bytesize=8, stopbits=2, parity='N', timeout=1)
time.sleep(0.1)

# Burst + Listen
cmd = [0x55, 0x00, 0x01]
cmd.append(cs(cmd[1:]))
ser.write(bytes(cmd))
time.sleep(0.2)

# Read TOF
cmd = [0x55, 0x05]
cmd.append(cs(cmd[1:]))
ser.write(bytes(cmd))
time.sleep(0.05)
resp = ser.read(6)

print("TOF response:", resp.hex())
ser.close()
