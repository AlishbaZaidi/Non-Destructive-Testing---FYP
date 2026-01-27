import serial
import time
import matplotlib.pyplot as plt

# -------------------------------
# CONFIGURATION
# -------------------------------
PORT = 'COM12'          # Change if needed
BAUDRATE = 115200

# -------------------------------
# CHECKSUM FUNCTION
# -------------------------------
def calculate_checksum(data):
    total = sum(data) & 0xFF
    return (~total) & 0xFF

# -------------------------------
# SEND AND RECEIVE FUNCTION
# -------------------------------
def send_and_receive(ser, cmd_bytes, read_delay=0.05):
    print(f"Sending: {bytes(cmd_bytes).hex(' ').upper()}")

    ser.reset_input_buffer()
    ser.write(bytes(cmd_bytes))
    ser.flush()

    time.sleep(read_delay)

    response = ser.read(ser.in_waiting)

    if response:
        print(f"Received ({len(response)} bytes): {response.hex(' ').upper()}")
    else:
        print("No response received")

    return response

# -------------------------------
# MAIN PROGRAM
# -------------------------------
def main():
    print(f"Opening serial port {PORT} at {BAUDRATE} baud...")

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        timeout=1,
        write_timeout=1
    )

    time.sleep(0.1)
    print("Serial port opened.\n")

    # ===================================================
    # STEP 1: ENABLE HIGH-FREQUENCY MODE (FREQ_SHIFT = 1)
    # Register: FREQ_CTRL (0x03)
    # Bit 0 = FREQ_SHIFT
    # ===================================================
    print("=== STEP 1: ENABLE HIGH FREQUENCY MODE ===")
    # Write register 0x03 with value 0x01
    hf_cmd = [0x55, 0x40, 0x03, 0x01]
    hf_cmd.append(calculate_checksum([0x40, 0x03, 0x01]))
    send_and_receive(ser, hf_cmd)

    time.sleep(0.05)

    # ===================================================
    # STEP 2: SET TX CENTER FREQUENCY ≈ 400 kHz
    # Register: FREQ (0x04)
    # Value for ~400 kHz ≈ 0x9C (example)
    # ===================================================
    print("\n=== STEP 2: SET FREQUENCY TO ~400 kHz ===")
    # Write register 0x04 with value 0x9C (approx 400 kHz)
    freq_cmd = [0x55, 0x40, 0x04, 0x9C]
    freq_cmd.append(calculate_checksum([0x40, 0x04, 0x9C]))
    send_and_receive(ser, freq_cmd)

    time.sleep(0.1)

    # ===================================================
    # STEP 3: LISTEN ONLY
    # Command: 0x02, Preset 1, 1 object
    # ===================================================
    print("\n=== STEP 3: LISTEN FOR ULTRASONIC SIGNAL ===")
    listen_cmd = [0x55, 0x02, 0x01]
    listen_cmd.append(calculate_checksum([0x02, 0x01]))
    send_and_receive(ser, listen_cmd)

    # Wait for echo capture
    time.sleep(0.3)

    # ===================================================
    # STEP 4: READ ECHO DATA
    # Command: 0x0A, start address 0x00
    # ===================================================
    print("\n=== STEP 4: READ ECHO DATA ===")
    read_echo_cmd = [0x55, 0x0A, 0x00]
    read_echo_cmd.append(calculate_checksum([0x0A, 0x00]))
    response = send_and_receive(ser, read_echo_cmd, read_delay=0.1)

    ser.close()

    # ===================================================
    # STEP 5: PLOT DATA
    # ===================================================
    if len(response) < 3:
        print("\n❌ No echo data received.")
        return

    print("\n=== STEP 5: PLOTTING ECHO DATA ===")

    # Remove diagnostic byte and checksum
    echo_data = list(response[1:-1])

    print(f"Echo samples received: {len(echo_data)}")

    plt.figure(figsize=(10, 5))
    plt.plot(echo_data)
    plt.title("Ultrasonic Echo Signal (PGA460-Q1 @ 400 kHz)")
    plt.xlabel("Sample Index (Time)")
    plt.ylabel("Amplitude")
    plt.grid(True)
    plt.show()

# -------------------------------
# RUN
# -------------------------------
if __name__ == "__main__":
    main()
