import serial
import time

def test_pga460_uart(port='COM12', baudrate=9600):
    print(f"Testing PGA460-Q1 on {port} at {baudrate} baud...")
    
    try:
        # Open serial port with explicit settings
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,  # IMPORTANT: 2 stop bits!
            timeout=1,
            write_timeout=1
        )
        print(f"Serial port {port} opened successfully")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return
    
    # Wait for device to stabilize
    time.sleep(0.1)
    
    def calculate_checksum(data):
        """Calculate inverted 8-bit sum of bytes"""
        total = sum(data) & 0xFF
        return (~total) & 0xFF
    
    def send_and_receive(cmd_bytes, expected_len=0):
        """Send command and receive response with timeout"""
        print(f"Sending: {bytes(cmd_bytes).hex(' ').upper()}")
        
        # Clear any pending data
        ser.reset_input_buffer()
        
        # Send command
        ser.write(bytes(cmd_bytes))
        ser.flush()
        
        # Wait for response
        time.sleep(0.05)  # Increased wait time
        
        # Read response
        response = ser.read(ser.in_waiting or 100)
        
        if response:
            print(f"Received ({len(response)} bytes): {response.hex(' ').upper()}")
        else:
            print("No response received")
        
        return response
    
    # Test 1: Send sync pattern first (optional but helps sync)
    print("\n--- Test 1: Sending sync pattern ---")
    sync_pattern = bytes([0x55, 0x55, 0x55])
    send_and_receive(sync_pattern)
    time.sleep(0.1)
    
    # Test 2: Simple register read (DEV_STAT0 at 0x4C)
    print("\n--- Test 2: Reading DEV_STAT0 register ---")
    cmd = [0x55, 0x09, 0x4C]
    cmd.append(calculate_checksum([0x09, 0x4C]))
    response = send_and_receive(cmd)
    
    if len(response) >= 3:
        print(f"✓ Device responded with {len(response)} bytes")
        diagnostic = response[0]
        reg_value = response[1]
        
        # Check diagnostic byte format (bits 7:6 should be 01 for PGA460)
        if (diagnostic & 0xC0) == 0x40:
            print("✓ Diagnostic byte format correct (01xxxxxx)")
        else:
            print(f"⚠ Unexpected diagnostic format: 0x{diagnostic:02X}")
        
        print(f"  DEV_STAT0 = 0x{reg_value:02X}")
        print(f"  REV_ID = {(reg_value >> 6) & 0x03}")
        print(f"  OPT_ID = {(reg_value >> 4) & 0x03}")
    else:
        print("✗ Incomplete response")
    
    # Test 3: Try temperature measurement (simple command)
    print("\n--- Test 3: Temperature measurement command ---")
    cmd = [0x55, 0x04, 0x00]  # Temperature measurement
    cmd.append(calculate_checksum([0x04, 0x00]))
    send_and_receive(cmd)
    
    # Wait for measurement
    time.sleep(0.1)
    
    # Read temperature result
    print("\n--- Test 4: Reading temperature result ---")
    cmd = [0x55, 0x06]
    cmd.append(calculate_checksum([0x06]))
    send_and_receive(cmd)
    
    # Test 5: Direct echo test (listen only command)
    print("\n--- Test 5: Listen Only command ---")
    cmd = [0x55, 0x02, 0x01]  # Listen Only Preset1, 1 object
    cmd.append(calculate_checksum([0x02, 0x01]))
    send_and_receive(cmd)
    
    # Close serial port
    ser.close()
    print("\nTest completed.")

# Run the test
test_pga460_uart('COM12', 115200)