import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

class PGA460Receiver:
    def __init__(self, port='COM12', baudrate=115200):
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        
    def checksum(self, data):
        """Calculate PGA460 checksum (same as your 300kHz code)"""
        return (~sum(data)) & 0xFF
    
    def send_command(self, cmd_bytes, delay=0.05):
        """Send command and read response"""
        if self.ser is None:
            print("Not connected!")
            return b''
        
        self.ser.reset_input_buffer()
        self.ser.write(bytes(cmd_bytes))
        self.ser.flush()
        time.sleep(delay)
        
        # Read available bytes
        response = self.ser.read(self.ser.in_waiting)
        return response
    
    def connect(self):
        """Connect to PGA460"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_TWO,  # PGA460 uses 2 stop bits
                parity=serial.PARITY_NONE,
                timeout=1
            )
            time.sleep(0.1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def set_to_80khz_default(self):
        """
        Configure PGA460 for 80kHz operation using default settings
        Based on PGA460 datasheet register map
        """
        print("\n[1/6] Configuring for 80kHz default mode...")
        
        # Set to Ultrasonic Mode Preset 1 (default-ish settings)
        # Command: 0x55, 0x05, PRESET, CHKSUM
        preset_cmd = [0x55, 0x05, 0x01]  # Preset 1
        preset_cmd.append(self.checksum(preset_cmd[1:]))
        
        resp = self.send_command(preset_cmd, delay=0.1)
        print(f"Preset set response: {resp.hex() if resp else 'No response'}")
        
        # Adjust frequency-related registers for 80kHz
        # These are example values - check PGA460 datasheet Table 7-5
        # FREQ_HI and FREQ_LO registers control frequency
        
        # For 80kHz, we need to calculate register values
        # PGA460 frequency formula: f = 1/(2 * (FREQ_REG + 1) * 125ns)
        # FREQ_REG = (1/(2 * f * 125e-9)) - 1
        
        target_freq = 80000  # 80 kHz
        freq_reg = int((1/(2 * target_freq * 125e-9)) - 1)
        
        # Split into high and low bytes
        freq_hi = (freq_reg >> 8) & 0xFF
        freq_lo = freq_reg & 0xFF
        
        print(f"80kHz Frequency Register: 0x{freq_reg:04X} (HI: 0x{freq_hi:02X}, LO: 0x{freq_lo:02X})")
        
        # Write to FREQ_LO register (address 0x14)
        write_freq_lo = [0x55, 0x04, 0x14, freq_lo]
        write_freq_lo.append(self.checksum(write_freq_lo[1:]))
        
        resp = self.send_command(write_freq_lo, delay=0.05)
        print(f"FREQ_LO write response: {resp.hex() if resp else 'No response'}")
        
        # Write to FREQ_HI register (address 0x15)
        write_freq_hi = [0x55, 0x04, 0x15, freq_hi]
        write_freq_hi.append(self.checksum(write_freq_hi[1:]))
        
        resp = self.send_command(write_freq_hi, delay=0.05)
        print(f"FREQ_HI write response: {resp.hex() if resp else 'No response'}")
        
        # Set to minimum gain for initial testing (address 0x0C = P1_GAIN)
        min_gain = 0x00  # Minimum gain
        write_gain = [0x55, 0x04, 0x0C, min_gain]
        write_gain.append(self.checksum(write_gain[1:]))
        
        resp = self.send_command(write_gain, delay=0.05)
        print(f"Gain set to minimum response: {resp.hex() if resp else 'No response'}")
        
        return True
    
    def listen_for_echo(self):
        """Put PGA460 in listen mode (same as your 300kHz code)"""
        print("\n[2/6] Setting to listen mode...")
        listen_cmd = [0x55, 0x02, 0x01]  # Listen command
        listen_cmd.append(self.checksum(listen_cmd[1:]))
        
        resp = self.send_command(listen_cmd, delay=0.3)
        print(f"Listen response: {resp.hex() if resp else 'No response'}")
        return resp
    
    def read_echo_samples(self, num_samples=128):
        """Read echo data from PGA460 RAM"""
        print(f"\n[3/6] Reading {num_samples} echo samples...")
        
        # Command: 0x55, 0x0B, START_ADDR_HI, START_ADDR_LO
        # Start address 0x0080 (128 in decimal) for echo data
        
        start_addr = 0x0080
        addr_hi = (start_addr >> 8) & 0xFF
        addr_lo = start_addr & 0xFF
        
        read_cmd = [0x55, 0x0B, addr_hi, addr_lo]
        read_cmd.append(self.checksum(read_cmd[1:]))
        
        resp = self.send_command(read_cmd, delay=0.1)
        
        if resp:
            # First byte is echo of command (0xFF for success)
            echo_data = resp[1:]  # Skip first byte
            print(f"Received {len(echo_data)} bytes")
            return echo_data
        else:
            print("No response from read command!")
            return b''
    
    def trigger_burst(self):
        """Trigger a single ultrasonic burst (optional)"""
        print("\n[Optional] Triggering burst for echo...")
        # Command for single burst: 0x55, 0x07, 0x01 (Preset 1)
        burst_cmd = [0x55, 0x07, 0x01]
        burst_cmd.append(self.checksum(burst_cmd[1:]))
        
        resp = self.send_command(burst_cmd, delay=0.5)  # Longer delay for echo
        print(f"Burst response: {resp.hex() if resp else 'No response'}")
        return resp
    
    def print_samples_detailed(self, samples, max_print=20):
        """Print samples in hex and decimal"""
        if not samples or len(samples) == 0:
            print("No samples to display!")
            return
        
        print("\n" + "="*60)
        print("RAW ECHO DATA (First 20 bytes):")
        print("="*60)
        
        # Print hex
        for i in range(0, min(len(samples), max_print)):
            print(f"Byte[{i:03d}]: 0x{samples[i]:02X}")
        
        print("\n" + "="*60)
        print(f"DECIMAL VALUES (First {max_print} samples):")
        print("="*60)
        
        # Print decimal
        for i in range(0, min(len(samples), max_print)):
            # PGA460 returns 8-bit samples (0-255)
            voltage = (samples[i] / 255) * 3.3  # Assuming 3.3V reference
            print(f"Sample[{i:03d}]: {samples[i]:3d} (≈ {voltage:.3f} V)")
        
        # Statistics
        print("\n" + "="*60)
        print("STATISTICS:")
        print("="*60)
        print(f"Total samples: {len(samples)}")
        if len(samples) > 0:
            print(f"Min value: {min(samples)}")
            print(f"Max value: {max(samples)}")
            print(f"Average: {np.mean(samples):.2f}")
            print(f"Std dev: {np.std(samples):.2f}")
            print(f"Non-zero samples: {sum(1 for x in samples if x > 0)}")
    
    def plot_echo_signal(self, samples, title="80kHz Ultrasonic Echo"):
        """Plot the echo signal"""
        if not samples or len(samples) == 0:
            print("No data to plot!")
            return
        
        # Convert to numpy array
        echo_array = np.array(samples)
        
        # Create figure
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        
        # Time domain plot
        axes[0, 0].plot(echo_array, 'b-', linewidth=1, marker='o', markersize=2)
        axes[0, 0].set_xlabel('Sample Index')
        axes[0, 0].set_ylabel('Amplitude (0-255)')
        axes[0, 0].set_title(f'{title}\n{len(samples)} samples')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Histogram
        axes[0, 1].hist(echo_array, bins=20, edgecolor='black', alpha=0.7)
        axes[0, 1].set_xlabel('Amplitude')
        axes[0, 1].set_ylabel('Count')
        axes[0, 1].set_title('Amplitude Distribution')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Zoomed view (first 50 samples)
        zoom_samples = min(50, len(echo_array))
        axes[1, 0].plot(range(zoom_samples), echo_array[:zoom_samples], 
                       'r-', linewidth=1.5, marker='s', markersize=3)
        axes[1, 0].set_xlabel('Sample Index')
        axes[1, 0].set_ylabel('Amplitude')
        axes[1, 0].set_title(f'First {zoom_samples} Samples (Zoomed)')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Check for periodic signal (autocorrelation)
        if len(echo_array) > 10:
            autocorr = np.correlate(echo_array - np.mean(echo_array), 
                                   echo_array - np.mean(echo_array), mode='full')
            autocorr = autocorr[len(autocorr)//2:]
            axes[1, 1].plot(autocorr[:50], 'g-')
            axes[1, 1].set_xlabel('Lag')
            axes[1, 1].set_ylabel('Autocorrelation')
            axes[1, 1].set_title('Signal Periodicity Check')
            axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"pga460_80khz_echo_{timestamp}.png"
        plt.savefig(filename, dpi=150)
        print(f"\nPlot saved as: {filename}")
        
        plt.show()
    
    def save_data(self, samples):
        """Save echo data to file"""
        if not samples:
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"pga460_80khz_data_{timestamp}.csv"
        
        with open(filename, 'w') as f:
            f.write("Sample_Index,Amplitude,Voltage\n")
            for i, val in enumerate(samples):
                voltage = (val / 255) * 3.3  # 8-bit to voltage
                f.write(f"{i},{val},{voltage:.4f}\n")
        
        print(f"Data saved to: {filename}")
    
    def run_80khz_test(self, use_burst=False):
        """Complete 80kHz test sequence"""
        print("="*60)
        print("PGA460 80kHz DEFAULT MODE TEST")
        print("="*60)
        
        # Connect
        if not self.connect():
            return
        
        try:
            # Configure for 80kHz
            self.set_to_80khz_default()
            
            # Optional: Trigger a burst if you want to test with transmission
            if use_burst:
                self.trigger_burst()
                time.sleep(0.2)  # Wait for echo
            
            # Listen for echo
            self.listen_for_echo()
            
            # Read samples
            raw_data = self.read_echo_samples(128)
            
            if not raw_data:
                print("\nWARNING: No echo data received!")
                print("Possible issues:")
                print("1. No ultrasonic pulse was transmitted")
                print("2. Nothing in front of transducer to reflect")
                print("3. Gain too low")
                print("4. Frequency mismatch with transducer")
                
                # Try reading noise floor (listen without burst)
                print("\n[4/6] Reading noise floor (no transmission)...")
                time.sleep(0.5)
                raw_data = self.read_echo_samples(128)
            
            # Convert to list of integers
            samples = list(raw_data)
            
            # Print detailed info
            print("\n[5/6] Analyzing samples...")
            self.print_samples_detailed(samples)
            
            # Plot
            print("\n[6/6] Generating plots...")
            self.plot_echo_signal(samples)
            
            # Save data
            self.save_data(samples)
            
            print("\n" + "="*60)
            print("TEST COMPLETE")
            print("="*60)
            
        except Exception as e:
            print(f"Error during test: {e}")
        finally:
            if self.ser:
                self.ser.close()
                print("Serial port closed")

def main():
    """Main function"""
    # Configuration
    PORT = 'COM12'  # Change to your COM port
    BAUD = 115200
    
    # Create PGA460 instance
    pga460 = PGA460Receiver(port=PORT, baudrate=BAUD)
    
    # Run test
    # Set use_burst=True if you want to transmit and receive echo
    # Set use_burst=False to just listen to ambient noise/80kHz signals
    pga460.run_80khz_test(use_burst=False)
    
    # For manual control (uncomment if needed):
    # pga460.connect()
    # pga460.set_to_80khz_default()
    # raw_data = pga460.read_echo_samples(128)
    # print(f"Raw data: {raw_data.hex()}")

if __name__ == "__main__":
    main()