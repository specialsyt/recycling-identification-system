#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import platform
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import time
from cir_difference import calculate_index

INITIATOR_SERIAL = "FED7218050533748392E3120FF151D39"

def find_arduino_port():
    """Find the serial port for the Arduino on different operating systems."""
    ports = list(serial.tools.list_ports.comports())
    
    # Print detailed info for debugging
    for port in ports:
        print(f"\nFound device:")
        print(f"  Port: {port.device}")
        print(f"  Serial Number: {port.serial_number}")
        print(f"  Manufacturer: {port.manufacturer}")
        print(f"  Product ID: {port.pid}")
        print(f"  Vendor ID: {port.vid}")
        print(f"  Description: {port.description}")

    # Filter for Arduino devices
    usb_ports = [port for port in ports if port.vid == 9114]  # Arduino vendor ID
    # or use specific serial numbers if known
    # initiator_port = next((port.device for port in ports if port.serial_number == "INITIATOR_SERIAL"), None)
    # responder_port = next((port.device for port in ports if port.serial_number == "RESPONDER_SERIAL"), None)
    
    return usb_ports

class CIRPlotter:
    def __init__(self):
        # Initialize the plot
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 12))
        self.fig.suptitle('Channel Impulse Response')
        
        # Initialize empty data for 1015 samples (skipping first garbage byte)
        self.x = np.arange(1015)  
        self.real_data = np.zeros(1015)
        self.imag_data = np.zeros(1015)
        self.mag_data = np.zeros(1015)
        
        # Create the lines
        self.line_real, = self.ax1.plot(self.x, self.real_data, 'b-', label='Real')
        self.line_imag, = self.ax2.plot(self.x, self.imag_data, 'r-', label='Imaginary')
        self.line_mag, = self.ax3.plot(self.x, self.mag_data, 'g-', label='Magnitude')
        
        # Set up the axes
        self.ax1.set_ylabel('Amplitude')
        self.ax1.set_title('Real Part')
        self.ax1.grid(True)
        self.ax1.legend()
        
        self.ax2.set_xlabel('Sample')
        self.ax2.set_ylabel('Amplitude')
        self.ax2.set_title('Imaginary Part')
        self.ax2.grid(True)
        self.ax2.legend()

        self.ax3.set_xlabel('Sample')
        self.ax3.set_ylabel('Amplitude')
        self.ax3.set_title('Magnitude')
        self.ax3.grid(True)
        self.ax3.legend()
        
        plt.tight_layout()
        
        # Add last update time tracking
        self.last_update_time = time.time()
        
        # Add validation thresholds
        self.max_amplitude = 40000  # Maximum reasonable amplitude
        self.min_variance = 100     # Minimum variance for valid data
        self.last_valid_data = None # Store last valid data
        
        # Add tracking for previous data and correlation
        self.prev_mag_data = None
        self.correlation_threshold = 0.90  # Adjust this value (0-1) for sensitivity
        self.max_shift = 100  # Maximum samples to check for alignment
        self.min_diff_threshold = 500  # Minimum difference to consider a change
        
    def validate_data(self, real_data, imag_data):
        """Validate the incoming data"""
        # Convert to numpy arrays if not already
        real = np.array(real_data)
        imag = np.array(imag_data)
        
        # Check for static/constant data
        real_var = np.var(real)
        imag_var = np.var(imag)
        if real_var < self.min_variance and imag_var < self.min_variance:
            print("Warning: Static data detected")
            return False
            
        # Check for unreasonable amplitudes
        if np.max(np.abs(real)) > self.max_amplitude or np.max(np.abs(imag)) > self.max_amplitude:
            print("Warning: Amplitude exceeds reasonable limits")
            return False
            
        # Check for all-zero data
        if np.all(real == 0) or np.all(imag == 0):
            print("Warning: Zero data detected")
            return False
            
        return True

    def align_signals(self, current, reference):
        """Align current signal with reference using cross-correlation"""
        if reference is None:
            return current, 0
            
        best_correlation = 0
        best_shift = 0
        
        # Try different shifts to find best alignment
        for shift in range(-self.max_shift, self.max_shift):
            if shift < 0:
                curr_slice = current[-shift:]
                ref_slice = reference[:shift]
            else:
                curr_slice = current[:-shift] if shift else current
                ref_slice = reference[shift:] if shift else reference
                
            min_len = min(len(curr_slice), len(ref_slice))
            if min_len < len(current) // 2:  # Skip if too little overlap
                continue
                
            correlation = np.corrcoef(curr_slice[:min_len], ref_slice[:min_len])[0,1]
            if correlation > best_correlation:
                best_correlation = correlation
                best_shift = shift
        
        # Apply the best shift
        if best_shift < 0:
            aligned = np.roll(current, -best_shift)
        else:
            aligned = np.roll(current, best_shift)
            
        return aligned, best_correlation

    def is_significantly_different(self, current_mag, prev_mag):
        """Check if the current signal is significantly different from previous"""
        if prev_mag is None:
            return True
            
        # Align signals first
        aligned_current, correlation = self.align_signals(current_mag, prev_mag)
        
        # If signals are too similar after alignment, ignore
        if correlation > self.correlation_threshold:
            diff = np.abs(aligned_current - prev_mag)
            if np.max(diff) < self.min_diff_threshold:
                return False
                
        return True

    def set_data(self, real_data, imag_data):
        # Validate the data
        if not self.validate_data(real_data, imag_data):
            if self.last_valid_data is not None:
                real_data, imag_data = self.last_valid_data
            else:
                return
        
        # Convert to numpy arrays
        real = np.array(real_data[:1015], dtype=float)
        imag = np.array(imag_data[:1015], dtype=float)
        current_mag = np.sqrt(np.square(real) + np.square(imag))
        
        # Check if significantly different from previous
        if not self.is_significantly_different(current_mag, self.prev_mag_data):
            return
            
        # Store valid data
        self.last_valid_data = (real_data.copy(), imag_data.copy())
        self.real_data = real
        self.imag_data = imag
        self.mag_data = current_mag
        self.prev_mag_data = current_mag.copy()
        self.new_data = True

    def update_plot(self, real_data, imag_data, mag_data, live=False, rate=1):
        current_time = time.time_ns() / 1e6  # Convert to ms
        # Only update if more than rate ms have passed
        if current_time - self.last_update_time >= rate or live:
            # Update the data
            self.real_data = real_data
            self.imag_data = imag_data
            self.mag_data = mag_data
            
            # Update the lines
            self.line_real.set_ydata(self.real_data)
            self.line_imag.set_ydata(self.imag_data)
            self.line_mag.set_ydata(self.mag_data)
            
            # Adjust y-axis limits if needed
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax2.relim()
            self.ax2.autoscale_view()
            self.ax3.relim()
            self.ax3.autoscale_view()
            # self.ax3.set_ylim(400, 1000)
            
            # Draw the updates
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
            # Update the last update time
            self.last_update_time = current_time

def process_cir_data(raw_data, plotter=None, live=False, rate=1):
    real_data = []
    imag_data = []
    mag_data = []
    
    # Process 4064 bytes as 1016 complex samples (including first garbage byte)
    # We'll process only 1015 samples, starting after the first byte
    max_samples = 1015  # Total samples we want to process
    
    # Process only up to max_samples, skipping the first byte
    for i in range(1, 4063, 4):  # 4063 ensures we don't exceed buffer
        if len(real_data) >= max_samples:
            break
            
        # Convert 4 bytes into real and imaginary parts
        real = (raw_data[i+1] << 8) | raw_data[i]
        imag = (raw_data[i+3] << 8) | raw_data[i+2]
        
        # Convert to signed integers
        if real > 32767:
            real -= 65536
        if imag > 32767:
            imag -= 65536
            
        real_data.append(real)
        imag_data.append(imag)
        mag_data.append(np.sqrt(real**2 + imag**2))
        # print(f"Sample {len(real_data)-1}: {real} + {imag}j")
    
    # Update plot if plotter is available
    if plotter is not None:
        plotter.update_plot(real_data, imag_data, mag_data, live, rate)

    return real_data, imag_data, mag_data

def reset_device(ser):
    """Send reset command to Arduino"""
    if ser is not None and ser.is_open:
        print("Sending reset command...")
        ser.write(b"RESET\n")
        time.sleep(1)  # Wait for device to reset
        ser.reset_input_buffer()  # Clear any startup messages

def initialize() -> tuple[serial.Serial, serial.Serial]:
    # Find the Arduino port
    usb_ports = find_arduino_port()
    if len(usb_ports) < 2:
        print(f"Error: Could not find both Arduino serial ports... Found {len(usb_ports)}")
        sys.exit(1)
    
    print(f"Connecting to Arduino on ports: {usb_ports}")
    
    initiator_port = [port.device for port in usb_ports if INITIATOR_SERIAL == port.serial_number][0]
    responder_port = [port.device for port in usb_ports if INITIATOR_SERIAL != port.serial_number][0]
    initiator_ser = None
    responder_ser = None
    try:
        initiator_ser = serial.Serial(initiator_port, 921600, timeout=1)
        responder_ser = serial.Serial(responder_port, 921600, timeout=1)
        # initiator_ser.reset_input_buffer()
        # responder_ser.reset_input_buffer()

        initiator_ser = initiator_ser
        responder_ser = responder_ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    return initiator_ser, responder_ser

def interactive_capture_time_delay(ser: serial.Serial, num_measurements: int = 2):
    """Interactive capture of CIR measurements to calculate time delay difference"""
    from cir_difference import calculate_index
    import numpy as np
    import sys

    # Lists to store measurements
    air_measurements = []
    object_measurements = []
    
    print("\nStarting CIR measurement collection...")
    print("Collecting measurements in air. Please ensure clear line of sight.")
    
    # Collect num_measurements measurements in air
    for i in range(num_measurements):
        raw_data = ser.read(4064)
        if len(raw_data) == 4064:
            real_data, imag_data, _ = process_cir_data(raw_data)
            air_measurements.append((real_data, imag_data))
        sys.stdout.write(f"\rCollecting air measurement {i+1}/{num_measurements}")
        sys.stdout.flush()
    print()  # New line after progress complete
    
    input("\nPress Enter when ready to collect measurements with object in between...")
    print("Collecting measurements with object. Please place object between devices.")
    
    # Collect num_measurements measurements with object
    for i in range(num_measurements):
        raw_data = ser.read(4064)
        if len(raw_data) == 4064:
            real_data, imag_data, _ = process_cir_data(raw_data)
            object_measurements.append((real_data, imag_data))
        sys.stdout.write(f"\rCollecting object measurement {i+1}/{num_measurements}")
        sys.stdout.flush()
    print()  # New line after progress complete
    
    # Calculate magnitude
    air_mag = []
    obj_mag = []
    for air in air_measurements:
        air_mag.append(np.sqrt(np.square(air[0]) + np.square(air[1])))
    for obj in object_measurements:
        obj_mag.append(np.sqrt(np.square(obj[0]) + np.square(obj[1])))
    
    # Calculate time delay using calculate_index
    time_delays = []
    for air in air_mag:
        for obj in obj_mag:
            # print(f"Calculating time delay for {air} and {obj}")
            time_delay = calculate_index(air, obj)
            print(f"\nCalculated time delay: {time_delay} ns")
            time_delays.append(time_delay)
    time_delay = np.average(time_delays)
    return time_delay

def main():
    
    # Initialize plotter if requested
    plotter = None
    if args.plot:
        plotter = CIRPlotter()
        plt.ion()  # Enable interactive mode
        plt.show()
    
    initiator_ser, responder_ser = initialize()
    ser = initiator_ser

    # Flush the buffer manually
    # ser.read(ser.in_waiting)
    
    try:
        while True:
            # Still read 4064 bytes but process only 4063 (skipping first byte)
            raw_data = ser.read(4064)
            if len(raw_data) == 4064:
                process_cir_data(raw_data, plotter, args.live, args.rate)
            elif len(raw_data) > 0:
                try:
                    text = raw_data.decode('utf-8').rstrip()
                    print(f"Message: {text}")
                except UnicodeDecodeError:
                    print(f"Received {len(raw_data)} bytes of unknown data")
                    
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")
        if args.plot:
            plt.close('all')


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Process CIR data from DW1000')
    parser.add_argument('-d', '--debug', action='store_true', help='Enable debug mode')
    parser.add_argument('-p', '--plot', action='store_true', help='Enable real-time plotting')
    parser.add_argument('-l', '--live', action='store_true', help='Live mode')
    parser.add_argument('-r', '--rate', type=int, default=1, help='Update rate in ms')
    parser.add_argument('--reset', action='store_true', help='Reset device before starting')
    args = parser.parse_args()
    
    if args.debug:
        initiator_ser, responder_ser = initialize()
        time_delay = interactive_capture_time_delay(initiator_ser, num_measurements=10)
        print(f"Average time delay: {time_delay} ns")
    else:
        main()
