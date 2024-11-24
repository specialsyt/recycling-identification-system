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

def find_arduino_port():
    """Find the serial port for the Arduino on different operating systems."""
    system = platform.system()
    
    # List all serial ports
    ports = list(serial.tools.list_ports.comports())
    
    for port in ports:
        if port.device == "/dev/cu.usbmodem1101":
            continue
        # macOS typically uses /dev/cu.usbmodem* for Arduino
        if system == 'Darwin' and 'usbmodem' in port.device.lower():
            return port.device
        # Linux (Raspberry Pi) typically uses /dev/ttyACM* or /dev/ttyUSB*
        elif system == 'Linux' and ('ACM' in port.device or 'USB' in port.device):
            return port.device
        # Windows typically uses COM*
        elif system == 'Windows' and 'COM' in port.device:
            return port.device
    
    return None

class CIRPlotter:
    def __init__(self):
        # Initialize the plot
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 12))
        self.fig.suptitle('Channel Impulse Response')
        
        # Initialize empty data for 253 samples (skipping first garbage byte)
        self.x = np.arange(253)  
        self.real_data = np.zeros(253)
        self.imag_data = np.zeros(253)
        self.mag_data = np.zeros(253)
        
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
    
    # Process 1016 bytes as 254 complex samples (including first garbage byte)
    # We'll process only 253 samples, starting after the first byte
    max_samples = 253  # Total samples we want to process
    
    # Process only up to max_samples, skipping the first byte
    for i in range(1, 1013, 4):  # 1013 ensures we don't exceed buffer
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

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Process CIR data from DW1000')
    parser.add_argument('-p', '--plot', action='store_true', help='Enable real-time plotting')
    parser.add_argument('-l', '--live', action='store_true', help='Live mode')
    parser.add_argument('-r', '--rate', type=int, default=1, help='Update rate in ms')
    args = parser.parse_args()
    
    # Initialize plotter if requested
    plotter = None
    if args.plot:
        plotter = CIRPlotter()
        plt.ion()  # Enable interactive mode
        plt.show()
    
    # Find the Arduino port
    port = find_arduino_port()
    if port is None:
        print("Error: Could not find Arduino serial port")
        sys.exit(1)
    
    print(f"Connecting to Arduino on port: {port}")
    
    try:
        ser = serial.Serial(port, 921600, timeout=1)  # Increased baud rate
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    
    try:
        while True:
            # Still read 1016 bytes but process only 1015 (skipping first byte)
            raw_data = ser.read(1016)
            if len(raw_data) == 1016:
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

