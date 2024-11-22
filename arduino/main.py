#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import platform
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def find_arduino_port():
    """Find the serial port for the Arduino on different operating systems."""
    system = platform.system()
    
    # List all serial ports
    ports = list(serial.tools.list_ports.comports())
    
    for port in ports:
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
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle('Channel Impulse Response')
        
        # Initialize empty data
        self.x = np.arange(254)  # 254 samples
        self.real_data = np.zeros(254)
        self.imag_data = np.zeros(254)
        
        # Create the lines
        self.line_real, = self.ax1.plot(self.x, self.real_data, 'b-', label='Real')
        self.line_imag, = self.ax2.plot(self.x, self.imag_data, 'r-', label='Imaginary')
        
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
        
        plt.tight_layout()

    def update_plot(self, real_data, imag_data):
        # Update the data
        self.real_data = real_data
        self.imag_data = imag_data
        
        # Update the lines
        self.line_real.set_ydata(self.real_data)
        self.line_imag.set_ydata(self.imag_data)
        
        # Adjust y-axis limits if needed
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        
        # Draw the updates
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def process_cir_data(raw_data, plotter=None):
    real_data = []
    imag_data = []
    
    # Process 1016 bytes of CIR data (254 complex samples)
    for i in range(0, len(raw_data), 4):
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
        print(f"Sample {i//4}: {real} + {imag}j")
    
    # Update plot if plotter is available
    if plotter is not None:
        plotter.update_plot(real_data, imag_data)

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Process CIR data from DW1000')
    parser.add_argument('-p', '--plot', action='store_true', help='Enable real-time plotting')
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
            # Read exactly 1016 bytes (254 complex samples)
            raw_data = ser.read(1016)
            if len(raw_data) == 1016:
                process_cir_data(raw_data, plotter)
            elif len(raw_data) > 0:
                # Try to decode as text for non-CIR messages
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

