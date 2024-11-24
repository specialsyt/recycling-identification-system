import serial
import serial.tools.list_ports
import platform
import sys
import numpy as np
from scipy.signal import find_peaks
import scipy.constants as const
import time


def find_arduino_port():
    """Find the serial port for the Arduino on different operating systems."""
    system = platform.system()
    
    # List all serial ports
    ports = list(serial.tools.list_ports.comports())
    
    for port in ports:
        if port.device == "/dev/cu.usbmodem1201":
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

def process_cir_data(raw_data):
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
    complex_data = np.array(real_data) + 1j * np.array(imag_data)
    return complex_data
        
def read_data(data_len=1016):
    try:
        raw_data = ser.read(data_len)
        if len(raw_data) == data_len:
            return process_cir_data(raw_data)
        elif len(raw_data) > 0:
            # Try to decode as text for non-CIR messages
            try:
                text = raw_data.decode('utf-8').rstrip()
                print(f"Message: {text}")
            except UnicodeDecodeError:
                print(f"Received {len(raw_data)} bytes of unknown data")
        else:
            raise ValueError("UWB is hanging and isn't outputting any data")
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        
def calculate_index(cir_no_object, cir_with_object, sampling_rate, thickness):
    """
    Calculate the refractive index of an object using two CIRs (with and without the object).
    
    Parameters:
        cir_no_object (np.array): CIR data without the object.
        cir_with_object (np.array): CIR data with the object.
        sampling_rate (float): Sampling rate of the CIR in Hz.
        thickness (float): Thickness of the object in meters.
    
    Returns:
        refractive_index (float): Estimated refractive index.
    """
    if cir_no_object is None or cir_with_object is None:
        raise ValueError("Data wasn't properly collected, see above errors and retry.")
        
    c = const.c
    
    # Step 1: Convert CIRs to magnitude
    cir_no_object_mag = np.abs(cir_no_object)
    cir_with_object_mag = np.abs(cir_with_object)
    
    # Step 2: Find the direct path peaks
    peaks_no_object, _ = find_peaks(cir_no_object_mag, height=np.max(cir_no_object_mag) * 0.5)
    peaks_with_object, _ = find_peaks(cir_with_object_mag, height=np.max(cir_with_object_mag) * 0.5)
    
    if len(peaks_no_object) == 0 or len(peaks_with_object) == 0:
        raise ValueError("Unable to detect direct path in one or both CIRs.")
    
    index_difference = peaks_with_object[0] - peaks_no_object[0]
    return index_difference
    
if __name__ == '__main__':    
    # Find the Arduino port
    port = find_arduino_port()
    if port is None:
        print("Error: Could not find Arduino serial port")
        sys.exit(1)
    
    print(f"Connecting to Arduino on port: {port}")
    
    try:
        ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
        ser.reset_input_buffer()
        ser.close()

        ser = serial.Serial(port, 921600, timeout=1)  # Increased baud rate
        ser.reset_input_buffer()
        
        start_time = time.time()
        air_data = read_data()
        end_time = time.time()
        time_taken = end_time - start_time
        
        input("Press Enter once you've placed the object")
        
        start_time = time.time()
        obj_data = read_data()
        end_time = time.time()
        time_taken_2 = end_time - start_time
        
        avg_time_taken = (time_taken_2 + time_taken) / 2
        sampling_rate = 1 / (avg_time_taken / 254)
        print("Sampling Rate: ", sampling_rate)
        #sampling_rate = 27 # Is this right? We have 254 samples, but it doesn't take 1 second to get them...
        thickness = 0.0002 # plastic bottles and cans are 0.2mm, glass bottles are often 2-3mm.
        
        print(calculate_index(air_data, obj_data, sampling_rate, thickness))
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")