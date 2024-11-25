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
        
def calculate_index(cir_no_object_mag, cir_with_object_mag):
    """Calculate the time delay between two CIR measurements"""
    print("\nDEBUG INFO:")
    print(f"Signal lengths: no_object={len(cir_no_object_mag)}, with_object={len(cir_with_object_mag)}")
    print(f"Max magnitudes: no_object={np.max(cir_no_object_mag):.2f}, with_object={np.max(cir_with_object_mag):.2f}")
    
    # Ensure signals aren't identical
    if np.array_equal(cir_no_object_mag, cir_with_object_mag):
        print("WARNING: Signals are identical!")
        return 0
    
    # Find multiple peaks with a moderate threshold
    threshold_no_obj = np.max(cir_no_object_mag) * 0.6
    threshold_with_obj = np.max(cir_with_object_mag) * 0.6
    
    print(f"Peak detection thresholds: no_object={threshold_no_obj:.2f}, with_object={threshold_with_obj:.2f}")
    
    peaks_no_object, properties_no_obj = find_peaks(cir_no_object_mag, 
                                                   height=threshold_no_obj,
                                                   distance=20)
    peaks_with_object, properties_with_obj = find_peaks(cir_with_object_mag, 
                                                       height=threshold_with_obj,
                                                       distance=20)
    
    print(f"Number of peaks found: no_object={len(peaks_no_object)}, with_object={len(peaks_with_object)}")
    
    # If no peaks found, return 0
    if len(peaks_no_object) == 0 or len(peaks_with_object) == 0:
        print("No peaks found in at least one signal")
        return 0
    
    # Get and print peak information
    heights_no_obj = properties_no_obj["peak_heights"]
    heights_with_obj = properties_with_obj["peak_heights"]
    
    print("\nNo object peaks:")
    for idx, (pos, height) in enumerate(zip(peaks_no_object, heights_no_obj)):
        print(f"Peak {idx}: position={pos}, height={height:.2f}")
        
    print("\nWith object peaks:")
    for idx, (pos, height) in enumerate(zip(peaks_with_object, heights_with_obj)):
        print(f"Peak {idx}: position={pos}, height={height:.2f}")
    
    # Sort peaks by height
    no_obj_sorted_idx = np.argsort(heights_no_obj)[::-1]
    with_obj_sorted_idx = np.argsort(heights_with_obj)[::-1]
    
    peaks_no_object = peaks_no_object[no_obj_sorted_idx]
    peaks_with_object = peaks_with_object[with_obj_sorted_idx]
    
    print("\nLooking for delays...")
    # Look at multiple peak pairs to find meaningful delay
    for i in range(min(3, len(peaks_no_object))):
        for j in range(min(3, len(peaks_with_object))):
            delay = peaks_with_object[j] - peaks_no_object[i]
            print(f"Comparing peaks: no_obj[{i}]={peaks_no_object[i]} with_obj[{j}]={peaks_with_object[j]} -> delay={delay}")
            if delay > 0:  # Only consider positive delays
                # Convert sample difference to nanoseconds
                time_delay = delay * (1000 / 1015)  # Scale factor may need adjustment
                print(f"Found positive delay: {delay} samples = {time_delay:.2f} ns")
                return time_delay
    
    print("No positive delays found between peak pairs")
    return 0
    
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