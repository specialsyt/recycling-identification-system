#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import platform
import time
import numpy as np

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
        elif system == 'Windows' and 'COM' in port.device and "USB" in port.description:
            return port.device
    
    return None

def get_load_sensor_reading(port=None):
    if port is None:
        port = find_arduino_port()
    ser = serial.Serial(port, 9600, timeout=1)
    ser.reset_input_buffer()

    adc_values = []
    start = time.time()
    while time.time() - start < 1:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            adc_value = float(line.split(" ")[2])
            adc_values.append(adc_value)

    adc_values = np.array(adc_values)
    avg_adc = np.average(adc_values)
    return avg_adc


if __name__ == '__main__':
    print(get_load_sensor_reading())