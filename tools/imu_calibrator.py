import serial
import serial.tools.list_ports
import time
import math
import re
import sys

def get_serial_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found!")
        sys.exit(1)
    
    print("Available ports:")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")
    
    idx = int(input("Select the port number for the ESP32: "))
    return ports[idx].device

def parse_imu_line(line):
    # Parses the [IMU] print format: 
    # [IMU] Accel: (X, Y, Z) m/s², Gyro: (X, Y, Z) °/s
    match = re.search(r"Accel:\s*\(([^,]+),\s*([^,]+),\s*([^)]+)\).*Gyro:\s*\(([^,]+),\s*([^,]+),\s*([^)]+)\)", line)
    if match:
        try:
            ax, ay, az = map(float, match.group(1, 2, 3))
            gx, gy, gz = map(float, match.group(4, 5, 6))
            return [ax, ay, az], [gx, gy, gz]
        except ValueError:
            pass
    return None, None

def collect_data(ser, duration=3.0):
    start_time = time.time()
    accel_samples = []
    gyro_samples = []
    
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            accel, gyro = parse_imu_line(line)
            if accel and gyro:
                accel_samples.append(accel)
                gyro_samples.append(gyro)
    
    if not accel_samples:
        print("Error: No IMU data received. Make sure printIMU() is being called in loop()!")
        sys.exit(1)
        
    # Calculate averages
    avg_accel = [sum(col)/len(col) for col in zip(*accel_samples)]
    avg_gyro = [sum(col)/len(col) for col in zip(*gyro_samples)]
    return avg_accel, avg_gyro

def identify_dominant_axis(avg_vector, threshold=7.0):
    # Find which axis has the highest absolute value (e.g. feeling 1G or high rotation)
    max_val = max(abs(x) for x in avg_vector)
    idx = [abs(x) for x in avg_vector].index(max_val)
    
    if max_val < threshold:
        return None, None # Movement/Gravity wasn't strong enough
        
    axis_name = ["X", "Y", "Z"][idx]
    sign = 1 if avg_vector[idx] > 0 else -1
    return axis_name, sign

def main():
    print("=== Rocket IMU 6DOF Axis Calibrator ===")
    port = get_serial_port()
    baud = 115200
    
    mapping = {}
    
    with serial.Serial(port, baud, timeout=1) as ser:
        print(f"\nConnected to {port} at {baud} baud.")
        print("Waiting for data stream...")
        time.sleep(2)
        ser.reset_input_buffer()
        
        # --- 1. Longitudinal Axis (Z-axis of rocket) ---
        input("\n[STEP 1] Point the NOSE of the rocket STRAIGHT UP to the sky. Hold it steady. Press Enter when ready...")
        print("Collecting data for 3 seconds...")
        avg_accel, _ = collect_data(ser)
        axis, sign = identify_dominant_axis(avg_accel, threshold=7.0)
        if axis:
            mapping['Longitudinal (Nose Up)'] = f"{'-' if sign < 0 else '+'}{axis}"
            print(f"-> Detected Longitudinal Axis: {mapping['Longitudinal (Nose Up)']}")
        else:
            print("-> Could not clearly detect gravity. Did you hold it upright?")
            
        # --- 2. Normal/Yaw Axis (Z-axis of board, Y of rocket maybe?) ---
        input("\n[STEP 2] Lay the rocket FLAT on the table. Point the 'TOP' hatch/side of the rocket straight up to the sky. Press Enter when ready...")
        print("Collecting data for 3 seconds...")
        avg_accel, _ = collect_data(ser)
        axis, sign = identify_dominant_axis(avg_accel, threshold=7.0)
        if axis:
            mapping['Normal (Top Up)'] = f"{'-' if sign < 0 else '+'}{axis}"
            print(f"-> Detected Normal Axis: {mapping['Normal (Top Up)']}")
        else:
            print("-> Could not clearly detect gravity.")
            
        # --- 3. Roll Axis (Gyro) ---
        input("\n[STEP 3] Hold the rocket and smoothly SPIN/ROLL it CLOCKWISE around its nose. Keep spinning and press Enter...")
        print("Collecting data for 3 seconds...")
        _, avg_gyro = collect_data(ser)
        axis, sign = identify_dominant_axis(avg_gyro, threshold=50.0) # threshold in deg/s
        if axis:
            mapping['Roll (Clockwise)'] = f"{'-' if sign < 0 else '+'}{axis}"
            print(f"-> Detected Roll Axis: {mapping['Roll (Clockwise)']}")
        else:
            print(f"-> Movement not strong enough. Max gyro reading: {max(abs(x) for x in avg_gyro):.1f} deg/s")

    print("\n===========================================")
    print("             CALIBRATION RESULT            ")
    print("===========================================")
    for k, v in mapping.items():
        print(f"{k:25} : IMU {v} Axis")
    print("===========================================")
    print("Use these mappings in your ekf.c or Dynamics.c to align the MPU6050 with your rocket!")

if __name__ == "__main__":
    main()