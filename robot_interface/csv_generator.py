"""
CSV Data Collection Script for IMU Measurements with Kalman Filtering

This script collects 1000 samples of IMU sensor data from the ESP32 motion
controller, applies Kalman filtering for velocity estimation, and exports
the results to a timestamped CSV file.

Functionality:
    1. Connect to ESP32 via USB serial (COM3 @ 115200 baud)
    2. Initialize Kalman filter with precalibrated noise parameters
    3. Collect 1000 samples of acceleration data
    4. Apply Kalman filter to estimate velocity from acceleration
    5. Export data to CSV with timestamp filename
    6. Post-process CSV for Excel compatibility (European format)

Output File Format:
    - Filename: measurements_YYYYMMDD-HHMMSS.csv
    - Columns: time, vx, ax, vy, ay, vz, az
    - Contains: 1000 rows of filtered IMU data
    - Format: Semicolon-delimited with comma as decimal separator

Data Flow:
    ESP32 → Serial → Kalman Filter → CSV File
    
Usage:
    python csv_generator.py
    
    The script will automatically:
    - Create timestamped CSV file
    - Display progress (sample count)
    - Handle Ctrl+C gracefully
    - Post-process for Excel compatibility

Note:
    This script uses precalibrated standard deviations obtained from
    prior calibration measurements. See calibration documentation for
    details on noise characterization.

Author: Gustavo Eismann
Date: 05/jan/2026
"""

import csv
import time
from serial_module_interface import USBSerialInterface
from kalman_filter_model import KalmanFilterModel

# ========================================================================
# HARDWARE CONFIGURATION
# ========================================================================

# Initialize USB serial connection to ESP32
# Port: COM3 (verify in Device Manager)
# Baud rate: 115200 (must match ESP32 configuration)
commInterface = USBSerialInterface(s_port="COM3", s_baudrate=115200, s_bytesize=8, s_timeout=5)

# ========================================================================
# KALMAN FILTER INITIALIZATION
# ========================================================================

# Precalibrated acceleration noise standard deviations (m/s²)
# Obtained from static noise characterization measurements
# [std_dev_x, std_dev_y, std_dev_z]
std_dev = [0.21568627921038, 0.245023609034093, 0.35943901629517]

# Initialize Kalman filter model with measured noise parameters
kfModel = KalmanFilterModel(a_std_dev=std_dev)

# ========================================================================
# DATA COLLECTION SETUP
# ========================================================================

# Sample counter
n = 0

# CSV file structure
# Columns: time (cumulative), velocity estimates (vx, vy, vz), 
#          accelerations (ax, ay, az)
header = ['time', 'vx', 'ax', 'vy', 'ay', 'vz', 'az']
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initialize with zeros

# Generate timestamped filename
# Format: measurements_YYYYMMDD-HHMMSS.csv
timestr = time.strftime("%Y%m%d-%H%M%S")
filename = 'measurements_' + timestr + '.csv'

print('Coletando dados...')  # "Collecting data..."
print(f"Output file: {filename}")
print("Collecting 1000 samples... (Press Ctrl+C to stop early)\n")

# ========================================================================
# DATA COLLECTION LOOP
# ========================================================================

# Open CSV file for writing
with open(filename, 'w+', encoding='UTF-8', newline='') as f:
    writer = csv.writer(f)
    
    # Write header row (column names)
    writer.writerow(header)
    # Write initial zero row (baseline)
    writer.writerow(data)

    # Main data collection loop - collect 1000 samples
    while n < 1000:
        try:
            # ==================== Step 1: Read IMU data ====================
            # Read next measurement from ESP32 via serial port
            # Returns dictionary: {'ax': ..., 'ay': ..., 'az': ..., 'dt': ...}
            meas = commInterface.readSerialPort()

            # ==================== Step 2: Apply Kalman filter ====================
            # Update filter with new acceleration measurement
            # Filter estimates velocity by integrating acceleration
            kfModel.KalmanFilterUpdate(meas)
            
            # Get filtered results (velocity estimates + original acceleration)
            filtered_meas = kfModel.getFilteredData()

            # ==================== Step 3: Prepare CSV row ====================
            # Accumulate time (cumulative timestamp)
            data[0] = data[0] + filtered_meas.get('dt')
            
            # Extract filtered velocity estimates
            data[1] = filtered_meas.get('vx_est')  # X-axis velocity (m/s)
            data[2] = filtered_meas.get('ax')      # X-axis acceleration (m/s²)
            data[3] = filtered_meas.get('vy_est')  # Y-axis velocity (m/s)
            data[4] = filtered_meas.get('ay')      # Y-axis acceleration (m/s²)
            data[5] = filtered_meas.get('vz_est')  # Z-axis velocity (m/s)
            data[6] = filtered_meas.get('az')      # Z-axis acceleration (m/s²)

            # ==================== Step 4: Write to CSV ====================
            writer.writerow(data)
            
            # Display progress (sample number)
            print(f"Sample {n + 1}/1000")
            
            # Uncomment to debug Kalman filter internal variables:
            # print(kfModel.getAllKalmanVariables())
            
            # Increment sample counter
            n = n + 1

        except (KeyboardInterrupt):
            # User pressed Ctrl+C - close file and exit gracefully
            print("\n\nKeyboard interrupt detected. Closing file...")
            f.close()
            break

# Ensure file is closed (redundant with 'with' statement, but safe)
f.close()
print(f"\nData collection complete. {n} samples collected.")

# ========================================================================
# POST-PROCESSING: CONVERT TO EUROPEAN CSV FORMAT
# ========================================================================
# Many European versions of Excel expect:
#   - Semicolon (;) as field separator instead of comma
#   - Comma (,) as decimal separator instead of period
#
# This conversion makes the CSV directly compatible with Excel in
# European locales (e.g., Portuguese, Spanish, German, etc.)

print("\nPost-processing CSV for Excel compatibility...")

# Read entire file content
data = ""
with open(filename, 'r') as file:
    # Replace field separator: comma → semicolon
    # Replace decimal separator: period → comma
    data = file.read().replace(',', ';').replace('.', ',')
file.close()

# Write converted data back to file
with open(filename, "w") as out_file:
    out_file.write(data)
out_file.close()

print(f"CSV post-processing complete.")
print(f"File ready: {filename}")
print(f"\nFile format: Semicolon-delimited, comma decimal separator")
print(f"Samples: {n}")
print(f"Columns: {', '.join(header)}")
print("\nYou can now open the file in Excel or import it for analysis.")
