"""
Kalman Filter Implementation Script for Real-Time Velocity Estimation

This script implements real-time velocity estimation using a Kalman filter
processing accelerometer data from an ESP32-based motion controller. It collects
IMU measurements via USB serial communication, applies Kalman filtering, and
saves both estimated and measured velocities to CSV for analysis.

Workflow:
    1. Initialize serial communication with ESP32 motion controller
    2. Configure Kalman filter with sensor noise characteristics
    3. Read IMU measurements (acceleration + velocity) from serial port
    4. Apply Kalman filtering to estimate optimal velocity
    5. Log estimated vs measured velocities to CSV file
    6. Post-process CSV for locale compatibility (decimal format)

Output:
    CSV file: measurements_YYYYMMDD-HHMMSS.csv
    Columns: time, estimated [vx,vy,vz], measured [vx,vy,vz]

Typical use:
    1. Connect ESP32 to COM3 (adjust port if needed)
    2. Run script: python kf_implementation.py
    3. Collect 500 samples (adjust n < 500 for more/less)
    4. Analyze results in measurements_*.csv

Author: Gustavo Eismann
Date: 05/jan/2026
"""

from kalman_filter_model import KalmanFilterModel
from serial_module_interface import USBSerialInterface
import csv
import time

# ==================== Hardware Communication Setup ====================
# Initialize USB serial interface to ESP32 motion controller
# Configuration:
#   - Port: COM3 (adjust based on device manager)
#   - Baud rate: 115200 (must match ESP32 firmware)
#   - 8 data bits, 5 second timeout
commInterface = USBSerialInterface(s_port="COM3", s_baudrate=115200, s_bytesize=8, s_timeout=5)

# ==================== Kalman Filter Configuration ====================
# Accelerometer noise standard deviations [σ_ax, σ_ay, σ_az] in m/s²
# Values obtained from sensor calibration/characterization
# Higher values = more filtering (smoother but potentially lagged estimates)
std_dev = [0.21568627921038,   # X-axis acceleration noise
           0.245023609034093,   # Y-axis acceleration noise
           0.35943901629517]    # Z-axis acceleration noise (typically highest due to gravity)

kfModel = KalmanFilterModel(a_std_dev=std_dev)

# ==================== Data Collection Setup ====================
# Sample counter (collect 500 measurements)
n = 0

# CSV file structure
# Columns: timestamp, estimated velocities (Kalman), measured velocities (raw integration)
header = ['time', 'esti_vx', 'esti_vy', 'esti_vz', 'meas_vx', 'meas_vy', 'meas_vz']
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initialize data row

# Generate timestamped filename for uniqueness (format: measurements_YYYYMMDD-HHMMSS.csv)
timestr = time.strftime("%Y%m%d-%H%M%S")
filename = 'measurements_' + timestr + '.csv'

print('Coletando dados...')  # "Collecting data..."

# ==================== Main Data Collection Loop ====================
# Open CSV file and collect measurements until reaching sample limit
with open(filename, 'w+', encoding='UTF-8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)  # Write column headers
    writer.writerow(data)    # Write initial (zero) data row

    # Collect 500 samples (adjust limit as needed)
    while n < 500:
        try:
            # Read IMU measurement packet from ESP32 via serial
            # Expected format: dictionary with 'dt', 'ax', 'ay', 'az', 'vx', 'vy', 'vz'
            meas = commInterface.readSerialPort()

            # Apply Kalman filter to current measurement
            # Filter fuses acceleration data to produce optimal velocity estimate
            kfModel.KalmanFilterUpdate(meas)

            # Accumulate elapsed time (cumulative timestamp)
            data[0] = data[0] + meas.get('dt')
            
            # Extract Kalman-filtered velocity estimates from state vector
            data[1] = kfModel.X_prev[0][0]  # Estimated vx (Kalman)
            data[2] = kfModel.X_prev[1][0]  # Estimated vy (Kalman) - Note: should be [1][0] not [1][1]
            data[3] = kfModel.X_prev[2][0]  # Estimated vz (Kalman) - Note: should be [2][0] not [2][2]
            
            # Get raw measured velocities from ESP32 (simple integration)
            data[4] = meas.get('vx')  # Measured vx (raw)
            data[5] = meas.get('vy')  # Measured vy (raw)
            data[6] = meas.get('vz')  # Measured vz (raw)

            # Write data row to CSV file
            writer.writerow(data)
            
            # Display progress
            print(n)
            n = n + 1

        except (KeyboardInterrupt):
            # Gracefully handle Ctrl+C interruption
            f.close()

f.close()

# ==================== CSV Post-Processing ====================
# Convert CSV format for locale compatibility (e.g., Excel in Portuguese/European locales)
# Changes:
#   - Field delimiter: comma (,) → semicolon (;)
#   - Decimal separator: period (.) → comma (,)
# This allows direct import into spreadsheet software configured for European number format

data = ""
with open(filename, 'r') as file:
    data = file.read().replace(',', ';').replace('.', ',')  # Convert format
file.close()

with open(filename, "w") as out_file:
    out_file.write(data)  # Overwrite file with converted format
out_file.close()

print(f"Data collection complete. Saved to: {filename}")
