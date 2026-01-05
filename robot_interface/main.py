"""
Real-Time IMU-Based Robotic Arm Control System

This is the main integration module that combines all system components to enable
real-time gesture-based control of the Dobot Magician robotic arm using IMU sensor
data from an ESP32 motion controller.

System Architecture:
    1. ESP32 Motion Controller (MPU9250 IMU sensor)
       ↓ USB/Bluetooth Serial Communication
    2. Serial Interface Module (data acquisition)
       ↓ Raw acceleration + velocity measurements
    3. Kalman Filter (optimal state estimation)
       ↓ Filtered velocity estimates
    4. Dobot Kinematics Interface (motion translation)
       ↓ Jog commands
    5. Dobot Magician Robot (physical motion)

Control Flow:
    - Read IMU measurements from ESP32 via serial
    - Apply Kalman filtering for noise reduction
    - Translate filtered data to robot jog commands
    - Execute motion when button pressed (b2)
    - Stop motion when button released
    - Emergency return to home on Ctrl+C

Hardware Setup:
    - ESP32: COM3 (USB) or COM14 (Bluetooth) @ 115200 baud
    - Dobot: COM6 @ 115200 baud
    - Button input: b2 (enable/disable motion)

Author: Gustavo Eismann
Date: 05/jan/2026
Project: TCC - Robot Manipulator with ESP32 Motion Controller
"""

# ==================== Core System Imports ====================
import DobotDllType as dType
from dobot_command_interface import DobotKinematics
from time_control import micros
from kalman_filter_model import KalmanFilterModel

# ==================== Communication Interface Setup ====================
# Select communication method with ESP32 motion controller
# COMM_TYPE: 0 = USB Serial (recommended for reliability)
#            1 = Bluetooth Serial (for wireless operation)
COMM_TYPE = 0

if COMM_TYPE == 1:
    # Bluetooth Serial configuration (wireless)
    from serial_module_interface import BluetoothInterface
    commInterface = BluetoothInterface(s_port="COM14", s_baudrate=115200)
    print("Using Bluetooth Serial on COM14")
else:
    # USB Serial configuration (wired - more stable)
    from serial_module_interface import USBSerialInterface
    commInterface = USBSerialInterface(s_port="COM3", s_baudrate=115200, s_bytesize=8, s_timeout=5)
    print("Using USB Serial on COM3")


CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

# Load Dobot DLL library and get API handle
api = dType.load()
print("Dobot DLL loaded successfully")

# Establish connection with Dobot Magician
# Port: COM6, Baud rate: 115200 (must match robot configuration)
state = dType.ConnectDobot(api, "COM6", 115200)[0]
print("Connect status:", CON_STR[state])

# ==================== Timing and Performance Measurement ====================
# Variables for tracking loop execution time and filter timing
delta_t = 0.0   # Elapsed time between iterations [seconds]
lastUpd = 0.0   # Timestamp of last update [microseconds]
current = 0.0   # Current timestamp [microseconds]

# Initialize timing (first measurement)
current = micros()
delta_t = ((current - lastUpd) / 1000000.0)  # Convert microseconds to seconds
lastUpd = current

# ==================== Kalman Filter Configuration ====================
# Initialize Kalman filter with accelerometer noise characteristics
# Standard deviations obtained from sensor calibration/characterization
std_dev = [0.21568627921038,   # σ_ax [m/s²]
           0.245023609034093,   # σ_ay [m/s²]
           0.35943901629517]    # σ_az [m/s²]
kfModel = KalmanFilterModel(a_std_dev=std_dev)
print("Kalman filter initialized with std_dev:", std_dev)

# ==================== Main Control System ====================
if (state == dType.DobotConnect.DobotConnect_NoError):
    print("\n=== System Initialization ===")
    
    # Initialize Dobot kinematics controller
    # kv=100: Velocity gain for scaling IMU velocity to robot motion
    dk = DobotKinematics(api, kv=100)
    print("Dobot kinematics interface initialized (kv=100)")

    # Clear any pending commands from previous sessions
    dType.SetQueuedCmdClear(api)
    print("Command queue cleared")

    # Configure PTP (Point-to-Point) motion parameters
    # Parameters: velocity_ratio=100%, acceleration_ratio=100%
    dType.SetPTPCommonParams(api, 100, 100, 0)
    print("PTP parameters configured")

    # Set current position as home (safe return point)
    # Note: Ensure robot is in safe configuration before running
    dk.setHomePosition()
    print("Home position set to current location")
    print("\n=== Starting Real-Time Control Loop ===")
    print("Press and hold button b2 to enable robot motion")
    print("Release button to stop. Press Ctrl+C to exit.\n")

    try:
        # ==================== Real-Time Control Loop ====================
        # Continuously read IMU data and control robot motion
        while True:
            # Clear command queue to ensure responsive control
            dType.SetQueuedCmdClear(api)

            # -------------------- Timing Measurement --------------------
            # Measure loop execution time for performance monitoring
            current = micros()
            delta_t = ((current - lastUpd) / 1000000.0)  # Convert μs to seconds
            lastUpd = current

            # -------------------- Step 1: Data Acquisition --------------------
            # Read IMU measurements from ESP32
            # Expected data: {ax, ay, az, vx, vy, vz, dt, b2, ...}
            meas = commInterface.readSerialPort()
            
            # -------------------- Step 2: Kalman Filtering --------------------
            # Apply optimal state estimation to reduce sensor noise
            kfModel.KalmanFilterUpdate(meas)
            filtered_meas = kfModel.getFilteredData()
            
            # filtered_meas contains: {dt, vx_est, vy_est, vz_est, ax, ay, az}
            # where vx_est, vy_est, vz_est are optimally filtered velocities

            # -------------------- Step 3: Motion Control --------------------
            # Enable robot motion only when button b2 is pressed
            if meas.get('b2'):
                # Button pressed: translate filtered measurements to robot motion
                # Uses dominant-axis algorithm to determine jog direction
                dk.setMeasurementsToJOG(filtered_meas)
            else:
                # Button released: stop robot motion immediatelyk.breakJogging()

            # -------------------- Step 4: Performance Monitoring --------------------
            # Measure and display loop execution time
            current = micros()
            delta_t = ((current - lastUpd) / 1000000.0)
            lastUpd = current
            
            print("Loop execution time: {:.6f} s ({:.1f} Hz)".format(delta_t, 1.0/delta_t if delta_t > 0 else 0))
            
            # Note: Target loop rate should be > 10 Hz for responsive control
            # Typical execution time: 20-100 ms (10-50 Hz)

    except KeyboardInterrupt:
        # ==================== Graceful Shutdown ====================
        # User pressed Ctrl+C - safely return robot to home position
        print("\n\n=== Keyboard Interrupt Detected ===")
        print("Stopping robot motion...")
        dk.breakJogging()
        
        print("Returning to home position...")
        dk.gotoHome()
        
        print("Robot safely returned to home.")

        print("\nThank you for using the IMU-based robot control system!")
    finally:
        # ==================== Cleanup and Disconnection ====================
        # Always execute cleanup, even if errors occurred
        print("\n=== System Shutdown ===")
        
        # Close serial communication
        print("Closing serial connection...")
        commInterface.close()
        
        # Disconnect from Dobot
        print("Disconnecting Dobot...")
        dType.DisconnectDobot(api)
        
        print("System shutdown complete.")
        print("\nThank you for using the IMU-based robot control system!")
