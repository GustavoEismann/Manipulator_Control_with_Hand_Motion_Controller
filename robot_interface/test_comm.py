"""
Serial Communication Test Script

Simple utility to test and verify serial communication with the ESP32
motion controller. Continuously reads and displays incoming IMU data
to verify connection and data format.

Usage:
    1. Configure COMM_TYPE (0 for USB, 1 for Bluetooth)
    2. Verify COM port in the interface initialization
    3. Run script: python test_comm.py
    4. Press Ctrl+C to stop

Expected Output:
    Continuous stream of IMU measurement dictionaries or lists
    e.g., {'ax': 1.2, 'ay': -0.3, 'az': 9.8, 'dt': 0.01, ...}

Troubleshooting:
    - No output: Check COM port and baud rate
    - Garbled data: Verify baud rate matches ESP32 (115200)
    - Connection errors: Check if port is in use by another program

Author: Gustavo Eismann
Date: 05/jan/2026
"""

# ==================== Communication Interface Selection ====================
# Select serial interface type
COMM_TYPE = 0   # 0 = USB Serial (wired), 1 = Bluetooth Serial (wireless)

# Initialize appropriate communication interface
if COMM_TYPE == 1:
    # Bluetooth configuration
    from serial_module_interface import BluetoothInterface
    commInterface = BluetoothInterface(s_port="COM14", s_baudrate=115200)
    print("Bluetooth interface initialized on COM14")
else:
    # USB configuration (recommended for testing)
    from serial_module_interface import USBSerialInterface
    commInterface = USBSerialInterface(s_port="COM3", s_baudrate=115200, s_bytesize=8, s_timeout=5)
    print("USB interface initialized on COM3")

print("\n=== Starting Communication Test ===")
print("Reading data from ESP32... (Press Ctrl+C to stop)\n")

try:
    # Main test loop - continuously read and display data
    while True:
        # Read next message from serial port
        # For Bluetooth: specify n_elements (e.g., 5 for 5 CSV values)
        # For USB: returns dictionary automatically
        if COMM_TYPE == 1:
            meas = commInterface.readBluetoothSerialPort(5)  # Adjust element count as needed
        else:
            meas = commInterface.readSerialPort()
        
        # Display received data
        print(meas)

except KeyboardInterrupt:
    # User pressed Ctrl+C - clean shutdown
    print("\n\n=== Keyboard Interrupt ===")
    print("Closing serial connection...")
    
    # Close appropriate serial interface
    if COMM_TYPE == 1:
        commInterface.btSerial.close()
    else:
        commInterface.ser.close()
    
    print("COM port closed successfully")
    print("Test complete.")
