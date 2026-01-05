"""
Serial Communication Interface for ESP32 Motion Controller

This module provides robust serial communication interfaces for receiving
IMU sensor data from an ESP32 motion controller. It supports both USB serial
and Bluetooth serial connections with automatic error handling and data validation.

Supported Communication Methods:
    - USB Serial: Reliable wired connection (recommended)
    - Bluetooth Serial: Wireless connection for portable applications

Data Format:
    - USB: Python dictionary string (e.g., "{'ax': 1.2, 'ay': 0.5, ...}")
    - Bluetooth: Comma-separated values (e.g., "1.2,0.5,0.3,0.1")

Features:
    - Automatic port recovery on connection errors
    - Message validation (format, encoding, completeness)
    - Buffer management to prevent data overflow
    - Type conversion and error handling

Typical Usage:
    # USB Serial
    usb = USBSerialInterface(s_port="COM3", s_baudrate=115200, 
                              s_bytesize=8, s_timeout=5)
    data = usb.readSerialPort()  # Returns dictionary
    
    # Bluetooth Serial
    bt = BluetoothInterface(s_port="COM14", s_baudrate=115200)
    data = bt.readBluetoothSerialPort(n_elements=4)  # Returns list

Author: Gustavo Eismann
Date: 05/jan/2026
"""

from serial import Serial
from serial import SerialException
from ast import literal_eval  # Safely evaluate dictionary strings


# ========================================================================
# USB SERIAL INTERFACE CLASS
# ========================================================================

class USBSerialInterface():
    """
    USB Serial interface for ESP32 IMU data acquisition.
    
    This class handles wired USB serial communication with the ESP32 motion
    controller. It receives IMU sensor data formatted as Python dictionary
    strings and automatically handles connection errors, data validation,
    and buffer management.
    
    Data Format Expected:
        Dictionary string transmitted over serial, e.g.:
        "{'ax': 1.23, 'ay': -0.45, 'az': 9.81, 'vx': 0.0, ...}\n"
    
    The class automatically:
        - Recovers from port connection errors
        - Validates message format and encoding
        - Converts string to Python dictionary
        - Clears buffer to prevent data lag
    
    Attributes:
        ser (Serial): PySerial Serial object for communication
        dataDict (dict): Most recently received and validated data dictionary
    """

    def __init__(self, s_port, s_baudrate, s_bytesize, s_timeout):
        """
        Initialize USB serial connection to ESP32.
        
        Args:
            s_port (str): COM port identifier (e.g., "COM3" on Windows)
            s_baudrate (int): Baud rate (must match ESP32, typically 115200)
            s_bytesize (int): Number of data bits (typically 8)
            s_timeout (float): Read timeout in seconds (5 recommended)
        
        Raises:
            SerialException: If port cannot be opened or is already in use
        
        Note:
            If the port is already open, this method attempts to close
            and reopen it automatically.
        """
        try:
            # Attempt to open serial port with specified parameters
            self.ser = Serial(port=s_port, baudrate=s_baudrate, bytesize=s_bytesize, timeout=s_timeout)
        """
        Safely close serial connection and clean up resources.
        
        Clears the input buffer to discard any pending unread data,
        then closes the serial port to release the resource for other
        applications.
        
        This method should always be called when finished with the
        serial interface to prevent port locking issues.
        
        Note:
            Call this in a finally block or exception handler to ensure
            proper cleanup even if errors occur.
        """
        print(f"Clearing {self.ser.port} serial input buffer. Closing serial port.")
        self.ser.reset_input_buffer()  # Discard any buffered data
        self.ser.close()                # Release port
        print(f"{self.ser.port} closed successfully"ection failed - attempt recovery
            print(f"SerialException: {e1}")

            if Serial(port=s_port, baudrate=s_baudrate, bytesize=s_bytesize, timeout=s_timeout).isOpen():
                # Port is already open (possibly by another process)
                print(f"{s_port} is already open. Attempting to reopen...")
            else:
                # Port exists but connection failed - try close and reopen
                Serial(port=s_port, baudrate=s_baudrate, bytesize=s_bytesize, timeout=s_timeout).close()
                # Reopen serial port
                self.ser = Serial(port=s_port, baudrate=s_baudrate, bytesize=s_bytesize, timeout=s_timeout)
        """
        Validate and parse serial message into Python dictionary.
        
        This method performs three-stage validation:
        1. Decode: Convert bytes to UTF-8 string
        2. Parse: Evaluate string as Python dictionary literal
        3. Store: Save validated dictionary to self.dataDict
        
        Args:
            serial_msg (bytes): Raw bytes received from serial port
                               Expected format: b"{'key': value, ...}\n"
        ==================== Stage 2: Parse string to dictionary ====================
        try:
            # Safely evaluate string as Python literal (dictionary)
            # literal_eval only accepts valid Python literal structures (safe)
            self.dataDict = literal_eval(decoded_bytes)

        except SyntaxError as e1:
            # String is not valid Python dictionary syntax
            print(f"SyntaxError: Invalid dictionary format - {e1}")
            print(f"Received: {decoded_bytes}")
            return False

        except ValueError as e2:
            # String contains invalid values or malformed literals
            print(f"ValueError: Invalid dictionary values - {e2}")
            print(f"Received: {decoded_bytes}")
            return False

        # ==================== Stage 3: Success ====================
        # Message successfully validated and parsedbytes to string ====================
        try:
            # Remove trailing newline (\n) and decode UTF-8
            decoded_bytes = serial_msg[0:len(serial_msg) - 1].decode("utf-8")
        except UnicodeDecodeError:
            # Decoding failed - corrupted data or wrong encoding
            print("UnicodeDecodeError: Unable to decode serial message")port))
        self.ser.reset_input_buffer()
        self.ser.close()


    def handleSerialDictionary(self, serial_msg):
        ''' Check if the data coming from Serial Port is useful or not.
            If it is useful, store it into the class variable dataList.
        '''

        # Try to decode message to String
        try:
            decoded_bytes = serial_msg[0:len(serial_msg) - 1].decode("utf-8")
        except UnicodeDecodeError:
            # Return False if the String cannot be decoded according to utf-8 standard
            return False

        # Split String into a Dictionary
        try:
            self.dataDict = literal_eval(decoded_bytes)

        except SyntaxError as e1:
            print(e1)
            print(decoded_bytes)
            return False

        except ValueError as e2:
            print(e2)
            print(decoded_bytes)
            return False

        # If everything went right, return True
        return True


    def readSerialPort(self):
        ''' Read Serial Port and check if the message is valid. '''

        while True:
            ser_bytes = self.ser.readline()

            if (ser_bytes != b''):
                if self.handleSerialDictionary(ser_bytes):
                    break

        self.ser.reset_input_buffer()
        return self.dataDict



class BluetoothInterface():

    def __init__(self, s_port, s_baudrate):
        ''' Constructor Method to the Sensor Module Bluetooth Interface. '''

        self.btSerial = Serial(s_port, s_baudrate)
        self.dataList = []


    def handleBluetoothMessage(self, bluetooth_msg, n_elements):
        ''' Check if the data coming from Bluetooth Serial Port is useful or not.
            If it is useful, store it into the class variable dataList.
        '''

        # Try to decode message to String
        try:
            decoded_bytes = bluetooth_msg[0:len(bluetooth_msg) - 1].decode("utf-8")
        except UnicodeDecodeError:
            # Return False if the String cannot be decoded according to utf-8 standard
            return False

        # Split String into a List
        self.dataList = decoded_bytes.split(",")

        # Check if the list has 'n_elements' elements
        del self.dataList[-1]
        if len(self.dataList) != n_elements:
            # Return False if the number of measurements is different than 'n_elements'
            return False

        # Try to convert the List elements into floats
        for i in range(n_elements):
            try:
                self.dataList[i] = float(self.dataList[i])
            except ValueError:
                # Return False if the String cannot be converted into float
                return False

        # If everything went right, return True
        return True


    def readBluetoothSerialPort(self, n_elements):
        ''' Read Bluetooth Serial Port and check if the message is valid. '''

        while True:
            ser_bytes = self.btSerial.readline()

            if (ser_bytes != b''):
                if self.handleBluetoothMessage(ser_bytes, n_elements):
                    break

        return self.dataList


    def requestData(self, n_elements):
        # Send request
        self.btSerial.write(b'G')

        while True:
            ser_bytes = self.btSerial.readline()
#             self.btSerial.write(b'K')
#             print(ser_bytes)

            if (ser_bytes != b''):
                if self.handleBluetoothMessage(ser_bytes, n_elements):
                    break

        return self.dataList
