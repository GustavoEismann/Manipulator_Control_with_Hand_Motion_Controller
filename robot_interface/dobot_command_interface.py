"""
Dobot Motion Control Interface for IMU-Based Manipulation

This module provides a high-level interface for controlling the Dobot Magician
robotic arm using IMU (Inertial Measurement Unit) sensor data. It translates
acceleration and velocity measurements from an ESP32 motion controller into
Dobot jog commands, enabling intuitive gesture-based control.

Key Features:
    - IMU-to-robot motion mapping (acceleration → velocity → jog commands)
    - Dominant axis detection for intuitive single-axis movements
    - Configurable velocity and acceleration scaling gains
    - Home position management and safe return functionality
    - Motion state tracking to prevent unnecessary command updates

Typical Workflow:
    1. Initialize DobotKinematics with API and gain parameters
    2. Set home position for safe return
    3. Read IMU measurements (acceleration/velocity)
    4. Call setMeasurementsToJOG() to translate to robot motion
    5. Robot moves in direction of dominant acceleration axis

Author: Gustavo Eismann
Date: 05/jan/2026
"""

import DobotDllType as dType
from enum import IntEnum


class Direction(IntEnum):
    """
    Enumeration of possible robot motion directions.
    
    Used to track and command single-axis jog movements in Cartesian space.
    Each direction corresponds to a specific Dobot jog command value.
    
    Attributes:
        IDLE: No movement (all axes stopped)
        X_UP: Positive X-axis motion (forward)
        X_DN: Negative X-axis motion (backward)
        Y_UP: Positive Y-axis motion (left)
        Y_DN: Negative Y-axis motion (right)
        Z_UP: Positive Z-axis motion (up)
        Z_DN: Negative Z-axis motion (down)
        R_UP: Positive rotation (counterclockwise)
        R_DN: Negative rotation (clockwise)
    """
    IDLE = 0   # Stationary (no jogging)
    X_UP = 1   # +X direction
    X_DN = 2   # -X direction
    Y_UP = 3   # +Y direction
    Y_DN = 4   # -Y direction
    Z_UP = 5   # +Z direction
    Z_DN = 6   # -Z direction
    R_UP = 7   # +R rotation
    R_DN = 8   # -R rotation


class DobotKinematics():
    """
    High-level interface for IMU-based control of Dobot Magician robot.
    
    This class manages the translation of IMU sensor data (acceleration and
    velocity) into Dobot jog commands. It implements a dominant-axis algorithm
    that selects the axis with the highest acceleration for single-axis motion,
    making the control intuitive for gesture-based manipulation.
    
    The class tracks both Cartesian position (x, y, z, r) and joint angles
    (j1-j4), manages home position, and implements motion state tracking to
    minimize redundant command transmission.
    
    Attributes:
        api: Dobot DLL API handle
        x, y, z, r: Current Cartesian position [mm, mm, mm, deg]
        j1, j2, j3, j4: Current joint angles [deg]
        ka, kv: Acceleration and velocity scaling gains
        fa, fv: Fixed acceleration and velocity (reserved)
        prev_move, curr_move: Motion state tracking
    """

    def __init__(self, api, ka=1, kv=1, fa=200, fv=200):
        """
        Initialize Dobot kinematics controller.
        
        Args:
            api: Dobot DLL API object (from dType.load())
            ka (float): Acceleration gain multiplier (scales IMU acceleration)
                       Higher values = more aggressive motion
                       Default: 1 (no scaling)
            kv (float): Velocity gain multiplier (scales IMU velocity)
                       Higher values = faster motion
                       Default: 1 (no scaling)
            fa (float): Fixed acceleration parameter (reserved for future use)
            fv (float): Fixed velocity parameter (reserved for future use)
        
        Note:
            Gain tuning guidelines:
            - Start with ka=1, kv=1 for testing
            - Increase kv for faster response to motion
            - Increase ka for more aggressive acceleration-based control
            - Typical range: 0.1 to 10.0 depending on sensor characteristics
        """
        self.api = api  # Dobot DLL API handle

        # ==================== Cartesian Position (Work Space Coordinates) ====================
        # Current end-effector position in millimeters and degrees
        self.x = 0.0  # X coordinate [mm] (forward/back)
        self.y = 0.0  # Y coordinate [mm] (left/right)
        self.z = 0.0  # Z coordinate [mm] (up/down)
        self.r = 0.0  # Rotation angle [deg] (end-effector orientation)

        # ==================== Joint Position (Configuration Space) ====================
        # Current joint angles in degrees
        self.j1 = 0.0  # Base rotation joint [deg]
        self.j2 = 0.0  # Shoulder joint [deg]
        self.j3 = 0.0  # Elbow joint [deg]
        self.j4 = 0.0  # Wrist rotation joint [deg]

        # ==================== Motion State Tracking ====================
        # Track movement direction to avoid redundant command updates
        self.prev_move = Direction.IDLE  # Previous jog direction
        self.curr_move = Direction.IDLE  # Current jog direction

        # ==================== Control Gains ====================
        # Scaling factors for IMU measurements
        self.ka = ka  # Acceleration gain (scales ax, ay, az)
        self.kv = kv  # Velocity gain (scales vx, vy, vz)
        self.fa = fa  # Fixed acceleration (reserved)
        self.fv = fv  # Fixed velocity (reserved)


    def getPose(self):
        """
        Query and update current robot pose from Dobot.
        
        Retrieves the current Cartesian position (x, y, z, r) and joint angles
        (j1-j4) from the robot and updates instance variables.
        
        This method should be called:
        - During initialization to establish current position
        - Before executing movements to ensure accurate state
        - After manual jogging or external position changes
        
        Updates:
            self.x, self.y, self.z, self.r: Cartesian coordinates
            self.j1, self.j2, self.j3, self.j4: Joint angles
        """
        # Query robot and unpack all 8 pose values (4 Cartesian + 4 joint)
        [self.x, self.y, self.z, self.r, self.j1, self.j2, self.j3, self.j4] = dType.GetPose(self.api)


    def setHomePosition(self, type=None, xJ1=None, yJ2=None, zJ3=None, rJ4=None):
        """
        Define the home position for safe return.
        
        Sets the reference position that the robot will return to when
        gotoHome() is called. Position can be specified in either Cartesian
        coordinates or joint angles.
        
        Args:
            type (str): Position specification type
                       "XYZR" - Cartesian coordinates (x, y, z, rotation)
                       "JOINT" - Joint space (j1, j2, j3, j4)
                       None - Use current robot position
            xJ1 (float): X coordinate [mm] or Joint1 angle [deg]
            yJ2 (float): Y coordinate [mm] or Joint2 angle [deg]
            zJ3 (float): Z coordinate [mm] or Joint3 angle [deg]
            rJ4 (float): Rotation [deg] or Joint4 angle [deg]
        
        Examples:
            # Set home to specific Cartesian position
            dobot.setHomePosition("XYZR", 200, 0, 50, 0)
            
            # Set home to specific joint configuration
            dobot.setHomePosition("JOINT", 0, 45, 45, 0)
            
            # Use current position as home
            dobot.setHomePosition()
        """
        # Check if all parameters are provided
        if (type is not None and xJ1 is not None and yJ2 is not None and zJ3 is not None and rJ4 is not None):
            if (type == "XYZR"):
                # Set home position in Cartesian coordinates
                self.x = xJ1  # X position [mm]
                self.y = yJ2  # Y position [mm]
                self.z = zJ3  # Z position [mm]
                self.r = rJ4  # Rotation [deg]

            if (type == "JOINT"):
                # Set home position in joint space
                self.j1 = xJ1  # Base joint [deg]
                self.j2 = yJ2  # Shoulder joint [deg]
                self.j3 = zJ3  # Elbow joint [deg]
                self.j4 = rJ4  # Wrist joint [deg]

        else:
            # No parameters provided - use current robot position as home
            self.getPose()


    def gotoHome(self):
        """
        Safely return robot to home position.
        
        Executes a controlled sequence to stop any ongoing motion and move
        the robot to the previously defined home position using PTP (Point-to-Point)
        linear motion mode.
        
        Safety sequence:
            1. Clear all queued commands
            2. Stop all jogging motion
            3. Wait for motion to stabilize
            4. Execute PTP move to home coordinates
        
        Note:
            - Uses PTP mode 2 (MOVL - linear interpolation)
            - Movement speed: 100% of configured PTP parameters
            - Home position must be set via setHomePosition() first
            - Ensure home position is reachable and collision-free
        """
        # Step 1: Clear command queue to remove any pending commands
        dType.SetQueuedCmdClear(self.api)

        # Step 2: Stop all jogging motion by zeroing velocity/acceleration
        dType.SetJOGCoordinateParams(self.api, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        dType.SetWAITCmd(self.api, 10, 0)  # Wait 10ms for parameters to apply
        
        # Send idle jog command to stop motion
        dType.SetJOGCmd(self.api, 0, 0, 0)
        dType.SetWAITCmd(self.api, 10, 0)  # Wait 10ms for motion to stop

        # Step 3: Configure PTP parameters and move to home position
        # Parameters: xyzVelocity=100%, rVelocity=100%, xyzAccel=100%, rAccel=100%
        dType.SetPTPCoordinateParams(self.api, 100, 100, 100, 100, 0)
        
        # Execute PTP move: mode=2 (MOVL - linear), target=(x,y,z,r), non-queued
        dType.SetPTPCmd(self.api, 2, self.x, self.y, self.z, self.r, 0)


    def breakJogging(self):
        """
        Emergency stop - immediately halt all robot motion.
        
        Stops jogging motion by zeroing all velocity and acceleration parameters
        and sending an idle jog command. This method provides a quick way to
        stop the robot during IMU-controlled motion.
        
        Use cases:
            - Emergency stop during gesture control
            - User releases control input
            - Sensor data becomes invalid or unreliable
            - Collision avoidance
        
        Note:
            This is a soft stop (deceleration, not instant). For true emergency
            stop, use the robot's hardware emergency stop button.
        """
        print("Break Enabled")  # Debug output
        
        # Zero all jog parameters (stops motion gracefully)
        dType.SetJOGCoordinateParams(self.api, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        
        # Send idle jog command
        dType.SetJOGCmd(self.api, 0, 0, 0)
        
        # Short wait to ensure command is processed
        dType.SetWAITCmd(self.api, 10, 0)


    def setMeasurementsToJOG(self, meas):
        """
        Translate IMU measurements into Dobot jog commands.
        
        This is the core method that implements gesture-based robot control.
        It processes acceleration and velocity data from the IMU, applies
        scaling gains, determines the dominant motion axis, and generates
        appropriate jog commands to move the robot.
        
        Algorithm:
            1. Extract and scale velocity/acceleration from IMU
            2. Compute magnitude for each axis
            3. Find dominant axis (highest acceleration magnitude)
            4. Determine direction (positive or negative)
            5. Send jog command for dominant axis only
            6. Track motion state to minimize redundant commands
        
        Args:
            meas (dict): IMU measurement dictionary containing:
                'vx_est', 'vy_est', 'vz_est': Estimated velocities [mm/s or m/s]
                'ax', 'ay', 'az': Accelerations [m/s² or g]
                'dt': Time step (not currently used)
        
        Motion mapping:
            - Positive X acceleration → Robot moves +X (forward)
            - Negative Y acceleration → Robot moves -Y (right)
            - Positive Z acceleration → Robot moves +Z (up)
            etc.
        
        Note:
            Only the axis with highest acceleration magnitude is commanded,
            resulting in intuitive single-axis motion for gesture control.
        """

        # ==================== Step 1: Extract and scale measurements ====================
        # Apply velocity gain to estimated velocities from Kalman filter
        vx = meas.get('vx_est') * self.kv  # X velocity (scaled)
        vy = meas.get('vy_est') * self.kv  # Y velocity (scaled)
        vz = meas.get('vz_est') * self.kv  # Z velocity (scaled)
        
        # Apply acceleration gain to raw accelerometer data
        ax = meas.get('ax') * self.ka  # X acceleration (scaled)
        ay = meas.get('ay') * self.ka  # Y acceleration (scaled)
        az = meas.get('az') * self.ka  # Z acceleration (scaled)

        # Compute magnitude (absolute value) for dominant axis detection
        m_ax = abs(ax)  # |acceleration_x|
        m_ay = abs(ay)  # |acceleration_y|
        m_az = abs(az)  # |acceleration_z|

        m_vx = abs(vx)  # |velocity_x|
        m_vy = abs(vy)  # |velocity_y|
        m_vz = abs(vz)  # |velocity_z|

        # ==================== Step 2: Configure jog parameters ====================
        # Set velocity and acceleration for each axis based on IMU measurements
        # Parameters: x_vel, x_acc, y_vel, y_acc, z_vel, z_acc, r_vel, r_acc, queued
        # Rotation (r) parameters set to 0 (no rotational control in this implementation)
        dType.SetJOGCoordinateParams(self.api, m_vx, m_ax, m_vy, m_ay, m_vz, m_az, 0, 0, 0)
        
        # Debug output: display current velocity and acceleration values
        print("xVel: {:7.1f} \t yVel: {:7.1f} \t zVel: {:7.1f}".format(m_vx, m_vy, m_vz))
        print("xAcc: {:7.1f} \t yAcc: {:7.1f} \t zAcc: {:7.1f}".format(m_ax, m_ay, m_az))

        # ==================== Step 3: Check for zero motion condition ====================
        # If all accelerations are below threshold, robot should stop
        if (m_ax < 1.0 and m_ay < 1.0 and m_az < 1.0):
            # All accelerations negligible → idle state
            print("Zero acceleration - Stopping robot")
            
            # Send idle jog command (stop motion)
            dType.SetJOGCmd(self.api, 0, 0, 0)
            dType.SetWAITCmd(self.api, 100, 0)  # Wait 100ms for smooth stop

        else:
            # ==================== Step 4: Dominant axis selection ====================
            # Determine which axis has the highest acceleration magnitude
            # This implements single-axis motion for intuitive gesture control
            
            if (m_ax > m_ay):
                # X or Z is dominant (X > Y)
                if (m_ax > m_az):
                    # X is dominant axis (|ax| > |ay| and |ax| > |az|)
                    if (ax > 0):
                        self.curr_move = Direction.X_UP  # Positive X acceleration
                    else:
                        self.curr_move = Direction.X_DN  # Negative X acceleration
                else:
                    # Z is dominant axis (|az| > |ax| > |ay|)
                    if (az > 0):
                        self.curr_move = Direction.Z_UP  # Positive Z acceleration
                    else:
                        self.curr_move = Direction.Z_DN  # Negative Z acceleration
            else:
                # Y or Z is dominant (Y ≥ X)
                if (m_ay > m_az):
                    # Y is dominant axis (|ay| > |ax| and |ay| > |az|)
                    if (ay > 0):
                        self.curr_move = Direction.Y_UP  # Positive Y acceleration
                    else:
                        self.curr_move = Direction.Y_DN  # Negative Y acceleration
                else:
                    # Z is dominant axis (|az| > |ay| ≥ |ax|)
                    if (az > 0):
                        self.curr_move = Direction.Z_UP  # Positive Z acceleration
                    else:
                        self.curr_move = Direction.Z_DN  # Negative Z acceleration

            # ==================== Step 5: Motion state tracking and command update ====================
            # Optimize command transmission by only sending new commands when direction changes
            
            if self.curr_move != self.prev_move:
                # Direction has changed → need to update jog command
                print(f"Direction change: {self.prev_move} → {self.curr_move}")
                
                # Stop current motion smoothly
                dType.SetWAITCmd(self.api, 100, 0)      # Wait for stability
                dType.SetJOGCmd(self.api, 0, 0, 0)      # Send stop command
                
                # Start new direction
                dType.SetJOGCmd(self.api, 0, self.curr_move, 0)  # Command new direction
                dType.SetWAITCmd(self.api, 100, 0)      # Wait for motion to start
                
                # Update state tracking
                self.prev_move = self.curr_move
            else:
                # Direction unchanged → robot continues current motion
                # Only send wait command to maintain timing
                dType.SetWAITCmd(self.api, 100, 0)
