"""
Dobot Magician Control Script

This module controls the Dobot Magician robotic arm through the Dobot DLL API.
It demonstrates basic operations including connection, homing, and point-to-point motion.
"""

import threading
import DobotDllType as dType

# Connection status dictionary for human-readable error messages
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

# Load the Dobot DLL into memory and get the CDLL instance
api = dType.load()

# Establish connection with Dobot (empty port string = auto-detect, 115200 baud rate)
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:", CON_STR[state])

# Only proceed if connection was successful
if state == dType.DobotConnect.DobotConnect_NoError:

    # Clear the command queue to ensure no pending commands
    dType.SetQueuedCmdClear(api)

    # Configure motion parameters (velocity and acceleration for each axis)
    # HOME parameters: velocity and acceleration for homing sequence (all set to 200)
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued=1)

    # PTP Joint parameters: velocity and acceleration for each joint (J1-J4)
    # Format: velocityJ1, velocityJ2, velocityJ3, velocityJ4, accelJ1, accelJ2, accelJ3, accelJ4
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)

    # PTP Common parameters: velocity ratio (100%) and acceleration ratio (100%)
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)

    # Execute homing sequence to calibrate the robot's position
    dType.SetHOMECmd(api, temp=0, isQueued=1)

    # Queue a series of PTP (Point-to-Point) movements
    # Creates 5 alternating movements with +50mm and -50mm offsets
    for i in range(0, 5):
        if i % 2 == 0:
            offset = 50
        else:
            offset = -50

        # Send PTP command in MOVL (linear movement) mode
        # Parameters: mode, x, y, z, r (rotation)
        # lastIndex tracks the final command in the queue
        lastIndex = dType.SetPTPCmd(
            api,
            dType.PTPMode.PTPMOVLXYZMode,
            200 + offset,  # X coordinate
            offset,         # Y coordinate
            offset,         # Z coordinate
            offset,         # R (end-effector rotation)
            isQueued=1
        )[0]

    # Start executing the queued commands
    dType.SetQueuedCmdStartExec(api)

    # Wait for all commands to complete
    # Polls current queue index until it reaches the last command
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)  # Sleep for 100ms between checks

    # Stop command queue execution
    dType.SetQueuedCmdStopExec(api)

# Disconnect from Dobot and release resources
dType.DisconnectDobot(api)
