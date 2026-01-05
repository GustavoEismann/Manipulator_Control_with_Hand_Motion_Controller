"""
Kalman Filter Model for Velocity Estimation from Accelerometer Data

This module implements a discrete-time Kalman filter for estimating velocity 
from noisy accelerometer measurements. The filter fuses acceleration data over 
time to produce optimal velocity estimates by balancing process uncertainty 
and measurement noise.

The implementation follows the standard Kalman filter recursive algorithm:
1. Prediction step: Estimate next state and covariance
2. Update step: Incorporate measurement and compute optimal gain
3. Correction step: Update state estimate with weighted measurement

Physical Model:
- State vector: [vx, vy, vz] (velocity in 3D space)
- Measurements: [ax, ay, az] (acceleration from IMU)
- Integration: v(k) = v(k-1) + a(k-1) * dt

Typical use case:
    filter = KalmanFilterModel(a_std_dev=[0.1, 0.1, 0.15])
    for measurement in sensor_data:
        filter.KalmanFilterUpdate(measurement)
        velocity = filter.getFilteredData()

Author: Gustavo Eismann
Date: 05/jan/2026
References:
    - Kalman, R. E. (1960). A New Approach to Linear Filtering and Prediction Problems
    - OpenIMU Process Covariance: https://openimu.readthedocs.io/en/latest/algorithms/Process_Covariance.html
"""

import numpy as np

# Suppress divide-by-zero and invalid operation warnings from numpy
# (handled explicitly in the filter implementation)
np.seterr(divide='ignore', invalid='ignore')


class KalmanFilterModel():
    """
    Discrete-time Kalman Filter for 3D velocity estimation.
    
    This class implements a linear Kalman filter that estimates velocity
    from accelerometer measurements using a constant velocity motion model
    with acceleration as the control input.
    
    State-space model:
        x(k) = A*x(k-1) + B*u(k) + w(k)    # State prediction
        y(k) = C*z(k) + v(k)                # Measurement model
    
    where:
        x = [vx, vy, vz]^T  : velocity state vector
        u = [ax, ay, az]^T  : acceleration input (from previous step)
        z = [ax, ay, az]^T  : current acceleration measurement
        w ~ N(0, Q)         : process noise
        v ~ N(0, R)         : measurement noise
    
    Attributes:
        X_prev (ndarray): Current velocity estimate [vx, vy, vz]
        P_prev (ndarray): State covariance matrix (uncertainty)
        K (ndarray): Kalman gain (optimal weighting)
        dt (float): Time step between measurements
    """

    def __init__(self, a_std_dev=[0.0, 0.0, 0.0], isQkZero=False):
        """
        Initialize the Kalman Filter with noise parameters.
        
        Args:
            a_std_dev (list): Standard deviation of accelerometer noise [σ_ax, σ_ay, σ_az]
                            Units: m/s² or g's depending on sensor configuration
                            Used to compute process noise covariance Q
            isQkZero (bool): If True, process noise Q is set to zero (assumes perfect model)
                           If False, Q is computed from accelerometer noise statistics
                           Default: False (recommended for real sensors)
        
        Note:
            Higher a_std_dev values indicate noisier sensors and lead to:
            - Larger process noise Q
            - Lower Kalman gain K (trusts measurements less)
            - Smoother but potentially lagged velocity estimates
        """

        # ==================== Global Configuration Parameters ====================
        self.dt = 0.1                                           # Elapsed-Time between one iteration and another (Δt)
        self.I3 = np.identity(3)                                # [3x3] Identity Matrix
        self.DEG_TO_RAD = np.pi / 180.0                         # Scalar Factor to convert Degrees to Radians
        self.RAD_TO_DEG = 180.0 / np.pi                         # Scalar Factor to convert Radians to Degrees
        self.isQkZero = isQkZero                                # Boolean variable to define if Q_k will be zero, or it should be computed

        # ==================== Sensor Noise Characteristics ====================
        # Accelerometer measurement noise (from sensor calibration or datasheet)
        self.aSD = np.array([[a_std_dev[0]],                    # X-axis acceleration std dev σ_ax [m/s²]
                             [a_std_dev[1]],                    # Y-axis acceleration std dev σ_ay [m/s²]
                             [a_std_dev[2]]])                   # Z-axis acceleration std dev σ_az [m/s²]

        # ==================== State Prediction Variables ====================
        # These variables implement: x(k|k-1) = A*x(k-1|k-1) + B*u(k) + w(k)
        self.X_kp = np.array([[0.0], [0.0], [0.0]])             # Predicted State Matrix
        self.A = np.identity(3)                                 # System Model Dynamics Matrix
        self.X_prev = np.array([[0.0], [0.0], [0.0]])           # Initial State Matrix / Previous Predicted State Matrix
        self.B = np.identity(3)                                 # System Model Input Matrix
        self.u_k = np.array([[0.0], [0.0], [0.0]])              # System Control Variables
        self.w_k = np.array([[0.0], [0.0], [0.0]])              # System Model State Noise Matrix

        self.P_kp = np.identity(3)                              # Predicted Process Covariance Matrix
        self.P_prev = np.identity(3)                            # Initial Covariance Matrix / Previous Predicted Process Covariance Matrix
        self.Q_k = 0.0                                          # Process Noise Covariance Matrix

        self.K = np.identity(3)                                 # Kalman Gain Matrix
        self.H = np.identity(3)                                 # Observation Matrix (in this case, it is an Identity Matrix)
        self.R = np.identity(3)                                 # Sensor Noise Covariance Matrix

        self.Y_k = np.array([[0.0], [0.0], [0.0]])              # Current Observed State Matrix
        self.Y_km_prev = np.array([[0.0], [0.0], [0.0]])        # Previous Observed State Matrix
        self.Y_km = np.array([[0.0], [0.0], [0.0]])             # Current Measurements Matrix
        self.C = np.identity(3)                                 # Output Matrix (Identity Matrix, for now)
        self.Z_k = 0.0                                          # Observation Noise Matrix


    # ========================================================================
    # KALMAN FILTER ALGORITHM - STEP-BY-STEP IMPLEMENTATION
    # ========================================================================

    def ComputePredictedStateMatrix(self):
        """
        PREDICTION STEP 1: Predict next state using motion model.
        
        Computes the a priori state estimate x(k|k-1) by propagating the
        previous corrected state forward in time using the system dynamics
        and control input.
        
        Mathematical model:
            x(k|k-1) = A·x(k-1|k-1) + B·u(k) + w(k)
        
        where:
            x(k|k-1)   : Predicted state (velocity) at time k
            A          : State transition matrix (identity for velocity)
            x(k-1|k-1) : Corrected state from previous time step
            B          : Control matrix (scaled by dt for acceleration integration)
            u(k)       : Control input (previous acceleration measurement)
            w(k)       : Process noise ~ N(0, Q)
        
        Physical interpretation:
            v(k) = v(k-1) + a(k-1) * dt
            Current velocity = Previous velocity + Acceleration × Time
        """
        # Scale control matrix by time step for acceleration integration

        self.B = self.B * self.dt

        self.X_kp = self.A.dot(self.X_prev) + self.B.dot(self.u_k) + self.w_k


    def ComputeProcessNoiseCovarianceMatrix(self):
        """
        PREDICTION STEP 2: Compute process noise covariance matrix Q(k).
        
        The process noise accounts for uncertainty in the motion model due to:
        - Unmodeled accelerations (vibrations, external forces)
        - Integration errors from discrete-time approximation
        - Sensor noise propagating through integration
        
        Mathematical formula (from OpenIMU methodology):
            Q(k) = (σ_a · dt)² · I₃
        
        where:
            Q(k)  : Process noise covariance [3×3 matrix]
            σ_a   : Accelerometer standard deviation [m/s²]
            dt    : Time step [seconds]
            I₃    : 3×3 identity matrix
        
        Derivation:
            Since v = ∫a·dt, the variance in velocity due to acceleration noise is:
            Var(v) = Var(a·dt) = Var(a)·dt² = σ_a²·dt²
        
        Reference:
            OpenIMU Process Covariance
            https://openimu.readthedocs.io/en/latest/algorithms/Process_Covariance.html
        """

        self.Q_k = ((self.aSD * self.dt) * (self.aSD * self.dt)) * (self.I3)
        # self.Q_k = 0.0


    def ComputePredictedProcessCovarianceMatrix(self):
        """
        PREDICTION STEP 3: Predict state covariance (uncertainty propagation).
        
        Propagates the uncertainty from the previous time step forward,
        accounting for both state transition uncertainty and process noise.
        
        Mathematical formula:
            P(k|k-1) = A·P(k-1|k-1)·A^T + Q(k)
        
        where:
            P(k|k-1)   : Predicted covariance (uncertainty in predicted state)
            A          : State transition matrix
            P(k-1|k-1) : Previous corrected covariance
            A^T        : Transpose of state transition matrix
            Q(k)       : Process noise covariance
        
        Note:
            The subsequent code forces P_kp to be diagonal by zeroing off-diagonal
            elements. This assumes no correlation between velocity components,
            simplifying computation but potentially reducing filter optimality.
        """

        # Propagate covariance: P(k|k-1) = A·P(k-1|k-1)·A^T + Q(k)
        self.P_kp = self.A.dot(self.P_prev.dot(np.transpose(self.A))) + self.Q_k

        # Force covariance to be diagonal (assume no inter-axis correlation)
        # This simplifies computation and improves numerical stability
        n1 = len(self.P_kp)
        d1 = self.P_kp.ravel()[::n1 + 1]    # Extract diagonal elements
        values = d1.copy()                   # Save diagonal values
        self.P_kp[:, :] = 0.0                # Zero entire matrix
        d1[:] = values                       # Restore diagonal


    def ComputeKalmanGain(self):
        """
        UPDATE STEP 1: Compute optimal Kalman gain.
        
        The Kalman gain determines how much to trust the measurement versus
        the prediction. It provides optimal weighting that minimizes the
        mean squared error of the state estimate.
        
        Mathematical formula:
                    P(k|k-1)·H^T
            K(k) = ─────────────────────
                   H·P(k|k-1)·H^T + R
        
        where:
            K(k)       : Kalman gain [3×3 matrix]
            P(k|k-1)   : Predicted covariance (prediction uncertainty)
            H          : Observation matrix (measurement model)
            H^T        : Transpose of observation matrix
            R          : Measurement noise covariance (sensor uncertainty)
        
        Interpretation:
            - Large P (uncertain prediction) → Large K → Trust measurement more
            - Large R (noisy sensor) → Small K → Trust prediction more
            - K acts as optimal blending weight: x = x_pred + K·(z - H·x_pred)
        """

        # Compute Kalman gain: K = P(k|k-1)·H^T / [H·P(k|k-1)·H^T + R]
        num = self.P_kp.dot(np.transpose(self.H))                    # Numerator
        den = self.H.dot(self.P_kp.dot(np.transpose(self.H))) + self.R  # Denominator (innovation covariance)
        self.K = np.divide(num, den)

        # Force Kalman gain to be diagonal (consistent with diagonal covariance assumption)
        n2 = len(self.K)
        d2 = self.K.ravel()[::n2 + 1]    # Extract diagonal elements
        values = d2.copy()                # Save diagonal values
        self.K[:, :] = 0.0                # Zero entire matrix
        d2[:] = values                    # Restore diagonal


    def ComputeObservedState(self, isInitial=False):
        """
        UPDATE STEP 2: Process measurement into observation space.
        
        Converts the raw sensor measurement into an observation that can be
        compared with the predicted state. For this velocity estimator,
        integrates acceleration to get a velocity observation.
        
        Mathematical formula:
            y(k) = C·z(k) + v(k)
        
        where:
            y(k)  : Observed state (velocity from integrated acceleration)
            C     : Output matrix (scaled by dt to convert acceleration to velocity)
            z(k)  : Raw measurement (acceleration from sensor)
            v(k)  : Observation noise ~ N(0, R)
        
        Physical interpretation:
            y(k) = a(k) · dt
            Velocity increment = Acceleration × Time step
        
        Args:
            isInitial (bool): Reserved for initial state setup (currently unused)
        """

        # Scale output matrix by dt to convert acceleration to velocity
        self.C = np.identity(3) * self.dt

        # Compute observation: y(k) = C·z(k) + v(k)
        self.Y_k = self.C.dot(self.Y_km) + self.Z_k


    def ComputeUpdatedStateAndProcessCovariance(self):
        """
        UPDATE STEP 3: Correct prediction using measurement (a posteriori estimate).
        
        Combines the predicted state with the measurement using the Kalman gain
        to produce an optimal estimate. Also updates the covariance to reflect
        reduced uncertainty after incorporating the measurement.
        
        Mathematical formulas:
            x(k|k) = x(k|k-1) + K·[y(k) - H·x(k|k-1)]
            P(k|k) = [I - K·H]·P(k|k-1)
        
        where:
            x(k|k)     : Corrected state estimate (optimal velocity)
            x(k|k-1)   : Predicted state
            K          : Kalman gain (optimal weighting)
            y(k)       : Observation from measurement
            H          : Observation matrix
            [y(k) - H·x(k|k-1)] : Innovation (measurement residual)
            P(k|k)     : Corrected covariance (reduced uncertainty)
            P(k|k-1)   : Predicted covariance
            I          : Identity matrix
        
        The innovation represents the "new information" from the measurement.
        The Kalman gain scales this innovation before adding it to the prediction.
        """

        # Compute corrected state estimate: x(k|k) = x(k|k-1) + K·innovation
        # Stored in X_prev since it becomes the "previous" state for next iteration
        innovation = self.Y_k - self.H.dot(self.X_kp)     # Measurement residual
        self.X_prev = self.X_kp + self.K.dot(innovation)  # Corrected state

        # Compute corrected covariance: P(k|k) = [I - K·H]·P(k|k-1)
        # Stored in P_prev since it becomes the "previous" covariance for next iteration
        # This is the Joseph form, which ensures numerical stability
        self.P_prev = (self.I3 - self.K.dot(self.H)).dot(self.P_kp)



    # ========================================================================
    # MAIN KALMAN FILTER UPDATE METHOD
    # ========================================================================

    def KalmanFilterUpdate(self, meas):
        """
        Execute one complete Kalman filter update cycle.
        
        This is the main method that orchestrates the entire filter operation,
        calling prediction and update steps in sequence. It processes a new
        measurement and produces an updated velocity estimate.
        
        Args:
            meas (dict): Measurement dictionary containing:
                - 'ax', 'ay', 'az': Acceleration measurements [m/s² or g]
                - 'dt': Time step since last measurement [seconds]
        
        Algorithm flow:
            1. Extract measurements and update time step
            2. Predict next state and covariance
            3. Compute optimal Kalman gain
            4. Process measurement into observation
            5. Correct state and covariance estimates
        
        After calling this method:
            - self.X_prev contains the updated velocity estimate
            - self.P_prev contains the updated uncertainty
            - Use getFilteredData() to retrieve results
        """

        # ==================== Step 0: Extract and prepare measurements ====================
        # self.Y_km[0][0] = self.Y_km_prev[0][0]
        # self.Y_km[1][0] = self.Y_km_prev[1][0]
        # self.Y_km[2][0] = self.Y_km_prev[2][0]

        self.Y_km[0][0] = meas.get('ax')
        self.Y_km[1][0] = meas.get('ay')
        self.Y_km[2][0] = meas.get('az')

        # self.u_k[0][0] = meas.get('ax')
        # self.u_k[1][0] = meas.get('ay')
        # self.u_k[2][0] = meas.get('az')

        self.u_k[0][0] = self.Y_km_prev[0][0]
        self.u_k[1][0] = self.Y_km_prev[1][0]
        self.u_k[2][0] = self.Y_km_prev[2][0]

        self.dt = meas.get('dt')

        # ==================== Step 1: Predict state ====================
        self.ComputePredictedStateMatrix()                # x(k|k-1) = A·x(k-1) + B·u(k)

        # ==================== Step 2: Predict covariance ====================
        if self.isQkZero:
            self.Q_k = 0.0                                # Assume perfect model (no process noise)
        else:
            self.ComputeProcessNoiseCovarianceMatrix()    # Q(k) = (σ_a·dt)²·I

        self.ComputePredictedProcessCovarianceMatrix()    # P(k|k-1) = A·P(k-1)·A^T + Q

        # ==================== Step 3: Compute Kalman gain ====================
        self.ComputeKalmanGain()                          # K = P(k|k-1)·H^T / [H·P(k|k-1)·H^T + R]

        # ==================== Step 4: Process measurement ====================
        self.ComputeObservedState()                       # y(k) = C·z(k)

        # ==================== Step 5: Correct state and covariance ====================
        self.ComputeUpdatedStateAndProcessCovariance()    # x(k|k) = x(k|k-1) + K·innovation
                                                          # P(k|k) = [I - K·H]·P(k|k-1)

        # Save current measurement for use as control input in next iteration
        self.Y_km_prev[0][0] = self.u_k[0][0]
        self.Y_km_prev[1][0] = self.u_k[1][0]
        self.Y_km_prev[2][0] = self.u_k[2][0]


    # ========================================================================
    # PUBLIC INTERFACE METHODS
    # ========================================================================

    def setInitialState(self, meas):
        """
        Set initial state from measurement array (legacy method).
        
        Args:
            meas (array-like): Measurement array [ax, ay, az, ..., mx, my, mz, ...]
                             Expected format: 9+ elements with IMU data
        
        Note:
            Currently not fully utilized in the filter initialization.
            The filter starts with zero velocity by default.
        """

        # Get measurements
        self.am = np.array([meas[0], meas[1], meas[2]])
        self.mm = np.array([meas[6], meas[7], meas[8]])

        # Compute the Initial Observed Quaternion to get the notion of direction
        self.ComputeObservedState(isInitial=True)

    def returnEstimatedState(self):
        """
        Get current velocity estimate.
        
        Returns:
            list: [vx, vy, vz] - Velocity components in m/s (or units matching sensor)
        """
        return [self.X_prev[0], self.X_prev[1], self.X_prev[2]]


    def printComputedVariables(self):
        """
        Print all Kalman Filter internal variables for debugging.
        
        Displays:
            - Predicted state x(k|k-1)
            - Process noise Q
            - Predicted covariance P(k|k-1)
            - Kalman gain K
            - Observation y(k)
            - Corrected state x(k|k)
            - Corrected covariance P(k|k)
        """

        print("\n X_kp = ", self.X_kp)   # Predicted State Matrix
        print("\n Q_k = ", self.Q_k)     # Predicted Process Noise Covariance Matrix
        print("\n P_kp = ", self.P_kp)   # Predicted Process Covariance Matrix
        print("\n K = ", self.K)         # Kalman Gain
        print("\n Y_k = ", self.Y_k)     # Observed State Matrix
        print("\n X_k = ", self.X_prev)  # Estimated State Matrix
        print("\n P_k = ", self.P_prev)  # Estimated Process Covariance Matrix
        print("\n--------------------------------------------------------------")


    def printVariables(self, n_iter):
        line0 = "Iteration: {:>4} - Elapsed Time from last iteration (Δt): {:.6f} s \n".format(n_iter, self.delta_t)
        line1 = "       | {:>8.5f} |         | {:>8.5f} |       | {:>8.5f}  {:>8.5f}  {:>8.5f}  {:>8.5f} |         | {:>8.5f} | \n"\
                .format(self.X_kp[0][0], self.Y_k[0][0], self.K[0][0], self.K[0][1], self.K[0][2], self.K[0][3], self.X_prev[0][0])
        line2 = "X_kp = | {:>8.5f} |   Y_k = | {:>8.5f} |   K = | {:>8.5f}  {:>8.5f}  {:>8.5f}  {:>8.5f} |   X_k = | {:>8.5f} | \n"\
                .format(self.X_kp[1][0], self.Y_k[1][0], self.K[1][0], self.K[1][1], self.K[1][2], self.K[1][3], self.X_prev[1][0])
        line3 = "       | {:>8.5f} |         | {:>8.5f} |       | {:>8.5f}  {:>8.5f}  {:>8.5f}  {:>8.5f} |         | {:>8.5f} | \n"\
                .format(self.X_kp[2][0], self.Y_k[2][0], self.K[2][0], self.K[2][1], self.K[2][2], self.K[2][3], self.X_prev[2][0])
        line4 = "       | {:>8.5f} |         | {:>8.5f} |       | {:>8.5f}  {:>8.5f}  {:>8.5f}  {:>8.5f} |         | {:>8.5f} | \n"\
                .format(self.X_kp[3][0], self.Y_k[3][0], self.K[3][0], self.K[3][1], self.K[3][2], self.K[3][3], self.X_prev[3][0])

        print(line0 + line1 + line2 + line3 + line4)


    def getFilteredData(self):
        """
        Get filtered velocity estimates with measurements.
        
        Returns:
            dict: Contains:
                - 'dt': Time step [seconds]
                - 'vx_est', 'vy_est', 'vz_est': Estimated velocities (rounded to 0.1)
                - 'ax', 'ay', 'az': Raw acceleration measurements
        
        Typical usage:
            data = filter.getFilteredData()
            print(f"Velocity: [{data['vx_est']}, {data['vy_est']}, {data['vz_est']}]")
        """
        return {
            "dt": self.dt,
            "vx_est": round(self.X_prev[0][0], 1),  # Estimated X velocity
            "vy_est": round(self.X_prev[1][0], 1),  # Estimated Y velocity
            "vz_est": round(self.X_prev[2][0], 1),  # Estimated Z velocity
            "ax": self.Y_km[0][0],                  # Raw X acceleration
            "ay": self.Y_km[1][0],                  # Raw Y acceleration
            "az": self.Y_km[2][0]                   # Raw Z acceleration
        }
    
    def getAllKalmanVariables(self):
        """
        Get all internal Kalman filter variables for analysis.
        
        Returns:
            dict: Complete filter state including:
                - Time step, states, covariances, gains, measurements
        
        Useful for:
            - Filter tuning and optimization
            - Debugging and visualization
            - Performance analysis
            - Research and documentation
        """
        return {
            "dt": self.dt,          # Time step
            "X_kp": self.X_kp,      # Predicted state
            "X_prev": self.X_prev,  # Corrected state (current estimate)
            "P_kp": self.P_kp,      # Predicted covariance
            "P_prev": self.P_prev,  # Corrected covariance
            "Q_k": self.Q_k,        # Process noise covariance
            "K": self.K,            # Kalman gain
            "Y_k": self.Y_k,        # Observation
            "Y_km": self.Y_km,      # Raw measurement
            "u_k": self.u_k         # Control input
        }
