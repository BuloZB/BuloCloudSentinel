"""
Kalman filter implementation for target tracking.

This module provides a Kalman filter implementation for tracking targets
with position and velocity state.
"""

import numpy as np
from typing import Optional, Tuple

class KalmanFilter:
    """
    Kalman filter for target tracking.
    
    This implementation uses a constant velocity model with 4 state variables:
    [latitude, longitude, latitude_velocity, longitude_velocity]
    """
    
    def __init__(
        self,
        initial_state: np.ndarray,
        process_noise: float = 1e-5,
        measurement_noise: float = 1e-3
    ):
        """
        Initialize the Kalman filter.
        
        Args:
            initial_state: Initial state vector [lat, lon, lat_vel, lon_vel]
            process_noise: Process noise covariance scalar
            measurement_noise: Measurement noise covariance scalar
        """
        # State vector [lat, lon, lat_vel, lon_vel]
        self.state = initial_state
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, 1, 0],  # lat = lat + lat_vel
            [0, 1, 0, 1],  # lon = lon + lon_vel
            [0, 0, 1, 0],  # lat_vel = lat_vel
            [0, 0, 0, 1]   # lon_vel = lon_vel
        ])
        
        # Measurement matrix (we only measure position)
        self.H = np.array([
            [1, 0, 0, 0],  # measure lat
            [0, 1, 0, 0]   # measure lon
        ])
        
        # Process noise covariance
        self.Q = np.eye(4) * process_noise
        
        # Measurement noise covariance
        self.R = np.eye(2) * measurement_noise
        
        # State covariance matrix
        self.P = np.eye(4)
    
    def predict(self, dt: float = 1.0) -> np.ndarray:
        """
        Predict the state forward by dt time units.
        
        Args:
            dt: Time step
            
        Returns:
            Predicted state
        """
        # Update state transition matrix for dt
        F_dt = np.copy(self.F)
        F_dt[0, 2] = dt  # lat += lat_vel * dt
        F_dt[1, 3] = dt  # lon += lon_vel * dt
        
        # Predict state
        self.state = F_dt @ self.state
        
        # Predict covariance
        self.P = F_dt @ self.P @ F_dt.T + self.Q
        
        return self.state
    
    def update(self, measurement: np.ndarray) -> np.ndarray:
        """
        Update the state with a new measurement.
        
        Args:
            measurement: Measurement vector [lat, lon]
            
        Returns:
            Updated state
        """
        # Calculate innovation (measurement residual)
        y = measurement - self.H @ self.state
        
        # Calculate innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Calculate Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        I = np.eye(self.state.shape[0])
        self.P = (I - K @ self.H) @ self.P
        
        return self.state
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the current state and covariance.
        
        Returns:
            Tuple of (state, covariance)
        """
        return self.state, self.P
    
    def get_position(self) -> Tuple[float, float]:
        """
        Get the current position.
        
        Returns:
            Tuple of (latitude, longitude)
        """
        return self.state[0], self.state[1]
    
    def get_velocity(self) -> Tuple[float, float]:
        """
        Get the current velocity.
        
        Returns:
            Tuple of (latitude_velocity, longitude_velocity)
        """
        return self.state[2], self.state[3]
    
    def get_position_covariance(self) -> np.ndarray:
        """
        Get the position covariance.
        
        Returns:
            Position covariance matrix (2x2)
        """
        return self.P[:2, :2]
    
    def get_velocity_covariance(self) -> np.ndarray:
        """
        Get the velocity covariance.
        
        Returns:
            Velocity covariance matrix (2x2)
        """
        return self.P[2:, 2:]
