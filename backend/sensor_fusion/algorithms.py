"""
Sensor Fusion Algorithms for Bulo.Cloud Sentinel Tactical Use Module.

This module provides various algorithms for fusing data from different sensor types
to create a coherent operational picture.
"""

import numpy as np
from typing import Dict, Any, List, Optional, Tuple
import json
from datetime import datetime


class KalmanFilter:
    """
    Kalman filter implementation for sensor fusion.
    
    This filter is particularly useful for fusing position and velocity data
    from different sensors with varying accuracy levels.
    """
    
    def __init__(self, state_dim: int, measurement_dim: int):
        """
        Initialize Kalman filter.
        
        Args:
            state_dim: Dimension of the state vector
            measurement_dim: Dimension of the measurement vector
        """
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim
        
        # State estimate
        self.x = np.zeros((state_dim, 1))
        
        # State covariance
        self.P = np.eye(state_dim)
        
        # Process noise covariance
        self.Q = np.eye(state_dim) * 0.01
        
        # Measurement noise covariance
        self.R = np.eye(measurement_dim) * 0.1
        
        # State transition matrix
        self.F = np.eye(state_dim)
        
        # Measurement matrix
        self.H = np.zeros((measurement_dim, state_dim))
        for i in range(measurement_dim):
            self.H[i, i] = 1.0
    
    def predict(self, dt: float = 1.0):
        """
        Predict step of the Kalman filter.
        
        Args:
            dt: Time step
        """
        # Update state transition matrix for position-velocity model
        if self.state_dim >= 4:  # At least 2D position-velocity
            self.F[0, 2] = dt  # x += vx * dt
            self.F[1, 3] = dt  # y += vy * dt
        
        # Predict state
        self.x = self.F @ self.x
        
        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x, self.P
    
    def update(self, z: np.ndarray):
        """
        Update step of the Kalman filter.
        
        Args:
            z: Measurement vector
        """
        # Reshape measurement if needed
        if z.ndim == 1:
            z = z.reshape(-1, 1)
        
        # Innovation
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x, self.P


class BayesianFusion:
    """
    Bayesian fusion for combining probability distributions from multiple sensors.
    
    This is useful for classification tasks where different sensors provide
    probability distributions over possible classes.
    """
    
    @staticmethod
    def fuse_probabilities(prob_dists: List[Dict[str, float]]) -> Dict[str, float]:
        """
        Fuse probability distributions using Bayesian fusion.
        
        Args:
            prob_dists: List of probability distributions, each as a dict mapping
                        class names to probabilities
        
        Returns:
            Fused probability distribution
        """
        if not prob_dists:
            return {}
        
        # Get all unique classes
        all_classes = set()
        for dist in prob_dists:
            all_classes.update(dist.keys())
        
        # Initialize with uniform prior
        result = {cls: 1.0 / len(all_classes) for cls in all_classes}
        
        # Apply Bayes' rule iteratively
        for dist in prob_dists:
            # Normalize the distribution if needed
            total = sum(dist.values())
            if total == 0:
                continue
                
            norm_dist = {k: v / total for k, v in dist.items()}
            
            # Update posterior
            total_posterior = 0
            posterior = {}
            for cls in all_classes:
                # P(cls | data) ∝ P(data | cls) * P(cls)
                likelihood = norm_dist.get(cls, 0.0001)  # Small value to avoid zeros
                prior = result[cls]
                posterior[cls] = likelihood * prior
                total_posterior += posterior[cls]
            
            # Normalize posterior
            if total_posterior > 0:
                result = {k: v / total_posterior for k, v in posterior.items()}
        
        return result


class DempsterShafer:
    """
    Dempster-Shafer evidence theory for sensor fusion.
    
    This approach is useful when dealing with uncertain or incomplete information
    from different sensors.
    """
    
    @staticmethod
    def combine_evidence(mass_functions: List[Dict[str, float]]) -> Dict[str, float]:
        """
        Combine evidence using Dempster's rule of combination.
        
        Args:
            mass_functions: List of mass functions, each as a dict mapping
                           focal elements to mass values
        
        Returns:
            Combined mass function
        """
        if not mass_functions:
            return {}
        
        # Start with the first mass function
        result = mass_functions[0].copy()
        
        # Combine with each additional mass function
        for mass in mass_functions[1:]:
            result = DempsterShafer._combine_two_mass_functions(result, mass)
        
        return result
    
    @staticmethod
    def _combine_two_mass_functions(m1: Dict[str, float], m2: Dict[str, float]) -> Dict[str, float]:
        """
        Combine two mass functions using Dempster's rule.
        
        Args:
            m1: First mass function
            m2: Second mass function
        
        Returns:
            Combined mass function
        """
        result = {}
        conflict = 0.0
        
        # Calculate combined masses
        for k1, v1 in m1.items():
            for k2, v2 in m2.items():
                # Intersection of focal elements
                if k1 == "Ø" or k2 == "Ø":
                    # Empty set
                    conflict += v1 * v2
                else:
                    # Parse sets from string representation
                    set1 = set(k1.split(','))
                    set2 = set(k2.split(','))
                    intersection = set1.intersection(set2)
                    
                    if not intersection:
                        # Empty intersection means conflict
                        conflict += v1 * v2
                    else:
                        # Non-empty intersection
                        intersection_key = ','.join(sorted(intersection))
                        result[intersection_key] = result.get(intersection_key, 0) + v1 * v2
        
        # Normalize to account for conflict
        if conflict < 1.0:
            normalization_factor = 1.0 / (1.0 - conflict)
            result = {k: v * normalization_factor for k, v in result.items()}
        
        return result


class SensorFusionAlgorithms:
    """
    Factory class for creating and using different sensor fusion algorithms.
    """
    
    @staticmethod
    def create_position_velocity_kalman(initial_state: Optional[np.ndarray] = None) -> KalmanFilter:
        """
        Create a Kalman filter for position-velocity fusion.
        
        Args:
            initial_state: Initial state vector [x, y, vx, vy]
        
        Returns:
            Configured Kalman filter
        """
        kf = KalmanFilter(state_dim=4, measurement_dim=2)
        
        # Configure for position-velocity model
        kf.F[0, 2] = 1.0  # x += vx * dt
        kf.F[1, 3] = 1.0  # y += vy * dt
        
        # Set initial state if provided
        if initial_state is not None:
            kf.x = initial_state.reshape(-1, 1)
        
        return kf
    
    @staticmethod
    def fuse_position_data(positions: List[Dict[str, float]], 
                          uncertainties: Optional[List[Dict[str, float]]] = None) -> Dict[str, float]:
        """
        Fuse position data from multiple sensors.
        
        Args:
            positions: List of position dictionaries, each with 'x', 'y' keys
            uncertainties: Optional list of uncertainty dictionaries, each with 'x', 'y' keys
        
        Returns:
            Fused position as a dictionary with 'x', 'y' keys
        """
        if not positions:
            return {}
        
        # Default uncertainties if not provided
        if uncertainties is None:
            uncertainties = [{'x': 1.0, 'y': 1.0} for _ in positions]
        
        # Weighted average based on uncertainties
        sum_weights_x = sum(1.0 / u['x'] for u in uncertainties)
        sum_weights_y = sum(1.0 / u['y'] for u in uncertainties)
        
        fused_x = sum(p['x'] / u['x'] for p, u in zip(positions, uncertainties)) / sum_weights_x
        fused_y = sum(p['y'] / u['y'] for p, u in zip(positions, uncertainties)) / sum_weights_y
        
        return {'x': fused_x, 'y': fused_y}
    
    @staticmethod
    def fuse_classifications(classifications: List[Dict[str, float]]) -> Dict[str, float]:
        """
        Fuse classification results from multiple sensors.
        
        Args:
            classifications: List of classification dictionaries, each mapping
                            class names to probabilities
        
        Returns:
            Fused classification as a dictionary mapping class names to probabilities
        """
        return BayesianFusion.fuse_probabilities(classifications)
    
    @staticmethod
    def fuse_detections(detections: List[Dict[str, Any]], 
                       max_distance: float = 10.0) -> List[Dict[str, Any]]:
        """
        Fuse object detections from multiple sensors.
        
        Args:
            detections: List of detection dictionaries, each with 'x', 'y', 'type', 'confidence' keys
            max_distance: Maximum distance for considering detections as the same object
        
        Returns:
            List of fused detections
        """
        if not detections:
            return []
        
        # Group detections by proximity
        groups = []
        for detection in detections:
            matched = False
            for group in groups:
                # Check if this detection is close to any detection in the group
                for group_detection in group:
                    dx = detection['x'] - group_detection['x']
                    dy = detection['y'] - group_detection['y']
                    distance = (dx**2 + dy**2)**0.5
                    if distance <= max_distance:
                        group.append(detection)
                        matched = True
                        break
                if matched:
                    break
            
            if not matched:
                # Create a new group
                groups.append([detection])
        
        # Fuse each group
        fused_detections = []
        for group in groups:
            # Average position
            avg_x = sum(d['x'] for d in group) / len(group)
            avg_y = sum(d['y'] for d in group) / len(group)
            
            # Fuse types using Bayesian fusion
            type_probs = {}
            for d in group:
                d_type = d['type']
                confidence = d['confidence']
                type_probs[d_type] = type_probs.get(d_type, 0) + confidence
            
            # Normalize
            total = sum(type_probs.values())
            type_probs = {k: v / total for k, v in type_probs.items()}
            
            # Get most likely type
            most_likely_type = max(type_probs.items(), key=lambda x: x[1])[0]
            
            # Average confidence
            avg_confidence = sum(d['confidence'] for d in group) / len(group)
            
            fused_detections.append({
                'x': avg_x,
                'y': avg_y,
                'type': most_likely_type,
                'confidence': avg_confidence,
                'fused_from': len(group)
            })
        
        return fused_detections
