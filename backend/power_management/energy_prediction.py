"""
Energy Prediction Service for Bulo.Cloud Sentinel.

This module provides services for predicting energy consumption
for drone missions based on various factors.
"""

import asyncio
import logging
import json
import math
import numpy as np
from typing import Dict, List, Any, Optional, Tuple
from datetime import datetime
import httpx

from backend.power_management.models import (
    EnergyPrediction, EnergyPredictionModel as PredictionModelType,
    BatteryMetrics
)
from backend.mission_planning.models import Mission, Position, Waypoint
from backend.weather.service import WeatherService
from dronecore.drone_command_telemetry_hub import DroneCommandHub

logger = logging.getLogger(__name__)

class EnergyPredictionService:
    """
    Service for predicting energy consumption for drone missions.
    
    This service uses various models to predict energy consumption
    based on mission parameters, drone specifications, and environmental
    factors.
    """
    
    def __init__(
        self,
        db_session=None,
        drone_command_hub=None,
        weather_service=None
    ):
        """
        Initialize the energy prediction service.
        
        Args:
            db_session: Database session for persistence
            drone_command_hub: Drone command hub for telemetry
            weather_service: Weather service for environmental data
        """
        self.db = db_session
        self.drone_command_hub = drone_command_hub or DroneCommandHub()
        self.weather_service = weather_service or WeatherService()
        
        # Drone specifications cache
        self._drone_specs: Dict[str, Dict[str, Any]] = {}
        
        # Model coefficients
        self._model_coefficients: Dict[str, Dict[str, Any]] = {
            "linear": {
                "distance": 0.05,  # % per km
                "altitude": 0.02,  # % per 10m
                "speed": 0.03,  # % per m/s
                "payload": 0.1,  # % per kg
                "wind": 0.08,  # % per m/s
                "temperature": 0.01,  # % per °C deviation from 20°C
                "base": 5.0  # % base consumption
            },
            "polynomial": {
                "distance_squared": 0.001,  # % per km²
                "altitude_squared": 0.0005,  # % per 10m²
                "speed_squared": 0.002,  # % per (m/s)²
                "wind_squared": 0.005,  # % per (m/s)²
                "cross_wind_factor": 0.1  # multiplier for crosswind effect
            }
        }
    
    async def predict_energy_consumption(
        self,
        mission_id: Optional[str] = None,
        drone_id: str = None,
        waypoints: Optional[List[Dict[str, Any]]] = None,
        payload_weight: Optional[float] = None,
        wind_speed: Optional[float] = None,
        wind_direction: Optional[float] = None,
        temperature: Optional[float] = None,
        model_type: PredictionModelType = PredictionModelType.LINEAR
    ) -> EnergyPrediction:
        """
        Predict energy consumption for a mission.
        
        Args:
            mission_id: Optional ID of the mission
            drone_id: ID of the drone
            waypoints: Optional list of waypoints (if mission_id not provided)
            payload_weight: Optional payload weight in kg
            wind_speed: Optional wind speed in m/s
            wind_direction: Optional wind direction in degrees
            temperature: Optional temperature in Celsius
            model_type: Type of prediction model to use
            
        Returns:
            Energy prediction
        """
        # Get mission waypoints
        if mission_id and not waypoints:
            # Get mission from database
            if not self.db:
                raise ValueError("Database session required to get mission by ID")
            
            # This is a placeholder - in a real implementation, you would fetch the mission from the database
            # mission = await self.db.get_mission(mission_id)
            # waypoints = mission.waypoints
            
            # For now, we'll just raise an error
            raise ValueError("Either mission_id with database or waypoints must be provided")
        
        if not waypoints:
            raise ValueError("Waypoints must be provided")
        
        # Get drone specifications
        drone_specs = await self._get_drone_specs(drone_id)
        
        # Get environmental data if not provided
        if not all([wind_speed, wind_direction, temperature]):
            # Get average position of waypoints
            avg_lat = sum(wp.get("latitude", wp.get("position", {}).get("latitude", 0)) for wp in waypoints) / len(waypoints)
            avg_lon = sum(wp.get("longitude", wp.get("position", {}).get("longitude", 0)) for wp in waypoints) / len(waypoints)
            
            # Get weather data
            try:
                weather_data = await self.weather_service.get_current_weather(avg_lat, avg_lon)
                wind_speed = wind_speed or weather_data.wind_speed
                wind_direction = wind_direction or weather_data.wind_direction
                temperature = temperature or weather_data.temperature
            except Exception as e:
                logger.warning(f"Error getting weather data: {str(e)}")
                # Use default values if weather data not available
                wind_speed = wind_speed or 0.0
                wind_direction = wind_direction or 0.0
                temperature = temperature or 20.0
        
        # Calculate mission parameters
        distance, duration, avg_speed, max_altitude = self._calculate_mission_parameters(waypoints)
        
        # Predict energy consumption based on model type
        if model_type == PredictionModelType.LINEAR:
            consumption, factors = self._predict_linear(
                distance, avg_speed, max_altitude, payload_weight, wind_speed, wind_direction, temperature, drone_specs
            )
        elif model_type == PredictionModelType.POLYNOMIAL:
            consumption, factors = self._predict_polynomial(
                distance, avg_speed, max_altitude, payload_weight, wind_speed, wind_direction, temperature, drone_specs
            )
        elif model_type == PredictionModelType.NEURAL_NETWORK:
            # This would use a trained neural network model
            # For now, we'll just use the linear model
            consumption, factors = self._predict_linear(
                distance, avg_speed, max_altitude, payload_weight, wind_speed, wind_direction, temperature, drone_specs
            )
        elif model_type == PredictionModelType.ENSEMBLE:
            # This would combine multiple models
            # For now, we'll just average linear and polynomial
            linear_consumption, linear_factors = self._predict_linear(
                distance, avg_speed, max_altitude, payload_weight, wind_speed, wind_direction, temperature, drone_specs
            )
            poly_consumption, poly_factors = self._predict_polynomial(
                distance, avg_speed, max_altitude, payload_weight, wind_speed, wind_direction, temperature, drone_specs
            )
            consumption = (linear_consumption + poly_consumption) / 2
            factors = {k: (linear_factors.get(k, 0) + poly_factors.get(k, 0)) / 2 for k in set(linear_factors) | set(poly_factors)}
        else:
            raise ValueError(f"Unsupported model type: {model_type}")
        
        # Add margin of error
        margin_of_error = 10.0  # 10% margin of error
        consumption_with_margin = consumption * (1 + margin_of_error / 100)
        
        # Check if mission is feasible
        is_feasible = consumption_with_margin < 90.0  # Less than 90% of battery
        
        # Generate recommendations
        recommendations = self._generate_recommendations(
            consumption, factors, is_feasible, distance, avg_speed, max_altitude, wind_speed
        )
        
        # Calculate estimated range
        battery_capacity = drone_specs.get("battery_capacity", 5000)  # mAh
        energy_per_percent = battery_capacity / 100.0  # mAh per %
        energy_per_km = (consumption / distance) * energy_per_percent if distance > 0 else 0
        estimated_range = (90.0 * energy_per_percent) / energy_per_km if energy_per_km > 0 else 0
        
        # Create prediction
        prediction = EnergyPrediction(
            mission_id=mission_id or "custom_mission",
            drone_id=drone_id,
            model_type=model_type,
            estimated_consumption=consumption,
            estimated_duration=duration,
            estimated_range=estimated_range,
            confidence=0.8,  # 80% confidence
            margin_of_error=margin_of_error,
            is_feasible=is_feasible,
            factors=factors,
            recommendations=recommendations
        )
        
        # Store prediction in database
        if self.db and mission_id:
            # This is a placeholder - in a real implementation, you would save to the database
            pass
        
        return prediction
    
    async def optimize_mission(
        self,
        mission_id: str,
        drone_id: str,
        current_battery: float,
        optimization_level: float = 0.5
    ) -> Tuple[EnergyPrediction, List[Dict[str, Any]]]:
        """
        Optimize a mission for energy efficiency.
        
        Args:
            mission_id: ID of the mission to optimize
            drone_id: ID of the drone
            current_battery: Current battery level (percentage)
            optimization_level: Level of optimization (0-1)
            
        Returns:
            Tuple of (energy prediction, optimized waypoints)
        """
        # Get mission from database
        if not self.db:
            raise ValueError("Database session required to optimize mission")
        
        # This is a placeholder - in a real implementation, you would fetch the mission from the database
        # mission = await self.db.get_mission(mission_id)
        # waypoints = mission.waypoints
        
        # For now, we'll just raise an error
        raise NotImplementedError("Mission optimization not implemented")
    
    async def _get_drone_specs(self, drone_id: str) -> Dict[str, Any]:
        """
        Get drone specifications.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Drone specifications
        """
        # Check cache
        if drone_id in self._drone_specs:
            return self._drone_specs[drone_id]
        
        # Get from database or drone command hub
        try:
            # This is a placeholder - in a real implementation, you would get from the database or drone
            specs = {
                "model": "Generic Drone",
                "weight": 1.5,  # kg
                "max_speed": 15.0,  # m/s
                "battery_capacity": 5000,  # mAh
                "battery_voltage": 11.1,  # V
                "max_payload": 0.5,  # kg
                "max_wind_resistance": 10.0,  # m/s
                "power_consumption": {
                    "hover": 100.0,  # W
                    "forward": 120.0,  # W
                    "ascend": 150.0,  # W
                    "descend": 80.0  # W
                }
            }
            
            # Cache specs
            self._drone_specs[drone_id] = specs
            
            return specs
        except Exception as e:
            logger.error(f"Error getting drone specifications for {drone_id}: {str(e)}")
            
            # Return default specs
            default_specs = {
                "model": "Unknown Drone",
                "weight": 1.5,  # kg
                "max_speed": 15.0,  # m/s
                "battery_capacity": 5000,  # mAh
                "battery_voltage": 11.1,  # V
                "max_payload": 0.5,  # kg
                "max_wind_resistance": 8.0,  # m/s
                "power_consumption": {
                    "hover": 100.0,  # W
                    "forward": 120.0,  # W
                    "ascend": 150.0,  # W
                    "descend": 80.0  # W
                }
            }
            
            return default_specs
    
    def _calculate_mission_parameters(self, waypoints: List[Dict[str, Any]]) -> Tuple[float, int, float, float]:
        """
        Calculate mission parameters from waypoints.
        
        Args:
            waypoints: List of waypoints
            
        Returns:
            Tuple of (distance in km, duration in seconds, average speed in m/s, max altitude in m)
        """
        if len(waypoints) < 2:
            return 0.0, 0, 0.0, 0.0
        
        total_distance = 0.0
        max_altitude = 0.0
        
        # Calculate distance and max altitude
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            
            # Extract coordinates
            lat1 = wp1.get("latitude", wp1.get("position", {}).get("latitude", 0))
            lon1 = wp1.get("longitude", wp1.get("position", {}).get("longitude", 0))
            alt1 = wp1.get("altitude", wp1.get("position", {}).get("altitude", 0))
            
            lat2 = wp2.get("latitude", wp2.get("position", {}).get("latitude", 0))
            lon2 = wp2.get("longitude", wp2.get("position", {}).get("longitude", 0))
            alt2 = wp2.get("altitude", wp2.get("position", {}).get("altitude", 0))
            
            # Calculate distance between waypoints
            distance = self._haversine_distance(lat1, lon1, lat2, lon2)
            total_distance += distance
            
            # Update max altitude
            max_altitude = max(max_altitude, alt1, alt2)
        
        # Estimate duration based on average speed of 10 m/s
        avg_speed = 10.0  # m/s
        duration = int((total_distance * 1000) / avg_speed)  # seconds
        
        return total_distance, duration, avg_speed, max_altitude
    
    def _haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate the great circle distance between two points on the earth.
        
        Args:
            lat1: Latitude of point 1 in degrees
            lon1: Longitude of point 1 in degrees
            lat2: Latitude of point 2 in degrees
            lon2: Longitude of point 2 in degrees
            
        Returns:
            Distance in kilometers
        """
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371  # Radius of earth in kilometers
        
        return c * r
    
    def _predict_linear(
        self,
        distance: float,
        speed: float,
        altitude: float,
        payload_weight: Optional[float],
        wind_speed: float,
        wind_direction: float,
        temperature: float,
        drone_specs: Dict[str, Any]
    ) -> Tuple[float, Dict[str, float]]:
        """
        Predict energy consumption using a linear model.
        
        Args:
            distance: Distance in kilometers
            speed: Average speed in m/s
            altitude: Maximum altitude in meters
            payload_weight: Payload weight in kg
            wind_speed: Wind speed in m/s
            wind_direction: Wind direction in degrees
            temperature: Temperature in Celsius
            drone_specs: Drone specifications
            
        Returns:
            Tuple of (consumption percentage, factor contributions)
        """
        coeffs = self._model_coefficients["linear"]
        
        # Default payload to 0 if not provided
        payload_weight = payload_weight or 0.0
        
        # Calculate individual factors
        distance_factor = distance * coeffs["distance"]
        altitude_factor = (altitude / 10.0) * coeffs["altitude"]
        speed_factor = speed * coeffs["speed"]
        payload_factor = payload_weight * coeffs["payload"]
        wind_factor = wind_speed * coeffs["wind"]
        temp_factor = abs(temperature - 20.0) * coeffs["temperature"]
        base_factor = coeffs["base"]
        
        # Calculate total consumption
        consumption = base_factor + distance_factor + altitude_factor + speed_factor + payload_factor + wind_factor + temp_factor
        
        # Factor contributions
        factors = {
            "base": base_factor,
            "distance": distance_factor,
            "altitude": altitude_factor,
            "speed": speed_factor,
            "payload": payload_factor,
            "wind": wind_factor,
            "temperature": temp_factor
        }
        
        return consumption, factors
    
    def _predict_polynomial(
        self,
        distance: float,
        speed: float,
        altitude: float,
        payload_weight: Optional[float],
        wind_speed: float,
        wind_direction: float,
        temperature: float,
        drone_specs: Dict[str, Any]
    ) -> Tuple[float, Dict[str, float]]:
        """
        Predict energy consumption using a polynomial model.
        
        Args:
            distance: Distance in kilometers
            speed: Average speed in m/s
            altitude: Maximum altitude in meters
            payload_weight: Payload weight in kg
            wind_speed: Wind speed in m/s
            wind_direction: Wind direction in degrees
            temperature: Temperature in Celsius
            drone_specs: Drone specifications
            
        Returns:
            Tuple of (consumption percentage, factor contributions)
        """
        # Get linear prediction first
        linear_consumption, linear_factors = self._predict_linear(
            distance, speed, altitude, payload_weight, wind_speed, wind_direction, temperature, drone_specs
        )
        
        # Add polynomial terms
        poly_coeffs = self._model_coefficients["polynomial"]
        
        # Calculate polynomial factors
        distance_squared_factor = (distance ** 2) * poly_coeffs["distance_squared"]
        altitude_squared_factor = ((altitude / 10.0) ** 2) * poly_coeffs["altitude_squared"]
        speed_squared_factor = (speed ** 2) * poly_coeffs["speed_squared"]
        wind_squared_factor = (wind_speed ** 2) * poly_coeffs["wind_squared"]
        
        # Calculate crosswind effect
        # Assuming wind_direction is relative to drone heading
        crosswind_factor = abs(math.sin(math.radians(wind_direction))) * wind_speed * poly_coeffs["cross_wind_factor"]
        
        # Add polynomial terms to linear consumption
        consumption = linear_consumption + distance_squared_factor + altitude_squared_factor + speed_squared_factor + wind_squared_factor + crosswind_factor
        
        # Factor contributions
        factors = linear_factors.copy()
        factors.update({
            "distance_squared": distance_squared_factor,
            "altitude_squared": altitude_squared_factor,
            "speed_squared": speed_squared_factor,
            "wind_squared": wind_squared_factor,
            "crosswind": crosswind_factor
        })
        
        return consumption, factors
    
    def _generate_recommendations(
        self,
        consumption: float,
        factors: Dict[str, float],
        is_feasible: bool,
        distance: float,
        speed: float,
        altitude: float,
        wind_speed: float
    ) -> List[str]:
        """
        Generate recommendations for optimizing energy consumption.
        
        Args:
            consumption: Predicted energy consumption
            factors: Factor contributions
            is_feasible: Whether the mission is feasible
            distance: Distance in kilometers
            speed: Average speed in m/s
            altitude: Maximum altitude in meters
            wind_speed: Wind speed in m/s
            
        Returns:
            List of recommendations
        """
        recommendations = []
        
        if not is_feasible:
            recommendations.append("Mission exceeds safe battery limits. Consider reducing distance or optimizing route.")
        
        # Find the largest contributing factors
        sorted_factors = sorted([(k, v) for k, v in factors.items() if k != "base"], key=lambda x: x[1], reverse=True)
        
        # Generate recommendations based on top factors
        if sorted_factors:
            top_factor = sorted_factors[0][0]
            
            if top_factor == "distance" or top_factor == "distance_squared":
                recommendations.append("Distance is a major factor. Consider optimizing the route or reducing waypoints.")
            
            elif top_factor == "speed" or top_factor == "speed_squared":
                if speed > 10.0:
                    recommendations.append(f"Reducing speed from {speed:.1f} m/s to {max(5.0, speed * 0.8):.1f} m/s could save energy.")
            
            elif top_factor == "altitude" or top_factor == "altitude_squared":
                if altitude > 50.0:
                    recommendations.append(f"Flying at a lower altitude (e.g., {max(20.0, altitude * 0.7):.1f}m) could reduce energy consumption.")
            
            elif top_factor == "wind" or top_factor == "wind_squared" or top_factor == "crosswind":
                if wind_speed > 5.0:
                    recommendations.append("High wind conditions will significantly increase energy consumption. Consider postponing the mission if possible.")
            
            elif top_factor == "payload":
                recommendations.append("Reducing payload weight would improve energy efficiency.")
            
            elif top_factor == "temperature":
                recommendations.append("Temperature conditions are affecting battery performance. Monitor battery temperature during flight.")
        
        # Add general recommendations
        if consumption > 70.0:
            recommendations.append("Consider adding a return-to-home waypoint at 30% battery to ensure safe return.")
        
        if distance > 5.0:
            recommendations.append("For long-distance missions, monitor battery levels closely and be prepared to abort if necessary.")
        
        return recommendations
