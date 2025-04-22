"""
Locust load testing for the Drone Show microservice.

This module provides load testing for the Drone Show microservice using Locust.
"""

import json
import random
import time
from locust import HttpUser, task, between


class DroneShowUser(HttpUser):
    """
    Simulated user for load testing the Drone Show microservice.
    
    This user simulates typical user behavior, including:
    - Browsing choreographies
    - Viewing choreography details
    - Creating choreographies
    - Simulating choreographies
    - Executing choreographies
    """
    
    wait_time = between(1, 5)  # Wait 1-5 seconds between tasks
    
    def on_start(self):
        """Initialize the user."""
        # Store choreography IDs
        self.choreography_ids = []
        self.simulation_ids = []
        self.execution_ids = []
        
        # Get existing choreographies
        response = self.client.get("/shows/")
        if response.status_code == 200:
            choreographies = response.json()
            self.choreography_ids = [c["id"] for c in choreographies]
    
    @task(10)
    def get_choreographies(self):
        """Get all choreographies."""
        self.client.get("/shows/")
    
    @task(5)
    def get_choreography(self):
        """Get a specific choreography."""
        if self.choreography_ids:
            choreography_id = random.choice(self.choreography_ids)
            self.client.get(f"/shows/{choreography_id}")
    
    @task(2)
    def create_choreography(self):
        """Create a new choreography."""
        # Create a simple choreography with 5 drones
        choreography_data = {
            "metadata": {
                "name": f"Load Test Choreography {int(time.time())}",
                "description": "A choreography created for load testing",
                "author": "Load Tester",
                "tags": ["load-test", "test"],
                "duration": 60.0,
                "drone_count": 5,
                "status": "draft"
            },
            "type": "waypoint",
            "trajectories": []
        }
        
        # Add trajectories for 5 drones
        for i in range(5):
            # Create waypoints
            waypoints = []
            led_states = []
            
            # Add waypoints and LED states
            for t in range(0, 61, 20):
                # Add waypoint
                waypoints.append({
                    "time": float(t),
                    "position": {
                        "lat": 37.7749 + (i % 3) * 0.0001,
                        "lon": -122.4194 + (i // 3) * 0.0001,
                        "alt": 10.0 + t / 10.0
                    },
                    "heading": float(t * 6)
                })
                
                # Add LED state
                led_states.append({
                    "time": float(t),
                    "color": {
                        "r": int(255 * (t / 60.0)),
                        "g": int(255 * (1 - t / 60.0)),
                        "b": int(128)
                    },
                    "effect": "solid"
                })
            
            # Add trajectory
            choreography_data["trajectories"].append({
                "drone_id": f"drone_{i+1}",
                "waypoints": waypoints,
                "led_states": led_states
            })
        
        # Send request
        response = self.client.post("/shows/", json=choreography_data)
        
        # Store choreography ID if successful
        if response.status_code == 201:
            choreography = response.json()
            self.choreography_ids.append(choreography["id"])
    
    @task(1)
    def simulate_choreography(self):
        """Simulate a choreography."""
        if self.choreography_ids:
            choreography_id = random.choice(self.choreography_ids)
            
            # Create simulation settings
            settings = {
                "start_time": 0.0,
                "end_time": 60.0,
                "speed_factor": 1.0,
                "include_takeoff_landing": True,
                "visualize_led": True,
                "visualize_trajectories": True,
                "drone_model": "generic"
            }
            
            # Send request
            response = self.client.post(f"/simulation/{choreography_id}", json=settings)
            
            # Store simulation ID if successful
            if response.status_code == 200:
                simulation = response.json()
                self.simulation_ids.append(simulation["id"])
    
    @task(1)
    def get_simulation(self):
        """Get a simulation."""
        if self.simulation_ids:
            simulation_id = random.choice(self.simulation_ids)
            self.client.get(f"/simulation/{simulation_id}")
    
    @task(1)
    def execute_choreography(self):
        """Execute a choreography."""
        if self.choreography_ids:
            choreography_id = random.choice(self.choreography_ids)
            
            # Create execution settings
            settings = {
                "include_takeoff_landing": True,
                "use_rtk": True,
                "safety_checks": True,
                "geofence_enabled": True,
                "return_home_on_low_battery": True,
                "return_home_on_connection_loss": True,
                "led_enabled": True
            }
            
            # Send request
            response = self.client.post(f"/execution/{choreography_id}", json=settings)
            
            # Store execution ID if successful
            if response.status_code == 200:
                execution = response.json()
                self.execution_ids.append(execution["id"])
    
    @task(1)
    def get_execution(self):
        """Get an execution."""
        if self.execution_ids:
            execution_id = random.choice(self.execution_ids)
            self.client.get(f"/execution/{execution_id}")
    
    @task(1)
    def get_execution_logs(self):
        """Get execution logs."""
        if self.execution_ids:
            execution_id = random.choice(self.execution_ids)
            self.client.get(f"/execution/{execution_id}/logs")
