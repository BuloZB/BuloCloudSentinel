from typing import List, Dict, Any, Optional, Tuple, Set
import asyncio
import numpy as np
import json
from datetime import datetime
from backend.sensor_fusion.algorithms import SensorFusionAlgorithms, KalmanFilter

class SensorType:
    CAMERA = "camera"
    RADAR = "radar"
    LIDAR = "lidar"
    GPS = "gps"
    IMU = "imu"
    THERMAL = "thermal"
    ACOUSTIC = "acoustic"

class SensorFusionEngine:
    def __init__(self):
        self.sensor_data_buffers: Dict[str, List[Dict[str, Any]]] = {}
        self.fused_data: Dict[str, Any] = {}
        self.position_kalman = SensorFusionAlgorithms.create_position_velocity_kalman()
        self.last_fusion_time = datetime.now()
        self.max_buffer_size = 100  # Limit buffer size to prevent memory issues
        self.registered_sensors: Set[str] = set()
        self.fusion_algorithms = {
            "position": self._fuse_position_data,
            "detection": self._fuse_detection_data,
            "classification": self._fuse_classification_data,
            "telemetry": self._fuse_telemetry_data
        }

    def register_sensor(self, sensor_id: str, sensor_type: str, capabilities: List[str]):
        """Register a new sensor with the fusion engine."""
        self.registered_sensors.add(sensor_id)
        if sensor_id not in self.sensor_data_buffers:
            self.sensor_data_buffers[sensor_id] = []

        # Store sensor metadata
        self.fused_data["sensors"] = self.fused_data.get("sensors", {})
        self.fused_data["sensors"][sensor_id] = {
            "type": sensor_type,
            "capabilities": capabilities,
            "last_update": None,
            "status": "registered"
        }

        return {"detail": f"Sensor {sensor_id} registered successfully"}

    def ingest_data(self, sensor_id: str, data: Dict[str, Any]):
        """Ingest data from a sensor into the fusion engine."""
        # Validate sensor registration
        if sensor_id not in self.registered_sensors:
            raise ValueError(f"Sensor {sensor_id} is not registered")

        # Add timestamp if not present
        if "timestamp" not in data:
            data["timestamp"] = datetime.now().isoformat()

        # Update sensor metadata
        if "sensors" in self.fused_data and sensor_id in self.fused_data["sensors"]:
            self.fused_data["sensors"][sensor_id]["last_update"] = data["timestamp"]
            self.fused_data["sensors"][sensor_id]["status"] = "active"

        # Add to buffer
        if sensor_id not in self.sensor_data_buffers:
            self.sensor_data_buffers[sensor_id] = []

        self.sensor_data_buffers[sensor_id].append(data)

        # Trim buffer if needed
        if len(self.sensor_data_buffers[sensor_id]) > self.max_buffer_size:
            self.sensor_data_buffers[sensor_id] = self.sensor_data_buffers[sensor_id][-self.max_buffer_size:]

        return {"detail": "Data ingested successfully"}

    async def fuse_data(self) -> Dict[str, Any]:
        """Fuse all sensor data to create a coherent operational picture."""
        # Calculate time delta since last fusion
        now = datetime.now()
        dt = (now - self.last_fusion_time).total_seconds()
        self.last_fusion_time = now

        # Initialize fused data structure if needed
        if "position" not in self.fused_data:
            self.fused_data["position"] = {}
        if "detections" not in self.fused_data:
            self.fused_data["detections"] = []
        if "classifications" not in self.fused_data:
            self.fused_data["classifications"] = {}
        if "telemetry" not in self.fused_data:
            self.fused_data["telemetry"] = {}

        # Apply Kalman prediction step
        self.position_kalman.predict(dt)

        # Apply fusion algorithms for each data type
        for fusion_type, fusion_func in self.fusion_algorithms.items():
            try:
                self.fused_data[fusion_type] = fusion_func(dt)
            except Exception as e:
                print(f"Error in {fusion_type} fusion: {str(e)}")

        # Add fusion timestamp
        self.fused_data["timestamp"] = now.isoformat()

        # Check sensor health
        self._update_sensor_health(now)

        return self.fused_data

    def _update_sensor_health(self, current_time: datetime):
        """Update health status of all registered sensors."""
        if "sensors" not in self.fused_data:
            return

        for sensor_id, sensor_info in self.fused_data["sensors"].items():
            if sensor_info["last_update"] is None:
                sensor_info["status"] = "inactive"
                continue

            last_update = datetime.fromisoformat(sensor_info["last_update"])
            time_diff = (current_time - last_update).total_seconds()

            # Mark sensors as stale if no updates in 10 seconds
            if time_diff > 10:
                sensor_info["status"] = "stale"
            # Mark sensors as inactive if no updates in 60 seconds
            if time_diff > 60:
                sensor_info["status"] = "inactive"

    def _fuse_position_data(self, dt: float) -> Dict[str, Any]:
        """Fuse position data from GPS, IMU, and other position sensors."""
        position_data = []
        uncertainties = []

        # Collect position data from relevant sensors
        for sensor_id, data_list in self.sensor_data_buffers.items():
            if not data_list:
                continue

            latest_data = data_list[-1]
            if "position" in latest_data:
                pos = latest_data["position"]
                if "x" in pos and "y" in pos:
                    position_data.append(pos)
                    # Use provided uncertainty or default
                    uncertainty = latest_data.get("uncertainty", {"x": 1.0, "y": 1.0})
                    uncertainties.append(uncertainty)

        if not position_data:
            # No position data available
            return self.fused_data.get("position", {})

        # Use weighted fusion for position data
        fused_position = SensorFusionAlgorithms.fuse_position_data(position_data, uncertainties)

        # Update Kalman filter with fused position
        if "x" in fused_position and "y" in fused_position:
            measurement = np.array([fused_position["x"], fused_position["y"]])
            self.position_kalman.update(measurement)

            # Get filtered state
            state = self.position_kalman.x.flatten()

            # Update fused position with filtered values and velocity
            fused_position["x"] = float(state[0])
            fused_position["y"] = float(state[1])
            fused_position["vx"] = float(state[2])
            fused_position["vy"] = float(state[3])
            fused_position["speed"] = float(np.sqrt(state[2]**2 + state[3]**2))
            fused_position["heading"] = float(np.arctan2(state[3], state[2]) * 180 / np.pi)

        return fused_position

    def _fuse_detection_data(self, dt: float) -> List[Dict[str, Any]]:
        """Fuse object detection data from cameras, radars, and LIDARs."""
        all_detections = []

        # Collect detection data from relevant sensors
        for sensor_id, data_list in self.sensor_data_buffers.items():
            if not data_list:
                continue

            latest_data = data_list[-1]
            if "detections" in latest_data and isinstance(latest_data["detections"], list):
                all_detections.extend(latest_data["detections"])

        if not all_detections:
            # No detection data available
            return self.fused_data.get("detections", [])

        # Fuse detections
        fused_detections = SensorFusionAlgorithms.fuse_detections(all_detections)

        return fused_detections

    def _fuse_classification_data(self, dt: float) -> Dict[str, Dict[str, float]]:
        """Fuse classification data from AI models and sensors."""
        classifications = {}

        # Collect classification data from relevant sensors
        for sensor_id, data_list in self.sensor_data_buffers.items():
            if not data_list:
                continue

            latest_data = data_list[-1]
            if "classifications" in latest_data and isinstance(latest_data["classifications"], dict):
                for obj_id, probs in latest_data["classifications"].items():
                    if obj_id not in classifications:
                        classifications[obj_id] = []
                    classifications[obj_id].append(probs)

        # Fuse classifications for each object
        fused_classifications = {}
        for obj_id, prob_list in classifications.items():
            fused_classifications[obj_id] = SensorFusionAlgorithms.fuse_classifications(prob_list)

        return fused_classifications

    def _fuse_telemetry_data(self, dt: float) -> Dict[str, Any]:
        """Fuse telemetry data from various sensors."""
        telemetry = {}

        # Collect telemetry data from relevant sensors
        for sensor_id, data_list in self.sensor_data_buffers.items():
            if not data_list:
                continue

            latest_data = data_list[-1]
            if "telemetry" in latest_data and isinstance(latest_data["telemetry"], dict):
                # Merge telemetry data, newer values overwrite older ones
                for key, value in latest_data["telemetry"].items():
                    telemetry[key] = value

        return telemetry

    async def run(self):
        """Run the fusion engine continuously."""
        while True:
            try:
                await self.fuse_data()
                # Publish fused data to subscribers could be implemented here
            except Exception as e:
                print(f"Error in fusion engine: {str(e)}")

            await asyncio.sleep(0.1)  # 10Hz fusion rate
