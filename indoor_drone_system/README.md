# Indoor Drone System

This microservice provides advanced indoor drone capabilities for the Bulo.Cloud Sentinel platform, inspired by the SU17 Smart Indoor Drone.

## Features

- **LiDAR-based SLAM**: Implements FAST-LIO algorithm for precise indoor positioning
- **Visual SLAM**: Uses ORB-SLAM3 for camera-based positioning and mapping
- **Sensor Fusion**: Combines LiDAR, visual, and IMU data for robust positioning
- **Autonomous Navigation**: Implements EGO-Swarm path planning algorithm
- **Mission Planning**: Supports waypoint-based mission planning and execution
- **3D Mapping**: Real-time 3D environment reconstruction and mapping

## Architecture

The Indoor Drone System is implemented as a set of microservices:

- **API Service**: FastAPI-based REST API for integration with Bulo.Cloud Sentinel
- **SLAM Service**: Implements LiDAR and visual SLAM algorithms
- **Path Planning Service**: Implements path planning algorithms
- **Sensor Fusion Service**: Combines data from multiple sensors
- **Simulation Service**: Provides simulation capabilities for testing
- **MAVROS Bridge**: Interfaces with drone flight controllers

## Hardware Support

The system is designed to work with the following hardware:

- **LiDAR**: Livox Mid-360 3D LiDAR
- **Cameras**: Intel RealSense D455 stereo cameras
- **Onboard Computer**: NVIDIA Jetson Xavier NX
- **Flight Controller**: Pixhawk 4

## Software Dependencies

- **ROS2 Humble**: Robot Operating System 2
- **FAST-LIO**: LiDAR-inertial odometry algorithm
- **ORB-SLAM3**: Visual SLAM algorithm
- **Open3D**: Point cloud processing library
- **PyTorch**: Deep learning framework for AI components

## API Endpoints

The Indoor Drone System provides the following API endpoints:

- `/drones`: Get all available drones and their states
- `/drones/{drone_id}`: Get a specific drone by ID
- `/drones/{drone_id}/command`: Send a command to a drone
- `/missions`: Create and manage mission plans
- `/missions/{mission_id}/execute`: Execute a mission with a specific drone
- `/maps`: Get all available maps
- `/maps/{map_id}`: Get a specific map by ID
- `/maps/create`: Start creating a new map with a specific drone
- `/ws/telemetry/{drone_id}`: WebSocket for real-time telemetry data

## Integration with Bulo.Cloud Sentinel

The Indoor Drone System integrates with the Bulo.Cloud Sentinel platform through:

1. **REST API**: For command and control
2. **WebSockets**: For real-time telemetry data
3. **Shared Data Storage**: For maps and mission plans

## Development

### Prerequisites

- Docker and Docker Compose
- NVIDIA GPU with CUDA support (for SLAM and AI components)
- ROS2 Humble (for local development)

### Running the System

```bash
# Clone the repository
git clone https://github.com/your-org/indoor-drone-system.git
cd indoor-drone-system

# Start the services
docker-compose up -d
```

### Testing

The system includes a simulation environment for testing without physical hardware:

```bash
# Start the simulation
docker-compose -f docker-compose.simulation.yml up -d

# Run tests
docker-compose exec api pytest
```
