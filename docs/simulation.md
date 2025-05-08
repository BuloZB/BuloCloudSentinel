# Digital Twin & Simulation

This document provides an overview of the Digital Twin & Simulation capabilities of the Bulo.CloudSentinel platform.

## Overview

The Digital Twin & Simulation module provides a deterministic, software-in-the-loop (SITL) environment that mirrors real deployments, enabling development and testing of swarm logic without risking physical drones. It uses Ignition Gazebo as the physics and rendering engine, with ROS 2 Humble as the robotics middleware.

## Architecture

The simulation environment consists of the following components:

- **Ignition Gazebo**: Core physics and rendering engine
- **ROS 2 Humble**: Robotics middleware for communication
- **ros_gz_bridge**: Bridge between Ignition and ROS 2
- **sim_swarm_gateway**: Container that relays ROS2 topics to existing `drone_core`
- **Drone Models**: Realistic drone models with physics properties
- **World Models**: Urban environments with buildings, streets, and obstacles

## Features

- **Realistic Physics**: Accurate simulation of drone dynamics, aerodynamics, and environmental effects
- **Sensor Simulation**: Simulation of various sensors including cameras, LiDAR, IMU, GPS, etc.
- **Environmental Effects**: Simulation of weather conditions, time of day, and other environmental factors
- **Multi-Drone Support**: Simulation of multiple drones in the same environment
- **Urban Environments**: Realistic urban environments with buildings, streets, and obstacles
- **Integration with Bulo.CloudSentinel**: Seamless integration with the existing platform

## Getting Started

### Prerequisites

- Docker and Docker Compose
- NVIDIA GPU with CUDA support (recommended)
- Kubernetes cluster (for production deployment)

### Running the Simulation

#### Using Docker Compose

```bash
# Start the simulation environment
docker-compose -f sim/docker/docker-compose.yml up -d

# Spawn a world with 5 drones
python sim/scripts/spawn_world.py --drones 5 --world urban_small

# Run tests
pytest sim/tests/
```

#### Using Kubernetes

```bash
# Install the Helm chart
helm install ignition-sim sim/helm/ignition-sim

# Port-forward to access the web UI
kubectl port-forward svc/ignition-sim-sim-web-ui 8080:8080
```

## Typical Workflows

### Development Workflow

1. Start the simulation environment
2. Spawn a world with the desired number of drones
3. Develop and test swarm logic
4. Run tests to verify functionality
5. Deploy to production

### Testing Workflow

1. Start the simulation environment
2. Run automated tests
3. Analyze results
4. Fix issues
5. Repeat

### CI/CD Integration

The simulation environment is integrated with GitHub Actions for continuous integration and deployment. The workflow is defined in `.github/workflows/sim.yml` and runs the following steps:

1. Build the simulation environment
2. Run unit tests
3. Start Ignition Gazebo
4. Run integration tests
5. Run swarm path test
6. Upload coverage report

## World Models

The simulation environment includes the following world models:

- **urban_small**: Small urban environment with a few buildings and streets
- **urban_large**: Large urban environment with many buildings and streets
- **suburban**: Suburban environment with houses and streets
- **rural**: Rural environment with fields and roads
- **indoor_warehouse**: Indoor warehouse environment
- **indoor_office**: Indoor office environment

## Drone Models

The simulation environment includes the following drone models:

- **quadcopter**: Standard quadcopter drone
- **hexacopter**: Hexacopter drone with six rotors
- **octocopter**: Octocopter drone with eight rotors
- **fixed_wing**: Fixed-wing drone

## Sensor Suites

The simulation environment includes the following sensor suites:

- **basic**: Basic sensors (IMU, GPS, barometer)
- **advanced**: Advanced sensors (IMU, GPS, barometer, cameras)
- **lidar**: LiDAR sensors (IMU, GPS, barometer, LiDAR)
- **thermal**: Thermal sensors (IMU, GPS, barometer, thermal camera)
- **full**: Full sensor suite (IMU, GPS, barometer, cameras, LiDAR, thermal camera)

## Weather Conditions

The simulation environment supports the following weather conditions:

- **clear**: Clear weather
- **cloudy**: Cloudy weather
- **rainy**: Rainy weather
- **foggy**: Foggy weather
- **stormy**: Stormy weather

## Time of Day

The simulation environment supports the following times of day:

- **dawn**: Dawn
- **morning**: Morning
- **noon**: Noon
- **afternoon**: Afternoon
- **dusk**: Dusk
- **night**: Night

## API Reference

### Spawn World API

The `spawn_world.py` script provides a command-line interface to spawn a Gazebo world with customizable parameters:

```bash
python sim/scripts/spawn_world.py --drones 5 --world urban_small --weather clear --time noon
```

Parameters:

- `--world`: World to spawn (default: urban_small)
- `--drones`: Number of drones to spawn (default: 1)
- `--weather`: Weather conditions (default: clear)
- `--time`: Time of day (default: noon)
- `--wind-speed`: Wind speed in m/s (default: 0.0)
- `--wind-direction`: Wind direction in degrees (default: 0.0)
- `--rain`: Rain intensity (0.0-1.0) (default: 0.0)
- `--fog`: Fog density (0.0-1.0) (default: 0.0)
- `--drone-type`: Type of drone to spawn (default: quadcopter)
- `--sensor-suite`: Sensor suite to equip on drones (default: basic)

### Swarm Gateway API

The Swarm Gateway provides a REST API for controlling the simulation:

- `GET /api/drones`: Get all drones
- `GET /api/drones/{drone_id}`: Get a specific drone
- `POST /api/drones/{drone_id}/command`: Send a command to a drone
- `GET /api/missions`: Get all missions
- `POST /api/missions`: Create a new mission
- `GET /api/missions/{mission_id}`: Get a specific mission
- `PUT /api/missions/{mission_id}`: Update a mission
- `DELETE /api/missions/{mission_id}`: Delete a mission
- `GET /api/telemetry/{drone_id}`: Get telemetry for a specific drone
- `GET /api/video/{drone_id}`: Get video stream for a specific drone

## Security & Compliance

- All containers run as non-root with read-only filesystem
- No privileged flags are used
- Parameters are validated through YAML schemas
- All new assets are licensed under CC-BY-4.0 or more permissive

## Troubleshooting

### Common Issues

- **Gazebo fails to start**: Check GPU drivers and ensure NVIDIA runtime is available
- **ROS 2 bridge fails to connect**: Check network configuration and ensure Gazebo is running
- **Drones not spawning**: Check world configuration and ensure Gazebo is running
- **Tests failing**: Check logs for errors and ensure all components are running

### Logs

- Gazebo logs: `/data/logs/gazebo.log`
- ROS 2 logs: `/data/logs/ros2.log`
- Swarm Gateway logs: `/data/logs/gateway.log`
- Web UI logs: `/data/logs/web.log`

## References

- [Ignition Gazebo Documentation](https://ignitionrobotics.org/docs/citadel)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [ros_gz Documentation](https://github.com/gazebosim/ros_gz)
