# Digital Twin & Simulation for Bulo.CloudSentinel

This module provides a deterministic, software-in-the-loop (SITL) environment that mirrors real deployments, enabling development and testing of swarm logic without risking physical drones.

## Features

- **Ignition Gazebo Integration**: Physics and rendering engine for realistic drone simulation
- **ROS 2 Humble Bridge**: Seamless integration with ROS 2 ecosystem
- **Swarm Simulation**: Support for multi-drone scenarios
- **Urban Environment**: Realistic urban world with buildings, streets, and GPS multipath effects
- **Telemetry Bridge**: Integration with existing Bulo.CloudSentinel platform
- **Automated Testing**: CI/CD integration for continuous validation

## Architecture

The simulation environment consists of the following components:

- **Ignition Gazebo**: Core physics and rendering engine
- **ROS 2 Humble**: Robotics middleware for communication
- **ros_gz_bridge**: Bridge between Ignition and ROS 2
- **sim_swarm_gateway**: Container that relays ROS2 topics to existing `drone_core`
- **Drone Models**: Realistic drone models with physics properties
- **World Models**: Urban environments with buildings, streets, and obstacles

## Directory Structure

```
sim/
├── bazel/                  # Bazel build configuration
├── docker/                 # Docker configuration files
├── helm/                   # Helm charts for Kubernetes deployment
├── models/                 # Gazebo models
│   ├── drones/             # Drone models
│   ├── environments/       # Environment models
│   └── sensors/            # Sensor models
├── ros2_ws/                # ROS 2 workspace
│   ├── src/                # ROS 2 packages
│   │   ├── sim_bridge/     # Bridge between ROS 2 and Bulo.CloudSentinel
│   │   ├── sim_control/    # Drone control in simulation
│   │   ├── sim_description/# Robot description files
│   │   └── sim_gazebo/     # Gazebo integration
├── scripts/                # Utility scripts
│   ├── spawn_world.py      # Script to spawn world with parameters
│   └── run_tests.py        # Script to run tests
└── tests/                  # Test files
    ├── integration/        # Integration tests
    └── unit/               # Unit tests
```

## Getting Started

### Prerequisites

- Docker and Docker Compose
- NVIDIA GPU with CUDA support (recommended)
- Kubernetes cluster (for production deployment)

### Running the Simulation

```bash
# Start the simulation environment
docker-compose -f sim/docker/docker-compose.yml up -d

# Spawn a world with 5 drones
python sim/scripts/spawn_world.py --drones 5 --world urban_small

# Run tests
pytest sim/tests/
```

## Integration with Bulo.CloudSentinel

The simulation environment integrates with the existing Bulo.CloudSentinel platform through the `sim_swarm_gateway` container, which relays ROS2 topics to the existing `drone_core` over the local network. This allows the SentinelWeb UI to display simulation data without modifications.

## Security & Compliance

- All containers run as non-root with read-only filesystem
- No privileged flags are used
- Parameters are validated through YAML schemas
- All new assets are licensed under CC-BY-4.0 or more permissive
