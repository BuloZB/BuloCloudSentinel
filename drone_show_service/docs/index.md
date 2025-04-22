# Drone Show Microservice

Welcome to the documentation for the Drone Show microservice, a component of the Bulo.Cloud Sentinel platform.

## Overview

The Drone Show microservice enables planning, simulation, and execution of choreographed drone light shows. It provides a comprehensive set of tools for creating, managing, and executing drone show choreographies.

## Features

- **Choreography Definition**: JSON-based format for defining LED patterns and waypoints
- **Simulation & Preview**: 3D visualization of drone positions and LED states
- **Execution Engine**: Synchronized execution of choreographies across a fleet of drones
- **Monitoring & Logging**: Real-time telemetry and logging of show execution
- **Integration**: Seamless integration with Bulo.Cloud Sentinel platform
- **Blender Integration**: Import animations from Blender for easy show creation
- **Music Synchronization**: Synchronize LED patterns with music
- **Time Synchronization**: Precise timing across all drones for coordinated shows
- **Formation Transitions**: Smooth transitions between different formations
- **Battery Management**: Optimize shows based on battery levels

## Architecture

The Drone Show microservice is built on a modern, scalable architecture:

- **FastAPI Backend**: High-performance, async API built with FastAPI
- **PostgreSQL Database**: Reliable storage for choreography and execution data
- **Redis Cache**: Fast in-memory cache for performance optimization
- **MinIO Object Storage**: Scalable object storage for choreography data and logs
- **WebSocket Support**: Real-time communication for live updates
- **Docker Containerization**: Easy deployment and scaling
- **Kubernetes Support**: Production-ready deployment with Kubernetes

## Getting Started

To get started with the Drone Show microservice, see the following guides:

- [API Documentation](api.md): Detailed information about the API endpoints
- [Blender Integration](blender_integration.md): How to create choreographies using Blender
- [Integration with Bulo.Cloud Sentinel](integration.md): How to integrate with the Bulo.Cloud Sentinel platform
- [Deployment](deployment.md): How to deploy the microservice

## Code Reference

For detailed information about the code, see the following references:

- [Models](reference/models.md): Data models used in the microservice
- [Services](reference/services.md): Services that implement the business logic
- [API](reference/api.md): API endpoints and request handlers
- [Utils](reference/utils.md): Utility functions and helpers
