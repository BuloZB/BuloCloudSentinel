"""
Sample data generation script for the Drone Show microservice.

This script generates sample data for the Drone Show microservice.
"""

import asyncio
import logging
import json
import math
import random
import secrets  # For secure random number generation when needed
from datetime import datetime
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker

from drone_show_service.core.config import settings
from drone_show_service.models.database import Base, ChoreographyDB
from drone_show_service.models.choreography import (
    Choreography, ChoreographyMetadata, ChoreographyType, ChoreographyStatus,
    DroneTrajectory, Waypoint, Position, LEDState, LEDColor, LEDEffect
)


async def generate_sample_data():
    """Generate sample data."""
    # Configure logging
    logging.basicConfig(
        level=getattr(logging, settings.LOG_LEVEL),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
    logger = logging.getLogger(__name__)

    # Create engine
    logger.info(f"Connecting to database: {settings.DATABASE_URL}")
    engine = create_async_engine(
        settings.DATABASE_URL,
        echo=True,
        future=True,
    )

    # Create session factory
    async_session = sessionmaker(
        autocommit=False,
        autoflush=False,
        bind=engine,
        class_=AsyncSession,
        expire_on_commit=False,
    )

    # Create tables
    logger.info("Creating tables...")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    # Generate sample data
    logger.info("Generating sample data...")

    # Create sample choreographies
    async with async_session() as session:
        # Create circle formation choreography
        await create_circle_formation(session, logger)

        # Create grid formation choreography
        await create_grid_formation(session, logger)

        # Create custom formation choreography
        await create_custom_formation(session, logger)

    logger.info("Sample data generated successfully")


async def create_circle_formation(session, logger):
    """Create a circle formation choreography."""
    logger.info("Creating circle formation choreography...")

    # Parameters
    num_drones = 12
    radius = 20.0  # meters
    center_lat = 37.7749
    center_lon = -122.4194
    altitude = 30.0  # meters
    duration = 120.0  # seconds

    # Create trajectories
    trajectories = []
    for i in range(num_drones):
        # Calculate initial position on circle
        angle = 2 * math.pi * i / num_drones
        lat = center_lat + (radius / 111320) * math.cos(angle)
        lon = center_lon + (radius / (111320 * math.cos(center_lat * math.pi / 180))) * math.sin(angle)

        # Create waypoints
        waypoints = []
        led_states = []

        # Add takeoff waypoint
        waypoints.append(
            Waypoint(
                time=0.0,
                position=Position(lat=lat, lon=lon, alt=0.0),
                heading=0.0,
            )
        )

        # Add waypoints for circle
        for t in range(0, int(duration) + 1, 10):
            # Calculate position on circle
            angle = 2 * math.pi * i / num_drones + (2 * math.pi * t / duration)
            lat = center_lat + (radius / 111320) * math.cos(angle)
            lon = center_lon + (radius / (111320 * math.cos(center_lat * math.pi / 180))) * math.sin(angle)

            # Add waypoint
            waypoints.append(
                Waypoint(
                    time=float(t),
                    position=Position(lat=lat, lon=lon, alt=altitude),
                    heading=math.degrees(angle) + 90.0,
                )
            )

            # Add LED state
            hue = (i / num_drones + t / duration) % 1.0
            r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
            led_states.append(
                LEDState(
                    time=float(t),
                    color=LEDColor(r=r, g=g, b=b),
                    effect=LEDEffect.SOLID,
                )
            )

        # Add landing waypoint
        waypoints.append(
            Waypoint(
                time=duration,
                position=Position(lat=lat, lon=lon, alt=0.0),
                heading=math.degrees(angle) + 90.0,
            )
        )

        # Create trajectory
        trajectories.append(
            DroneTrajectory(
                drone_id=f"drone_{i+1}",
                waypoints=waypoints,
                led_states=led_states,
            )
        )

    # Create choreography
    choreography = Choreography(
        metadata=ChoreographyMetadata(
            name="Circle Formation",
            description="A simple circle formation with color gradient",
            author="Drone Show Generator",
            tags=["circle", "formation", "sample"],
            duration=duration,
            drone_count=num_drones,
            status=ChoreographyStatus.READY,
        ),
        type=ChoreographyType.FORMATION,
        trajectories=trajectories,
    )

    # Convert to database model
    db_choreography = ChoreographyDB(
        name=choreography.metadata.name,
        description=choreography.metadata.description,
        author=choreography.metadata.author,
        tags=choreography.metadata.tags,
        duration=choreography.metadata.duration,
        drone_count=choreography.metadata.drone_count,
        status=choreography.metadata.status,
        type=choreography.type,
        trajectories=json.loads(json.dumps([t.dict() for t in choreography.trajectories])),
    )

    # Add to database
    session.add(db_choreography)
    await session.commit()

    logger.info(f"Created circle formation choreography with ID: {db_choreography.id}")


async def create_grid_formation(session, logger):
    """Create a grid formation choreography."""
    logger.info("Creating grid formation choreography...")

    # Parameters
    rows = 5
    cols = 5
    spacing = 10.0  # meters
    center_lat = 37.7749
    center_lon = -122.4194
    altitude = 30.0  # meters
    duration = 120.0  # seconds

    # Create trajectories
    trajectories = []
    for row in range(rows):
        for col in range(cols):
            # Calculate position in grid
            lat = center_lat + ((row - (rows - 1) / 2) * spacing / 111320)
            lon = center_lon + ((col - (cols - 1) / 2) * spacing / (111320 * math.cos(center_lat * math.pi / 180)))

            # Create waypoints
            waypoints = []
            led_states = []

            # Add takeoff waypoint
            waypoints.append(
                Waypoint(
                    time=0.0,
                    position=Position(lat=lat, lon=lon, alt=0.0),
                    heading=0.0,
                )
            )

            # Add waypoints for grid
            for t in range(0, int(duration) + 1, 10):
                # Calculate altitude oscillation
                alt = altitude + 5.0 * math.sin(2 * math.pi * t / duration)

                # Add waypoint
                waypoints.append(
                    Waypoint(
                        time=float(t),
                        position=Position(lat=lat, lon=lon, alt=alt),
                        heading=0.0,
                    )
                )

                # Add LED state
                hue = (row / rows + col / cols + t / duration) % 1.0
                r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
                led_states.append(
                    LEDState(
                        time=float(t),
                        color=LEDColor(r=r, g=g, b=b),
                        effect=LEDEffect.SOLID,
                    )
                )

            # Add landing waypoint
            waypoints.append(
                Waypoint(
                    time=duration,
                    position=Position(lat=lat, lon=lon, alt=0.0),
                    heading=0.0,
                )
            )

            # Create trajectory
            trajectories.append(
                DroneTrajectory(
                    drone_id=f"drone_{row*cols+col+1}",
                    waypoints=waypoints,
                    led_states=led_states,
                )
            )

    # Create choreography
    choreography = Choreography(
        metadata=ChoreographyMetadata(
            name="Grid Formation",
            description="A grid formation with altitude oscillation",
            author="Drone Show Generator",
            tags=["grid", "formation", "sample"],
            duration=duration,
            drone_count=rows * cols,
            status=ChoreographyStatus.READY,
        ),
        type=ChoreographyType.FORMATION,
        trajectories=trajectories,
    )

    # Convert to database model
    db_choreography = ChoreographyDB(
        name=choreography.metadata.name,
        description=choreography.metadata.description,
        author=choreography.metadata.author,
        tags=choreography.metadata.tags,
        duration=choreography.metadata.duration,
        drone_count=choreography.metadata.drone_count,
        status=choreography.metadata.status,
        type=choreography.type,
        trajectories=json.loads(json.dumps([t.dict() for t in choreography.trajectories])),
    )

    # Add to database
    session.add(db_choreography)
    await session.commit()

    logger.info(f"Created grid formation choreography with ID: {db_choreography.id}")


async def create_custom_formation(session, logger):
    """Create a custom formation choreography."""
    logger.info("Creating custom formation choreography...")

    # Parameters
    num_drones = 20
    center_lat = 37.7749
    center_lon = -122.4194
    altitude = 30.0  # meters
    duration = 120.0  # seconds

    # Create trajectories
    trajectories = []
    for i in range(num_drones):
        # Create waypoints
        waypoints = []
        led_states = []

        # Add takeoff waypoint
        # Using deterministic pattern instead of random for sample data
        # Note: For security-sensitive applications, use secrets module instead
        lat = center_lat + (0.001 * math.cos(i * 2 * math.pi / num_drones))
        lon = center_lon + (0.001 * math.sin(i * 2 * math.pi / num_drones))
        waypoints.append(
            Waypoint(
                time=0.0,
                position=Position(lat=lat, lon=lon, alt=0.0),
                heading=0.0,
            )
        )

        # Add waypoints for custom formation
        for t in range(0, int(duration) + 1, 10):
            # Calculate position based on time
            angle = 2 * math.pi * t / duration
            radius = 20.0 + 10.0 * math.sin(3 * angle)
            lat = center_lat + (radius / 111320) * math.cos(angle + i * 2 * math.pi / num_drones)
            lon = center_lon + (radius / (111320 * math.cos(center_lat * math.pi / 180))) * math.sin(angle + i * 2 * math.pi / num_drones)

            # Add waypoint
            waypoints.append(
                Waypoint(
                    time=float(t),
                    position=Position(lat=lat, lon=lon, alt=altitude),
                    heading=math.degrees(angle) + 90.0,
                )
            )

            # Add LED state
            hue = (i / num_drones + t / duration) % 1.0
            r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
            led_states.append(
                LEDState(
                    time=float(t),
                    color=LEDColor(r=r, g=g, b=b),
                    effect=LEDEffect.SOLID,
                )
            )

        # Add landing waypoint
        waypoints.append(
            Waypoint(
                time=duration,
                position=Position(lat=lat, lon=lon, alt=0.0),
                heading=math.degrees(angle) + 90.0,
            )
        )

        # Create trajectory
        trajectories.append(
            DroneTrajectory(
                drone_id=f"drone_{i+1}",
                waypoints=waypoints,
                led_states=led_states,
            )
        )

    # Create choreography
    choreography = Choreography(
        metadata=ChoreographyMetadata(
            name="Flower Formation",
            description="A flower-like formation with dynamic movement",
            author="Drone Show Generator",
            tags=["flower", "formation", "sample"],
            duration=duration,
            drone_count=num_drones,
            status=ChoreographyStatus.READY,
        ),
        type=ChoreographyType.FORMATION,
        trajectories=trajectories,
    )

    # Convert to database model
    db_choreography = ChoreographyDB(
        name=choreography.metadata.name,
        description=choreography.metadata.description,
        author=choreography.metadata.author,
        tags=choreography.metadata.tags,
        duration=choreography.metadata.duration,
        drone_count=choreography.metadata.drone_count,
        status=choreography.metadata.status,
        type=choreography.type,
        trajectories=json.loads(json.dumps([t.dict() for t in choreography.trajectories])),
    )

    # Add to database
    session.add(db_choreography)
    await session.commit()

    logger.info(f"Created custom formation choreography with ID: {db_choreography.id}")


def hsv_to_rgb(h, s, v):
    """Convert HSV color to RGB."""
    if s == 0.0:
        return int(v * 255), int(v * 255), int(v * 255)

    i = int(h * 6.0)
    f = (h * 6.0) - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i %= 6

    if i == 0:
        return int(v * 255), int(t * 255), int(p * 255)
    if i == 1:
        return int(q * 255), int(v * 255), int(p * 255)
    if i == 2:
        return int(p * 255), int(v * 255), int(t * 255)
    if i == 3:
        return int(p * 255), int(q * 255), int(v * 255)
    if i == 4:
        return int(t * 255), int(p * 255), int(v * 255)
    if i == 5:
        return int(v * 255), int(p * 255), int(q * 255)


if __name__ == "__main__":
    asyncio.run(generate_sample_data())
