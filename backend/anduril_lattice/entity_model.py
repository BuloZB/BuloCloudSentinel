"""
Anduril Lattice Entity Model for Bulo.Cloud Sentinel.

This module defines the Entity data model compatible with Anduril's Lattice platform.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Any, Optional


class EntityType(Enum):
    """Entity template types in Anduril's Lattice platform."""
    TRACK = "TEMPLATE_TRACK"
    SENSOR_POINT_OF_INTEREST = "TEMPLATE_SENSOR_POINT_OF_INTEREST"
    SIGNAL_OF_INTEREST = "TEMPLATE_SIGNAL_OF_INTEREST"
    ASSET = "TEMPLATE_ASSET"
    GEO = "TEMPLATE_GEO"


class EntityComponent:
    """Base class for entity components."""
    pass


@dataclass
class Entity:
    """
    Entity data model compatible with Anduril's Lattice platform.

    An entity represents any type of known object that can be referenced or commanded
    in the Common Operational Picture (COP).
    """

    # Required components
    entity_id: str = ""
    is_live: bool = True
    expiry_time: Optional[str] = None  # ISO-8601 format

    # Provenance information
    provenance: Dict[str, str] = field(default_factory=dict)

    # Common components
    aliases: Dict[str, str] = field(default_factory=dict)
    description: Optional[str] = None
    created_time: Optional[str] = None  # ISO-8601 format

    # Location components
    location: Optional[Dict[str, Any]] = None

    # Military view components
    mil_view: Optional[Dict[str, str]] = None

    # Ontology components
    ontology: Optional[Dict[str, str]] = None

    # Geo components
    geo_shape: Optional[Dict[str, Any]] = None
    geo_details: Optional[Dict[str, Any]] = None

    # Signal components
    signal: Optional[Dict[str, Any]] = None

    # Sensor components
    sensors: Optional[List[Dict[str, Any]]] = None

    # Additional components
    additional_components: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the entity to a dictionary representation.

        Returns:
            Dictionary representation of the entity
        """
        result = {
            "entity_id": self.entity_id,
            "is_live": self.is_live
        }

        if self.expiry_time:
            result["expiry_time"] = self.expiry_time

        if self.provenance:
            result["provenance"] = self.provenance

        if self.aliases:
            result["aliases"] = self.aliases

        if self.description:
            result["description"] = self.description

        if self.created_time:
            result["created_time"] = self.created_time

        if self.location:
            result["location"] = self.location

        if self.mil_view:
            result["mil_view"] = self.mil_view

        if self.ontology:
            result["ontology"] = self.ontology

        if self.geo_shape:
            result["geo_shape"] = self.geo_shape

        if self.geo_details:
            result["geo_details"] = self.geo_details

        if self.signal:
            result["signal"] = self.signal

        if self.sensors:
            result["sensors"] = self.sensors

        # Add any additional components
        result.update(self.additional_components)

        return result

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Entity':
        """
        Create an entity from a dictionary.

        Args:
            data: Dictionary representation of the entity

        Returns:
            Entity instance
        """
        # Extract known components
        entity_id = data.pop("entity_id", "")
        is_live = data.pop("is_live", True)
        expiry_time = data.pop("expiry_time", None)
        provenance = data.pop("provenance", {})
        aliases = data.pop("aliases", {})
        description = data.pop("description", None)
        created_time = data.pop("created_time", None)
        location = data.pop("location", None)
        mil_view = data.pop("mil_view", None)
        ontology = data.pop("ontology", None)
        geo_shape = data.pop("geo_shape", None)
        geo_details = data.pop("geo_details", None)
        signal = data.pop("signal", None)
        sensors = data.pop("sensors", None)

        # Create entity
        entity = cls(
            entity_id=entity_id,
            is_live=is_live,
            expiry_time=expiry_time,
            provenance=provenance,
            aliases=aliases,
            description=description,
            created_time=created_time,
            location=location,
            mil_view=mil_view,
            ontology=ontology,
            geo_shape=geo_shape,
            geo_details=geo_details,
            signal=signal,
            sensors=sensors
        )

        # Add any remaining components as additional components
        entity.additional_components = data

        return entity


def create_track_entity(
    entity_id: str,
    name: str,
    latitude: float,
    longitude: float,
    altitude: Optional[float] = None,
    disposition: str = "DISPOSITION_UNKNOWN",
    environment: str = "ENVIRONMENT_SURFACE",
    platform_type: str = "Unknown",
    integration_name: str = "BuloCloudSentinel",
    data_type: str = "SurveillancePlatform",
    expiry_minutes: int = 5
) -> Entity:
    """
    Create a track entity.

    Args:
        entity_id: Unique entity ID
        name: Human-readable name
        latitude: Latitude in degrees
        longitude: Longitude in degrees
        altitude: Optional altitude in meters
        disposition: Military disposition (FRIENDLY, HOSTILE, etc.)
        environment: Environment (AIR, SURFACE, etc.)
        platform_type: Type of platform (Airplane, Vehicle, etc.)
        integration_name: Name of the integration
        data_type: Type of data
        expiry_minutes: Expiry time in minutes from now

    Returns:
        Track entity
    """
    from datetime import datetime, timedelta, timezone

    # Create position
    position = {
        "latitude_degrees": latitude,
        "longitude_degrees": longitude
    }

    if altitude is not None:
        position["altitude_hae_meters"] = altitude

    # Create entity
    entity = Entity(
        entity_id=entity_id,
        is_live=True,
        expiry_time=(datetime.now(timezone.utc) + timedelta(minutes=expiry_minutes)).isoformat(),
        aliases={"name": name},
        location={"position": position},
        mil_view={
            "disposition": disposition,
            "environment": environment
        },
        ontology={
            "template": EntityType.TRACK.value,
            "platform_type": platform_type
        },
        provenance={
            "integration_name": integration_name,
            "data_type": data_type,
            "source_update_time": datetime.now(timezone.utc).isoformat()
        }
    )

    return entity


def create_asset_entity(
    entity_id: str,
    name: str,
    latitude: float,
    longitude: float,
    altitude: Optional[float] = None,
    disposition: str = "DISPOSITION_FRIENDLY",
    environment: str = "ENVIRONMENT_SURFACE",
    platform_type: str = "Surveillance",
    integration_name: str = "BuloCloudSentinel",
    data_type: str = "SurveillancePlatform",
    expiry_minutes: int = 30
) -> Entity:
    """
    Create an asset entity.

    Args:
        entity_id: Unique entity ID
        name: Human-readable name
        latitude: Latitude in degrees
        longitude: Longitude in degrees
        altitude: Optional altitude in meters
        disposition: Military disposition (FRIENDLY, HOSTILE, etc.)
        environment: Environment (AIR, SURFACE, etc.)
        platform_type: Type of platform (UAV, Radar, etc.)
        integration_name: Name of the integration
        data_type: Type of data
        expiry_minutes: Expiry time in minutes from now

    Returns:
        Asset entity
    """
    from datetime import datetime, timedelta, timezone

    # Create position
    position = {
        "latitude_degrees": latitude,
        "longitude_degrees": longitude
    }

    if altitude is not None:
        position["altitude_hae_meters"] = altitude

    # Create entity
    entity = Entity(
        entity_id=entity_id,
        is_live=True,
        expiry_time=(datetime.now(timezone.utc) + timedelta(minutes=expiry_minutes)).isoformat(),
        aliases={"name": name},
        location={"position": position},
        mil_view={
            "disposition": disposition,
            "environment": environment
        },
        ontology={
            "template": EntityType.ASSET.value,
            "platform_type": platform_type
        },
        provenance={
            "integration_name": integration_name,
            "data_type": data_type,
            "source_update_time": datetime.now(timezone.utc).isoformat()
        }
    )

    return entity


def create_geo_entity(
    entity_id: str,
    name: str,
    geo_type: str = "GEO_TYPE_GENERAL",
    integration_name: str = "BuloCloudSentinel",
    data_type: str = "SurveillancePlatform",
    expiry_minutes: int = 60,
    **geo_shape_args
) -> Entity:
    """
    Create a geo entity.

    Args:
        entity_id: Unique entity ID
        name: Human-readable name
        geo_type: Type of geo entity (GENERAL, CONTROL_AREA, etc.)
        integration_name: Name of the integration
        data_type: Type of data
        expiry_minutes: Expiry time in minutes from now
        geo_shape_args: Arguments for the geo shape (point, polygon, etc.)

    Returns:
        Geo entity
    """
    from datetime import datetime, timedelta, timezone

    # Create entity
    entity = Entity(
        entity_id=entity_id,
        is_live=True,
        expiry_time=(datetime.now(timezone.utc) + timedelta(minutes=expiry_minutes)).isoformat(),
        aliases={"name": name},
        geo_details={"type": geo_type},
        ontology={"template": EntityType.GEO.value},
        provenance={
            "integration_name": integration_name,
            "data_type": data_type,
            "source_update_time": datetime.now(timezone.utc).isoformat()
        }
    )

    # Add geo shape
    if "point" in geo_shape_args:
        entity.geo_shape = {"point": geo_shape_args["point"]}
    elif "polygon" in geo_shape_args:
        entity.geo_shape = {"polygon": geo_shape_args["polygon"]}
    elif "ellipse" in geo_shape_args:
        entity.geo_shape = {"ellipse": geo_shape_args["ellipse"]}
    elif "line" in geo_shape_args:
        entity.geo_shape = {"line": geo_shape_args["line"]}

    return entity
