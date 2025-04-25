"""
Repository for sensor fusion data persistence.
"""

from typing import List, Dict, Any, Optional
from datetime import datetime, timezone, timedelta
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import desc, func
from backend.infrastructure.persistence.sensor_fusion_models import (
    SensorModel,
    SensorDataModel,
    FusedDataModel,
    SensorFusionConfigModel
)


class SensorFusionRepository:
    """
    Repository for sensor fusion data persistence operations.
    """

    def __init__(self, session: AsyncSession):
        """
        Initialize the repository with a database session.

        Args:
            session: SQLAlchemy async session
        """
        self.session = session

    async def register_sensor(self, sensor_id: str, sensor_type: str, capabilities: List[str]) -> SensorModel:
        """
        Register a new sensor or update an existing one.

        Args:
            sensor_id: Unique identifier for the sensor
            sensor_type: Type of sensor (camera, radar, lidar, etc.)
            capabilities: List of sensor capabilities

        Returns:
            The created or updated sensor model
        """
        # Check if sensor already exists
        stmt = select(SensorModel).where(SensorModel.sensor_id == sensor_id)
        result = await self.session.execute(stmt)
        sensor = result.scalars().first()

        if sensor:
            # Update existing sensor
            sensor.sensor_type = sensor_type
            sensor.capabilities = capabilities
            sensor.status = "registered"
        else:
            # Create new sensor
            sensor = SensorModel(
                sensor_id=sensor_id,
                sensor_type=sensor_type,
                capabilities=capabilities,
                status="registered",
                created_at=datetime.now(timezone.utc)
            )
            self.session.add(sensor)

        await self.session.commit()
        await self.session.refresh(sensor)
        return sensor

    async def update_sensor_status(self, sensor_id: str, status: str, last_update: Optional[datetime] = None) -> bool:
        """
        Update a sensor's status and last update time.

        Args:
            sensor_id: Unique identifier for the sensor
            status: New status (active, stale, inactive)
            last_update: Last update timestamp

        Returns:
            True if the sensor was updated, False otherwise
        """
        stmt = select(SensorModel).where(SensorModel.sensor_id == sensor_id)
        result = await self.session.execute(stmt)
        sensor = result.scalars().first()

        if not sensor:
            return False

        sensor.status = status
        if last_update:
            sensor.last_update = last_update

        await self.session.commit()
        return True

    async def store_sensor_data(self, sensor_id: str, data: Dict[str, Any], timestamp: Optional[datetime] = None) -> SensorDataModel:
        """
        Store a data point from a sensor.

        Args:
            sensor_id: Unique identifier for the sensor
            data: Sensor data payload
            timestamp: Data timestamp (defaults to current time)

        Returns:
            The created sensor data model
        """
        # Get sensor by sensor_id
        stmt = select(SensorModel).where(SensorModel.sensor_id == sensor_id)
        result = await self.session.execute(stmt)
        sensor = result.scalars().first()

        if not sensor:
            raise ValueError(f"Sensor {sensor_id} not found")

        # Update sensor status and last update time
        sensor.status = "active"
        sensor.last_update = timestamp or datetime.now(timezone.utc)

        # Create sensor data entry
        sensor_data = SensorDataModel(
            sensor_id=sensor.id,
            timestamp=timestamp or datetime.now(timezone.utc),
            data=data
        )

        self.session.add(sensor_data)
        await self.session.commit()
        await self.session.refresh(sensor_data)
        return sensor_data

    async def store_fused_data(self, fused_data: Dict[str, Any]) -> FusedDataModel:
        """
        Store a snapshot of fused data.

        Args:
            fused_data: Fused data dictionary

        Returns:
            The created fused data model
        """
        timestamp = datetime.fromisoformat(fused_data.get("timestamp", datetime.now(timezone.utc).isoformat()))

        fused_data_model = FusedDataModel(
            timestamp=timestamp,
            position=fused_data.get("position"),
            detections=fused_data.get("detections"),
            classifications=fused_data.get("classifications"),
            telemetry=fused_data.get("telemetry")
        )

        self.session.add(fused_data_model)
        await self.session.commit()
        await self.session.refresh(fused_data_model)
        return fused_data_model

    async def get_latest_fused_data(self) -> Optional[FusedDataModel]:
        """
        Get the latest fused data snapshot.

        Returns:
            The latest fused data model or None if no data exists
        """
        stmt = select(FusedDataModel).order_by(desc(FusedDataModel.timestamp)).limit(1)
        result = await self.session.execute(stmt)
        return result.scalars().first()

    async def get_fused_data_history(self, start_time: datetime, end_time: datetime) -> List[FusedDataModel]:
        """
        Get fused data snapshots within a time range.

        Args:
            start_time: Start of time range
            end_time: End of time range

        Returns:
            List of fused data models within the time range
        """
        stmt = select(FusedDataModel).where(
            FusedDataModel.timestamp >= start_time,
            FusedDataModel.timestamp <= end_time
        ).order_by(FusedDataModel.timestamp)

        result = await self.session.execute(stmt)
        return result.scalars().all()

    async def get_sensor_data_history(self, sensor_id: str, start_time: datetime, end_time: datetime) -> List[SensorDataModel]:
        """
        Get sensor data points within a time range.

        Args:
            sensor_id: Unique identifier for the sensor
            start_time: Start of time range
            end_time: End of time range

        Returns:
            List of sensor data models within the time range
        """
        # Get sensor by sensor_id
        stmt = select(SensorModel).where(SensorModel.sensor_id == sensor_id)
        result = await self.session.execute(stmt)
        sensor = result.scalars().first()

        if not sensor:
            return []

        # Get sensor data points
        stmt = select(SensorDataModel).where(
            SensorDataModel.sensor_id == sensor.id,
            SensorDataModel.timestamp >= start_time,
            SensorDataModel.timestamp <= end_time
        ).order_by(SensorDataModel.timestamp)

        result = await self.session.execute(stmt)
        return result.scalars().all()

    async def get_all_sensors(self) -> List[SensorModel]:
        """
        Get all registered sensors.

        Returns:
            List of all sensor models
        """
        stmt = select(SensorModel)
        result = await self.session.execute(stmt)
        return result.scalars().all()

    async def get_sensor_by_id(self, sensor_id: str) -> Optional[SensorModel]:
        """
        Get a sensor by its ID.

        Args:
            sensor_id: Unique identifier for the sensor

        Returns:
            The sensor model or None if not found
        """
        stmt = select(SensorModel).where(SensorModel.sensor_id == sensor_id)
        result = await self.session.execute(stmt)
        return result.scalars().first()

    async def set_config(self, key: str, value: Any, description: Optional[str] = None) -> SensorFusionConfigModel:
        """
        Set a configuration value.

        Args:
            key: Configuration key
            value: Configuration value
            description: Optional description

        Returns:
            The created or updated config model
        """
        stmt = select(SensorFusionConfigModel).where(SensorFusionConfigModel.key == key)
        result = await self.session.execute(stmt)
        config = result.scalars().first()

        if config:
            # Update existing config
            config.value = value
            if description:
                config.description = description
            config.updated_at = datetime.now(timezone.utc)
        else:
            # Create new config
            config = SensorFusionConfigModel(
                key=key,
                value=value,
                description=description,
                updated_at=datetime.now(timezone.utc)
            )
            self.session.add(config)

        await self.session.commit()
        await self.session.refresh(config)
        return config

    async def get_config(self, key: str) -> Optional[Any]:
        """
        Get a configuration value.

        Args:
            key: Configuration key

        Returns:
            The configuration value or None if not found
        """
        stmt = select(SensorFusionConfigModel).where(SensorFusionConfigModel.key == key)
        result = await self.session.execute(stmt)
        config = result.scalars().first()

        if not config:
            return None

        return config.value

    async def cleanup_old_data(self, days_to_keep: int = 30) -> int:
        """
        Clean up old sensor data and fused data.

        Args:
            days_to_keep: Number of days of data to keep

        Returns:
            Number of records deleted
        """
        cutoff_date = datetime.utcnow() - timedelta(days=days_to_keep)

        # Delete old sensor data
        stmt = select(func.count()).select_from(SensorDataModel).where(SensorDataModel.timestamp < cutoff_date)
        result = await self.session.execute(stmt)
        sensor_data_count = result.scalar()

        if sensor_data_count > 0:
            await self.session.execute(
                f"DELETE FROM {SensorDataModel.__tablename__} WHERE timestamp < :cutoff",
                {"cutoff": cutoff_date}
            )

        # Delete old fused data
        stmt = select(func.count()).select_from(FusedDataModel).where(FusedDataModel.timestamp < cutoff_date)
        result = await self.session.execute(stmt)
        fused_data_count = result.scalar()

        if fused_data_count > 0:
            await self.session.execute(
                f"DELETE FROM {FusedDataModel.__tablename__} WHERE timestamp < :cutoff",
                {"cutoff": cutoff_date}
            )

        await self.session.commit()
        return sensor_data_count + fused_data_count
