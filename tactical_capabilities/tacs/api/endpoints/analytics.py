"""
API endpoints for analytics.
"""

from typing import List, Optional, Dict, Any
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from datetime import datetime, timedelta

from api.schemas import (
    TargetAnalytics, SensorAnalytics
)
from core.security import get_current_user, has_permission
from db.session import get_db_session
from db.models import (
    Target as TargetModel,
    Track as TrackModel,
    Sensor as SensorModel,
    AnalyticsSnapshot as AnalyticsSnapshotModel
)

router = APIRouter()

@router.get("/targets", response_model=TargetAnalytics)
async def get_target_analytics(
    time_period: str = Query("last_24h", description="Time period (last_24h, last_7d, last_30d, all)"),
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get target analytics.
    
    Args:
        time_period: Time period
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Target analytics
    """
    # Calculate time threshold based on time period
    now = datetime.utcnow()
    if time_period == "last_24h":
        time_threshold = now - timedelta(hours=24)
    elif time_period == "last_7d":
        time_threshold = now - timedelta(days=7)
    elif time_period == "last_30d":
        time_threshold = now - timedelta(days=30)
    else:  # all
        time_threshold = datetime.min
    
    # Check if we have a recent analytics snapshot
    if time_period != "all":
        snapshot_query = select(AnalyticsSnapshotModel).filter(
            AnalyticsSnapshotModel.snapshot_type == "target_analytics",
            AnalyticsSnapshotModel.time_period == time_period,
            AnalyticsSnapshotModel.timestamp >= now - timedelta(hours=1)  # Use snapshots less than 1 hour old
        ).order_by(AnalyticsSnapshotModel.timestamp.desc())
        
        snapshot_result = await db.execute(snapshot_query)
        snapshot = snapshot_result.scalars().first()
        
        if snapshot:
            # Return snapshot data
            return TargetAnalytics(**snapshot.data, time_period=time_period)
    
    # Get targets
    target_query = select(TargetModel).filter(TargetModel.created_at >= time_threshold)
    target_result = await db.execute(target_query)
    targets = target_result.scalars().all()
    
    # Get tracks
    track_query = select(TrackModel).filter(TrackModel.created_at >= time_threshold)
    track_result = await db.execute(track_query)
    tracks = track_result.scalars().all()
    
    # Calculate analytics
    total_targets = len(targets)
    
    # Count targets by type
    targets_by_type = {}
    for target in targets:
        target_type = target.type
        if target_type in targets_by_type:
            targets_by_type[target_type] += 1
        else:
            targets_by_type[target_type] = 1
    
    # Count targets by status
    targets_by_status = {}
    for target in targets:
        target_status = target.status
        if target_status in targets_by_status:
            targets_by_status[target_status] += 1
        else:
            targets_by_status[target_status] = 1
    
    # Calculate average confidence
    average_confidence = sum(target.confidence for target in targets) / total_targets if total_targets > 0 else 0.0
    
    # Calculate average track quality
    average_track_quality = sum(track.quality for track in tracks) / len(tracks) if tracks else 0.0
    
    # Calculate detection rate (targets per hour)
    time_diff = (now - time_threshold).total_seconds() / 3600  # hours
    detection_rate = total_targets / time_diff if time_diff > 0 else 0.0
    
    # Create analytics
    analytics = TargetAnalytics(
        total_targets=total_targets,
        targets_by_type=targets_by_type,
        targets_by_status=targets_by_status,
        average_confidence=average_confidence,
        average_track_quality=average_track_quality,
        detection_rate=detection_rate,
        time_period=time_period
    )
    
    # Store analytics snapshot
    snapshot = AnalyticsSnapshotModel(
        snapshot_type="target_analytics",
        timestamp=now,
        time_period=time_period,
        data=analytics.dict(exclude={"time_period"})
    )
    
    db.add(snapshot)
    await db.commit()
    
    return analytics

@router.get("/sensors", response_model=SensorAnalytics)
async def get_sensor_analytics(
    time_period: str = Query("last_24h", description="Time period (last_24h, last_7d, last_30d, all)"),
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get sensor analytics.
    
    Args:
        time_period: Time period
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Sensor analytics
    """
    # Calculate time threshold based on time period
    now = datetime.utcnow()
    if time_period == "last_24h":
        time_threshold = now - timedelta(hours=24)
    elif time_period == "last_7d":
        time_threshold = now - timedelta(days=7)
    elif time_period == "last_30d":
        time_threshold = now - timedelta(days=30)
    else:  # all
        time_threshold = datetime.min
    
    # Check if we have a recent analytics snapshot
    if time_period != "all":
        snapshot_query = select(AnalyticsSnapshotModel).filter(
            AnalyticsSnapshotModel.snapshot_type == "sensor_analytics",
            AnalyticsSnapshotModel.time_period == time_period,
            AnalyticsSnapshotModel.timestamp >= now - timedelta(hours=1)  # Use snapshots less than 1 hour old
        ).order_by(AnalyticsSnapshotModel.timestamp.desc())
        
        snapshot_result = await db.execute(snapshot_query)
        snapshot = snapshot_result.scalars().first()
        
        if snapshot:
            # Return snapshot data
            return SensorAnalytics(**snapshot.data, time_period=time_period)
    
    # Get sensors
    sensor_query = select(SensorModel)
    sensor_result = await db.execute(sensor_query)
    sensors = sensor_result.scalars().all()
    
    # Calculate analytics
    total_sensors = len(sensors)
    
    # Count sensors by type
    sensors_by_type = {}
    for sensor in sensors:
        sensor_type = sensor.type
        if sensor_type in sensors_by_type:
            sensors_by_type[sensor_type] += 1
        else:
            sensors_by_type[sensor_type] = 1
    
    # Count sensors by status
    sensors_by_status = {}
    for sensor in sensors:
        sensor_status = sensor.status
        if sensor_status in sensors_by_status:
            sensors_by_status[sensor_status] += 1
        else:
            sensors_by_status[sensor_status] = 1
    
    # In a real implementation, this would calculate detection counts and average confidence
    # For now, we'll just use placeholder values
    detection_counts = {str(sensor.id): 0 for sensor in sensors}
    average_confidence = {str(sensor.id): 0.0 for sensor in sensors}
    
    # Create analytics
    analytics = SensorAnalytics(
        total_sensors=total_sensors,
        sensors_by_type=sensors_by_type,
        sensors_by_status=sensors_by_status,
        detection_counts=detection_counts,
        average_confidence=average_confidence,
        time_period=time_period
    )
    
    # Store analytics snapshot
    snapshot = AnalyticsSnapshotModel(
        snapshot_type="sensor_analytics",
        timestamp=now,
        time_period=time_period,
        data=analytics.dict(exclude={"time_period"})
    )
    
    db.add(snapshot)
    await db.commit()
    
    return analytics
