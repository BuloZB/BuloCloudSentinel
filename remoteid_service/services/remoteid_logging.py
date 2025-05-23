"""
Remote ID logging service for the Remote ID & Regulatory Compliance Service.

This module provides functionality for logging Remote ID broadcasts.
"""

import asyncio
import logging
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Tuple

from sqlalchemy import select, func, delete
from sqlalchemy.ext.asyncio import AsyncSession
from geoalchemy2.functions import ST_MakePoint, ST_SetSRID

from remoteid_service.api.schemas.remoteid import (
    BroadcastLog,
    BroadcastMethod,
    Position,
    RemoteIDMode,
    Velocity,
)
from remoteid_service.core.settings import Settings
from remoteid_service.db.models import RemoteIDBroadcast

# Configure logging
logger = logging.getLogger(__name__)

class RemoteIDLoggingService:
    """
    Remote ID logging service.
    
    This service manages logging of Remote ID broadcasts and provides
    functionality for retrieving and managing logs.
    """
    
    def __init__(self, settings: Settings):
        """
        Initialize the Remote ID logging service.
        
        Args:
            settings: Application settings
        """
        self.settings = settings
        self.maintenance_task = None
        self.running = False
    
    async def start(self) -> None:
        """
        Start the Remote ID logging service.
        """
        if self.running:
            return
        
        self.running = True
        
        # Start maintenance task
        self.maintenance_task = asyncio.create_task(self._maintenance_loop())
        logger.info("Started Remote ID logging service")
    
    async def stop(self) -> None:
        """
        Stop the Remote ID logging service.
        """
        if not self.running:
            return
        
        self.running = False
        
        # Stop maintenance task
        if self.maintenance_task:
            self.maintenance_task.cancel()
            try:
                await self.maintenance_task
            except asyncio.CancelledError:
                pass
            self.maintenance_task = None
        
        logger.info("Stopped Remote ID logging service")
    
    async def _maintenance_loop(self) -> None:
        """
        Maintenance loop for the Remote ID logging service.
        
        This method runs in a background task and periodically performs
        maintenance tasks, such as purging old logs.
        """
        while self.running:
            try:
                # Perform maintenance tasks
                await self._purge_old_logs()
                
                # Wait for next maintenance interval (every hour)
                await asyncio.sleep(3600)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in maintenance loop: {str(e)}")
                await asyncio.sleep(60)  # Avoid tight loop on error
    
    async def _purge_old_logs(self) -> None:
        """
        Purge old Remote ID broadcast logs.
        
        This method deletes logs that are older than the retention period.
        """
        try:
            # Create async session
            async with AsyncSession() as session:
                # Calculate cutoff time
                cutoff_time = datetime.utcnow() - timedelta(hours=self.settings.BROADCAST_LOG_RETENTION_HOURS)
                
                # Delete old logs
                result = await session.execute(
                    delete(RemoteIDBroadcast).where(RemoteIDBroadcast.timestamp < cutoff_time)
                )
                
                # Commit changes
                await session.commit()
                
                # Log result
                logger.info(f"Purged {result.rowcount} old Remote ID broadcast logs")
        except Exception as e:
            logger.error(f"Error purging old logs: {str(e)}")
    
    async def log_broadcast(
        self,
        drone_id: str,
        mode: RemoteIDMode,
        method: BroadcastMethod,
        position: Position,
        velocity: Optional[Velocity] = None,
        operator_id: Optional[str] = None,
        serial_number: Optional[str] = None,
        session_id: Optional[str] = None,
        message_data: Optional[Dict[str, Any]] = None,
        timestamp: Optional[datetime] = None,
        db: AsyncSession = None,
    ) -> None:
        """
        Log a Remote ID broadcast.
        
        Args:
            drone_id: Drone ID
            mode: Broadcast mode
            method: Broadcast method
            position: Position
            velocity: Velocity
            operator_id: Operator ID
            serial_number: Serial number
            session_id: Session ID
            message_data: Message data
            timestamp: Timestamp
            db: Database session
        """
        try:
            # Create new session if not provided
            close_session = False
            if db is None:
                db = AsyncSession()
                close_session = True
            
            # Create log entry
            log_entry = RemoteIDBroadcast(
                drone_id=drone_id,
                timestamp=timestamp or datetime.utcnow(),
                mode=mode,
                method=method,
                location=ST_SetSRID(
                    ST_MakePoint(position.longitude, position.latitude),
                    4326
                ),
                altitude=position.altitude,
                speed=velocity.speed_horizontal if velocity else None,
                heading=velocity.heading if velocity else None,
                operator_id=operator_id,
                serial_number=serial_number,
                session_id=session_id,
                message_data=message_data,
            )
            
            # Add to session
            db.add(log_entry)
            
            # Commit if we created the session
            if close_session:
                await db.commit()
                await db.close()
            
            logger.debug(f"Logged Remote ID broadcast for drone {drone_id}")
        except Exception as e:
            logger.error(f"Error logging Remote ID broadcast: {str(e)}")
            
            # Close session if we created it
            if close_session and db:
                await db.close()
    
    async def get_logs(
        self,
        drone_id: Optional[str] = None,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        limit: Optional[int] = 100,
        offset: Optional[int] = 0,
        db: AsyncSession = None,
    ) -> Tuple[List[BroadcastLog], int]:
        """
        Get Remote ID broadcast logs.
        
        Args:
            drone_id: Drone ID
            start_time: Start time
            end_time: End time
            limit: Limit
            offset: Offset
            db: Database session
            
        Returns:
            Tuple[List[BroadcastLog], int]: Logs and total count
        """
        try:
            # Create new session if not provided
            close_session = False
            if db is None:
                db = AsyncSession()
                close_session = True
            
            # Build query
            query = select(RemoteIDBroadcast)
            count_query = select(func.count()).select_from(RemoteIDBroadcast)
            
            # Add filters
            if drone_id:
                query = query.where(RemoteIDBroadcast.drone_id == drone_id)
                count_query = count_query.where(RemoteIDBroadcast.drone_id == drone_id)
            
            if start_time:
                query = query.where(RemoteIDBroadcast.timestamp >= start_time)
                count_query = count_query.where(RemoteIDBroadcast.timestamp >= start_time)
            
            if end_time:
                query = query.where(RemoteIDBroadcast.timestamp <= end_time)
                count_query = count_query.where(RemoteIDBroadcast.timestamp <= end_time)
            
            # Add order by
            query = query.order_by(RemoteIDBroadcast.timestamp.desc())
            
            # Add limit and offset
            if limit is not None:
                query = query.limit(limit)
            
            if offset is not None:
                query = query.offset(offset)
            
            # Execute queries
            result = await db.execute(query)
            count_result = await db.execute(count_query)
            
            # Get results
            logs = result.scalars().all()
            total = count_result.scalar()
            
            # Close session if we created it
            if close_session:
                await db.close()
            
            # Convert to response models
            broadcast_logs = []
            for log in logs:
                # Extract position from geography
                position = Position(
                    latitude=log.location.latitude if hasattr(log.location, 'latitude') else 0,
                    longitude=log.location.longitude if hasattr(log.location, 'longitude') else 0,
                    altitude=log.altitude,
                )
                
                # Create velocity if available
                velocity = None
                if log.speed is not None or log.heading is not None:
                    velocity = Velocity(
                        speed_horizontal=log.speed or 0,
                        heading=log.heading,
                    )
                
                # Create broadcast log
                broadcast_log = BroadcastLog(
                    id=log.id,
                    drone_id=log.drone_id,
                    timestamp=log.timestamp,
                    mode=log.mode,
                    method=log.method,
                    position=position,
                    velocity=velocity,
                    operator_id=log.operator_id,
                    serial_number=log.serial_number,
                    session_id=log.session_id,
                    message_data=log.message_data,
                )
                
                broadcast_logs.append(broadcast_log)
            
            return broadcast_logs, total
        except Exception as e:
            logger.error(f"Error getting Remote ID broadcast logs: {str(e)}")
            
            # Close session if we created it
            if close_session and db:
                await db.close()
            
            # Re-raise exception
            raise
