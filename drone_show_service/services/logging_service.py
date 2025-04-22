"""
Logging service for the Drone Show microservice.

This module provides services for logging drone show execution.
"""

import logging
import json
from typing import Dict, Any, List, Optional
from datetime import datetime
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

from drone_show_service.models.database import ExecutionLogDB
from drone_show_service.core.config import settings
from drone_show_service.services.minio_service import MinioService

logger = logging.getLogger(__name__)


class LoggingService:
    """
    Service for logging drone show execution.
    
    This service provides methods for logging events during drone show execution,
    including errors, warnings, and informational messages.
    """
    
    def __init__(self, minio_service: Optional[MinioService] = None):
        """
        Initialize the logging service.
        
        Args:
            minio_service: MinIO service for storing log data
        """
        self.minio_service = minio_service or MinioService()
    
    async def log_event(
        self, db: AsyncSession, execution_id: str, level: str, message: str, data: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Log an event.
        
        Args:
            db: Database session
            execution_id: Execution ID
            level: Log level (INFO, WARNING, ERROR)
            message: Log message
            data: Additional data
            
        Returns:
            Log entry ID
        """
        # Create database model
        db_log = ExecutionLogDB(
            execution_id=execution_id,
            level=level,
            message=message,
            data=data,
        )
        
        # Add to database
        db.add(db_log)
        await db.commit()
        await db.refresh(db_log)
        
        # Log to console
        log_method = getattr(logger, level.lower(), logger.info)
        log_method(f"[{execution_id}] {message}")
        
        return db_log.id
    
    async def log_info(
        self, db: AsyncSession, execution_id: str, message: str, data: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Log an informational message.
        
        Args:
            db: Database session
            execution_id: Execution ID
            message: Log message
            data: Additional data
            
        Returns:
            Log entry ID
        """
        return await self.log_event(db, execution_id, "INFO", message, data)
    
    async def log_warning(
        self, db: AsyncSession, execution_id: str, message: str, data: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Log a warning message.
        
        Args:
            db: Database session
            execution_id: Execution ID
            message: Log message
            data: Additional data
            
        Returns:
            Log entry ID
        """
        return await self.log_event(db, execution_id, "WARNING", message, data)
    
    async def log_error(
        self, db: AsyncSession, execution_id: str, message: str, data: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Log an error message.
        
        Args:
            db: Database session
            execution_id: Execution ID
            message: Log message
            data: Additional data
            
        Returns:
            Log entry ID
        """
        return await self.log_event(db, execution_id, "ERROR", message, data)
    
    async def get_logs(
        self, db: AsyncSession, execution_id: str, level: Optional[str] = None, skip: int = 0, limit: int = 100
    ) -> List[Dict[str, Any]]:
        """
        Get logs for an execution.
        
        Args:
            db: Database session
            execution_id: Execution ID
            level: Filter by log level
            skip: Number of records to skip
            limit: Maximum number of records to return
            
        Returns:
            List of log entries
        """
        # Build query
        query = select(ExecutionLogDB).where(ExecutionLogDB.execution_id == execution_id)
        
        # Apply filters
        if level:
            query = query.where(ExecutionLogDB.level == level)
        
        # Apply sorting
        query = query.order_by(ExecutionLogDB.timestamp.desc())
        
        # Apply pagination
        query = query.offset(skip).limit(limit)
        
        # Execute query
        result = await db.execute(query)
        db_logs = result.scalars().all()
        
        # Convert to dictionaries
        return [
            {
                "id": log.id,
                "execution_id": log.execution_id,
                "timestamp": log.timestamp.isoformat(),
                "level": log.level,
                "message": log.message,
                "data": log.data,
            }
            for log in db_logs
        ]
    
    async def export_logs(self, db: AsyncSession, execution_id: str) -> Optional[str]:
        """
        Export logs for an execution.
        
        Args:
            db: Database session
            execution_id: Execution ID
            
        Returns:
            URL to exported logs
        """
        try:
            # Get all logs
            logs = await self.get_logs(db, execution_id, skip=0, limit=10000)
            
            # Export to MinIO
            file_key = f"logs/{execution_id}/logs.json"
            success = await self.minio_service.put_json(file_key, logs)
            
            if success:
                return file_key
            
            return None
        except Exception as e:
            logger.error(f"Error exporting logs for execution {execution_id}: {str(e)}")
            return None
