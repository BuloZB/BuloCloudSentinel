"""
Database initialization script for the Drone Show microservice.

This script initializes the database for the Drone Show microservice.
"""

import asyncio
import logging
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.sql import text

from drone_show_service.core.config import settings
from drone_show_service.models.database import Base


async def init_db():
    """Initialize the database."""
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
    
    # Create tables
    logger.info("Creating tables...")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
        await conn.run_sync(Base.metadata.create_all)
    
    logger.info("Database initialized successfully")


if __name__ == "__main__":
    asyncio.run(init_db())
