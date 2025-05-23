"""
Database session module for the Remote ID & Regulatory Compliance Service.

This module provides database session management for the service.
"""

import logging
from typing import AsyncGenerator

from sqlalchemy.ext.asyncio import (
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)
from sqlalchemy.orm import declarative_base

from remoteid_service.core.settings import get_settings

# Configure logging
logger = logging.getLogger(__name__)

# Create SQLAlchemy base
Base = declarative_base()

# Create async engine
engine = create_async_engine(
    get_settings().DATABASE_URL,
    echo=get_settings().DEBUG,
    pool_size=get_settings().DATABASE_POOL_SIZE,
    max_overflow=get_settings().DATABASE_MAX_OVERFLOW,
    pool_recycle=get_settings().DATABASE_POOL_RECYCLE,
)

# Create session factory
async_session_factory = async_sessionmaker(
    engine,
    expire_on_commit=False,
    autoflush=False,
    autocommit=False,
)

async def create_db_and_tables() -> None:
    """
    Create database tables.
    
    This function creates all tables defined in SQLAlchemy models.
    """
    try:
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
        logger.info("Database tables created successfully")
    except Exception as e:
        logger.error(f"Error creating database tables: {str(e)}")
        raise

async def get_db_session() -> AsyncGenerator[AsyncSession, None]:
    """
    Get a database session.
    
    This function provides a database session for dependency injection.
    
    Yields:
        AsyncSession: Database session
    """
    async with async_session_factory() as session:
        try:
            yield session
        finally:
            await session.close()
