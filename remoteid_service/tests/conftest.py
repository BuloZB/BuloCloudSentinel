"""
Pytest configuration for the Remote ID & Regulatory Compliance Service.

This module provides fixtures for testing the Remote ID & Regulatory Compliance Service.
"""

import asyncio
import os
import pytest
from typing import AsyncGenerator, Dict, Any

from fastapi import FastAPI
from fastapi.testclient import TestClient
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import NullPool

from remoteid_service.core.settings import Settings, get_settings
from remoteid_service.db.session import Base
from remoteid_service.main import app as main_app

# Test database URL
TEST_DATABASE_URL = "postgresql+asyncpg://postgres:placeholderpassword@localhost:5432/test_remoteid"

# Override settings for testing
@pytest.fixture
def test_settings() -> Settings:
    """
    Test settings fixture.
    
    Returns:
        Settings: Test settings
    """
    return Settings(
        DATABASE_URL=TEST_DATABASE_URL,
        REDIS_URL="redis://:placeholderpassword@localhost:6379/1",
        SECRET_KEY="test_secret_key",
        ENVIRONMENT="test",
        DEBUG=True,
        LOG_LEVEL="DEBUG",
        ENABLE_BROADCAST=False,
        ENABLE_HARDWARE=False,
    )

# Override get_settings for testing
@pytest.fixture
def app(test_settings: Settings) -> FastAPI:
    """
    FastAPI app fixture.
    
    Args:
        test_settings: Test settings
        
    Returns:
        FastAPI: FastAPI app
    """
    # Override get_settings
    main_app.dependency_overrides[get_settings] = lambda: test_settings
    
    return main_app

# Test client
@pytest.fixture
def client(app: FastAPI) -> TestClient:
    """
    Test client fixture.
    
    Args:
        app: FastAPI app
        
    Returns:
        TestClient: Test client
    """
    return TestClient(app)

# Async database session
@pytest.fixture
async def db_engine():
    """
    Database engine fixture.
    
    Yields:
        AsyncEngine: Async database engine
    """
    engine = create_async_engine(
        TEST_DATABASE_URL,
        echo=True,
        poolclass=NullPool,
    )
    
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
        await conn.run_sync(Base.metadata.create_all)
    
    yield engine
    
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
    
    await engine.dispose()

@pytest.fixture
async def db_session(db_engine) -> AsyncGenerator[AsyncSession, None]:
    """
    Database session fixture.
    
    Args:
        db_engine: Database engine
        
    Yields:
        AsyncSession: Async database session
    """
    async_session = sessionmaker(
        db_engine,
        class_=AsyncSession,
        expire_on_commit=False,
    )
    
    async with async_session() as session:
        yield session
        await session.rollback()

# Event loop
@pytest.fixture(scope="session")
def event_loop():
    """
    Event loop fixture.
    
    Yields:
        EventLoop: Event loop
    """
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()
