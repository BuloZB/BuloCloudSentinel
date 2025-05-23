"""
Worker application for the Mapping Service.
"""

import asyncio
import logging
import os
import signal
import sys
import uuid
from typing import List

from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker

from mapping_service.core.config import settings
from mapping_service.processing.worker import ProcessingWorker

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)

logger = logging.getLogger(__name__)

# Create async engine
engine = create_async_engine(
    settings.SQLALCHEMY_DATABASE_URI,
    echo=False,
    future=True,
)

# Create async session factory
async_session_factory = sessionmaker(
    engine, class_=AsyncSession, expire_on_commit=False
)

# Global variables
workers: List[ProcessingWorker] = []
worker_tasks: List[asyncio.Task] = []
shutdown_event = asyncio.Event()


async def run_worker(worker_id: int):
    """
    Run a worker.
    
    Args:
        worker_id: Worker ID
    """
    logger.info(f"Starting worker {worker_id}")
    
    # Create session
    async with async_session_factory() as session:
        # Create worker
        worker = ProcessingWorker(session)
        workers.append(worker)
        
        # Run worker until shutdown
        while not shutdown_event.is_set():
            try:
                # Get next job
                job = await worker.get_next_job()
                
                if job:
                    # Process job
                    await worker.process_job(job)
                else:
                    # No jobs, wait before checking again
                    await asyncio.sleep(5)
            except Exception as e:
                logger.error(f"Error in worker {worker_id}: {str(e)}")
                await asyncio.sleep(10)  # Wait longer after an error
    
    logger.info(f"Worker {worker_id} stopped")


async def shutdown():
    """Shutdown the worker application."""
    logger.info("Shutting down workers")
    
    # Set shutdown event
    shutdown_event.set()
    
    # Wait for workers to stop
    if worker_tasks:
        await asyncio.gather(*worker_tasks, return_exceptions=True)
    
    logger.info("All workers stopped")


def handle_signals():
    """Handle signals."""
    loop = asyncio.get_event_loop()
    
    # Add signal handlers
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(shutdown()))


async def main():
    """Main entry point."""
    logger.info("Starting Mapping Service Worker")
    
    # Create temporary directory if it doesn't exist
    os.makedirs(settings.TEMP_DIRECTORY, exist_ok=True)
    
    # Handle signals
    handle_signals()
    
    # Start workers
    for i in range(settings.WORKER_CONCURRENCY):
        task = asyncio.create_task(run_worker(i))
        worker_tasks.append(task)
    
    # Wait for shutdown
    await shutdown_event.wait()
    
    logger.info("Mapping Service Worker stopped")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Unhandled exception: {str(e)}")
        sys.exit(1)
