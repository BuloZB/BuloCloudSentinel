"""
Main module for the SATCOM / 5G Fallback Connectivity module.

This module provides the main entry point for the module.
"""

import argparse
import asyncio
import logging
import os
import signal
import sys
from typing import Any, Dict, List, Optional

from .adapters import CommsAdapter, WifiMeshAdapter, FiveGAdapter, IridiumCertusAdapter
from .services import ConnectionMonitor, FallbackManager, WireGuardManager
from .utils import load_config, get_config, setup_logging, RedisStore

logger = logging.getLogger(__name__)


async def main(config_path: str):
    """
    Main entry point.

    Args:
        config_path: Path to configuration file
    """
    # Load configuration
    config = load_config(config_path)
    if not config:
        logger.error(f"Failed to load configuration from {config_path}")
        return 1

    # Set up logging
    log_config = config.get("logging", {})
    setup_logging(
        log_level=log_config.get("level", "INFO"),
        log_file=log_config.get("file"),
        log_format=log_config.get("format"),
    )

    # Initialize Redis store
    redis_config = config.get("redis", {})
    redis_store = RedisStore(
        redis_url=redis_config.get("url", "redis://localhost:6379/0"),
        prefix=redis_config.get("prefix", "comms_fallback:"),
        ttl=redis_config.get("ttl", 3600),
    )

    # Connect to Redis
    if not await redis_store.connect():
        logger.warning("Failed to connect to Redis, continuing without state persistence")

    # Initialize adapters
    adapters = {}
    adapter_configs = config.get("adapters", {})

    for adapter_id, adapter_config in adapter_configs.items():
        if not adapter_config.get("enabled", True):
            logger.info(f"Adapter {adapter_id} is disabled, skipping")
            continue

        try:
            if adapter_id == "wifi_mesh":
                adapter = WifiMeshAdapter(adapter_config)
            elif adapter_id == "five_g":
                adapter = FiveGAdapter(adapter_config)
            elif adapter_id == "iridium":
                adapter = IridiumCertusAdapter(adapter_config)
            else:
                logger.warning(f"Unknown adapter type: {adapter_id}")
                continue

            adapters[adapter_id] = adapter
            logger.info(f"Initialized adapter: {adapter}")

        except Exception as e:
            logger.error(f"Error initializing adapter {adapter_id}: {e}")

    if not adapters:
        logger.error("No adapters initialized, exiting")
        return 1

    # Initialize fallback manager
    fallback_config = config.get("fallback", {})
    monitor_config = config.get("monitor", {})
    wireguard_config = config.get("wireguard", {})

    manager_config = {
        "fallback": fallback_config,
        "monitor": monitor_config,
        "wireguard": wireguard_config,
    }

    fallback_manager = FallbackManager(
        config=manager_config,
        adapters=adapters,
        redis_store=redis_store,
    )

    # Set up signal handlers
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(shutdown(fallback_manager, redis_store)))

    # Start fallback manager
    await fallback_manager.start()
    logger.info("Fallback manager started")

    # Keep running until interrupted
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass

    return 0


async def shutdown(fallback_manager: FallbackManager, redis_store: RedisStore):
    """
    Shutdown the application.

    Args:
        fallback_manager: Fallback manager
        redis_store: Redis store
    """
    logger.info("Shutting down...")

    # Stop fallback manager
    await fallback_manager.stop()

    # Disconnect from Redis
    await redis_store.disconnect()

    # Stop event loop
    loop = asyncio.get_event_loop()
    loop.stop()


def run():
    """Run the application."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="SATCOM / 5G Fallback Connectivity")
    parser.add_argument(
        "--config",
        "-c",
        type=str,
        default="config/comms_fallback.yaml",
        help="Path to configuration file",
    )
    args = parser.parse_args()

    # Run main function
    try:
        loop = asyncio.get_event_loop()
        exit_code = loop.run_until_complete(main(args.config))
        sys.exit(exit_code)
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.exception(f"Unhandled exception: {e}")
        sys.exit(1)


if __name__ == "__main__":
    run()
