"""
Secret rotation for Bulo.Cloud Sentinel.

This module provides functionality for rotating secrets automatically.
"""

import os
import time
import logging
import threading
import random
import string
from typing import Dict, Optional, Callable, List, Any
from datetime import datetime, timedelta

from .secrets_manager import secrets_manager, get_secret, set_secret

# Configure logging
logger = logging.getLogger(__name__)


class SecretRotator:
    """
    Secret rotator for automatic secret rotation.
    
    This class provides functionality for rotating secrets automatically
    based on a schedule or on-demand.
    """
    
    def __init__(self, rotation_interval: int = 86400):
        """
        Initialize the secret rotator.
        
        Args:
            rotation_interval: Default rotation interval in seconds (default: 1 day)
        """
        self.rotation_interval = rotation_interval
        self.secret_configs: Dict[str, Dict[str, Any]] = {}
        self.last_rotation: Dict[str, float] = {}
        self.rotation_lock = threading.RLock()
        self.rotation_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        
        logger.info(f"Initialized secret rotator with default interval {rotation_interval}s")
    
    def add_secret(
        self,
        key: str,
        generator: Optional[Callable[[], str]] = None,
        interval: Optional[int] = None,
        backend_index: int = 0,
        notify_callback: Optional[Callable[[str, str], None]] = None
    ):
        """
        Add a secret for rotation.
        
        Args:
            key: Secret key
            generator: Function to generate a new secret value
            interval: Rotation interval in seconds (default: class default)
            backend_index: Backend index to store the secret
            notify_callback: Callback function to notify about rotation
        """
        with self.rotation_lock:
            self.secret_configs[key] = {
                "generator": generator,
                "interval": interval or self.rotation_interval,
                "backend_index": backend_index,
                "notify_callback": notify_callback
            }
            
            # Initialize last rotation time
            self.last_rotation[key] = time.time()
            
            logger.info(f"Added secret '{key}' for rotation with interval {interval or self.rotation_interval}s")
    
    def remove_secret(self, key: str):
        """
        Remove a secret from rotation.
        
        Args:
            key: Secret key
        """
        with self.rotation_lock:
            if key in self.secret_configs:
                del self.secret_configs[key]
                if key in self.last_rotation:
                    del self.last_rotation[key]
                
                logger.info(f"Removed secret '{key}' from rotation")
    
    def rotate_secret(self, key: str) -> bool:
        """
        Rotate a secret.
        
        Args:
            key: Secret key
            
        Returns:
            True if rotation was successful, False otherwise
        """
        with self.rotation_lock:
            if key not in self.secret_configs:
                logger.warning(f"[REDACTED])
                return False
            
            config = self.secret_configs[key]
            
            try:
                # Get current value
                old_value = get_secret(key)
                
                # Generate new value
                generator = config.get("generator")
                if generator:
                    new_value = generator()
                else:
                    # Default generator: 32 random characters
                    new_value = "".join(random.choices(
                        string.ascii_letters + string.digits,
                        k=32
                    ))
                
                # Set new value
                backend_index = config.get("backend_index", 0)
                if not set_secret(key, new_value, backend_index):
                    logger.error(f"Failed to set new value for [REDACTED])
                    return False
                
                # Update last rotation time
                self.last_rotation[key] = time.time()
                
                # Notify callback
                notify_callback = config.get("notify_callback")
                if notify_callback and old_value:
                    try:
                        notify_callback(key, new_value)
                    except Exception as e:
                        logger.error(f"Error in notify callback for [REDACTED])}")
                
                logger.info(f"Rotated [REDACTED])
                return True
            
            except Exception as e:
                logger.error(f"Error rotating [REDACTED])}")
                return False
    
    def rotate_all_secrets(self) -> Dict[str, bool]:
        """
        Rotate all secrets.
        
        Returns:
            Dictionary mapping secret keys to rotation success
        """
        results = {}
        
        with self.rotation_lock:
            for key in list(self.secret_configs.keys()):
                results[key] = self.rotate_secret(key)
        
        return results
    
    def rotate_due_secrets(self) -> Dict[str, bool]:
        """
        Rotate secrets that are due for rotation.
        
        Returns:
            Dictionary mapping secret keys to rotation success
        """
        results = {}
        now = time.time()
        
        with self.rotation_lock:
            for key, config in list(self.secret_configs.items()):
                interval = config.get("interval", self.rotation_interval)
                last_rotation = self.last_rotation.get(key, 0)
                
                if now - last_rotation >= interval:
                    results[key] = self.rotate_secret(key)
        
        return results
    
    def start_rotation_thread(self, check_interval: int = 60):
        """
        Start the automatic rotation thread.
        
        Args:
            check_interval: Interval in seconds to check for due secrets
        """
        if self.rotation_thread and self.rotation_thread.is_alive():
            logger.warning("Rotation thread already running")
            return
        
        self.stop_event.clear()
        
        def rotation_worker():
            logger.info("Started secret rotation thread")
            
            while not self.stop_event.is_set():
                try:
                    # Rotate due secrets
                    results = self.rotate_due_secrets()
                    
                    # Log results
                    rotated = [key for key, success in results.items() if success]
                    failed = [key for key, success in results.items() if not success]
                    
                    if rotated:
                        logger.info(f"Rotated {len(rotated)} secrets: {', '.join(rotated)}")
                    
                    if failed:
                        logger.error(f"Failed to rotate {len(failed)} secrets: {', '.join(failed)}")
                
                except Exception as e:
                    logger.error(f"Error in rotation thread: {str(e)}")
                
                # Wait for next check
                self.stop_event.wait(check_interval)
            
            logger.info("Stopped secret rotation thread")
        
        self.rotation_thread = threading.Thread(
            target=rotation_worker,
            daemon=True,
            name="SecretRotator"
        )
        self.rotation_thread.start()
        
        logger.info(f"Started secret rotation thread with check interval {check_interval}s")
    
    def stop_rotation_thread(self):
        """Stop the automatic rotation thread."""
        if not self.rotation_thread or not self.rotation_thread.is_alive():
            logger.warning("Rotation thread not running")
            return
        
        self.stop_event.set()
        self.rotation_thread.join(timeout=10)
        
        if self.rotation_thread.is_alive():
            logger.warning("Rotation thread did not stop gracefully")
        else:
            logger.info("Stopped secret rotation thread")
    
    def get_rotation_status(self) -> Dict[str, Dict[str, Any]]:
        """
        Get the rotation status of all secrets.
        
        Returns:
            Dictionary mapping secret keys to rotation status
        """
        status = {}
        now = time.time()
        
        with self.rotation_lock:
            for key, config in self.secret_configs.items():
                interval = config.get("interval", self.rotation_interval)
                last_rotation = self.last_rotation.get(key, 0)
                
                # Calculate next rotation time
                next_rotation = last_rotation + interval
                
                # Calculate time until next rotation
                time_until_next = max(0, next_rotation - now)
                
                # Calculate percentage until next rotation
                percentage = min(100, max(0, (now - last_rotation) / interval * 100))
                
                status[key] = {
                    "last_rotation": datetime.fromtimestamp(last_rotation).isoformat(),
                    "next_rotation": datetime.fromtimestamp(next_rotation).isoformat(),
                    "time_until_next": str(timedelta(seconds=int(time_until_next))),
                    "percentage": round(percentage, 2),
                    "interval": str(timedelta(seconds=interval)),
                    "due": time_until_next <= 0
                }
        
        return status


# Global secret rotator instance
secret_rotator = SecretRotator()


def add_secret_for_rotation(
    key: str,
    generator: Optional[Callable[[], str]] = None,
    interval: Optional[int] = None,
    backend_index: int = 0,
    notify_callback: Optional[Callable[[str, str], None]] = None
):
    """
    Add a secret for rotation.
    
    Args:
        key: Secret key
        generator: Function to generate a new secret value
        interval: Rotation interval in seconds
        backend_index: Backend index to store the secret
        notify_callback: Callback function to notify about rotation
    """
    secret_rotator.add_secret(
        key=key,
        generator=generator,
        interval=interval,
        backend_index=backend_index,
        notify_callback=notify_callback
    )


def remove_secret_from_rotation(key: str):
    """
    Remove a secret from rotation.
    
    Args:
        key: Secret key
    """
    secret_rotator.remove_secret(key)


def rotate_secret(key: str) -> bool:
    """
    Rotate a secret.
    
    Args:
        key: Secret key
        
    Returns:
        True if rotation was successful, False otherwise
    """
    return secret_rotator.rotate_secret(key)


def rotate_all_secrets() -> Dict[str, bool]:
    """
    Rotate all secrets.
    
    Returns:
        Dictionary mapping secret keys to rotation success
    """
    return secret_rotator.rotate_all_secrets()


def start_secret_rotation(check_interval: int = 60):
    """
    Start automatic secret rotation.
    
    Args:
        check_interval: Interval in seconds to check for due secrets
    """
    secret_rotator.start_rotation_thread(check_interval)


def stop_secret_rotation():
    """Stop automatic secret rotation."""
    secret_rotator.stop_rotation_thread()


def get_rotation_status() -> Dict[str, Dict[str, Any]]:
    """
    Get the rotation status of all secrets.
    
    Returns:
        Dictionary mapping secret keys to rotation status
    """
    return secret_rotator.get_rotation_status()
