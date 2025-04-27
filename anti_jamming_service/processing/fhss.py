"""
Frequency-Hopping Spread Spectrum (FHSS) implementation.

This module provides an implementation of FHSS for resilient communication.
"""

import logging
import numpy as np
from typing import Dict, List, Optional, Tuple, Any, Union
import asyncio
import time
import hashlib
import struct
import os
import secrets
import hmac

from anti_jamming_service.hardware.interfaces import ILoRaInterface
from anti_jamming_service.utils.config import get_config

logger = logging.getLogger(__name__)


class HoppingPattern:
    """
    Frequency hopping pattern generator.

    This class generates pseudo-random frequency hopping patterns for FHSS.
    """

    def __init__(self, num_channels: int, seed: Optional[bytes] = None):
        """
        Initialize the hopping pattern generator.

        Args:
            num_channels: Number of available frequency channels.
            seed: Optional seed for the random number generator.
        """
        self.num_channels = num_channels
        self.seed = seed or os.urandom(16)
        self.pattern = []
        self.index = 0

        # Generate initial pattern
        self._generate_pattern()

    def _generate_pattern(self):
        """Generate a new hopping pattern."""
        # Create a cryptographically secure deterministic sequence
        # Use HMAC-SHA256 for secure deterministic randomness
        self.pattern = list(range(self.num_channels))

        # Shuffle the pattern using Fisher-Yates algorithm with secure randomness
        for i in range(len(self.pattern) - 1, 0, -1):
            # Generate secure random index based on seed and current position
            h = hmac.new(self.seed, f"{i}".encode(), hashlib.sha256)
            # Use the first 4 bytes of the hash as a random value
            j = int.from_bytes(h.digest()[:4], byteorder='big') % (i + 1)
            # Swap elements
            self.pattern[i], self.pattern[j] = self.pattern[j], self.pattern[i]

        # Reset index
        self.index = 0

    def next_channel(self) -> int:
        """
        Get the next channel in the hopping pattern.

        Returns:
            int: Channel index.
        """
        # Get current channel
        channel = self.pattern[self.index]

        # Increment index
        self.index = (self.index + 1) % len(self.pattern)

        # Regenerate pattern if we've used all channels
        if self.index == 0:
            # Use current time and last pattern as seed for next pattern
            hash_input = self.seed + struct.pack("!d", time.time())
            self.seed = hashlib.sha256(hash_input).digest()
            self._generate_pattern()

        return channel

    def reset(self, seed: Optional[bytes] = None):
        """
        Reset the hopping pattern.

        Args:
            seed: Optional new seed for the random number generator.
        """
        if seed:
            self.seed = seed
        self._generate_pattern()


class FHSSProtocol:
    """
    Frequency-Hopping Spread Spectrum (FHSS) protocol.

    This class implements a simple FHSS protocol for resilient communication.
    """

    def __init__(self, lora_interface: ILoRaInterface, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the FHSS protocol.

        Args:
            lora_interface: LoRa interface for communication.
            config: Optional configuration parameters.
        """
        self.lora = lora_interface
        self.config = config or {}
        self.enabled = True
        self.num_channels = 10
        self.hop_interval = 1.0  # seconds
        self.base_frequency = 868000000  # Hz
        self.channel_spacing = 200000  # Hz
        self.hopping_pattern = None
        self.current_channel = 0
        self.hopping_task = None
        self.receive_task = None
        self.message_queue = asyncio.Queue()
        self.running = False

        # Load configuration
        self._load_config()

        # Initialize hopping pattern
        self._initialize_hopping_pattern()

    def _load_config(self):
        """Load configuration from the config file."""
        config = get_config().get("processing", {}).get("fhss", {})
        self.config.update(config)

        # Update parameters from config
        self.enabled = self.config.get("enabled", True)
        self.num_channels = self.config.get("num_channels", 10)
        self.hop_interval = self.config.get("hop_interval", 1.0)
        self.base_frequency = self.config.get("base_frequency", 868000000)
        self.channel_spacing = self.config.get("channel_spacing", 200000)

    def _initialize_hopping_pattern(self):
        """Initialize the hopping pattern."""
        # Get seed from config or generate random seed
        seed = self.config.get("seed")
        if seed and isinstance(seed, str):
            seed = seed.encode()

        self.hopping_pattern = HoppingPattern(self.num_channels, seed)

    def _get_frequency(self, channel: int) -> int:
        """
        Get the frequency for a channel.

        Args:
            channel: Channel index.

        Returns:
            int: Frequency in Hz.
        """
        return self.base_frequency + channel * self.channel_spacing

    async def initialize(self) -> bool:
        """
        Initialize the FHSS protocol.

        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        logger.info("Initializing FHSS protocol")

        try:
            # Initialize LoRa interface
            if not await self.lora.initialize():
                logger.error("Failed to initialize LoRa interface")
                return False

            # Set initial frequency
            self.current_channel = self.hopping_pattern.next_channel()
            frequency = self._get_frequency(self.current_channel)
            if not await self.lora.set_frequency(frequency):
                logger.error(f"Failed to set initial frequency: {frequency} Hz")
                return False

            # Enable frequency hopping on LoRa module
            if not await self.lora.set_frequency_hopping(True, int(self.hop_interval * 1000)):
                logger.warning("Failed to enable frequency hopping on LoRa module")
                # Continue anyway, we'll handle hopping in software

            # Start hopping task
            if self.enabled:
                self.running = True
                self.hopping_task = asyncio.create_task(self._hopping_loop())
                self.receive_task = asyncio.create_task(self._receive_loop())

            logger.info("FHSS protocol initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize FHSS protocol: {str(e)}")
            return False

    async def shutdown(self) -> bool:
        """
        Shutdown the FHSS protocol.

        Returns:
            bool: True if shutdown was successful, False otherwise.
        """
        logger.info("Shutting down FHSS protocol")

        try:
            # Stop tasks
            self.running = False

            if self.hopping_task:
                self.hopping_task.cancel()
                try:
                    await self.hopping_task
                except asyncio.CancelledError:
                    pass
                self.hopping_task = None

            if self.receive_task:
                self.receive_task.cancel()
                try:
                    await self.receive_task
                except asyncio.CancelledError:
                    pass
                self.receive_task = None

            # Disable frequency hopping on LoRa module
            await self.lora.set_frequency_hopping(False)

            # Shutdown LoRa interface
            await self.lora.shutdown()

            logger.info("FHSS protocol shut down successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to shut down FHSS protocol: {str(e)}")
            return False

    async def configure(self, config: Dict[str, Any]) -> bool:
        """
        Configure the FHSS protocol.

        Args:
            config: Configuration parameters.

        Returns:
            bool: True if configuration was successful, False otherwise.
        """
        logger.info("Configuring FHSS protocol")

        try:
            # Update configuration
            self.config.update(config)

            # Update parameters
            if "enabled" in config:
                self.enabled = config["enabled"]

            if "num_channels" in config:
                self.num_channels = config["num_channels"]
                self._initialize_hopping_pattern()

            if "hop_interval" in config:
                self.hop_interval = config["hop_interval"]
                # Update hopping interval on LoRa module
                await self.lora.set_frequency_hopping(True, int(self.hop_interval * 1000))

            if "base_frequency" in config:
                self.base_frequency = config["base_frequency"]

            if "channel_spacing" in config:
                self.channel_spacing = config["channel_spacing"]

            if "seed" in config:
                seed = config["seed"]
                if isinstance(seed, str):
                    seed = seed.encode()
                self.hopping_pattern.reset(seed)

            # Restart tasks if enabled
            if self.hopping_task:
                self.hopping_task.cancel()
                try:
                    await self.hopping_task
                except asyncio.CancelledError:
                    pass
                self.hopping_task = None

            if self.receive_task:
                self.receive_task.cancel()
                try:
                    await self.receive_task
                except asyncio.CancelledError:
                    pass
                self.receive_task = None

            if self.enabled:
                self.running = True
                self.hopping_task = asyncio.create_task(self._hopping_loop())
                self.receive_task = asyncio.create_task(self._receive_loop())

            logger.info("FHSS protocol configured successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to configure FHSS protocol: {str(e)}")
            return False

    async def send_message(self, message: bytes) -> bool:
        """
        Send a message using the FHSS protocol.

        Args:
            message: Message to send.

        Returns:
            bool: True if the message was sent successfully, False otherwise.
        """
        if not self.running:
            logger.error("FHSS protocol not running")
            return False

        try:
            # Add packet header with sequence number and timestamp
            # Use cryptographically secure random number for sequence
            seq_num = int.from_bytes(secrets.token_bytes(4), byteorder='big')
            header = struct.pack("!IQ", seq_num, int(time.time()))
            packet = header + message

            # Send packet
            return await self.lora.send_packet(packet)
        except Exception as e:
            logger.error(f"Failed to send message: {str(e)}")
            return False

    async def receive_message(self, timeout: float = 0.0) -> Optional[bytes]:
        """
        Receive a message using the FHSS protocol.

        Args:
            timeout: Timeout in seconds (0 = no timeout).

        Returns:
            Optional[bytes]: Received message, or None if no message was received.
        """
        if not self.running:
            logger.error("FHSS protocol not running")
            return None

        try:
            # Wait for a message with timeout
            if timeout > 0:
                try:
                    message = await asyncio.wait_for(self.message_queue.get(), timeout)
                    return message
                except asyncio.TimeoutError:
                    return None
            else:
                # No timeout, check if there's a message available
                if not self.message_queue.empty():
                    message = await self.message_queue.get()
                    return message
                else:
                    return None
        except Exception as e:
            logger.error(f"Failed to receive message: {str(e)}")
            return None

    async def get_status(self) -> Dict[str, Any]:
        """
        Get the status of the FHSS protocol.

        Returns:
            Dict[str, Any]: Status information.
        """
        return {
            "enabled": self.enabled,
            "running": self.running,
            "num_channels": self.num_channels,
            "hop_interval": self.hop_interval,
            "base_frequency": self.base_frequency,
            "channel_spacing": self.channel_spacing,
            "current_channel": self.current_channel,
            "current_frequency": self._get_frequency(self.current_channel)
        }

    async def _hopping_loop(self):
        """Background task for frequency hopping."""
        logger.info("Starting frequency hopping loop")

        while self.running:
            try:
                # Sleep for hop interval
                await asyncio.sleep(self.hop_interval)

                # Get next channel
                self.current_channel = self.hopping_pattern.next_channel()
                frequency = self._get_frequency(self.current_channel)

                # Set new frequency
                if not await self.lora.set_frequency(frequency):
                    logger.error(f"Failed to set frequency: {frequency} Hz")
                else:
                    logger.debug(f"Hopped to channel {self.current_channel} ({frequency} Hz)")
            except asyncio.CancelledError:
                logger.info("Frequency hopping loop cancelled")
                break
            except Exception as e:
                logger.error(f"Error in frequency hopping loop: {str(e)}")
                await asyncio.sleep(1)

    async def _receive_loop(self):
        """Background task for receiving messages."""
        logger.info("Starting message receive loop")

        while self.running:
            try:
                # Receive packet with short timeout
                packet = await self.lora.receive_packet(0.1)

                if packet:
                    # Parse packet header
                    if len(packet) >= 12:  # 4 bytes sequence + 8 bytes timestamp
                        header = packet[:12]
                        message = packet[12:]

                        # Extract sequence number and timestamp
                        seq_num, timestamp = struct.unpack("!IQ", header)

                        # Add message to queue
                        await self.message_queue.put(message)

                        logger.debug(f"Received message: seq={seq_num}, time={timestamp}")
            except asyncio.CancelledError:
                logger.info("Message receive loop cancelled")
                break
            except Exception as e:
                logger.error(f"Error in message receive loop: {str(e)}")
                await asyncio.sleep(1)
