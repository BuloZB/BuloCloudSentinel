"""
Attack manager service for the EW service.

This service is responsible for managing electronic attacks, including:
- Creating and scheduling attacks
- Monitoring attack status
- Evaluating attack effectiveness
- Terminating attacks
"""

import asyncio
import logging
import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any
from uuid import UUID

from core.config import settings

logger = logging.getLogger(__name__)

class AttackManager:
    """Attack manager service."""
    
    def __init__(self):
        """Initialize the attack manager."""
        self.attacks = {}
        self.attack_tasks = {}
        self.running = False
    
    async def create_attack(
        self,
        platform_id: str,
        attack_type: str,
        frequency: float,
        bandwidth: float,
        power: float,
        duration: int,
        waveform_id: Optional[str] = None,
        target_id: Optional[str] = None,
        direction: Optional[Dict[str, float]] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Create a new electronic attack.
        
        Args:
            platform_id: EW platform ID
            attack_type: Type of attack (jamming, spoofing, etc.)
            frequency: Center frequency in Hz
            bandwidth: Bandwidth in Hz
            power: Power in Watts
            duration: Duration in seconds
            waveform_id: Optional waveform template ID
            target_id: Optional target ID
            direction: Optional direction (azimuth, elevation)
            metadata: Optional metadata
            
        Returns:
            Created electronic attack
        """
        # Validate parameters
        if power > settings.ATTACK_POWER_LIMIT:
            raise ValueError(f"Power {power} W exceeds maximum allowed power {settings.ATTACK_POWER_LIMIT} W")
        
        if duration > settings.ATTACK_MAX_DURATION:
            raise ValueError(f"Duration {duration} seconds exceeds maximum allowed duration {settings.ATTACK_MAX_DURATION} seconds")
        
        # Create attack
        attack_id = str(uuid.uuid4())
        now = datetime.utcnow()
        attack = {
            "id": attack_id,
            "platform_id": platform_id,
            "attack_type": attack_type,
            "frequency": frequency,
            "bandwidth": bandwidth,
            "power": power,
            "waveform_id": waveform_id,
            "target_id": target_id,
            "start_time": now,
            "end_time": now + timedelta(seconds=duration),
            "status": "scheduled",
            "effectiveness": None,
            "direction": direction,
            "metadata": metadata or {},
            "created_at": now,
            "updated_at": now
        }
        
        # Store attack
        self.attacks[attack_id] = attack
        
        # Schedule attack execution
        if self.running:
            self.attack_tasks[attack_id] = asyncio.create_task(
                self._execute_attack(attack_id)
            )
        
        logger.info(f"Created attack {attack_id} of type {attack_type} on platform {platform_id}")
        
        return attack
    
    async def get_attack(self, attack_id: str) -> Optional[Dict[str, Any]]:
        """
        Get an electronic attack.
        
        Args:
            attack_id: Attack ID
            
        Returns:
            Electronic attack or None if not found
        """
        return self.attacks.get(attack_id)
    
    async def update_attack(self, attack_id: str, update_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Update an electronic attack.
        
        Args:
            attack_id: Attack ID
            update_data: Update data
            
        Returns:
            Updated electronic attack or None if not found
        """
        if attack_id not in self.attacks:
            logger.warning(f"Attack {attack_id} not found")
            return None
        
        # Get attack
        attack = self.attacks[attack_id]
        
        # Check if attack can be updated
        if attack["status"] not in ["scheduled", "active"]:
            raise ValueError(f"Cannot update attack with status {attack['status']}")
        
        # Update attack
        for key, value in update_data.items():
            if key in ["id", "platform_id", "created_at"]:
                continue
            attack[key] = value
        
        # Update timestamp
        attack["updated_at"] = datetime.utcnow()
        
        logger.info(f"Updated attack {attack_id}")
        
        return attack
    
    async def cancel_attack(self, attack_id: str) -> Optional[Dict[str, Any]]:
        """
        Cancel an electronic attack.
        
        Args:
            attack_id: Attack ID
            
        Returns:
            Cancelled electronic attack or None if not found
        """
        if attack_id not in self.attacks:
            logger.warning(f"Attack {attack_id} not found")
            return None
        
        # Get attack
        attack = self.attacks[attack_id]
        
        # Check if attack can be cancelled
        if attack["status"] not in ["scheduled", "active"]:
            raise ValueError(f"Cannot cancel attack with status {attack['status']}")
        
        # Cancel attack task
        if attack_id in self.attack_tasks:
            self.attack_tasks[attack_id].cancel()
            try:
                await self.attack_tasks[attack_id]
            except asyncio.CancelledError:
                pass
            del self.attack_tasks[attack_id]
        
        # Update attack status
        attack["status"] = "cancelled"
        attack["end_time"] = datetime.utcnow()
        attack["updated_at"] = datetime.utcnow()
        
        logger.info(f"Cancelled attack {attack_id}")
        
        return attack
    
    async def _execute_attack(self, attack_id: str):
        """
        Execute an electronic attack.
        
        Args:
            attack_id: Attack ID
        """
        if attack_id not in self.attacks:
            logger.warning(f"Attack {attack_id} not found")
            return
        
        # Get attack
        attack = self.attacks[attack_id]
        
        try:
            # Wait until start time
            now = datetime.utcnow()
            if attack["start_time"] > now:
                wait_seconds = (attack["start_time"] - now).total_seconds()
                logger.info(f"Waiting {wait_seconds} seconds to start attack {attack_id}")
                await asyncio.sleep(wait_seconds)
            
            # Start attack
            logger.info(f"Starting attack {attack_id}")
            attack["status"] = "active"
            attack["updated_at"] = datetime.utcnow()
            
            # In a real implementation, this would send commands to the platform
            # For now, we'll just simulate the attack
            
            # Wait until end time
            now = datetime.utcnow()
            if attack["end_time"] > now:
                wait_seconds = (attack["end_time"] - now).total_seconds()
                logger.info(f"Attack {attack_id} running for {wait_seconds} seconds")
                await asyncio.sleep(wait_seconds)
            
            # End attack
            logger.info(f"Ending attack {attack_id}")
            attack["status"] = "completed"
            attack["effectiveness"] = 0.8  # Simulated effectiveness
            attack["updated_at"] = datetime.utcnow()
            
        except asyncio.CancelledError:
            logger.info(f"Attack {attack_id} cancelled")
            attack["status"] = "cancelled"
            attack["updated_at"] = datetime.utcnow()
            raise
        
        except Exception as e:
            logger.exception(f"Error executing attack {attack_id}: {e}")
            attack["status"] = "failed"
            attack["updated_at"] = datetime.utcnow()
            attack["metadata"]["error"] = str(e)
    
    async def evaluate_effectiveness(self, attack_id: str) -> Optional[float]:
        """
        Evaluate the effectiveness of an electronic attack.
        
        Args:
            attack_id: Attack ID
            
        Returns:
            Effectiveness (0.0 to 1.0) or None if not available
        """
        if attack_id not in self.attacks:
            logger.warning(f"Attack {attack_id} not found")
            return None
        
        # Get attack
        attack = self.attacks[attack_id]
        
        # Check if attack is active or completed
        if attack["status"] not in ["active", "completed"]:
            return None
        
        # In a real implementation, this would evaluate the actual effectiveness
        # For now, we'll just return a simulated value
        effectiveness = 0.8
        
        # Update attack
        attack["effectiveness"] = effectiveness
        attack["updated_at"] = datetime.utcnow()
        
        return effectiveness
